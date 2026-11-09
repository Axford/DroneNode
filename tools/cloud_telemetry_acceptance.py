#!/usr/bin/env python3
"""Explicit local fault endpoint and sanitized summaries; never opens/programs a node."""
import argparse
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import socketserver
import struct
import threading
import ipaddress
import re
import subprocess
import tempfile
import os
import ssl
import time

DUMMY_API_KEY = 'G4-DUMMY-ONLY'
HTTP_MODES = ('valid','partial','stall','lost','oversize','redirect','malformed','header-oversize','slow-response','status-401','status-404','status-409','status-429','status-503','unexpected-200','tls-stall','tls-reject')

METRICS = ('freeHeap', 'minHeap', 'stackBytes', 'serviceGapMs', 'reportDurationMs', 'httpStatus')


def packet_count(encoded):
    raw = base64.b64decode(encoded, validate=True)
    if base64.b64encode(raw).decode() != encoded or not 6 <= len(raw) <= 672:
        raise ValueError('invalid packed length/base64')
    count = offset = 0
    while offset < len(raw):
        if len(raw) - offset < 6:
            raise ValueError('truncated packet')
        length = 5 + (raw[offset + 4] & 15) + 1
        offset += length
        count += 1
        if offset > len(raw) or count > 32:
            raise ValueError('invalid packet boundary/count')
    return count


def handler(mode, delay):
    class FaultHandler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *_):
            pass  # Neither API keys nor private packet/identity data enter logs.

        def do_POST(self):
            if self.path != '/addPackets':
                self.send_error(404)
                return
            try:
                size = int(self.headers.get('Content-Length', '-1'))
                if not 0 < size <= 4096:
                    raise ValueError('body limit')
                request = json.loads(self.rfile.read(size))
                if not isinstance(request,dict):raise ValueError('object required')
                if request.get('apiKey') != DUMMY_API_KEY:
                    self.send_error(403, 'Use the dummy fixture key');return
                count = packet_count(request['packets'])
            except (ValueError, KeyError, TypeError):
                self.send_error(400)
                self.close_connection = True
                return
            print(json.dumps({'requestBytes': size, 'packets': count, 'mode': mode}), flush=True)
            if mode == 'stall':
                self.server.pause(delay) if hasattr(self.server, 'pause') else time.sleep(delay)
            if mode == 'lost':
                self.close_connection = True
                self.connection.shutdown(socket.SHUT_RDWR)
                return
            if mode == 'oversize':
                self.send_response(202)
                self.send_header('Content-Length', '2048')
                self.end_headers()
                self.wfile.write(b'x' * 2048)
                return
            if mode == 'redirect':
                self.send_response(302)
                self.send_header('Location', 'http://invalid.example/addPackets')
                self.send_header('Content-Length', '0')
                self.end_headers()
                return
            if mode == 'header-oversize':
                self.send_response(202);self.send_header('X-Fault', 'x'*1500);self.send_header('Content-Length','0');self.end_headers();return
            if mode.startswith('status-'):
                self.send_response(int(mode[7:]));self.send_header('Content-Length','0');self.end_headers();return
            partial = mode == 'partial'
            response = {'received': count, 'admitted': max(0, count - 1) if partial else count, 'complete': not partial}
            if partial:
                response['code'] = 'admission-incomplete'
            body = b'{malformed' if mode == 'malformed' else json.dumps(response, separators=(',', ':')).encode()
            self.send_response(200 if mode == 'unexpected-200' else 503 if partial else 202)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            if mode == 'slow-response':
                for byte in body:
                    if hasattr(self.server,'pause') and not self.server.pause(delay/len(body)):break
                    self.wfile.write(bytes([byte]));self.wfile.flush()
            else:
                self.wfile.write(body)
    return FaultHandler


def local_ipv4(value):
    address=ipaddress.IPv4Address(value)
    if not (address.is_private or address.is_loopback) or address.is_unspecified or address.is_multicast or address.is_reserved:
        raise ValueError('use an explicit local unicast IPv4 address')
    return str(address)


class FaultServer(ThreadingHTTPServer):
    """Bounded accepted connections; TLS handshakes occur in worker threads."""
    daemon_threads=True
    block_on_close=False
    def __init__(self,address,mode,delay,context):
        if mode not in HTTP_MODES or not 0<=delay<=120:raise ValueError('invalid fault mode/delay')
        self.mode,self.delay,self.context=mode,delay,context
        self.stopping=threading.Event();self.slots=threading.BoundedSemaphore(8)
        self.connections=set();self.guard=threading.Lock()
        super().__init__(address,handler(mode,delay))
    def pause(self,delay):return not self.stopping.wait(delay)
    def process_request(self,request,address):
        if not self.slots.acquire(blocking=False):self.shutdown_request(request);return
        try:super().process_request(request,address)
        except BaseException:self.slots.release();raise
    def process_request_thread(self,request,address):
        try:
            with self.guard:
                if self.stopping.is_set():request.close();return
                self.connections.add(request)
            super().process_request_thread(request,address)
        finally:
            with self.guard:self.connections.discard(request)
            self.slots.release()
    def finish_request(self,request,address):
        request.settimeout(6)
        if self.mode in ('tls-stall','tls-reject'):
            print(json.dumps({'connection':1,'mode':self.mode}),flush=True)
            if self.mode=='tls-stall':self.pause(self.delay)
            else:request.sendall(b'not a TLS response')
            return
        # Keep ownership visible while wrapping so shutdown can close a stalled
        # handshake as well as a stalled HTTP response.
        with self.guard:
            if self.stopping.is_set():return
            secure=self.context.wrap_socket(request,server_side=True,do_handshake_on_connect=False)
            self.connections.add(secure)
        try:
            secure.settimeout(6);secure.do_handshake()
            self.RequestHandlerClass(secure,address,self)
        finally:
            with self.guard:self.connections.discard(secure)
            secure.close()
    def handle_error(self,*_):
        pass # Client timeout/disconnect is expected. Never print request/exception text.
    def server_close(self):
        self.stopping.set()
        with self.guard:
            for connection in self.connections:
                try:connection.shutdown(socket.SHUT_RDWR)
                except OSError:pass
                connection.close()
        super().server_close()


def dns_response(raw,name,answer,mode):
    """Single exact-name IPv4 test zone, TTL0, no recursion or forwarding."""
    if len(raw)<12 or len(raw)>512:return None
    ident,flags,qd,an,ns,ar=struct.unpack('!6H',raw[:12])
    if flags&0xf800 or (qd,an,ns,ar)!=(1,0,0,0):return None
    labels=[];offset=12
    while offset<len(raw):
        size=raw[offset];offset+=1
        if not size:break
        if size>63 or offset+size>len(raw):return None
        try:labels.append(raw[offset:offset+size].decode('ascii').lower())
        except UnicodeError:return None
        offset+=size
    else:return None
    if offset+4!=len(raw):return None
    kind,klass=struct.unpack('!HH',raw[offset:]);question=raw[12:]
    matched='.'.join(labels)==name.lower() and klass==1
    if matched and mode=='silent':return None
    rcode=5 if not matched else 3 if mode=='nxdomain' else 0
    payload=b''
    if matched and mode=='answer' and kind==1:
        payload=b'\xc0\x0c'+struct.pack('!HHIH',1,1,0,4)+socket.inet_aton(answer)
    return struct.pack('!6H',ident,0x8400|(flags&0x100)|rcode,1,bool(payload),0,0)+question+payload


class DnsServer(socketserver.UDPServer):
    def __init__(self,address,name,answer,mode):
        if mode not in ('answer','nxdomain','silent') or not re.fullmatch(r'[a-z0-9-]+(?:\.[a-z0-9-]+)*\.test',name) or len(name)>200:
            raise ValueError('use a bounded explicit .test name and DNS mode')
        if answer:answer=local_ipv4(answer)
        if mode=='answer' and not answer:raise ValueError('answer mode needs a local IPv4 address')
        class Handler(socketserver.BaseRequestHandler):
            def handle(self):
                raw,sock=self.request
                response=dns_response(raw,name,answer,mode)
                # Probe validation without retaining/logging names or packet bytes.
                probe=dns_response(raw,name,answer,'nxdomain')
                if probe is not None:
                    print(json.dumps({'dnsQuery':1,'matched':int((probe[3]&15)==3),'rcode':-1 if response is None else response[3]&15,'mode':mode}),flush=True)
                if response is not None:sock.sendto(response,self.client_address)
        super().__init__(address,Handler)


def certificates(output,name):
    if not re.fullmatch(r'[a-z0-9-]+(?:\.[a-z0-9-]+)*\.test',name) or len(name)>200:raise ValueError('use a bounded .test name')
    output=Path(output);output.mkdir(mode=0o700,parents=False,exist_ok=False)
    def run(args):subprocess.run(['openssl',*args],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    key=output/'key.pem'
    run(['genpkey','-algorithm','RSA','-pkeyopt','rsa_keygen_bits:2048','-out',str(key)]);key.chmod(0o600)
    for filename,hostname in [('valid.pem',name),('wrong-host.pem','different.invalid')]:
        run(['req','-new','-x509','-key',str(key),'-days','1','-subj','/CN='+hostname,'-addext','subjectAltName=DNS:'+hostname,'-out',str(output/filename)])
    with tempfile.TemporaryDirectory(prefix='cloud-fault-cert-') as temporary:
        directory=Path(temporary);(directory/'index').write_text('');(directory/'serial').write_text('01\n');(directory/'newcerts').mkdir()
        (directory/'ca.cnf').write_text('[ca]\ndefault_ca=local\n[local]\ndatabase='+str(directory/'index')+'\nserial='+str(directory/'serial')+'\nnew_certs_dir='+str(directory/'newcerts')+'\ndefault_md=sha256\npolicy=names\n[names]\ncommonName=supplied\n[leaf]\nsubjectAltName=DNS:'+name+'\n')
        csr=directory/'request.pem'
        run(['req','-new','-key',str(key),'-subj','/CN='+name,'-out',str(csr)])
        run(['ca','-config',str(directory/'ca.cnf'),'-batch','-selfsign','-keyfile',str(key),'-in',str(csr),'-startdate','20200101000000Z','-enddate','20200102000000Z','-extensions','leaf','-out',str(output/'expired.pem'),'-notext'])
    return {'certificates':3,'privateKeyMode':'0600'}


def tcp_drop_rule(node,workstation,interface,port,output):
    node,workstation=local_ipv4(node),local_ipv4(workstation)
    if node==workstation or not re.fullmatch(r'[a-z][a-z0-9]{0,14}',interface) or not 1024<=port<=65535:
        raise ValueError('use distinct local IPv4 addresses, explicit interface and unprivileged test port')
    rule=f'block drop in quick on {interface} inet proto tcp from {node} to {workstation} port = {port} flags S/SA\n'
    fd=os.open(output,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'w') as stream:stream.write(rule)
    return {'dryRun':True,'rules':1,'applied':False,'anchor':'com.apple/dronelink-g4'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest='command', required=True)
    serve = modes.add_parser('serve', help='Run an explicitly selected controlled TLS endpoint, not staging')
    serve.add_argument('--bind', default='127.0.0.1')
    serve.add_argument('--port', type=int, default=8443)
    serve.add_argument('--cert', required=True, type=Path)
    serve.add_argument('--key', required=True, type=Path)
    serve.add_argument('--mode', choices=HTTP_MODES, default='valid')
    serve.add_argument('--delay', type=float, default=10)
    dns=modes.add_parser('dns',help='Owned single-name UDP resolver; never forwards')
    dns.add_argument('--bind',required=True);dns.add_argument('--port',type=int,default=53)
    dns.add_argument('--name',required=True);dns.add_argument('--answer')
    dns.add_argument('--mode',required=True,choices=('answer','nxdomain','silent'))
    cert=modes.add_parser('certificates',help='Create temporary valid/wrong-host/expired self-signed certificates')
    cert.add_argument('--output-dir',type=Path,required=True);cert.add_argument('--name',required=True)
    drop=modes.add_parser('tcp-drop-rule',help='Write one scoped macOS PF SYN-drop rule; never runs pfctl')
    drop.add_argument('--node',required=True);drop.add_argument('--workstation',required=True)
    drop.add_argument('--interface',required=True);drop.add_argument('--port',type=int,required=True)
    drop.add_argument('--output',type=Path,required=True)
    analyze = modes.add_parser('analyze', help='Summarize whitelisted numeric metrics from a private JSON array')
    analyze.add_argument('--input', required=True, type=Path)
    args = parser.parse_args()
    if args.command == 'serve':
        if not 0 < args.port < 65536 or not 0 <= args.delay <= 120:
            parser.error('port or delay out of range')
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(args.cert, args.key)
        server = FaultServer((local_ipv4(args.bind), args.port), args.mode,args.delay,context)
        print(json.dumps({'bind': args.bind, 'port': args.port, 'mode': args.mode}), flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
    elif args.command=='tcp-drop-rule':
        print(json.dumps(tcp_drop_rule(args.node,args.workstation,args.interface,args.port,args.output)))
    elif args.command=='certificates':
        print(json.dumps(certificates(args.output_dir,args.name)))
    elif args.command=='dns':
        if not 0<args.port<65536:parser.error('port out of range')
        server=DnsServer((local_ipv4(args.bind),args.port),args.name,args.answer,args.mode)
        print(json.dumps({'bind':args.bind,'port':args.port,'mode':args.mode}),flush=True)
        try:server.serve_forever()
        except KeyboardInterrupt:pass
        finally:server.server_close()
    else:
        rows = json.loads(args.input.read_text())
        if not isinstance(rows, list) or len(rows) > 100000:
            parser.error('expected a bounded array of metric objects')
        result = {}
        for field in METRICS:
            values = [row[field] for row in rows if isinstance(row, dict) and type(row.get(field)) is int and 0 <= row[field] <= 0xffffffff]
            if values:
                result[field] = {'samples': len(values), 'min': min(values), 'max': max(values)}
        print(json.dumps(result, sort_keys=True))


if __name__ == '__main__':
    main()

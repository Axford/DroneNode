"""Operator tooling never selects/programs a device or contacts managed hosting."""
import contextlib
import http.client
import importlib.util
import io
import json
from pathlib import Path
import shutil
import ssl
import subprocess
import tempfile
import threading
import unittest
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('cloud_acceptance', ROOT/'tools/cloud_telemetry_acceptance.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
class OperatorToolTests(unittest.TestCase):
    def test_known_packet_boundaries_and_sanitized_metrics(self):
        self.assertEqual(module.packet_count('AAsECAAJDAwFCkJBQkM='), 2)
        for invalid in ('', 'AAsECAA=', 'not base64', 'AAsECAAJ='):
            with self.assertRaises(ValueError):
                module.packet_count(invalid)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'metrics.json'
            path.write_text(json.dumps([{'freeHeap': 60000, 'apiKey': 'DO-NOT-PRINT'}, {'freeHeap': 50000, 'serviceGapMs': 10}]))
            result = subprocess.check_output(['python3', str(ROOT/'tools/cloud_telemetry_acceptance.py'), 'analyze', '--input', str(path)], text=True)
            self.assertNotIn('DO-NOT-PRINT', result)
            self.assertEqual(json.loads(result)['freeHeap'], {'samples': 2, 'min': 50000, 'max': 60000})

    @unittest.skipUnless(shutil.which('openssl'), 'openssl supplies only the temporary local test certificate')
    def test_controlled_tls_fault_endpoint(self):
        from http.server import ThreadingHTTPServer
        with tempfile.TemporaryDirectory() as directory:
            cert, key = Path(directory)/'cert.pem', Path(directory)/'key.pem'
            subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '1', '-subj', '/CN=untrusted.test', '-keyout', str(key), '-out', str(cert)], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain(cert, key)
            for mode, status in [('valid', 202), ('partial', 503), ('oversize', 202), ('redirect', 302)]:
                server = ThreadingHTTPServer(('127.0.0.1', 0), module.handler(mode, 0))
                server.socket = context.wrap_socket(server.socket, server_side=True)
                thread = threading.Thread(target=server.serve_forever, daemon=True)
                thread.start()
                output = io.StringIO()
                try:
                    client = http.client.HTTPSConnection('127.0.0.1', server.server_port, context=ssl._create_unverified_context(), timeout=2)
                    with contextlib.redirect_stdout(output):
                        client.request('POST', '/addPackets', json.dumps({'packets': 'AAsECAAJ', 'apiKey': module.DUMMY_API_KEY}), {'Content-Type': 'application/json'})
                        response = client.getresponse()
                        self.assertEqual(response.status, status)
                        response.read()
                    client.close()
                    self.assertNotIn(module.DUMMY_API_KEY, output.getvalue())
                finally:
                    server.shutdown()
                    server.server_close()
                    thread.join()

    def test_dns_wire_responses_and_real_silent_query(self):
        import socket,struct
        query=bytes.fromhex('123401000001000000000000')+b'\x03run\x05cloud\x04test\x00'+struct.pack('!HH',1,1)
        response=module.dns_response(query,'run.cloud.test','192.168.1.50','answer')
        self.assertEqual(response[:12].hex(),'123485000001000100000000')
        self.assertEqual(response[-4:],bytes([192,168,1,50]));self.assertEqual(response[-10:-6],bytes(4)) # TTL0
        self.assertEqual(module.dns_response(query,'run.cloud.test',None,'nxdomain')[3]&15,3)
        self.assertIsNone(module.dns_response(query,'run.cloud.test',None,'silent'))
        self.assertEqual(module.dns_response(query,'other.test',None,'silent')[3]&15,5)
        for raw in [b'',query[:-1],query+b'x',query[:12]+b'\xc0\x0c\x00\x01\x00\x01']:
            self.assertIsNone(module.dns_response(raw,'run.cloud.test',None,'silent'))
        output=io.StringIO()
        with contextlib.redirect_stdout(output):
            server=module.DnsServer(('127.0.0.1',0),'run.cloud.test',None,'silent')
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            try:
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as client:
                    client.settimeout(0.15);client.sendto(query,server.server_address)
                    with self.assertRaises(TimeoutError):client.recvfrom(512)
            finally:server.shutdown();server.server_close();thread.join()
        self.assertIn('"rcode": -1',output.getvalue());self.assertIn('"matched": 1',output.getvalue());self.assertNotIn('run.cloud',output.getvalue())

    @unittest.skipUnless(shutil.which('openssl'),'fixture certificates require local OpenSSL')
    def test_production_fault_server_certificates_and_modes(self):
        import socket
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)/'certs';module.certificates(directory,'run.cloud.test')
            self.assertEqual((directory/'key.pem').stat().st_mode&0o777,0o600)
            for certificate in ['valid.pem','wrong-host.pem','expired.pem']:
                context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER);context.load_cert_chain(directory/certificate,directory/'key.pem')
                server=module.FaultServer(('127.0.0.1',0),'valid',0,context)
                thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
                try:
                    # Trust each self-signed leaf so hostname/expiry checks,
                    # rather than unknown-CA alone, distinguish the cases.
                    verified=ssl.create_default_context(cafile=str(directory/certificate))
                    with socket.create_connection(server.server_address,timeout=2) as raw:
                        if certificate=='valid.pem':
                            with verified.wrap_socket(raw,server_hostname='run.cloud.test'):pass
                        else:
                            with self.assertRaises(ssl.SSLCertVerificationError):verified.wrap_socket(raw,server_hostname='run.cloud.test')
                    client=http.client.HTTPSConnection(*server.server_address,context=ssl._create_unverified_context(),timeout=2)
                    with contextlib.redirect_stdout(io.StringIO()):
                        client.request('POST','/addPackets',json.dumps({'packets':'AAsECAAJ','apiKey':module.DUMMY_API_KEY}),{'Content-Type':'application/json'})
                        self.assertEqual(client.getresponse().status,202);client.close()
                finally:server.shutdown();server.server_close();thread.join()
            for mode in ['malformed','header-oversize','status-401','status-404','status-409','status-429','status-503','unexpected-200','slow-response','lost','tls-stall','tls-reject']:
                server=module.FaultServer(('127.0.0.1',0),mode,0.4 if mode=='tls-stall' else 0.02,context)
                thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
                try:
                    client=http.client.HTTPSConnection(*server.server_address,context=ssl._create_unverified_context(),timeout=0.15 if mode=='tls-stall' else 2)
                    with contextlib.redirect_stdout(io.StringIO()):
                        if mode in ['tls-stall','tls-reject','lost']:
                            with self.assertRaises((OSError,http.client.RemoteDisconnected)):
                                client.request('POST','/addPackets',json.dumps({'packets':'AAsECAAJ','apiKey':module.DUMMY_API_KEY}),{'Content-Type':'application/json'});client.getresponse()
                        else:
                            client.request('POST','/addPackets',json.dumps({'packets':'AAsECAAJ','apiKey':module.DUMMY_API_KEY}),{'Content-Type':'application/json'})
                            response=client.getresponse();body=response.read()
                            if mode=='malformed':self.assertEqual(body,b'{malformed')
                            elif mode=='header-oversize':self.assertEqual(len(response.getheader('X-Fault')),1500)
                            elif mode.startswith('status-'):self.assertEqual(response.status,int(mode[7:]))
                            elif mode=='unexpected-200':self.assertEqual(response.status,200)
                            else:self.assertEqual(response.status,202)
                        client.close()
                finally:server.shutdown();server.server_close();thread.join()

    def test_tcp_drop_rule_is_scoped_dry_run_only(self):
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'drop.conf'
            result=module.tcp_drop_rule('192.168.1.32','192.168.1.50','en1',8443,path)
            self.assertFalse(result['applied']);self.assertEqual(path.stat().st_mode&0o777,0o600)
            self.assertEqual(path.read_text(),'block drop in quick on en1 inet proto tcp from 192.168.1.32 to 192.168.1.50 port = 8443 flags S/SA\n')
            with self.assertRaises(FileExistsError):module.tcp_drop_rule('192.168.1.32','192.168.1.50','en1',8443,path)
            for bad in ['any','0.0.0.0','8.8.8.8']:
                with self.assertRaises(ValueError):module.tcp_drop_rule(bad,'192.168.1.50','en1',8443,path)

    def test_fault_workers_are_bounded_and_shutdown_interrupts_stalls(self):
        import socket,time
        server=module.FaultServer(('127.0.0.1',0),'tls-stall',120,None)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();clients=[]
        output=io.StringIO()
        with contextlib.redirect_stdout(output):
            try:
                for _ in range(8):clients.append(socket.create_connection(server.server_address,timeout=2))
                extra=socket.create_connection(server.server_address,timeout=2);extra.settimeout(2)
                self.assertEqual(extra.recv(1),b'');extra.close()
                started=time.monotonic();server.shutdown();server.server_close();thread.join()
                self.assertLess(time.monotonic()-started,2)
                for client in clients:client.settimeout(2);self.assertEqual(client.recv(1),b'')
            finally:
                for client in clients:client.close()
                if thread.is_alive():server.shutdown();server.server_close();thread.join()

    @unittest.skipUnless(shutil.which('openssl'),'fixture certificates require local OpenSSL')
    def test_fixture_refuses_non_dummy_key_without_echo(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)/'certs';module.certificates(directory,'run.cloud.test')
            context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER);context.load_cert_chain(directory/'valid.pem',directory/'key.pem')
            server=module.FaultServer(('127.0.0.1',0),'valid',0,context)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();output=io.StringIO()
            try:
                with contextlib.redirect_stdout(output):
                    client=http.client.HTTPSConnection(*server.server_address,context=ssl._create_unverified_context(),timeout=2)
                    client.request('POST','/addPackets',json.dumps({'packets':'AAsECAAJ','apiKey':'PRIVATE-MUST-NOT-ECHO'}),{'Content-Type':'application/json'})
                    response=client.getresponse();self.assertEqual(response.status,403);self.assertNotIn(b'PRIVATE-MUST-NOT-ECHO',response.read());client.close()
                self.assertNotIn('PRIVATE-MUST-NOT-ECHO',output.getvalue())
            finally:server.shutdown();server.server_close();thread.join()

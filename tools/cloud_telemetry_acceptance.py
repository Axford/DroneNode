#!/usr/bin/env python3
"""Explicit local fault endpoint and sanitized summaries; never opens/programs a node."""
import argparse
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import ssl
import time

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
                count = packet_count(request['packets'])
            except (ValueError, KeyError, TypeError):
                self.send_error(400)
                self.close_connection = True
                return
            print(json.dumps({'requestBytes': size, 'packets': count, 'mode': mode}), flush=True)
            if mode == 'stall':
                time.sleep(delay)
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
            partial = mode == 'partial'
            response = {'received': count, 'admitted': max(0, count - 1) if partial else count, 'complete': not partial}
            if partial:
                response['code'] = 'admission-incomplete'
            body = json.dumps(response, separators=(',', ':')).encode()
            self.send_response(503 if partial else 202)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
    return FaultHandler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest='command', required=True)
    serve = modes.add_parser('serve', help='Run an explicitly selected controlled TLS endpoint, not staging')
    serve.add_argument('--bind', default='127.0.0.1')
    serve.add_argument('--port', type=int, default=8443)
    serve.add_argument('--cert', required=True, type=Path)
    serve.add_argument('--key', required=True, type=Path)
    serve.add_argument('--mode', choices=('valid', 'partial', 'stall', 'lost', 'oversize', 'redirect'), default='valid')
    serve.add_argument('--delay', type=float, default=10)
    analyze = modes.add_parser('analyze', help='Summarize whitelisted numeric metrics from a private JSON array')
    analyze.add_argument('--input', required=True, type=Path)
    args = parser.parse_args()
    if args.command == 'serve':
        if not 0 < args.port < 65536 or not 0 <= args.delay <= 120:
            parser.error('port or delay out of range')
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(args.cert, args.key)
        server = ThreadingHTTPServer((args.bind, args.port), handler(args.mode, args.delay))
        server.socket = context.wrap_socket(server.socket, server_side=True)
        print(json.dumps({'bind': args.bind, 'port': args.port, 'mode': args.mode}), flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
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

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
                        client.request('POST', '/addPackets', json.dumps({'packets': 'AAsECAAJ', 'apiKey': 'DO-NOT-PRINT'}), {'Content-Type': 'application/json'})
                        response = client.getresponse()
                        self.assertEqual(response.status, status)
                        response.read()
                    client.close()
                    self.assertNotIn('DO-NOT-PRINT', output.getvalue())
                finally:
                    server.shutdown()
                    server.server_close()
                    thread.join()

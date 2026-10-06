#!/usr/bin/env python3
"""Real localhost-only checks; no Tailscale, credentials, services or release data."""
import contextlib
import hashlib
import http.client
import importlib.util
import io
import os
from pathlib import Path
import socket
import tempfile
import threading
import time
import unittest

spec = importlib.util.spec_from_file_location('staging_origin', Path(__file__).with_name('staging-origin.py'))
origin = importlib.util.module_from_spec(spec)
spec.loader.exec_module(origin)
HOST = 'selected.example.test'
COMMIT = 'a' * 40
HASH = 'b' * 64
ARTIFACT = '/artifacts/' + COMMIT + '/SKAGER-Beta2-Setup.exe'


class Tests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'repository'
        self.public = self.root / 'public'
        self.public.mkdir(parents=True)
        self.payload = bytes(range(256)) * 1025
        self.write('1.root.json', b'{"root":1}')
        self.write('timestamp.json', b'{"timestamp":1}')
        self.write('1.snapshot.json', b'{"snapshot":1}')
        self.write('1.targets.json', b'{"targets":1}')
        self.write('targets/releases/' + HASH + '.beta.json', b'{"policy":1}')
        self.write(ARTIFACT[1:], self.payload)

    def write(self, path, data):
        target = self.public / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        return target

    def start(self, **kwargs):
        server = origin.Origin(str(self.root), HOST, port=0, **kwargs)
        thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .01})
        thread.start()
        def stop():
            server.shutdown()
            thread.join(2)
            server.server_close()
        self.addCleanup(stop)
        self.assertEqual(server.server_address[0], '127.0.0.1')
        return server

    def request(self, server, path='/1.root.json', method='GET', host=HOST):
        client = http.client.HTTPConnection(*server.server_address, timeout=2)
        try:
            client.request(method, path, headers={'Host': host})
            response = client.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            client.close()

    def raw(self, server, request):
        with socket.create_connection(server.server_address, timeout=2) as client:
            client.sendall(request)
            result = bytearray()
            while chunk := client.recv(65536):
                result.extend(chunk)
            return bytes(result)

    def test_get_head_binary_and_atomic_timestamp(self):
        server = self.start()
        for path in ['/1.root.json', '/1.targets.json', '/1.snapshot.json',
                     '/targets/releases/' + HASH + '.beta.json', ARTIFACT]:
            status, headers, data = self.request(server, path)
            self.assertEqual(status, 200)
            self.assertEqual(data, (self.public / path[1:]).read_bytes())
            self.assertEqual(int(headers['Content-Length']), len(data))
            self.assertEqual(headers['X-Content-Type-Options'], 'nosniff')
            self.assertIn('immutable', headers['Cache-Control'])
            head_status, head_headers, body = self.request(server, path, 'HEAD')
            self.assertEqual(head_status, 200)
            self.assertEqual(head_headers['Content-Length'], headers['Content-Length'])
            self.assertEqual(body, b'')
        self.assertEqual(hashlib.sha256(self.request(server, ARTIFACT)[2]).digest(),
                         hashlib.sha256(self.payload).digest())
        self.assertEqual(self.request(server, '/timestamp.json')[1]['Cache-Control'], 'no-store')
        self.write('.pending-stamp', b'{"timestamp":2}').replace(self.public / 'timestamp.json')
        self.assertEqual(self.request(server, '/timestamp.json')[2], b'{"timestamp":2}')
        self.assertEqual(self.request(server, '/1.snapshot.json')[0], 200)

    def test_only_allowlisted_paths_no_reflection_or_logs(self):
        (self.root / 'state.json').write_text('PRIVATE-STATE')
        self.write('state.json', b'PRIVATE-STATE')
        self.write('keys.json', b'PRIVATE-KEY')
        self.write('.pending-secret', b'PRIVATE-PENDING')
        server = self.start()
        targets = ['/', '/artifacts/', '/state.json', '/keys.json', '/.pending-secret',
                   '/../state.json', '/%2e%2e/state.json', '/%252e%252e/state.json',
                   '/1.root.json?secret=DO-NOT-REFLECT', '/1.root.json#fragment',
                   '//1.root.json', '/./1.root.json', '/a/../1.root.json',
                   '/1%2eroot.json', '/1.root.json%00', '/1.root.json\\..\\keys',
                   'http://' + HOST + '/1.root.json', '/targets/releases/beta.json',
                   '/artifacts/' + COMMIT + '/arbitrary.exe', '/01.root.json']
        with contextlib.redirect_stdout(io.StringIO()) as out, contextlib.redirect_stderr(io.StringIO()) as err:
            for path in targets:
                result = self.raw(server, ('GET ' + path + ' HTTP/1.1\r\nHost: ' + HOST + '\r\n\r\n').encode())
                self.assertIn(b' 404 ', result.split(b'\r\n')[0], path)
                self.assertEqual(result.split(b'\r\n\r\n')[1], b'')
                self.assertNotIn(b'DO-NOT-REFLECT', result)
            self.assertEqual(out.getvalue() + err.getvalue(), '')

    def test_host_methods_and_framing(self):
        server = self.start()
        self.assertEqual(self.request(server, host='unselected.example.test')[0], 421)
        self.assertEqual(self.request(server, host=HOST + ':443')[0], 200)
        for method in ['POST', 'PUT', 'DELETE', 'OPTIONS', 'TRACE', 'CONNECT']:
            self.assertEqual(self.request(server, method=method)[0], 405)
        for headers in ['Host: ' + HOST + '\r\nHost: ' + HOST,
                        'X-Forwarded-Host: ' + HOST, 'Host: localhost']:
            result = self.raw(server, ('GET /1.root.json HTTP/1.1\r\n' + headers + '\r\n\r\n').encode())
            self.assertIn(b' 421 ', result)
        for extra in ['Content-Length: 1', 'Content-Length: 0\r\nContent-Length: 0',
                      'Transfer-Encoding: chunked', 'Expect: 100-continue', 'Upgrade: websocket',
                      ' Folded: header', 'Bad Header: value']:
            result = self.raw(server, ('GET /1.root.json HTTP/1.1\r\nHost: ' + HOST + '\r\n' + extra + '\r\n\r\n').encode())
            self.assertIn(b' 400 ', result)
            self.assertNotIn(b'100 Continue', result)
        result = self.raw(server, b'GET /1.root.json HTTP/1.1\r\nX: ' + b'x' * origin.MAX_HEADERS)
        self.assertIn(b' 431 ', result)
        result = self.raw(server, ('GET /1.root.json HTTP/1.1\r\nHost: ' + HOST + '\r\n\r\nBODY').encode())
        self.assertIn(b' 400 ', result)

    def test_symlinks_hardlinks_special_files_and_ancestors(self):
        server = self.start()
        outside = Path(self.temp.name) / 'private'
        outside.write_bytes(b'PRIVATE')
        leaf = self.public / '2.root.json'
        leaf.symlink_to(outside)
        self.assertEqual(self.request(server, '/2.root.json')[0], 404)
        leaf.unlink()
        os.link(outside, leaf)
        self.assertEqual(self.request(server, '/2.root.json')[0], 404)
        leaf.unlink()
        os.mkfifo(leaf)
        self.assertEqual(self.request(server, '/2.root.json')[0], 404)
        (self.public / '3.root.json').mkdir()
        self.assertEqual(self.request(server, '/3.root.json')[0], 404)
        commit_link = self.public / 'artifacts' / ('c' * 40)
        commit_link.symlink_to(outside.parent, target_is_directory=True)
        self.assertEqual(self.request(server, '/artifacts/' + 'c' * 40 + '/SKAGER-Beta2-Setup.exe')[0], 404)
        link = Path(self.temp.name) / 'linked'
        link.symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(OSError):
            origin.Origin(str(link), HOST, port=0)
        other = Path(self.temp.name) / 'other'
        other.mkdir()
        (other / 'public').symlink_to(self.public, target_is_directory=True)
        with self.assertRaises(OSError):
            origin.Origin(str(other), HOST, port=0)
        self.write('4.root.json', b'x' * ((1 << 20) + 1))
        self.assertEqual(self.request(server, '/4.root.json')[0], 404)

    def test_bind_failure_and_slow_download_release_worker(self):
        artifact = self.public / ARTIFACT[1:]
        with artifact.open('wb') as file:
            file.truncate(16 << 20)
        server = self.start(workers=1, write_timeout=.1, transfer_timeout=1)
        with self.assertRaises(OSError):
            origin.Origin(str(self.root), HOST, port=server.server_address[1])
        finished = threading.Event()
        original = server.process_request_thread
        def observed(*args):
            try:
                original(*args)
            finally:
                finished.set()
        server.process_request_thread = observed
        with socket.socket() as slow:
            slow.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
            slow.settimeout(2)
            slow.connect(server.server_address)
            slow.sendall(('GET ' + ARTIFACT + ' HTTP/1.1\r\nHost: ' + HOST + '\r\n\r\n').encode())
            # Do not consume the response: the worker must stop when the local
            # socket's bounded write expires, without waiting for this client.
            self.assertTrue(finished.wait(2))
            self.assertIn(b' 200 ', slow.recv(4096).split(b'\r\n')[0])
            self.assertEqual(self.request(server)[0], 200)

    def test_header_deadline_and_concurrency_limit_recover(self):
        server = self.start(workers=1, header_timeout=.3)
        started = threading.Event()
        original = server.process_request_thread
        def observed(*args):
            started.set()
            original(*args)
        server.process_request_thread = observed
        with socket.create_connection(server.server_address, timeout=2) as slow:
            slow.sendall(b'GET ')
            self.assertTrue(started.wait(1))
            self.assertEqual(self.request(server)[0], 503)
            # Keep sending bytes faster than the per-socket timeout. The whole
            # incomplete header must still expire at its absolute deadline.
            stop = threading.Event()
            def trickle():
                while not stop.wait(.03):
                    try:
                        slow.sendall(b'x')
                    except OSError:
                        return
            writer = threading.Thread(target=trickle)
            writer.start()
            try:
                self.assertEqual(slow.recv(1), b'')
            finally:
                stop.set()
                writer.join(1)
        # Server closes the connection before its worker releases the slot.
        deadline = time.monotonic() + 1
        while True:
            status = self.request(server)[0]
            if status == 200:
                break
            self.assertEqual(status, 503)
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.01)


if __name__ == '__main__':
    unittest.main()

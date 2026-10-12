#!/usr/bin/env python3
"""Inert transport and custody tests; no DNS, TLS service, or tailnet access."""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock


spec = importlib.util.spec_from_file_location('private_origin_probe', Path(__file__).with_name('probe-private-updater-origin.py'))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class Socket:
    def __init__(self, response):
        self.response = response
        self.sent = b''
        self.timeouts = []

    def __enter__(self):
        return self

    def __exit__(self, *unused):
        pass

    def sendall(self, data):
        self.sent += data

    def settimeout(self, value):
        self.timeouts.append(value)

    def recv(self, size):
        result, self.response = self.response[:size], self.response[size:]
        return result


class ProbeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.host = 'selected.tail123.ts.net'
        self.root = b'{"signatures":[],"signed":{"_type":"root","version":1}}'
        self.config = json.dumps({'schema': 1, 'bootstrapRoot': json.loads(self.root),
                                  'metadataUrl': 'https://' + self.host,
                                  'artifactOrigin': 'https://' + self.host,
                                  'channel': 'beta', 'localCompatibilityAllowlist': []}).encode()
        self.config_path = self.base / 'config.json'
        self.root_path = self.base / '1.root.json'
        self.config_path.write_bytes(self.config)
        self.root_path.write_bytes(self.root)

    def run_probe(self, fetch=None):
        return probe.probe(self.config_path, probe.digest(self.config), self.root_path,
                           probe.digest(self.root), fetch or (lambda host, size: self.root))

    def response(self, body=None, status=b'200 OK', headers=b''):
        body = self.root if body is None else body
        return b'HTTP/1.1 ' + status + b'\r\nContent-Length: ' + str(len(self.root)).encode() + b'\r\n' + headers + b'\r\n' + body

    def test_exact_pinned_success_and_request_shape(self):
        sock = Socket(self.response())
        def fetch(host, size):
            self.assertEqual(host, self.host)
            return probe.fetch_root(host, size, connector=lambda selected: sock)
        result = self.run_probe(fetch)
        self.assertEqual(result['status'], 'passed')
        self.assertTrue(result['transportOnly'])
        self.assertEqual(sock.sent, (f'GET /1.root.json HTTP/1.1\r\nHost: {self.host}\r\n'
                                      'Accept: application/json\r\nCache-Control: no-cache\r\n'
                                      'Connection: close\r\n\r\n').encode())
        self.assertTrue(sock.timeouts)

    def test_default_connection_uses_selected_host_and_system_tls(self):
        raw, wrapped = mock.Mock(), mock.Mock()
        context = mock.Mock()
        context.wrap_socket.return_value = wrapped
        address = (probe.socket.AF_INET, ('127.0.0.1', 443))
        with mock.patch.object(probe, 'resolved_addresses', return_value=[address]) as resolve, \
             mock.patch.object(probe.socket, 'socket', return_value=raw) as connect, \
             mock.patch.object(probe.ssl, 'create_default_context', return_value=context) as tls:
            self.assertIs(probe.verified_connection(self.host), wrapped)
        resolve.assert_called_once_with(self.host)
        connect.assert_called_once_with(probe.socket.AF_INET, probe.socket.SOCK_STREAM, probe.socket.IPPROTO_TCP)
        raw.connect.assert_called_once_with(('127.0.0.1', 443))
        tls.assert_called_once_with()
        context.wrap_socket.assert_called_once_with(raw, server_hostname=self.host)

    def test_dns_deadline_refuses_without_connecting(self):
        worker = mock.Mock()
        worker.is_alive.return_value = True
        with mock.patch.object(probe.threading, 'Thread', return_value=worker):
            with self.assertRaises(ValueError):
                probe.resolved_addresses(self.host)
        worker.join.assert_called_once_with(5)

    def test_pinned_inputs_and_original_root_binding_refuse_changes(self):
        with self.assertRaises(ValueError):
            probe.probe(self.config_path, '0' * 64, self.root_path, probe.digest(self.root), lambda *_: self.root)
        with self.assertRaises(ValueError):
            probe.probe(self.config_path, probe.digest(self.config), self.root_path, '0' * 64, lambda *_: self.root)
        changed = self.config.replace(b'"version": 1', b'"version": 2')
        self.config_path.write_bytes(changed)
        with self.assertRaises(ValueError):
            self.run_probe()

    def test_duplicate_json_and_ambiguous_endpoints_refuse(self):
        bad = [self.config.replace(b'"schema": 1', b'"schema": 1, "schema": 1'),
               self.config.replace(self.host.encode(), b'OTHER.ts.net', 1),
               self.config.replace(self.host.encode(), (self.host + ':443').encode()),
               self.config.replace(self.host.encode(), (self.host + '/other').encode()),
               self.config.replace(b'https://', b'http://', 1)]
        for raw in bad:
            with self.subTest(raw=raw[-40:]), self.assertRaises(ValueError):
                probe.selected_host(raw, self.root)

    def test_network_response_must_be_exact_200_length_and_bytes(self):
        bad = [self.response(status=b'302 Found'), self.response(status=b'401 Unauthorized'),
               self.response(headers=b'Content-Length: 55\r\n'),
               self.response(headers=b'Transfer-Encoding: chunked\r\n'),
               self.response(headers=b'Location: https://other.ts.net/\r\n'),
               self.response(headers=b'Content-Encoding: gzip\r\n'),
               self.response(body=self.root[:-1]),
               self.response(body=self.root + b'!'),
               b'HTTP/1.1 200 OK\r\nX-Long: ' + b'x' * probe.MAX_HEADER + b'\r\n\r\n' + self.root]
        for raw in bad:
            with self.subTest(raw=raw[:60]), self.assertRaises(ValueError):
                probe.fetch_root(self.host, len(self.root), connector=lambda _: Socket(raw))
        with self.assertRaises(ValueError):
            self.run_probe(lambda *_: b'X' * len(self.root))

    def test_original_inputs_are_rechecked_after_request(self):
        def changed_root(*unused):
            self.root_path.write_bytes(self.root + b' ')
            return self.root
        with self.assertRaises(ValueError):
            self.run_probe(changed_root)

    def test_symlink_input_refuses_and_no_fetch(self):
        alias = self.base / 'alias.json'
        alias.symlink_to(self.root_path)
        with self.assertRaises(ValueError):
            probe.probe(self.config_path, probe.digest(self.config), alias, probe.digest(self.root),
                        lambda *_: self.fail('network attempted'))

    def test_fresh_evidence_and_sanitized_refusal(self):
        args = ['--config', str(self.config_path), '--config-sha256', probe.digest(self.config),
                '--original-root', str(self.root_path), '--original-root-sha256', probe.digest(self.root),
                '--evidence', str(self.base / 'evidence')]
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(probe.main(args, fetch=lambda *_: self.root), 0)
        result = json.loads((self.base / 'evidence' / 'result.json').read_text())
        self.assertEqual(result['status'], 'passed')
        with contextlib.redirect_stderr(io.StringIO()) as errors:
            self.assertEqual(probe.main(args), 1)
        self.assertEqual(errors.getvalue().strip(), probe.REFUSAL)
        self.assertEqual(json.loads((self.base / 'evidence' / 'result.json').read_text()), result)


if __name__ == '__main__':
    unittest.main()

"""Actual local socket/HTTP framing tests; no external traffic or credentials."""
import importlib.util
import json
import math
from pathlib import Path
import socket
import ssl
import threading
import time
import unittest
from unittest.mock import Mock, patch

import installed_updater_approval_http as client

spec = importlib.util.spec_from_file_location('transport_fixture',
    Path(__file__).with_name('test-installed-updater-approval-transport.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)

ENDPOINT = client.BASE + 'runs/700/attempts/1'
ZIP = client.BASE + 'artifacts/702/zip'
LOCATION = 'https://productionresultssa19.blob.core.windows.net/actions-results/fixture/approval.zip?sig=fixture'


def reply(body=b'{}', *, status=200, headers=()):
    return (f'HTTP/1.1 {status} Test\r\n'.encode() +
            b''.join(f'{k}: {v}\r\n'.encode() for k, v in headers) +
            f'Content-Length: {len(body)}\r\n\r\n'.encode() + body)


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.reads = client.GitHubReads('fixture-token')
        self.requests, self.threads, self.sockets, self.received = [], [], [], []
        self.addCleanup(self.close)

    def close(self):
        for sock in self.sockets:
            sock.close()
        for thread in self.threads:
            thread.join(timeout=1)
            self.assertFalse(thread.is_alive())

    def connection(self, route):
        def connect(host, deadline):
            local, remote = socket.socketpair()
            self.sockets.append(remote)
            def serve():
                try:
                    remote.settimeout(1)
                    request = b''
                    while not request.endswith(b'\r\n\r\n'):
                        block = remote.recv(4096)
                        if not block:
                            return
                        request += block
                    self.requests.append((host, request))
                    value = route(host, request)
                    if callable(value):
                        value(remote)
                    else:
                        remote.sendall(value)
                except (OSError, AssertionError):
                    pass
                finally:
                    remote.close()
            thread = threading.Thread(target=serve, daemon=True)
            self.threads.append(thread)
            thread.start()
            received = self.received
            class CountedSocket:
                def recv_into(self, buffer, count):
                    length = local.recv_into(buffer, count)
                    received.append(length)
                    return length
                def __getattr__(self, name):
                    return getattr(local, name)
            return CountedSocket()
        return patch.object(client, '_connect', side_effect=connect)

    def api(self, **changes):
        options = dict(timeout_seconds=1, max_bytes=1024)
        options.update(changes)
        return self.reads.read_api(ENDPOINT, **options)

    def test_fixed_get_request_and_strict_json(self):
        with self.connection(lambda *args: reply(b'{"id":700}')):
            self.assertEqual(self.api(), {'id': 700})
        host, request = self.requests[0]
        self.assertEqual(host, 'api.github.com')
        self.assertTrue(request.startswith(('GET /' + ENDPOINT + ' HTTP/1.1\r\n').encode()))
        self.assertIn(b'Authorization: Bearer fixture-token\r\n', request)
        self.assertIn(b'Accept-Encoding: identity\r\n', request)

    def test_only_fixed_status_endpoint_accepts_bounded_top_level_list(self):
        endpoint = 'repos/ThereptileII/Work/commits/' + 'a' * 40 + '/statuses?per_page=100&page=1'
        with self.connection(lambda *args: reply(b'[{"context":"fixed"}]')):
            self.assertEqual(self.reads.read_api(endpoint, timeout_seconds=1, max_bytes=1024), [{'context': 'fixed'}])
        for path, raw in ((ENDPOINT, b'[]'), (endpoint, b'{}'),
                          (endpoint, json.dumps([{}] * 101).encode())):
            with self.subTest(path=path, body=raw[:10]):
                with self.connection(lambda *args: reply(raw)):
                    with self.assertRaises(ValueError):
                        self.reads.read_api(path, timeout_seconds=1, max_bytes=4096)

    def test_status_locator_endpoint_has_no_repository_ref_or_pagination_escape(self):
        endpoint = 'repos/ThereptileII/Work/commits/' + 'a' * 40 + '/statuses?per_page=100&page=1'
        for path in (endpoint.replace('page=1', 'page=0'), endpoint.replace('&page=1', '&page=6'),
                     endpoint.replace('a' * 40, 'main'), endpoint.replace('ThereptileII/Work', 'other/repo'),
                     endpoint + '&extra=1', endpoint.replace('per_page=100', 'per_page=50')):
            with self.subTest(path=path), patch.object(client, '_connect') as connect:
                with self.assertRaises(ValueError): self.reads.read_api(path, timeout_seconds=1, max_bytes=1024)
                connect.assert_not_called()

    def test_receive_approval_composes_existing_original_binding_over_http(self):
        f = fixture.TransportTests()
        f.setUp()
        def route(host, request):
            path = request.split(b' ')[1].decode().lstrip('/')
            if host != 'api.github.com':
                self.assertNotIn(b'Authorization', request)
                self.assertNotIn(b'fixture-token', request)
                return reply(f.raw)
            if path.endswith('/zip'):
                return reply(b'', status=302, headers=[('Location', LOCATION)])
            value = f.read_api(path, timeout_seconds=1, max_bytes=client.transport.MAX_API_BYTES)
            return reply(json.dumps(value).encode())
        with self.connection(route):
            bound = f.receive(read_api=self.reads.read_api, download_archive=self.reads.download_archive)
        self.assertEqual(bound.original_approval_bytes, f.f.approval_bytes)
        self.assertEqual(bound.request.original_bytes, f.f.request_bytes)
        self.assertEqual(len(self.requests), 6)

    def test_endpoint_allowlist_refuses_before_connection(self):
        paths = [ENDPOINT + '/cancel', ENDPOINT.replace('700', '0700'),
                 ENDPOINT.replace('ThereptileII/Work', 'other/Work'),
                 'https://api.github.com/' + ENDPOINT, '/' + ENDPOINT,
                 ENDPOINT + '?page=2', ENDPOINT + '\r\nX: bad',
                 client.BASE + 'runs/700/attempts/1/jobs?per_page=100&page=2',
                 client.BASE + 'artifacts/702/../703', ZIP]
        with patch.object(client, '_connect') as connect:
            for path in paths:
                with self.subTest(path=path), self.assertRaises(ValueError):
                    self.reads.read_api(path, timeout_seconds=1, max_bytes=1024)
            connect.assert_not_called()

    def test_invalid_limits_and_tokens_refuse_without_io(self):
        with patch.object(client, '_connect') as connect:
            for value in [True, 0, -1, 31, math.inf, math.nan, '1']:
                with self.subTest(timeout=value), self.assertRaises(ValueError):
                    self.api(timeout_seconds=value)
            for value in [True, 0, -1, client.transport.MAX_API_BYTES + 1, 1.5]:
                with self.subTest(cap=value), self.assertRaises(ValueError):
                    self.api(max_bytes=value)
            for token in ['', 'x\r\ny', 'x y', 'é', None]:
                with self.subTest(token=token), self.assertRaises(ValueError):
                    client.GitHubReads(token)
            connect.assert_not_called()

    def test_duplicate_nested_json_keys_nonfinite_bad_utf8_and_trailing_bytes_refuse(self):
        for body in [b'{"x":{"k":1,"k":2}}', b'{"x":NaN}', b'{"x":Infinity}', b'{"x":1e9999}',
                     b'\xff', b'{}{}', b'[]', b'\xef\xbb\xbf{}']:
            with self.subTest(body=body), self.connection(lambda *args: reply(body)), self.assertRaises(ValueError):
                self.api()

    def test_content_length_prevents_oversized_body_read(self):
        with self.connection(lambda *args: b'HTTP/1.1 200 OK\r\nContent-Length: 999999\r\n\r\n'), self.assertRaises(ValueError):
            self.api(max_bytes=10)

    def test_streaming_size_cap_without_content_length(self):
        header = b'HTTP/1.1 200 OK\r\nConnection: close\r\n\r\n'
        data = header + b' ' * 100
        with self.connection(lambda *args: data), self.assertRaises(ValueError):
            self.api(max_bytes=10)
        self.assertEqual(sum(self.received), len(header) + 11)

    def test_chunked_success_and_bad_framing_refusal(self):
        header = b'HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n'
        with self.connection(lambda *args: header + b'1\r\n{\r\n1\r\n}\r\n0\r\n\r\n'):
            self.assertEqual(self.api(), {})
        for tail in [b'2\r\n{}XX0\r\n\r\n', b'3\r\n{}\r\n', b'-1\r\n',
                     b'2\n{}\r\n0\r\n\r\n', b'2\r\n{}\r\n0\r\nX: y\r\n\r\n',
                     b'2;foo=bar\r\n{}\r\n0\r\n\r\n']:
            with self.subTest(tail=tail), self.connection(lambda *args: header + tail), self.assertRaises(ValueError):
                self.api()

    def test_ambiguous_framing_encoding_and_truncated_body_refuse(self):
        for headers, body in [([('Content-Length', '2')], b'{}'),
                              ([('Transfer-Encoding', 'chunked')], b'{}'),
                              ([('Content-Encoding', 'gzip')], b'{}'),
                              ([('Transfer-Encoding', 'gzip')], b'{}')]:
            with self.subTest(headers=headers), self.connection(lambda *args: reply(body, headers=headers)), self.assertRaises(ValueError):
                self.api()
        with self.connection(lambda *args: b'HTTP/1.1 200 OK\r\nContent-Length: 3\r\n\r\n{}'), self.assertRaises(ValueError):
            self.api()

    def test_api_errors_and_redirects_never_retried(self):
        for status in [302, 401, 403, 404, 429, 500, 503]:
            old = len(self.requests)
            with self.subTest(status=status), self.connection(lambda *args: reply(status=status)), self.assertRaises(ValueError):
                self.api()
            self.assertEqual(len(self.requests), old + 1)

    def test_archive_redirect_one_hop_with_no_auth(self):
        with self.connection(lambda host, request: reply(b'', status=302, headers=[('Location', LOCATION)])
                             if host == 'api.github.com' else reply(b'ZIP')):
            self.assertEqual(self.reads.download_archive(ZIP, timeout_seconds=1, max_bytes=3), b'ZIP')
        self.assertEqual(len(self.requests), 2)
        self.assertNotIn(b'Authorization', self.requests[1][1])
        self.assertNotIn(b'fixture-token', self.requests[1][1])

    def test_redirect_host_scheme_port_userinfo_path_and_fragment_refuse(self):
        locations = ['http:' + LOCATION[6:], LOCATION.replace('productionresultssa19', 'evil'),
                     LOCATION.replace('.net/', '.net.evil/'), LOCATION.replace('.net/', '.net:443/'),
                     LOCATION.replace('https://', 'https://user@'), LOCATION + '#fragment',
                     LOCATION.replace('/actions-results/', '/other/'), LOCATION.replace('/fixture/', '/../'),
                     LOCATION + '\x01', 'https://127.0.0.1/zip',
                     LOCATION.replace('https://', 'https://fixture-token@')]
        for location in locations:
            old = len(self.requests)
            with self.subTest(location=location), self.connection(lambda *args: reply(b'', status=302, headers=[('Location', location)])), self.assertRaises(ValueError):
                self.reads.download_archive(ZIP, timeout_seconds=1, max_bytes=3)
            self.assertEqual(len(self.requests), old + 1)

    def test_storage_redirect_and_duplicate_locations_refuse_without_third_request(self):
        with self.connection(lambda *args: reply(b'', status=302, headers=[('Location', LOCATION)])), self.assertRaises(ValueError):
            self.reads.download_archive(ZIP, timeout_seconds=1, max_bytes=3)
        self.assertEqual(len(self.requests), 2)
        with self.connection(lambda *args: reply(b'', status=302, headers=[('Location', LOCATION), ('Location', LOCATION)])), self.assertRaises(ValueError):
            self.reads.download_archive(ZIP, timeout_seconds=1, max_bytes=3)
        self.assertEqual(len(self.requests), 3)

    def test_trickling_headers_and_body_use_total_deadline(self):
        for prefix, data in [(b'', b'HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\n{}'),
                             (b'HTTP/1.1 200 OK\r\nContent-Length: 100\r\n\r\n', b' ' * 100)]:
            def trickle(sock):
                sock.sendall(prefix)
                for part in data:
                    sock.sendall(bytes([part]))
                    time.sleep(.01)
            started = time.monotonic()
            with self.connection(lambda *args: trickle), self.assertRaises(ValueError):
                self.api(timeout_seconds=.08)
            self.assertLess(time.monotonic() - started, .4)

    def test_redirect_and_storage_share_one_deadline(self):
        def route(host, request):
            def delayed(sock):
                time.sleep(.05)
                sock.sendall(reply(b'', status=302, headers=[('Location', LOCATION)])
                             if host == 'api.github.com' else reply(b'ZIP'))
            return delayed
        started = time.monotonic()
        with self.connection(route), self.assertRaises(ValueError):
            self.reads.download_archive(ZIP, timeout_seconds=.08, max_bytes=3)
        self.assertEqual(len(self.requests), 2)
        self.assertLess(time.monotonic() - started, .15)

    def test_default_tls_verification_failure_closes_without_http_request(self):
        raw = Mock()
        context = ssl.create_default_context()
        self.assertTrue(context.check_hostname)
        self.assertEqual(context.verify_mode, ssl.CERT_REQUIRED)
        with patch.object(client, '_resolve', return_value=(socket.AF_INET, socket.SOCK_STREAM, 0, '', ('127.0.0.1', 443))), \
                patch.object(client.socket, 'socket', return_value=raw), \
                patch.object(client.ssl, 'create_default_context', return_value=context), \
                patch.object(context, 'wrap_socket', side_effect=ssl.SSLCertVerificationError()) as wrap:
            with self.assertRaises(ValueError):
                self.api()
        wrap.assert_called_once_with(raw, server_hostname='api.github.com')
        raw.close.assert_called_once()
        raw.sendall.assert_not_called()

    def test_header_and_chunk_overhead_bounded(self):
        for data in [b'HTTP/1.1 200 OK\r\nX: ' + b'a' * client.MAX_OVERHEAD + b'\r\n\r\n',
                     b'HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n' + b'1\r\nx\r\n' * 500]:
            with self.subTest(length=len(data)), self.connection(lambda *args: data), self.assertRaises(ValueError):
                self.api(max_bytes=10)

    def test_dns_timeout_has_at_most_one_worker_across_clients(self):
        release = threading.Event()
        started = threading.Event()
        def stalled(*args, **kwargs):
            started.set()
            release.wait(1)
            return []
        try:
            with patch.object(client.socket, 'getaddrinfo', side_effect=stalled) as resolve:
                for _ in range(3):
                    with self.assertRaises(ValueError):
                        client._resolve('api.github.com', client._Deadline(.03))
                self.assertTrue(started.is_set())
                self.assertEqual(resolve.call_count, 1)
        finally:
            release.set()
            self.assertTrue(client._RESOLVER_SLOT.acquire(timeout=1))
            client._RESOLVER_SLOT.release()

    def test_error_text_never_discloses_token_or_signed_url(self):
        with patch.object(client, '_connect', side_effect=OSError('fixture-token ' + LOCATION)):
            with self.assertRaisesRegex(ValueError, '^' + client.FAILURE + '$'):
                self.api()


if __name__ == '__main__':
    unittest.main()

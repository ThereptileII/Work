#!/usr/bin/env python3
"""Bounded loopback regression for the provider fixture's HTTP-header reader."""
from pathlib import Path
import socket
import subprocess
import sys
import unittest

from provider_tests import read_headers


def probe(case):
    payload = {
        'empty-eof': b'',
        'partial-eof': b'GET / HTTP/1.1\r\nHost: localhost\r\n',
        'complete': b'GET / HTTP/1.1\r\nHost: localhost\r\n\r\n',
        'oversized': b'x' * 16384,
    }[case]
    # Only disposable loopback sockets; no external endpoint, TLS key or worker.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(1)
        with socket.create_connection(listener.getsockname(), timeout=1) as client:
            connection, _ = listener.accept()
            with connection:
                connection.settimeout(1)
                client.sendall(payload)
                client.shutdown(socket.SHUT_WR)
                try:
                    actual = read_headers(connection)
                except EOFError:
                    if not case.endswith('-eof'):
                        raise
                except AssertionError:
                    if case != 'oversized':
                        raise
                else:
                    if case != 'complete' or actual != payload:
                        raise AssertionError('Incomplete or oversized headers accepted')
    print('PASS', case, 'reader returned and all owned sockets closed')


class HeaderCleanupTests(unittest.TestCase):
    def test_loopback_header_completion_and_cleanup(self):
        for case in ('empty-eof', 'partial-eof', 'complete', 'oversized'):
            with self.subTest(case=case):
                # A socket timeout cannot stop recv()==b'' busy-looping. A fresh
                # process deadline makes that regression fail without leaking a
                # non-daemon fixture worker into the rest of the test suite.
                result = subprocess.run(
                    [sys.executable, str(Path(__file__).resolve()), '--probe', case],
                    capture_output=True, text=True, timeout=3, check=False)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn('PASS ' + case, result.stdout)


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--probe':
        probe(sys.argv[2])
    else:
        unittest.main()

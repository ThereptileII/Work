#!/usr/bin/env python3
"""Loopback-only origin for the private SKAGER Staging repository (Linux)."""
import argparse
import io
import os
import re
import socket
import stat
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

MAX_HEADERS = 16384
MAX_FILE = 4 << 30
CHUNK = 64 << 10
ARTIFACTS = (
    'SKAGER-Beta2-Release-Notes.md', 'SKAGER-Beta2-Setup.exe',
    'SKAGER-Beta2-Portable-Recovery.zip', 'SKAGER-Beta2-source.zip',
    'SOURCE_AND_LICENSES.md',
)
PATH = re.compile(r'/(?:timestamp\.json|[1-9][0-9]{0,18}\.(?:root|targets|snapshot)\.json|'
                  r'targets/releases/[0-9a-f]{64}\.beta\.json|artifacts/[0-9a-f]{40}/(?:' +
                  '|'.join(re.escape(name) for name in ARTIFACTS) + r'))\Z')
HOST = re.compile(r'(?=.{1,253}\Z)(?:[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?\.)+'
                  r'[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?\Z')
HEADER = re.compile(rb"[!#$%&'*+.^_`|~0-9A-Za-z-]+:[\t\x20-\x7e]*\Z")


def open_directory(path):
    """Pin every directory by fd without following any symlink, including ancestors."""
    if sys.platform != 'linux' or not hasattr(os, 'O_NOFOLLOW') or not hasattr(os, 'O_DIRECTORY'):
        raise ValueError('This private origin requires Linux descriptor-relative file access')
    if not os.path.isabs(path) or any(p in ('.', '..') for p in path.split('/')):
        raise ValueError('An absolute repository directory without dot components is required')
    fd = os.open('/', os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        for part in path.split('/'):
            if part:
                next_fd = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=fd)
                os.close(fd)
                fd = next_fd
        return fd
    except BaseException:
        os.close(fd)
        raise


class Origin(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    block_on_close = False
    request_queue_size = 16
    allow_reuse_address = True

    def __init__(self, repository, host, port=8765, workers=4,
                 header_timeout=5.0, write_timeout=30.0, transfer_timeout=600.0):
        if not HOST.fullmatch(host) or host != host.lower():
            raise ValueError('A canonical selected HTTPS hostname is required')
        if not 0 <= port <= 65535 or not 1 <= workers <= 32:
            raise ValueError('Invalid local port or concurrency bound')
        if not all(0 < value <= 600 for value in (header_timeout, write_timeout, transfer_timeout)):
            raise ValueError('Invalid deadline')
        self.expected_host = host
        self.header_timeout = header_timeout
        self.write_timeout = write_timeout
        self.transfer_timeout = transfer_timeout
        self.slots = threading.BoundedSemaphore(workers)
        self.fd_lock = threading.Lock()
        self.public_fd = open_directory(os.path.join(repository, 'public'))
        try:
            super().__init__(('127.0.0.1', port), Handler)
        except BaseException:
            with self.fd_lock:
                if self.public_fd is not None:
                    os.close(self.public_fd)
                    self.public_fd = None
            raise

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            # No extra thread or waiting queue per accepted excess connection.
            try:
                request.settimeout(0.1)
                request.sendall(b'HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\nContent-Length: 0\r\n\r\n')
            except OSError:
                pass
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()

    def handle_error(self, request, client_address):
        # Never print request paths, headers, operator paths or tracebacks.
        pass

    def open_public(self, path):
        if not PATH.fullmatch(path):
            raise FileNotFoundError('Unavailable')
        with self.fd_lock:
            if self.public_fd is None:
                raise OSError("Origin closed")
            fd = os.dup(self.public_fd)
        try:
            parts = path[1:].split('/')
            for part in parts[:-1]:
                next_fd = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=fd)
                os.close(fd)
                fd = next_fd
            before = os.stat(parts[-1], dir_fd=fd, follow_symlinks=False)
            if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
                raise ValueError('Unavailable')
            result = os.open(parts[-1], os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=fd)
            try:
                info = os.fstat(result)
                limit = MAX_FILE if path.startswith('/artifacts/') else 1 << 20
                if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or not 0 < info.st_size <= limit:
                    raise ValueError('Unavailable')
                return os.fdopen(result, 'rb'), info.st_size
            except BaseException:
                os.close(result)
                raise
        finally:
            os.close(fd)

    def server_close(self):
        super().server_close()
        with self.fd_lock:
            if self.public_fd is not None:
                os.close(self.public_fd)
                self.public_fd = None


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'SKAGER-Staging'
    sys_version = ''

    def log_message(self, format, *args):
        pass

    def send_error(self, code, message=None, explain=None):
        # Fixed, empty errors do not reflect an untrusted URL/header or local path.
        self.request_version = 'HTTP/1.1'
        self.send_response(code)
        self.send_header('Content-Length', '0')
        self.send_header('Connection', 'close')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.close_connection = True

    def handle_one_request(self):
        self.close_connection = True
        self.requestline = ''
        self.request_version = 'HTTP/1.1'
        self.command = None
        deadline = time.monotonic() + self.server.header_timeout
        data = bytearray()
        try:
            # Absolute deadline defeats byte-at-a-time trickling. Parse only a
            # bounded, complete header block with the stdlib HTTP parser.
            while b'\r\n\r\n' not in data:
                left = deadline - time.monotonic()
                if left <= 0:
                    raise TimeoutError
                self.connection.settimeout(left)
                chunk = self.connection.recv(min(4096, MAX_HEADERS + 1 - len(data)))
                if not chunk:
                    return
                data.extend(chunk)
                if len(data) > MAX_HEADERS:
                    self.send_error(431)
                    return
            head, extra = bytes(data).split(b'\r\n\r\n', 1)
            lines = head.split(b'\r\n')
            if extra or len(lines) > 41 or not re.fullmatch(rb'[A-Z]{1,16} [!-~]{1,2048} HTTP/1\.[01]', lines[0]):
                self.send_error(400)
                return
            if any(len(line) > 4096 or not HEADER.fullmatch(line) for line in lines[1:]):
                self.send_error(400)
                return
            self.raw_requestline = lines[0] + b'\r\n'
            # Check the original target before the stdlib's // normalization.
            if not PATH.fullmatch(lines[0].split(b' ')[1].decode('ascii')):
                self.send_error(404)
                return
            self.rfile = io.BytesIO(b'\r\n'.join(lines[1:]) + b'\r\n\r\n')
            if not self.parse_request():
                return
            self.close_connection = True
            hosts = self.headers.get_all('Host', [])
            if len(hosts) != 1 or hosts[0] not in (self.server.expected_host, self.server.expected_host + ':443'):
                self.send_error(421)
                return
            lengths = self.headers.get_all('Content-Length', [])
            if (lengths and lengths != ['0']) or any(name in self.headers for name in ('Transfer-Encoding', 'Expect', 'Upgrade')):
                self.send_error(400)
                return
            if self.command not in ('GET', 'HEAD'):
                self.send_error(405)
                return
            self.serve_file()
        except (OSError, ValueError):
            # Timeout/disconnect, including during binary transfer: close only.
            self.close_connection = True

    def handle_expect_100(self):
        self.send_error(400)
        return False

    def serve_file(self):
        try:
            file, size = self.server.open_public(self.path)
        except (OSError, ValueError):
            self.send_error(404)
            return
        with file:
            deadline = time.monotonic() + self.server.transfer_timeout
            self.connection.settimeout(self.server.write_timeout)
            self.send_response(200)
            self.send_header('Content-Length', str(size))
            kind = 'application/json' if self.path.endswith('.json') else 'application/octet-stream'
            self.send_header('Content-Type', kind)
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Connection', 'close')
            self.send_header('Cache-Control', 'no-store' if self.path == '/timestamp.json' else 'private, max-age=86400, immutable')
            self.end_headers()
            if self.command == 'HEAD':
                return
            remaining = size
            while remaining:
                left = deadline - time.monotonic()
                if left <= 0:
                    raise TimeoutError
                self.connection.settimeout(min(self.server.write_timeout, left))
                chunk = file.read(min(CHUNK, remaining))
                if not chunk:
                    return  # A truncated transfer fails Content-Length verification.
                self.wfile.write(chunk)
                remaining -= len(chunk)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', required=True, help='Absolute selected operator repository; only its public child is served')
    parser.add_argument('--host', required=True, help='Exact selected private HTTPS hostname received from the trusted proxy')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error('Choose an unprivileged loopback port')
    try:
        with Origin(args.repository, args.host, args.port) as server:
            print('Private Staging origin ready on 127.0.0.1:' + str(args.port), flush=True)
            server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError):
        print('Private Staging origin unavailable; check the selected directory, host and local port')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

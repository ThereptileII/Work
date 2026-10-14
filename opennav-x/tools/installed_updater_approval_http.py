"""Bounded HTTPS callbacks for installed_updater_approval_transport.receive_approval.

Use GitHubReads(explicit_read_only_token).read_api and .download_archive. This
module never obtains credentials, retries requests, writes API resources/files,
produces approval, consumes challenges or invokes enrollment. Caller still owns
all external authorization and provenance gates. No proxy environment is used.

Artifact redirects are limited to HTTPS productionresultssa<digits> Azure hosts,
under /actions-results/, received directly from the fixed GitHub archive API.
GitHub documents that host family at
https://github.blog/changelog/2023-01-31-github-actions-job-summary-updates/
and artifact storage at https://docs.github.com/en/actions/reference/runners/self-hosted-runners.
Other storage backends fail closed and need source review before support.
"""
import http.client
import io
import json
import math
import queue
import re
import socket
import ssl
import threading
import time
from urllib.parse import urlsplit

import installed_updater_approval_transport as transport

FAILURE = 'Installed updater approval HTTP refused'
BASE = 'repos/' + transport.binding.REPOSITORY + '/actions/'
ID = r'[1-9][0-9]{0,19}'
API_PATH = re.compile(re.escape(BASE) + rf'(?:runs/{ID}/attempts/{ID}(?:/jobs\?per_page=100&page=1)?|artifacts/{ID})')
ZIP_PATH = re.compile(re.escape(BASE) + rf'artifacts/{ID}/zip')
STATUS_PATH = re.compile(r'repos/ThereptileII/Work/commits/[a-f0-9]{40}/statuses\?per_page=100&page=[1-5]')
MAX_OVERHEAD = 64 << 10
# A stuck OS DNS resolver may outlive its caller. Only one exists process-wide;
# later calls time out waiting for this slot rather than accumulating threads.
_RESOLVER_SLOT = threading.BoundedSemaphore(1)


def require(value):
    if not value:
        raise ValueError(FAILURE)


class _Deadline:
    def __init__(self, seconds):
        require(type(seconds) in (int, float) and math.isfinite(seconds) and 0 < seconds <= 30)
        self.end = time.monotonic() + seconds

    def remaining(self):
        seconds = self.end - time.monotonic()
        require(seconds > 0)
        return seconds


def _resolve(host, deadline):
    require(_RESOLVER_SLOT.acquire(timeout=deadline.remaining()))
    results = queue.Queue(maxsize=1)

    def resolve():
        try:
            results.put(socket.getaddrinfo(host, 443, type=socket.SOCK_STREAM))
        except OSError:
            results.put(None)
        finally:
            _RESOLVER_SLOT.release()
    try:
        threading.Thread(target=resolve, daemon=True).start()
    except BaseException:
        _RESOLVER_SLOT.release()
        raise
    try:
        addresses = results.get(timeout=deadline.remaining())
    except queue.Empty:
        raise ValueError(FAILURE) from None
    deadline.remaining()
    require(addresses)
    return addresses[0]  # One connection attempt, never automatic address retries.


def _connect(host, deadline):
    family, kind, proto, _, address = _resolve(host, deadline)
    context = ssl.create_default_context()  # System roots and hostname verification.
    raw = socket.socket(family, kind, proto)
    secured = None
    try:
        raw.settimeout(deadline.remaining())
        raw.connect(address)
        raw.settimeout(deadline.remaining())
        secured = context.wrap_socket(raw, server_hostname=host)
        deadline.remaining()
        return secured
    except BaseException:
        if secured is not None:
            secured.close()
        raw.close()
        raise


class _Wire(io.RawIOBase):
    def __init__(self, sock, deadline, cap):
        self.sock, self.deadline, self.left = sock, deadline, cap

    def readable(self):
        return True

    def readinto(self, buffer):
        self.sock.settimeout(self.deadline.remaining())
        require(self.left > 0)
        count = self.sock.recv_into(buffer, min(len(buffer), self.left))
        self.deadline.remaining()
        self.left -= count
        return count


class _Lines:
    """Bound all status/header/chunk/trailer bytes, including buffered reads."""
    def __init__(self, raw, deadline):
        self.stream = io.BufferedReader(raw, 4096)
        self.deadline, self.left = deadline, MAX_OVERHEAD

    def readline(self, limit=-1):
        self.deadline.remaining()
        limit = min(self.left + 1, limit) if limit >= 0 else self.left + 1
        # BufferedReader.readline may prefetch body bytes beyond a tiny caller
        # body cap. read1 requests exactly one byte while framing is unknown.
        line = bytearray()
        while len(line) < limit:
            byte = self.stream.read1(1)
            if not byte:
                break
            line.extend(byte)
            if byte == b'\n':
                break
        self.left -= len(line)
        require(self.left >= 0 and line.endswith(b'\r\n'))
        return bytes(line)

    def read(self, size=-1):
        self.deadline.remaining()
        require(size >= 0)
        data = bytearray()
        while len(data) < size:
            block = self.stream.read1(size - len(data))
            if not block:
                break
            data.extend(block)
        return bytes(data)

    def read1(self, size=-1):
        self.deadline.remaining()
        require(size >= 0)
        return self.stream.read1(size)

    def close(self):
        self.stream.close()

    def flush(self):
        self.stream.flush()


class _Response(http.client.HTTPResponse):
    def _read_next_chunk_size(self):
        line = self.fp.readline(128)
        require(re.fullmatch(b'[0-9a-fA-F]{1,8}\r\n', line))
        return int(line, 16)

    def _get_chunk_left(self):
        # stdlib discards these bytes without checking that they are CRLF.
        if self.chunk_left == 0:
            require(self._safe_read(2) == b'\r\n')
        if not self.chunk_left:
            self.chunk_left = self._read_next_chunk_size()
            if self.chunk_left == 0:
                require(self.fp.readline(128) == b'\r\n')  # No trailers.
                self._close_conn()
                self.chunk_left = None
        return self.chunk_left


class _ResponseSocket:
    def __init__(self, stream):
        self.stream = stream

    def makefile(self, *args):
        return self.stream


def _one_header(response, name):
    values = response.headers.get_all(name, [])
    require(len(values) <= 1)
    return values[0] if values else None


def _get(host, path, headers, deadline, cap, *, redirect=False):
    sock = response = None
    try:
        sock = _connect(host, deadline)
        request = ('GET ' + path + ' HTTP/1.1\r\nHost: ' + host + '\r\n' +
                   ''.join(k + ': ' + v + '\r\n' for k, v in headers.items()) +
                   'Connection: close\r\nAccept-Encoding: identity\r\n\r\n').encode('ascii')
        sock.settimeout(deadline.remaining())
        sock.sendall(request)
        deadline.remaining()
        stream = _Lines(_Wire(sock, deadline, cap + MAX_OVERHEAD), deadline)
        response = _Response(_ResponseSocket(stream), method='GET')
        response.begin()
        require(not response.headers.defects)
        require(_one_header(response, 'Content-Encoding') in (None, 'identity'))
        length = _one_header(response, 'Content-Length')
        encoding = _one_header(response, 'Transfer-Encoding')
        require(encoding in (None, 'chunked') and not (encoding and length))
        if length is not None:
            require(re.fullmatch(r'0|[1-9][0-9]{0,9}', length) and int(length) <= cap)
        if redirect:
            require(response.status == 302)
            location = _one_header(response, 'Location')
            require(location is not None)
            deadline.remaining()
            return location  # Never consume or forward the redirect's body.
        require(response.status == 200 and _one_header(response, 'Location') is None)
        result = bytearray()
        while True:
            block = response.read1(min(4096, cap - len(result) + 1))
            deadline.remaining()
            require(len(result) + len(block) <= cap)
            if not block:
                require(response.length in (None, 0) and (not response.chunked or response.chunk_left is None))
                return bytes(result)
            result.extend(block)
    finally:
        try:
            if response is not None:
                response.close()
        finally:
            if sock is not None:
                sock.close()


def _storage(location):
    require(type(location) is str and len(location) <= 8192 and
            all(33 <= ord(char) <= 126 for char in location) and '\\' not in location)
    url = urlsplit(location)
    require(url.scheme == 'https' and url.netloc == url.hostname and
            re.fullmatch(r'productionresultssa[0-9]{1,5}\.blob\.core\.windows\.net', url.hostname or '') and
            not url.fragment and url.path.startswith('/actions-results/') and url.query and
            re.fullmatch(r'/[A-Za-z0-9._%/-]+', url.path) and
            not any(piece in ('.', '..') for piece in url.path.split('/')))
    return url.hostname, url.path + '?' + url.query


def _pairs(items):
    result = {}
    for key, value in items:
        require(key not in result)
        result[key] = value
    return result


def _invalid_constant(value):
    raise ValueError(FAILURE)


def _finite_float(value):
    result = float(value)
    require(math.isfinite(result))
    return result


class GitHubReads:
    """Read-only transport callbacks; token must be explicitly supplied by caller.

    Connection injection is private test machinery; it cannot be selected using
    submitted records, URL arguments, environment variables or JSON fields.
    """
    def __init__(self, token):
        require(type(token) is str and 0 < len(token) <= 4096 and
                re.fullmatch(r'[A-Za-z0-9_.-]+', token))
        self._headers = {'Authorization': 'Bearer ' + token,
                         'Accept': 'application/vnd.github+json',
                         'X-GitHub-Api-Version': '2022-11-28', 'User-Agent': 'SKAGER-approval/1'}

    def read_api(self, endpoint, *, timeout_seconds, max_bytes):
        try:
            require(type(endpoint) is str and (API_PATH.fullmatch(endpoint) or STATUS_PATH.fullmatch(endpoint)))
            require(type(max_bytes) is int and 0 < max_bytes <= transport.MAX_API_BYTES)
            deadline = _Deadline(timeout_seconds)
            raw = _get('api.github.com', '/' + endpoint, self._headers, deadline, max_bytes)
            result = json.loads(raw.decode('utf-8'), object_pairs_hook=_pairs,
                                parse_constant=_invalid_constant, parse_float=_finite_float)
            require(type(result) is list and len(result) <= 100 if STATUS_PATH.fullmatch(endpoint) else type(result) is dict)
            deadline.remaining()
            return result
        except (OSError, ValueError, http.client.HTTPException, RecursionError, OverflowError):
            raise ValueError(FAILURE) from None

    def download_archive(self, endpoint, *, timeout_seconds, max_bytes):
        try:
            require(type(endpoint) is str and ZIP_PATH.fullmatch(endpoint))
            require(type(max_bytes) is int and 0 < max_bytes <= transport.MAX_ARCHIVE_BYTES)
            deadline = _Deadline(timeout_seconds)
            location = _get('api.github.com', '/' + endpoint, self._headers, deadline, MAX_OVERHEAD, redirect=True)
            host, path = _storage(location)
            return _get(host, path, {'Accept': 'application/zip', 'User-Agent': 'SKAGER-approval/1'}, deadline, max_bytes)
        except (OSError, ValueError, http.client.HTTPException, RecursionError, OverflowError):
            raise ValueError(FAILURE) from None

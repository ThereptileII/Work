#!/usr/bin/env python3
"""Read-only, pinned HTTPS transport probe for the private SKAGER update root.

This is not a TUF validator, tailnet enrollment, or installed-product test.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import ssl
import stat
import sys
import threading
import time


REFUSAL = 'Private updater origin transport preflight refused; retain evidence and inputs.'
HEX = re.compile(r'[0-9a-f]{64}\Z')
HOST = re.compile(r'(?:[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?\.)+[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?\.ts\.net\Z')
MAX_CONFIG = 576 << 10
MAX_ROOT = 1 << 20
MAX_HEADER = 8192
MAX_HEADER_LINE = 2048
MAX_HEADERS = 32


def require(ok):
    if not ok:
        raise ValueError(REFUSAL)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def strict_json(data):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result)
            result[key] = value
        return result
    return json.loads(data.decode('utf-8'), object_pairs_hook=pairs,
                      parse_constant=lambda _: (_ for _ in ()).throw(ValueError(REFUSAL)))


def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False)


def identity(info):
    return (info.st_dev, info.st_ino, info.st_mode, info.st_size, info.st_mtime_ns,
            info.st_ctime_ns, info.st_nlink, getattr(info, 'st_file_attributes', 0))


def path_identity(info):
    # Directory size/mtime/ctime/link count can change when unrelated entries
    # are created. Retain physical ancestry and type, not incidental contents.
    return (info.st_dev, info.st_ino, stat.S_IFMT(info.st_mode),
            bool(getattr(info, 'st_file_attributes', 0) & 0x400))


def plain_chain(path):
    path = Path(path)
    require(path.is_absolute() and os.path.normpath(str(path)) == str(path) and
            '..' not in path.parts and '.' not in path.parts)
    chain = [path, *path.parents]
    observed = []
    for entry in chain:
        info = entry.lstat()
        require(not stat.S_ISLNK(info.st_mode) and
                not (getattr(info, 'st_file_attributes', 0) & 0x400))
        observed.append((entry, path_identity(info)))
    return observed


class PinnedInput:
    def __init__(self, path, approved_sha, limit):
        require(isinstance(approved_sha, str) and HEX.fullmatch(approved_sha))
        self.path = Path(path)
        self.chain = plain_chain(self.path)
        before = identity(self.path.lstat())
        require(stat.S_ISREG(before[2]) and before[6] == 1 and 0 < before[3] <= limit)
        self.file_identity = before
        self.file = self.path.open('rb')
        try:
            require(identity(os.fstat(self.file.fileno())) == before)
            self.bytes = self.file.read(limit + 1)
            require(0 < len(self.bytes) <= limit and len(self.bytes) == before[3] and
                    digest(self.bytes) == approved_sha)
            self.check()
        except BaseException:
            self.file.close()
            raise

    def check(self):
        self.file.seek(0)
        require(self.file.read(len(self.bytes) + 1) == self.bytes and
                identity(os.fstat(self.file.fileno())) == self.file_identity and
                identity(self.path.lstat()) == self.file_identity and
                plain_chain(self.path) == self.chain)

    def close(self):
        self.file.close()


def selected_host(config, root):
    value = strict_json(config)
    require(type(value) is dict and set(value) == {'schema', 'bootstrapRoot', 'metadataUrl',
                                                   'artifactOrigin', 'channel', 'localCompatibilityAllowlist'} and
            type(value['schema']) is int and value['schema'] == 1 and value['channel'] == 'beta')
    original = strict_json(root)
    require(type(original) is dict and set(original) == {'signatures', 'signed'} and
            canonical_json(value['bootstrapRoot']) == canonical_json(original) and
            type(original['signed']) is dict and
            original['signed'].get('_type') == 'root')
    url = value['metadataUrl']
    require(type(url) is str and type(value['artifactOrigin']) is str and
            value['artifactOrigin'] == url and url.startswith('https://'))
    host = url[len('https://'):]
    require(HOST.fullmatch(host) and len(host) <= 253)
    return host


def resolved_addresses(host):
    # getaddrinfo can ignore socket timeouts. A daemon resolver has an explicit
    # five-second wait; after refusal it can perform no HTTP or TLS request.
    result = []
    def resolve():
        try:
            result.append(socket.getaddrinfo(host, 443, type=socket.SOCK_STREAM,
                                             proto=socket.IPPROTO_TCP))
        except OSError:
            result.append(None)
    worker = threading.Thread(target=resolve, daemon=True)
    worker.start()
    worker.join(5)
    require(not worker.is_alive() and result and result[0] and len(result[0]) <= 16)
    addresses = []
    for family, kind, protocol, _, address in result[0]:
        require(family in (socket.AF_INET, socket.AF_INET6) and
                kind == socket.SOCK_STREAM and protocol == socket.IPPROTO_TCP)
        addresses.append((family, address))
    return addresses


def verified_connection(host):
    # The system default CA and hostname verification remain enabled. No proxy
    # environment or alternate host is consulted; SNI and the Host header agree.
    addresses = resolved_addresses(host)
    context = ssl.create_default_context()
    deadline = time.monotonic() + 5
    for family, address in addresses:
        raw = socket.socket(family, socket.SOCK_STREAM, socket.IPPROTO_TCP)
        try:
            remaining = deadline - time.monotonic()
            require(remaining > 0)
            raw.settimeout(remaining)
            raw.connect(address)
            remaining = deadline - time.monotonic()
            require(remaining > 0)
            raw.settimeout(remaining)
            return context.wrap_socket(raw, server_hostname=host)
        except (OSError, ValueError):
            raw.close()
    raise ValueError(REFUSAL)


def receive(sock, deadline, size):
    left = deadline - time.monotonic()
    require(left > 0)
    sock.settimeout(left)
    return sock.recv(size)


def fetch_root(host, expected_size, connector=verified_connection):
    require(HOST.fullmatch(host) and 0 < expected_size <= MAX_ROOT)
    with connector(host) as connection:
        request = ('GET /1.root.json HTTP/1.1\r\nHost: ' + host +
                   '\r\nAccept: application/json\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n')
        connection.settimeout(5)
        connection.sendall(request.encode('ascii'))
        deadline = time.monotonic() + 10
        response = bytearray()
        while b'\r\n\r\n' not in response:
            block = receive(connection, deadline, min(4096, MAX_HEADER + 1 - len(response)))
            require(block)
            response.extend(block)
            if b'\r\n\r\n' not in response:
                require(len(response) <= MAX_HEADER)
        head, body = bytes(response).split(b'\r\n\r\n', 1)
        require(len(head) + 4 <= MAX_HEADER)
        lines = head.split(b'\r\n')
        require(re.fullmatch(rb'HTTP/1\.1 200 [\x20-\x7e]{1,64}', lines[0]) and
                1 <= len(lines) - 1 <= MAX_HEADERS)
        headers = {}
        for line in lines[1:]:
            require(len(line) <= MAX_HEADER_LINE and
                    re.fullmatch(rb"[!#$%&'*+.^_`|~0-9A-Za-z-]+:[\t\x20-\x7e]*", line))
            name, value = line.split(b':', 1)
            name = name.lower()
            require(name not in headers)
            headers[name] = value.strip()
        require(b'transfer-encoding' not in headers and b'location' not in headers and
                b'content-encoding' not in headers and
                headers.get(b'content-length') == str(expected_size).encode('ascii') and
                len(body) <= expected_size)
        while len(body) < expected_size:
            block = receive(connection, deadline, min(65536, expected_size - len(body)))
            require(block)
            body += block
        require(len(body) == expected_size)
        return body


def probe(config_path, config_sha, root_path, root_sha, fetch=fetch_root):
    config = PinnedInput(config_path, config_sha, MAX_CONFIG)
    try:
        root = PinnedInput(root_path, root_sha, MAX_ROOT)
        try:
            host = selected_host(config.bytes, root.bytes)
            try:
                actual = fetch(host, len(root.bytes))
                require(type(actual) is bytes and actual == root.bytes and digest(actual) == root_sha)
            finally:
                config.check()
                root.check()
            return {'schema': 1, 'status': 'passed', 'transportOnly': True,
                    'configSha256': config_sha, 'originalRootSha256': root_sha,
                    'responseBytes': len(actual), 'httpStatus': 200}
        finally:
            root.close()
    finally:
        config.close()


def main(argv=None, fetch=fetch_root):
    class Parser(argparse.ArgumentParser):
        def error(self, message):
            raise ValueError(REFUSAL)

    class Once(argparse.Action):
        def __call__(self, parser, namespace, values, option_string=None):
            seen = getattr(namespace, '_seen', set())
            require(self.dest not in seen)
            namespace._seen = seen | {self.dest}
            setattr(namespace, self.dest, values)

    parser = Parser(description=__doc__, allow_abbrev=False)
    for name in ('config', 'config-sha256', 'original-root', 'original-root-sha256', 'evidence'):
        parser.add_argument('--' + name, required=True, action=Once)
    result = {'schema': 1, 'status': 'refused', 'transportOnly': True}
    output = None
    try:
        args = parser.parse_args(argv)
        evidence = Path(args.evidence)
        require(not evidence.exists() and not evidence.is_symlink())
        parent = plain_chain(evidence.parent)
        require(parent[0][1][2] == stat.S_IFDIR)
        evidence.mkdir(mode=0o700)
        output = evidence / 'result.json'
        result = probe(args.config, args.config_sha256, args.original_root, args.original_root_sha256, fetch)
        code = 0
    except (Exception, KeyboardInterrupt):
        print(REFUSAL, file=sys.stderr)
        code = 1
    if output is not None:
        try:
            with output.open('x', encoding='utf-8') as stream:
                json.dump(result, stream, sort_keys=True, separators=(',', ':'))
                stream.write('\n')
        except Exception:
            print(REFUSAL, file=sys.stderr)
            return 1
    return code


if __name__ == '__main__':
    raise SystemExit(main())

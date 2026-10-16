#!/usr/bin/env python3
"""Opt-in >20 real minutes without AIS reports over responsive loopback TLS.

No firewall, packet-loss injection, real AIS credential or external endpoint.
The client is excluded from ordinary builds; this fixture is never in CTest.
"""
import argparse
import base64
import hashlib
import ipaddress
import json
from pathlib import Path
import select
import socket
import ssl
import struct
import subprocess
import tempfile
import threading
import time
import zlib

from network_tests import frame
from provider_tests import read, read_headers, subscription


def control_frame(stream):
    header = read(stream, 2)
    assert header[0] & 128 and not header[0] & 112, 'Invalid control frame'
    opcode, length = header[0] & 15, header[1] & 127
    assert opcode in (9, 10) and length < 126, 'Unexpected application/close frame'
    assert header[1] & 128, 'Unmasked client control frame'
    mask = read(stream, 4)
    payload = bytes(c ^ mask[i % 4] for i, c in enumerate(read(stream, length)))
    return opcode, payload


def run_case(client, ipv6=False):
    with tempfile.TemporaryDirectory(prefix='xnav-report-outage-') as directory:
        path = Path(directory)
        cert, key = path / 'cert.pem', path / 'key.pem'
        subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
                        '-days', '1', '-subj', '/CN=localhost', '-addext',
                        'subjectAltName=DNS:localhost', '-keyout', str(key),
                        '-out', str(cert)], check=True, capture_output=True, timeout=15)
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(cert, key)
        with socket.socket(socket.AF_INET6 if ipv6 else socket.AF_INET) as server:
            server.bind(('::1' if ipv6 else '127.0.0.1', 0))
            server.listen(2)
            server.settimeout(12)
            url = f'wss://localhost:{server.getsockname()[1]}/'
            output, phases, endpoints = [], {}, []
            with subprocess.Popen([str(client.resolve()), url, str(cert)],
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  text=True, bufsize=1) as process:
                def collect():
                    for line in process.stdout:
                        output.append(line.rstrip())
                        print(line.rstrip(), flush=True)
                        if line.startswith('PHASE '):
                            phases[line.split()[1]] = time.monotonic()

                reader = threading.Thread(target=collect)
                reader.start()
                watchdog = threading.Timer(20 * 60 + 35, process.kill)
                watchdog.start()
                metrics = {'clientPings': 0, 'serverPongs': 0, 'maxPongSeconds': 0}

                def wait_phase(name, timeout):
                    deadline = time.monotonic() + timeout
                    while name not in phases:
                        if process.poll() is not None:
                            reader.join(1)  # drain the final flushed phase before judging exit
                            assert name in phases, 'Provider exited before ' + name
                            return
                        assert time.monotonic() < deadline, 'Missing phase ' + name
                        time.sleep(0.02)

                def open_stream():
                    connection, _ = server.accept()
                    endpoints.append((connection.getpeername(), connection.getsockname()))
                    connection.settimeout(5)
                    try:
                        stream = context.wrap_socket(connection, server_side=True)
                    except BaseException:
                        connection.close()
                        raise
                    try:
                        request = read_headers(stream)
                        fields = dict(line.split(b':', 1) for line in request.split(b'\r\n')[1:]
                                      if b':' in line)
                        wskey = next(v.strip() for k, v in fields.items()
                                     if k.lower() == b'sec-websocket-key')
                        accept = base64.b64encode(hashlib.sha1(
                            wskey + b'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest())
                        stream.sendall(b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n'
                                       b'Connection: Upgrade\r\nSec-WebSocket-Accept: ' + accept +
                                       b'\r\nSec-WebSocket-Extensions: permessage-deflate\r\n\r\n')
                        area = subscription(stream, zlib.decompressobj(wbits=-15))
                        return stream, area
                    except BaseException:
                        stream.close()
                        raise

                confirmation = frame(b'{"MessageType":"SubscriptionConfirmation",'
                                     b'"Message":{"CompressionEnabled":true}}')
                report = {'MessageType': 'PositionReport', 'MetaData': {'MMSI': 123456789},
                          'Message': {'PositionReport': {'Valid': True, 'UserID': 123456789,
                                      'Latitude': 59.1, 'Longitude': 18.2, 'Sog': 4.5, 'Cog': 120.2}}}
                try:
                    stream, first_area = open_stream()
                    with stream:
                        stream.sendall(confirmation)
                        stream.sendall(frame(json.dumps(report).encode()))
                        sent_at = time.monotonic()
                        wait_phase('initial', 5)
                        next_ping, pending = sent_at, {}
                        while 'lost' not in phases:
                            now = time.monotonic()
                            assert now - sent_at < 20 * 60 + 12, 'Report outage deadline expired'
                            assert process.poll() is None, 'Provider exited during report outage'
                            if now >= next_ping:
                                payload = str(len(pending) + metrics['serverPongs']).encode()
                                pending[payload] = now
                                stream.sendall(frame(payload, opcode=9))
                                next_ping = now + 10
                            if stream.pending() or select.select([stream], [], [], 0.2)[0]:
                                opcode, payload = control_frame(stream)
                                if opcode == 9:
                                    stream.sendall(frame(payload, opcode=10))
                                    metrics['clientPings'] += 1
                                else:
                                    assert payload in pending, 'Unexpected server pong'
                                    latency = time.monotonic() - pending.pop(payload)
                                    assert latency < 5, 'Unresponsive transport'
                                    metrics['serverPongs'] += 1
                                    metrics['maxPongSeconds'] = max(metrics['maxPongSeconds'], latency)
                            assert all(time.monotonic() - at < 5 for at in pending.values()), 'Pong missing'
                        metrics['reportSilenceSeconds'] = time.monotonic() - sent_at
                        assert metrics['reportSilenceSeconds'] > 20 * 60
                        assert 'stale' in phases and metrics['clientPings'] >= 3
                        assert metrics['serverPongs'] >= 120 and not pending
                        # Genuine TCP teardown, after the actual provider has
                        # observed lost state. Cache/enable are never reset.
                        stream.shutdown(socket.SHUT_RDWR)
                    disconnected_at = time.monotonic()
                    wait_phase('disconnected', 5)
                    stream, replacement_area = open_stream()
                    with stream:
                        metrics['reconnectSeconds'] = time.monotonic() - disconnected_at
                        assert metrics['reconnectSeconds'] >= 1.8, 'Reconnect storm'
                        assert replacement_area == first_area, 'Reconnect lost viewport'
                        stream.sendall(confirmation)
                        wait_phase('confirmed_still_lost', 5)
                        # Confirmation alone must not invent a position receipt.
                        time.sleep(0.25)
                        assert 'recovered' not in phases, 'Confirmation refreshed lost target'
                        report['Message']['PositionReport']['Latitude'] = 59.2
                        stream.sendall(frame(json.dumps(report).encode()))
                        wait_phase('recovered', 5)
                    process.wait(timeout=8)
                    assert process.returncode == 0 and any(line.startswith('PASS ') for line in output)
                finally:
                    watchdog.cancel()
                    if process.poll() is None:
                        process.kill()
                    process.wait(timeout=5)
                    reader.join(5)
                    assert not reader.is_alive(), 'Output reader did not stop'

            actual = [list(map(int, line.split()[1:])) for line in output if line.startswith('OBS ')]
            assert len(actual) == len(endpoints) == 2, 'Missing/extra connection observations'
            assert actual[1][0] > actual[0][0], 'Connection generation did not advance'
            for values, (local, remote) in zip(actual, endpoints):
                _, family, local_port, remote_port, local_scope, remote_scope, *addresses = values
                assert family == (6 if ipv6 else 4)
                assert (local_port, remote_port) == (local[1], remote[1]), 'Socket ports differ'
                assert (local_scope, remote_scope) == ((local[3], remote[3]) if ipv6 else (0, 0))
                def address_bytes(endpoint):
                    raw = ipaddress.ip_address(endpoint[0]).packed
                    return list(raw) + [0] * (16 - len(raw))
                assert addresses == address_bytes(local) + address_bytes(remote), 'Socket addresses differ'
            metrics['family'] = 6 if ipv6 else 4
            metrics['generations'] = [entry[0] for entry in actual]
            print('PASS server ' + json.dumps(metrics, sort_keys=True), flush=True)
            return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--client', type=Path, required=True)
    parser.add_argument('--ipv6', action='store_true', help='Use IPv6 instead of IPv4')
    args = parser.parse_args()
    run_case(args.client, args.ipv6)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Loopback-only TLS and adversarial WebSocket tests. No service or key needed."""
import argparse
import base64
import hashlib
import socket
import ssl
import struct
import subprocess
import tempfile
import threading
import time
import zlib
from pathlib import Path


def frame(data=b'', opcode=2, fin=True, compressed=False):
    header = bytes([(0x80 if fin else 0) | (0x40 if compressed else 0) | opcode])
    size = len(data)
    header += bytes([size]) if size < 126 else (b'\x7e' + struct.pack('!H', size) if size <= 65535 else b'\x7f' + struct.pack('!Q', size))
    return header + data


def compressed(data):
    c = zlib.compressobj(wbits=-15)
    return (c.compress(data) + c.flush(zlib.Z_SYNC_FLUSH))[:-4]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--client', type=Path, required=True)
    args = parser.parse_args()
    cases = [
        ('binary', frame(b'{"MessageType":"SubscriptionConfirmation"}'), 1, 0),
        ('compressed', frame(compressed(b'a'*4096), compressed=True), 1, 0),
        ('exact-limit', frame(b'x'*65536), 1, 0),
        ('message-too-large', frame(b'x'*65537), 0, 1009),
        ('compressed-exact-limit', frame(compressed(b'x'*65536), compressed=True), 1, 0),
        ('inflate-bomb', frame(compressed(b'x'*(4*1024*1024)), compressed=True), 0, 1009),
        ('fragmented', frame(b'a'*32000, fin=False)+frame(b'b'*32000, opcode=0), 1, 0),
        ('fragment-overflow', frame(b'a'*40000, fin=False)+frame(b'b'*40000, opcode=0), 0, 1009),
        # Both valid reports must arrive even when bounded dispatch leaves
        # decrypted bytes in OpenSSL and the peer sends no more network data.
        ('two-reports-across-receive-cap', frame(b'a'*40000)+frame(b'b'*40000), 2, 0),
        ('empty-fragment-flood', frame(fin=False)+frame(opcode=0,fin=False)*256, 0, 1009),
        ('advertised-32bit-overflow', b'\x82\x7f'+struct.pack('!Q', 1<<32), 0, 1009),
        ('advertised-64bit-overflow', b'\x82\x7f'+struct.pack('!Q', (1<<64)-1), 0, 1009),
        ('continuous-small-reports', frame(b'a'*1000)*500, 500, 0),
        ('http-status-overflow', None, 0, 0),
        ('http-header-overflow', None, 0, 0),
        ('redirect-refused', None, 0, 0),
        ('untrusted-certificate', None, 0, 0),
        ('hostname-mismatch', None, 0, 0),
    ]
    with tempfile.TemporaryDirectory(prefix='xnav-transport-test-') as directory:
        path=Path(directory)
        cert, key = path/'cert.pem', path/'key.pem'
        subprocess.run(['openssl','req','-x509','-newkey','rsa:2048','-nodes','-days','1',
                        '-subj','/CN=localhost','-addext','subjectAltName=DNS:localhost',
                        '-keyout',str(key),'-out',str(cert)],check=True,capture_output=True)
        context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(cert,key)
        for name, payload, messages, close in cases:
            server=socket.socket()
            server.bind(('127.0.0.1',0));server.listen(1);server.settimeout(8)
            port=server.getsockname()[1]
            errors=[]
            def serve():
                try:
                    conn,_=server.accept()
                    with conn:
                        conn.settimeout(7)
                        with context.wrap_socket(conn,server_side=True) as stream:
                            request=b''
                            while b'\r\n\r\n' not in request:
                                data=stream.recv(4096)
                                if not data: return
                                request+=data
                                if len(request)>16384: raise RuntimeError('test request too large')
                            if name=='http-status-overflow':
                                stream.sendall(b'HTTP/1.1 101 '+b'x'*5000+b'\r\n');return
                            if name=='http-header-overflow':
                                stream.sendall(b'HTTP/1.1 101 Switching Protocols\r\n'+b'X-many: '+b'x'*950+b'\r\n'+(b'X-more: '+b'x'*950+b'\r\n')*20);return
                            if name=='redirect-refused':
                                stream.sendall(b'HTTP/1.1 302 Found\r\nLocation: ws://127.0.0.1:1/\r\n\r\n');return
                            fields=dict(line.split(b':',1) for line in request.split(b'\r\n')[1:] if b':' in line)
                            wskey=next(v.strip() for k,v in fields.items() if k.lower()==b'sec-websocket-key')
                            accept=base64.b64encode(hashlib.sha1(wskey+b'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest())
                            header=b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: '+accept+b'\r\nSec-WebSocket-Extensions: permessage-deflate\r\n\r\n'
                            stream.sendall(header)
                            if payload: stream.sendall(payload)
                            # Keep peer alive until normal client close/rejection.
                            while stream.recv(4096): pass
                except (BrokenPipeError,ConnectionResetError,ssl.SSLError): pass
                except Exception as error: errors.append(type(error).__name__)
                finally: server.close()
            worker=threading.Thread(target=serve);worker.start()
            host='127.0.0.1' if name=='hostname-mismatch' else 'localhost'
            trust='SYSTEM' if name=='untrusted-certificate' else str(cert)
            run=subprocess.run([str(args.client.resolve()),f'wss://{host}:{port}/',trust,str(messages),str(close)],capture_output=True,text=True,timeout=12)
            worker.join(9)
            if run.returncode or worker.is_alive() or errors:
                raise AssertionError(f'{name}: {run.returncode} {run.stdout} {run.stderr} {errors}')
            print(f'PASS {name}: {run.stdout.strip()}',flush=True)
    print(f'{len(cases)} TLS/transport scenarios passed')

if __name__=='__main__': main()

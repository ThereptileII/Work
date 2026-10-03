#!/usr/bin/env python3
"""Actual provider lifecycle against a TLS loopback server, never live AIS."""
import argparse
import base64
import hashlib
import json
import ipaddress
from pathlib import Path
import socket
import ssl
import struct
import subprocess
import tempfile
import threading
import time
import zlib
from network_tests import frame


def read(stream, length):
    out=b''
    while len(out)<length:
        data=stream.recv(length-len(out))
        if not data: raise EOFError()
        out+=data
    return out


def subscription(stream, inflater):
    header=read(stream,2)
    if not header[0]&128: raise AssertionError('Client fragmented subscription')
    size=header[1]&127
    if size==126:size=struct.unpack('!H',read(stream,2))[0]
    elif size==127:size=struct.unpack('!Q',read(stream,8))[0]
    if size>8192:raise AssertionError('Oversized client subscription')
    if not header[1]&128:raise AssertionError('Unmasked client')
    mask=read(stream,4)
    payload=bytes(c^mask[i%4] for i,c in enumerate(read(stream,size)))
    if header[0]&15==9:
        stream.sendall(frame(payload,opcode=10))
        return subscription(stream,inflater)
    if header[0]&15==8:raise AssertionError('Client closed before subscription')
    if header[0]&64:payload=inflater.decompress(payload+b'\x00\x00\xff\xff',16384)
    data=json.loads(payload)
    assert data['APIKey']=='loopback-test-not-a-real-key', 'Test key mismatch'
    assert data['FilterMessageTypes']==['PositionReport','StandardClassBPositionReport','ExtendedClassBPositionReport','ShipStaticData','StaticDataReport']
    assert len(data['BoundingBoxes'])==1
    return data['BoundingBoxes']


def run_case(client, scenario, ipv6=False):
    with tempfile.TemporaryDirectory(prefix='xnav-provider-') as directory:
        path=Path(directory);cert,key=path/'cert.pem',path/'key.pem'
        subprocess.run(['openssl','req','-x509','-newkey','rsa:2048','-nodes','-days','1',
                        '-subj','/CN=localhost','-addext','subjectAltName=DNS:localhost',
                        '-keyout',str(key),'-out',str(cert)],check=True,capture_output=True)
        context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER);context.load_cert_chain(cert,key)
        server=socket.socket(socket.AF_INET6 if ipv6 else socket.AF_INET);server.bind(('::1' if ipv6 else '127.0.0.1',0));server.listen(2);server.settimeout(15)
        port=server.getsockname()[1];errors=[];observations=[];endpoints=[]
        delayed_open=scenario.startswith('open-')
        discarded=[]
        def serve():
            try:
                previous=None;disconnected=None
                for connection_index in range(3 if delayed_open else 2):
                    attempt=connection_index-int(delayed_open)
                    conn,_=server.accept()
                    endpoints.append((conn.getpeername(), conn.getsockname()))
                    if disconnected is not None:
                        elapsed=time.monotonic()-disconnected
                        assert elapsed>=1.8, 'Reconnect storm'
                    with conn:
                        conn.settimeout(12)
                        with context.wrap_socket(conn,server_side=True) as stream:
                            request=b''
                            while b'\r\n\r\n' not in request:
                                request+=stream.recv(4096)
                                assert len(request)<16384
                            fields=dict(line.split(b':',1) for line in request.split(b'\r\n')[1:] if b':' in line)
                            wskey=next(v.strip() for k,v in fields.items() if k.lower()==b'sec-websocket-key')
                            accept=base64.b64encode(hashlib.sha1(wskey+b'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest())
                            stream.sendall(b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: '+accept+b'\r\nSec-WebSocket-Extensions: permessage-deflate\r\n\r\n')
                            if attempt < 0:
                                # A genuine 101/TLS connection is paused at the client's
                                # Open callback. It must close without an AIS subscription.
                                while True:
                                    head=read(stream,2);length=head[1]&127
                                    assert length < 126, 'Unexpected discarded-connection payload'
                                    mask=read(stream,4) if head[1]&128 else b''
                                    payload=read(stream,length)
                                    opcode=head[0]&15
                                    assert opcode in (8,9,10), 'Stale successful Open sent application data'
                                    if opcode==8:
                                        stream.sendall(frame(struct.pack('!H',1000),opcode=8))
                                        break
                                discarded.append('successful upgrade, no subscription')
                                continue
                            opened=time.monotonic();inflater=zlib.decompressobj(wbits=-15)
                            area=subscription(stream,inflater)
                            assert time.monotonic()-opened<3, 'Initial subscription late'
                            if attempt:assert area==previous,'Reconnect lost viewport'
                            stream.sendall(frame(b'{"MessageType":"SubscriptionConfirmation","Message":{"CompressionEnabled":true}}'))
                            latitude,longitude=(59.1,18.2) if not attempt else (40.1,10.2)
                            report={'MessageType':'PositionReport','MetaData':{'MMSI':123456789},'Message':{'PositionReport':{'Valid':True,'UserID':123456789,'Latitude':latitude,'Longitude':longitude,'Sog':4.5,'Cog':120.2}}}
                            stream.sendall(frame(json.dumps(report).encode()))
                            observations.append('confirmed position')
                            if not attempt:
                                previous=subscription(stream,inflater)
                                assert previous!=area,'Viewport change was not sent'
                                assert time.monotonic()-opened>=4.8, 'Viewport replacement storm'
                                stream.sendall(frame(b'{"MessageType":"SubscriptionConfirmation","Message":{"CompressionEnabled":true}}'))
                                stream.sendall(frame(struct.pack('!H',1000),opcode=8))
                                disconnected=time.monotonic()
                            else:
                                while stream.recv(4096):pass
            except (BrokenPipeError,ConnectionResetError):pass
            except Exception as error:errors.append(type(error).__name__+': '+str(error))
            finally:server.close()
        worker=threading.Thread(target=serve);worker.start()
        run=subprocess.run([str(client.resolve()),f'wss://localhost:{port}/',str(cert),scenario],capture_output=True,text=True,timeout=25)
        worker.join(15)
        assert not worker.is_alive() and not errors and run.returncode==0, f'{run.returncode} {run.stdout} {run.stderr} {errors}'
        assert len(observations)==2
        actual=[list(map(int,line.split()[1:])) for line in run.stdout.splitlines() if line.startswith('OBS ')]
        assert len(discarded)==int(delayed_open)
        expected_endpoints=endpoints[1:] if delayed_open else endpoints
        assert len(actual)==len(expected_endpoints)==2, 'Missing/extra provider connection observations'
        for values,(local,remote) in zip(actual,expected_endpoints):
            _,family,local_port,remote_port,local_scope,remote_scope,*addresses=values
            assert family==(6 if ipv6 else 4)
            assert (local_port,remote_port)==(local[1],remote[1]), 'Wrong actual socket ports'
            assert (local_scope,remote_scope)==((local[3],remote[3]) if ipv6 else (0,0))
            def address_bytes(address):
                raw=ipaddress.ip_address(address[0]).packed
                return list(raw)+[0]*(16-len(raw))
            assert addresses==address_bytes(local)+address_bytes(remote), 'Wrong actual socket addresses'
        print(scenario, 'IPv6' if ipv6 else 'IPv4')
        print(run.stdout.strip())
        print('PASS server: complete prompt subscriptions, compression, 5s pan cadence, bounded reconnect, key never logged')

def main():
    p=argparse.ArgumentParser();p.add_argument('--client',type=Path,required=True);args=p.parse_args()
    for scenario in ('normal', 'disable-enable', 'credential-change', 'open-disable-enable', 'open-credential-change'):
        run_case(args.client, scenario)
    run_case(args.client, 'normal', ipv6=True)

if __name__=='__main__':main()

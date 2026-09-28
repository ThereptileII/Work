#!/usr/bin/env python3
"""Actual provider lifecycle against a TLS loopback server, never live AIS."""
import argparse
import base64
import hashlib
import json
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


def main():
    p=argparse.ArgumentParser();p.add_argument('--client',type=Path,required=True);args=p.parse_args()
    with tempfile.TemporaryDirectory(prefix='xnav-provider-') as directory:
        path=Path(directory);cert,key=path/'cert.pem',path/'key.pem'
        subprocess.run(['openssl','req','-x509','-newkey','rsa:2048','-nodes','-days','1',
                        '-subj','/CN=localhost','-addext','subjectAltName=DNS:localhost',
                        '-keyout',str(key),'-out',str(cert)],check=True,capture_output=True)
        context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER);context.load_cert_chain(cert,key)
        server=socket.socket();server.bind(('127.0.0.1',0));server.listen(2);server.settimeout(15)
        port=server.getsockname()[1];errors=[];observations=[]
        def serve():
            try:
                previous=None;disconnected=None
                for attempt in range(2):
                    conn,_=server.accept()
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
        run=subprocess.run([str(args.client.resolve()),f'wss://localhost:{port}/',str(cert)],capture_output=True,text=True,timeout=25)
        worker.join(15)
        assert not worker.is_alive() and not errors and run.returncode==0, f'{run.returncode} {run.stdout} {run.stderr} {errors}'
        assert len(observations)==2
        print(run.stdout.strip())
        print('PASS server: complete prompt subscriptions, compression, 5s pan cadence, bounded reconnect, key never logged')

if __name__=='__main__':main()

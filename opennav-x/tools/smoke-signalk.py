#!/usr/bin/env python3
"""Loopback WebSocket -> actual OpenCPN Signal K driver -> Vessel Data.

No third-party test server, credentials, discovery, physical network or output.
"""
import base64
import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from diagnostic_snapshot import read_json_snapshot

root=Path(__file__).resolve().parents[1];windows=sys.platform=='win32'
evidence=root/'evidence/local';evidence.mkdir(parents=True,exist_ok=True)
temporary=tempfile.TemporaryDirectory(prefix='OpenNav Signal K ')
profile=Path(temporary.name)/'profile'
subprocess.run([sys.executable,str(root/'tools/prepare-test-profile.py'),'--build',
               str(root/('build/xnav-windows' if windows else 'build/xnav-linux')),
               '--profile',str(profile)],check=True)
server=socket.socket();server.bind(('127.0.0.1',0));server.listen(2);server.settimeout(.5)
with (profile/'opencpn.conf').open('a') as f:
    f.write('\n[Settings/NMEADataSource]\nDataConnections='+
            f'1;3;127.0.0.1;{server.getsockname()[1]};2;;4800;1;0;0;;0;;0;0;0;0;1;SIMULATED Signal K commissioning;0;;0;1;\n')
stop=threading.Event();connected=threading.Event();phase=['live'];errors=[]
report={'authority':'native Windows' if windows else 'Linux development','checks':[],
        'protocol':'Actual OpenCPN Signal K WebSocket input on loopback only','frames_sent':0}
def frame(peer,text):
    content=text.encode('utf-8');length=len(content)
    header=bytes([0x81,length]) if length<126 else b'\x81\x7e'+struct.pack('!H',length) if length<65536 else b'\x81\x7f'+struct.pack('!Q',length)
    peer.sendall(header+content);report['frames_sent']+=1
def transmit():
    while not stop.is_set():
        try:peer,address=server.accept()
        except socket.timeout:continue
        except OSError:return
        try:
            assert address[0]=='127.0.0.1';peer.settimeout(2)
            request=peer.recv(8192)
            # Upstream tries TLS first, then plain WS. This isolated fixture has
            # no TLS listener; close that attempt and allow its normal fallback.
            if not request.startswith(b'GET '):continue
            while b'\r\n\r\n' not in request and len(request)<8192:request+=peer.recv(8192)
            assert b'/signalk/v1/stream?' in request
            headers=dict(line.split(b':',1) for line in request.split(b'\r\n')[1:] if b':' in line)
            key=next(value.strip() for name,value in headers.items() if name.lower()==b'sec-websocket-key')
            accept=base64.b64encode(hashlib.sha1(key+b'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest())
            peer.sendall(b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: '+accept+b'\r\n\r\n')
            frame(peer,json.dumps({'version':'test fixture','self':'vessels.opennav-fixture'}));connected.set()
            last=''
            while not stop.wait(.25):
                mode=phase[0]
                if mode=='malformed':
                    if last!=mode:
                        for bad in ['['*10000+'0'+']'*10000, ' '*262145,
                                    '{"version":4}', '{"self":[]}', '{"context":true}',
                                    '{"context":"vessels.\\u0000invalid"}', '{bad', 'null']:
                            frame(peer,bad)
                    last=mode;continue
                last=mode
                if mode=='quiet':continue
                values=[{'path':'environment.depth.belowTransducer','value':None if mode=='invalid' else 8.4},
                        {'path':'electrical.batteries.house.voltage','value':343},
                        {'path':'electrical.batteries.house.capacity.stateOfCharge','value':.68},
                        {'path':'propulsion.main.revolutions','value':820/60}]
                frame(peer,json.dumps({'context':'vessels.opennav-fixture','updates':[{
                    '$source':'test.Åland','timestamp':datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds').replace('+00:00','Z'),
                    'values':values}]}))
        except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError):pass
        except Exception as error:
            if not stop.is_set():errors.append(repr(error))
        finally:peer.close()
thread=threading.Thread(target=transmit,daemon=True);thread.start()
env=dict(os.environ);app=xserver=None;ui=None
def data(predicate,timeout=20):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        assert app.poll() is None,('Unexpected process exit',app.returncode)
        try:
            d=read_json_snapshot(profile/'opennav-diagnostics.json',.5)
            values={v['name']:v for v in d['data']}
            if predicate(values):return d
        except FileNotFoundError:pass
        time.sleep(.15)
    raise AssertionError(('Signal K predicate timed out',values if 'values' in locals() else 'no diagnostics'))
try:
    if windows:
        spec=importlib.util.spec_from_file_location('ui',root/'tools/windows-ui.py')
        ui=importlib.util.module_from_spec(spec);spec.loader.exec_module(ui);ui.ensure_desktop()
    else:
        number=181
        while Path(f'/tmp/.X{number}-lock').exists():number+=1
        env['DISPLAY']=f':{number}'
        xserver=subprocess.Popen(['Xvfb',env['DISPLAY'],'-screen','0','1280x800x24','-nolisten','tcp'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        time.sleep(1)
    exe=root/('build/xnav-install/opencpn.exe' if windows else 'build/xnav-install/bin/opencpn')
    with (evidence/'signalk-launch.log').open('w') as log:
        app=subprocess.Popen([str(exe),'--configdir',str(profile),'--no_opengl','--xnav'],env=env,stdout=log,stderr=log)
    assert connected.wait(30),'Actual driver did not connect to loopback WebSocket'
    d=data(lambda v: v['Depth below transducer']['quality']=='LIVE' and v['Battery SOC'].get('value')==68)
    values={v['name']:v for v in d['data']}
    assert values['Depth below transducer']['value']==8.4 and values['Battery voltage']['value']==343
    assert abs(values['Motor speed']['value']-820)<.001
    assert all('SignalK/' in values[name]['source'] for name in ['Depth below transducer','Battery SOC','Battery voltage','Motor speed'])
    report['build_commit']=d['build_commit']
    report['checks'].append('Actual WebSocket driver: depth, pack V/SOC, motor RPM and UTF-8 source provenance')
    phase[0]='malformed'
    data(lambda v:v['Depth below transducer']['quality']=='STALE')
    report['checks'].append('Deep/oversized/wrong-type/control-text/malformed JSON: process survives and retained input expires')
    phase[0]='live';data(lambda v:v['Depth below transducer']['quality']=='LIVE')
    report['checks'].append('Valid input recovers through same connection after rejected frames')
    phase[0]='invalid';data(lambda v:'value' not in v['Depth below transducer'] and v['Depth below transducer']['validity']=='Invalid')
    report['checks'].append('Explicit null invalidates depth instead of displaying zero')
    phase[0]='quiet';data(lambda v:v['Battery SOC']['quality']=='STALE')
    report['checks'].append('Whole marine input loss expires battery and suppresses predictions')
    assert not errors,errors
    if windows:
        handle,_=ui.wait_window('OpenNav X / OpenCPN',app.pid)
        monitor=ui.monitor_process(app.pid);ui.close(handle);ui.wait_clean_exit(monitor)
    else:subprocess.run([str(exe),'--configdir',str(profile),'--remote','--quit'],env=env,capture_output=True,check=True,timeout=15)
    assert app.wait(timeout=30)==0
    report['status']='passed'
except BaseException as error:
    report['status']='failed';report['error']=repr(error);raise
finally:
    stop.set();server.close();thread.join(timeout=4)
    if app and app.poll() is None:
        app.terminate()
        try:app.wait(timeout=10)
        except subprocess.TimeoutExpired:app.kill();app.wait(timeout=5)
    if xserver:xserver.terminate();xserver.wait(timeout=10)
    for name in ['opencpn.log','opennav-diagnostics.json']:
        if (profile/name).exists():shutil.copy2(profile/name,evidence/('signalk-'+name))
    report['transport_errors']=errors
    (evidence/'signalk-results.json').write_text(json.dumps(report,indent=2)+'\n')
    temporary.cleanup()
print(json.dumps(report,indent=2))

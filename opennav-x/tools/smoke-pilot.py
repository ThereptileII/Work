#!/usr/bin/env python3
"""Actual OpenCPN TCP/manual-UI pilot path against a loopback-only translator.

No serial/CAN interface or physical pilot is opened. This is desktop software
qualification, not evidence of boat command delivery.
"""
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
from diagnostic_snapshot import read_json_snapshot

root=Path(__file__).resolve().parents[1]
windows=sys.platform=='win32'
evidence=root/'evidence/local';evidence.mkdir(parents=True,exist_ok=True)
temporary=tempfile.TemporaryDirectory(prefix='OpenNav pilot loopback ')
profile=Path(temporary.name).resolve()/'profile'
variant='xnav-windows' if windows else 'xnav-linux'
subprocess.run([sys.executable,str(root/'tools/prepare-test-profile.py'),'--build',str(root/'build'/variant),'--profile',str(profile)],check=True)
server=socket.socket();server.bind(('127.0.0.1',0));server.listen(2);server.settimeout(.2)
port=server.getsockname()[1];iface=f'TCP:127.0.0.1:{port}';name='c0508700e76004d2'
settings={k:'' for k in ['battery','capacity','corridor','draft','efficiency','hotel','margin','reserve']}
settings.update({'consumption':'measured','current':'unconfigured','minimum_speed':'0.5',
                 'model_source':'Unconfigured loopback test / no boat assumptions',
                 'pilot.interface':iface,'pilot.name':name,'pilot.permission':'manual'})
encoded='OpenNavXSettings 1\n'+''.join(json.dumps(k)+' '+json.dumps(v)+'\n' for k,v in sorted(settings.items()))
with (profile/'opencpn.conf').open('a',encoding='utf-8') as out:
    out.write('\n[Settings/NMEADataSource]\nDataConnections='
              f'1;0;127.0.0.1;{port};1;;4800;1;1;1;;0;;0;0;0;0;1;'
              'SIMULATED loopback ST4000 test;0;;0;1;\n')
    out.write('\n[OpenNav]\nAlphaSettings='+encoded.replace('\\','\\\\').replace('\n','\\n')+'\n')
stop=threading.Event();connected=threading.Event()
mode=['STANDBY'];target=[328.0];actual=328.0
silence=threading.Event();disconnect=threading.Event();claims=[True]
sent=[];failures=[];connections=[0]

def packet(pgn,data):return f'A001001.732 CCFF3 {pgn:05X} {data.hex().upper()}\r\n'.encode()
def angle(d):return int(round(math.radians(d)/.0001)).to_bytes(2,'little')
def transmitter():
    peer=None;buffer=b'';pending=None;last=0
    try:
        while not stop.is_set():
            if peer is None:
                try:peer,address=server.accept()
                except TimeoutError:continue
                assert address[0]=='127.0.0.1';peer.settimeout(.02)
                connections[0]+=1;connected.set();buffer=b'';last=0
            if disconnect.is_set():
                peer.close();peer=None;disconnect.clear();continue
            try:chunk=peer.recv(4096)
            except TimeoutError:chunk=None
            if chunk==b'':peer.close();peer=None;continue
            if chunk:
                buffer+=chunk
                assert len(buffer)<8192,'Unbounded/unterminated output'
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1)
                    words=line.strip().split();assert len(words)==4,line
                    pgn=int(words[2],16);data=bytes.fromhex(words[3].decode())
                    destination=int(words[1][2:4],16)
                    if pgn==59904:
                        assert destination==255 and data==bytes.fromhex('00ee00'),line
                        claims[0]=True;continue
                    assert pgn==126208 and destination==204,line
                    assert data[:-1]==bytes.fromhex('0163ff00ff03013b07030406'),line
                    key=data[-1];assert key in [0,0x40,0x50,0x7f,0x51,0xd1],line
                    sent.append({'key':key,'wire':line.decode(),'time':time.monotonic()})
                    pending=(key,time.monotonic()+.7)
            if pending and time.monotonic()>=pending[1] and not silence.is_set():
                key=pending[0];pending=None
                if key==0:mode[0]='STANDBY'
                elif key==0x40:mode[0]='AUTO';target[0]=actual
                else:
                    assert mode[0]=='AUTO','Step outside confirmed AUTO'
                    target[0]=(target[0]+{0x50:-10,0x7f:-1,0x51:1,0xd1:10}[key])%360
            if time.monotonic()-last>=.25:
                last=time.monotonic();payload=b''
                if claims[0]:payload+=packet(60928,int(name,16).to_bytes(8,'little'))
                if not silence.is_set():
                    value=0x40 if mode[0]=='AUTO' else 0
                    payload+=packet(65379,bytes.fromhex('3b9f')+value.to_bytes(2,'little')+bytes.fromhex('ffff')+bytes([value,255]))
                    payload+=packet(127250,b'\x01'+angle(actual)+bytes.fromhex('fffffffffd'))
                    if mode[0]=='AUTO':payload+=packet(65360,bytes.fromhex('3b9f01ffff')+angle(target[0])+b'\xff')
                if payload:peer.sendall(payload)
    except (BrokenPipeError,ConnectionResetError):
        if not stop.is_set():failures.append('Unexpected loopback disconnection')
    except BaseException as error:failures.append(repr(error))
    finally:
        if peer:peer.close()

thread=threading.Thread(target=transmitter,daemon=True);thread.start()
env=dict(os.environ);xserver=None;app=None;ui=None;handle=None
report={'authority':'native Windows' if windows else 'Linux development',
        'hardware':'NONE; loopback simulated translator','checks':[],'screenshots':[]}
def data(predicate=lambda d:True,timeout=15):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        assert not failures,failures
        try:
            d=read_json_snapshot(profile/'opennav-diagnostics.json')
            if predicate(d):return d
        except (FileNotFoundError,ValueError,PermissionError):pass
        time.sleep(.15)
    raise AssertionError('Pilot diagnostics timed out: '+json.dumps(d.get('runtime',{}).get('pilot',{})))
def pilot(predicate=lambda p:True,timeout=15):
    return data(lambda d:predicate(d.get('runtime',{}).get('pilot',{})),timeout)['runtime']['pilot']
def xdo(*args):return subprocess.check_output(['xdotool',*map(str,args)],env=env,text=True).strip()
def capture(name):
    p=evidence/(name+('.png' if windows else '-linux.png'))
    if windows:ui.capture(handle,p)
    else:subprocess.run(['import','-window','root',str(p)],env=env,check=True)
    report['screenshots'].append(p.name)
def click(label,x,y):
    if windows:ui.click_text(app.pid,label)
    else:
        xdo('windowfocus',handle);xdo('mousemove','--window',handle,x,y)
        time.sleep(.15);xdo('mousedown',1);time.sleep(.08);xdo('mouseup',1);time.sleep(.4)
def confirm(label):
    if windows:ui.click_text(app.pid,label)
    else:
        # The common sheet is centred at 1280x800; two footer actions.
        time.sleep(.2)
        title='Enable physical pilot control?' if label=='Enable manual control' else label
        dialogs=xdo('search','--onlyvisible','--pid',app.pid,'--name',title.replace('?','[?]')).splitlines()
        # Modal sheets are separate X11 windows; click relative to their own
        # dimensions instead of assuming their centering within the frame.
        modal=next((w for w in dialogs if w!=handle and
                    'WIDTH=520' in xdo('getwindowgeometry','--shell',w)),None)
        assert modal,('No confirmation sheet',dialogs)
        xdo('windowraise',modal);xdo('windowfocus',modal);time.sleep(.2)
        geometry=dict(line.split('=',1) for line in xdo('getwindowgeometry','--shell',modal).splitlines())
        report.setdefault('confirmations',[]).append({'title':title,'geometry':geometry,'name':xdo('getwindowname',modal)})
        capture('pilot-confirmation')
        xdo('mousemove','--window',modal,int(geometry['WIDTH'])*3//4,int(geometry['HEIGHT'])-40)
        xdo('click',1);time.sleep(.4)
def show_pilot():
    if windows:ui.click_text(app.pid,'Menu');ui.click_text(app.pid,'Manual autopilot')
    else:xdo('windowfocus',handle);xdo('key','ctrl+shift+y');time.sleep(.5)
    data(lambda d:d['ui_page']=='Manual autopilot')

try:
    if windows:
        spec=importlib.util.spec_from_file_location('ui',root/'tools/windows-ui.py');ui=importlib.util.module_from_spec(spec);spec.loader.exec_module(ui)
        report['display']=ui.ensure_desktop();exe=root/'build/xnav-install/opencpn.exe'
    else:
        n=120
        while Path(f'/tmp/.X{n}-lock').exists():n+=1
        env['DISPLAY']=f':{n}';xserver=subprocess.Popen(['Xvfb',env['DISPLAY'],'-screen','0','1280x800x24','-nolisten','tcp'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);time.sleep(1)
        exe=root/'build/xnav-install/bin/opencpn'
    with (evidence/'pilot-loopback-launch.log').open('w') as output:
        app=subprocess.Popen([str(exe),'--configdir',str(profile),'--no_opengl','--xnav'],env=env,stdout=output,stderr=output)
    deadline=time.monotonic()+60
    while time.monotonic()<deadline:
        assert app.poll() is None,'Application exited'
        log=profile/'opencpn.log'
        if log.exists() and 'OnInitTimer...Finalize Canvases' in log.read_text(errors='replace'):break
        time.sleep(.2)
    else:raise RuntimeError('Startup timeout')
    assert connected.wait(10),'No loopback driver connection'
    if windows:handle,_=ui.wait_window('OpenNav X / OpenCPN',app.pid)
    else:
        handle=xdo('search','--onlyvisible','--pid',app.pid,'--name','^OpenNav X / OpenCPN$').splitlines()[0]
        xdo('windowsize',handle,1280,800);xdo('windowmove',handle,0,0);xdo('windowfocus',handle)
    state=pilot(lambda p:p.get('fresh') and p.get('mode')=='STANDBY')
    assert not state['enabled'] and not sent,'Permission cannot auto-enable or emit controls'
    assert not state['track_capability'] and not state['wind_capability']
    show_pilot();capture('pilot-01-status-only')
    click('AUTO',475,406);confirm('Request AUTO')
    pilot(lambda p:p.get('command_state')=='Disabled');assert not sent
    click('Enable / disable manual control',550,345);confirm('Enable manual control')
    pilot(lambda p:p.get('enabled'))
    for label,key,delta,x in [('AUTO',0x40,0,475),('+1° magnetic course',0x51,1,790),
                            ('-1° magnetic course',0x7f,-1,475),('+10° magnetic course',0xd1,10,1100),
                            ('-10° magnetic course',0x50,-10,170)]:
        before=len(sent);previous=int(pilot()['command_id'])
        click(label,x,406 if label=='AUTO' else 467)
        if label=='AUTO':confirm('Request AUTO')
        state=pilot(lambda p:p.get('command_state')=='Confirmed' and
                    int(p.get('command_id','0'))>previous and len(sent)>before)
        assert sent[-1]['key']==key and state['mode']=='AUTO',state
        assert abs(state['locked_heading_magnetic_deg']-target[0])<.02,state
    capture('pilot-02-confirmed-manual')
    silence.set();before=len(sent)
    click('+1° magnetic course',790,467)
    pilot(lambda p:p.get('command_state')=='TimedOut',timeout=10)
    assert len(sent)==before+1,'No automatic resend after missing feedback'
    time.sleep(1)
    state=pilot(lambda p:not p.get('fresh'))
    capture('pilot-03-communication-loss')
    click('STANDBY',170,406)
    deadline=time.monotonic()+5
    while len(sent)<before+2 and time.monotonic()<deadline:time.sleep(.1)
    assert len(sent)==before+2 and sent[-1]['key']==0,'Standby may be attempted with stale mode'
    silence.clear();pilot(lambda p:p.get('command_state')=='Confirmed' and p.get('mode')=='STANDBY')
    claims[0]=False;disconnect.set()
    pilot(lambda p:not p.get('enabled'),timeout=12)
    # The same configured driver reconnects: its old NAME must not survive.
    deadline=time.monotonic()+15
    while connections[0]<2 and time.monotonic()<deadline:time.sleep(.2)
    assert connections[0]>=2,'Expected upstream reconnect'
    time.sleep(1)
    state=pilot(lambda p:not p.get('control_capability'))
    assert state['mode']=='UNAVAILABLE' and not state['enabled'],state
    claims[0]=True
    state=pilot(lambda p:p.get('fresh') and p.get('mode')=='STANDBY')
    assert not state['enabled'],'Reconnect cannot restore session enablement'
    capture('pilot-04-reconnected-control-off')
    report['checks']=['Read-only startup despite saved manual permission',
      'Actual UI and OpenCPN TCP serialization of AUTO/-1/+1/-10/+10/STANDBY',
      'New matching physical-style feedback confirms; no optimistic target change',
      'Missing feedback times out without retry; manual STANDBY remains available',
      'Same-driver reconnect clears identity and session enablement; new claim required',
      'Live TRACK/WIND unavailable; no SmartNav control path']
    if windows:monitor=ui.monitor_process(app.pid);ui.close(handle);ui.wait_clean_exit(monitor)
    else:subprocess.run([str(exe),'--configdir',str(profile),'--remote','--quit'],env=env,check=True,capture_output=True)
    assert app.wait(timeout=30)==0
    report['result']='passed; screenshot review required'
except BaseException as error:
    report['error']=str(error)
    if handle:
        try:capture('pilot-failure')
        except Exception:pass
    raise
finally:
    stop.set();thread.join(timeout=3);server.close()
    if app and app.poll() is None:
        app.terminate()
        try:app.wait(timeout=5)
        except subprocess.TimeoutExpired:app.kill();app.wait(timeout=10)
    if xserver:xserver.terminate();xserver.wait(timeout=10)
    report['sent']=sent;report['transport_failures']=failures
    for name in ['opencpn.log','opennav-diagnostics.json','opencpn.conf']:
        if (profile/name).exists():shutil.copy2(profile/name,evidence/('pilot-'+name))
    (evidence/'pilot-results.json').write_text(json.dumps(report,indent=2)+'\n')
    temporary.cleanup()
print(json.dumps(report,indent=2))

#!/usr/bin/env python3
"""Exercise actual recording/replay controls in a disposable, offline profile."""
import ctypes
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
from diagnostic_snapshot import read_json_snapshot

root=Path(__file__).resolve().parents[1]
windows=sys.platform=='win32'
evidence=root/'evidence/local';evidence.mkdir(parents=True,exist_ok=True)
temporary=tempfile.TemporaryDirectory(prefix='OpenNav recording ')
profile=Path(temporary.name).resolve()/'profile'
variant='xnav-windows' if windows else 'xnav-linux'
subprocess.run([sys.executable,str(root/'tools/prepare-test-profile.py'),'--build',str(root/'build'/variant),'--profile',str(profile)],check=True)
env=dict(os.environ);xserver=None;app=None;ui=None
report={'authority':'native Windows' if windows else 'Linux development','checks':[],'screenshots':[]}

def data(predicate=lambda d:True,timeout=15):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:
            d=read_json_snapshot(profile/'opennav-diagnostics.json')
            if predicate(d):return d
        except (FileNotFoundError,ValueError,PermissionError):pass
        time.sleep(.2)
    raise AssertionError('Recording diagnostic predicate timed out')
def xdo(*args):return subprocess.check_output(['xdotool',*map(str,args)],env=env,text=True).strip()
def click(label,x,y):
    if windows:ui.click_text(app.pid,label)
    else:xdo('mousemove','--window',handle,x,y);xdo('click',1);time.sleep(.4)
def page(label,shortcut):
    if windows:
        if label=='Commissioning & recordings':ui.click_text(app.pid,'Menu')
        ui.click_text(app.pid,label)
    else:
        xdo('windowfocus',handle)
        if label=='Energy':xdo('mousemove','--window',handle,260,773);xdo('click',1)
        else:xdo('key','ctrl+shift+'+shortcut)
        time.sleep(.4)
    expected={'Energy':'Energy','Commissioning & recordings':'Commissioning & recordings','Diagnostics':'Diagnostics','System diagnostics':'Diagnostics'}[label]
    data(lambda d:d['ui_page']==expected)
    if windows and label=='Energy':ui.assert_preview_page(handle,'Energy')
def capture(name):
    p=evidence/(name+('.png' if windows else '-linux.png'))
    if windows:ui.capture(handle,p)
    else:subprocess.run(['import','-window','root',str(p)],env=env,check=True)
    report['screenshots'].append(p.name)
def file_dialog(title,path,accept):
    if windows:
        dialog,_=ui.wait_window(title,app.pid)
        edits=[]
        for child,_ in ui.children(dialog):
            name=ctypes.create_unicode_buffer(128);ui.GetClassNameW(child,name,len(name))
            if name.value.lower()=='edit' and ui.IsWindowEnabled(child):
                rect=ui.W.RECT();ui.GetWindowRect(child,ctypes.byref(rect));edits.append((rect.top,child))
        assert edits,('No filename editor',ui.children(dialog))
        edit=max(edits)[1]
        # Common Item Dialog maintains a filename model separate from edit text.
        # Send real keyboard input so its change notifications and validation
        # run, rather than merely changing the HWND caption with WM_SETTEXT.
        rect=ui.W.RECT();assert ui.GetWindowRect(edit,ctypes.byref(rect))
        ui.SetForegroundWindow(dialog)
        ui.SetCursorPos((rect.left+rect.right)//2,(rect.top+rect.bottom)//2)
        ui.MouseEvent(2,0,0,0,0);ui.MouseEvent(4,0,0,0,0)
        key=ui.declare(ui.user,'keybd_event',None,ctypes.c_ubyte,ctypes.c_ubyte,ui.W.DWORD,ctypes.c_size_t)
        key(0x11,0,0,0);key(0x41,0,0,0);key(0x41,0,2,0);key(0x11,0,2,0)
        # WM_CHAR follows actual focus/selection and triggers normal EN_CHANGE.
        encoded=str(path).encode('utf-16-le')
        for i in range(0,len(encoded),2):
            ui.SendMessageW(edit,0x0102,int.from_bytes(encoded[i:i+2],'little'),0)
        assert ui.control_text(edit)==str(path)
        if accept=='Save':ui.capture(dialog,evidence/'recording-calibration-file-dialog.png')
        ui.dismiss_native_dialog(dialog,accept)
    else:
        xdo('key','ctrl+l');time.sleep(.2);xdo('type','--clearmodifiers','--',str(path));xdo('key','Return');time.sleep(.6)

def recording_frames(path):
    lines=path.read_text().splitlines();assert lines[0]=='OpenNavXRecording\t1\t0'
    assert lines[-1].startswith('END\t')
    assert not any(x.startswith(('R\t','L\t')) for x in lines)
    samples=[line.split('\t') for line in lines if line.startswith('S\t')]
    names={bytes.fromhex(s[1]).decode() for s in samples}
    assert 'Latitude' not in names and 'Longitude' not in names
    assert {'Speed through water','Battery SOC','Motor speed','Whole-pack net discharge'}<=names
    return int(lines[-1].split('\t')[1])

try:
    if windows:
        spec=importlib.util.spec_from_file_location('ui',root/'tools/windows-ui.py');ui=importlib.util.module_from_spec(spec);spec.loader.exec_module(ui)
        report['display']=ui.ensure_desktop();exe=root/'build/xnav-install/opencpn.exe'
    else:
        n=120
        while Path(f'/tmp/.X{n}-lock').exists():n+=1
        env['DISPLAY']=f':{n}';xserver=subprocess.Popen(['Xvfb',env['DISPLAY'],'-screen','0','1280x800x24','-nolisten','tcp'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);time.sleep(1)
        exe=root/'build/xnav-install/bin/opencpn'
    with (evidence/'recording-launch.log').open('w') as output:app=subprocess.Popen([str(exe),'--configdir',str(profile),'--no_opengl','--xnav','--xnav-demo'],env=env,stdout=output,stderr=output)
    deadline=time.monotonic()+60
    while time.monotonic()<deadline:
        assert app.poll() is None,'Recording test application exited'
        log=profile/'opencpn.log'
        if log.exists() and 'OnInitTimer...Finalize Canvases' in log.read_text(errors='replace'):break
        time.sleep(.2)
    else:raise RuntimeError('Deferred startup failed')
    if windows:handle,_=ui.wait_window('OpenNav X / OpenCPN',app.pid)
    else:
        handle=xdo('search','--onlyvisible','--pid',app.pid,'--name','^OpenNav X / OpenCPN$').splitlines()[0]
        xdo('windowsize',handle,1280,800);xdo('windowmove',handle,0,0);xdo('windowfocus',handle)
    assert not data()['runtime']['recording']['active']
    page('Commissioning & recordings','c');capture('recording-01-commissioning')
    click('Record instruments only',260,363)
    data(lambda d:d['runtime']['recording']['active'])
    time.sleep(3.3)
    capture('recording-02-active')
    page('Commissioning & recordings','c')
    click('Stop & save recording',260,407)
    stopped=data(lambda d:not d['runtime']['recording']['active'] and d['runtime']['recording']['published']>=3)
    files=list((profile/'recordings').glob('session-*/*.onxr'));assert len(files)==1,files
    count=recording_frames(files[0]);assert count>=3
    shutil.copy2(files[0],evidence/'recording-private.onxr')
    report['checks'].append(f'Actual UI captured/saved {count} normalized frames without positions/route')
    page('Commissioning & recordings','c')
    click('Open recording for REPLAY...',850,407)
    file_dialog('Open normalized recording',files[0],'Open')
    replay=data(lambda d:d['data_mode']=='REPLAY')
    assert replay['runtime']['replay']['active'] and not replay['runtime']['replay']['allows_hardware_control']
    assert not replay.get('source_candidates'),'Live source candidates blended into replay'
    page('Energy','e');capture('recording-03-replay-energy')
    time.sleep(count+6)
    stale=data(lambda d:next(x for x in d['data'] if x['name']=='Battery SOC')['quality']=='STALE')
    capture('recording-04-replay-ended-stale')
    assert 'arrival_soc' not in stale['energy'],stale['energy']
    report['checks'].append('Replay is identified; separate source state; hardware control blocked; end ages into stale')
    page('Commissioning & recordings','c')
    click('Rewind REPLAY',615,565)
    data(lambda d:next(x for x in d['data'] if x['name']=='Battery SOC')['quality']!='STALE')
    click('Pause / resume REPLAY',230,565)
    paused=data(lambda d:d['runtime']['replay'].get('paused'))
    elapsed=paused['runtime']['replay']['elapsed_ms'];time.sleep(1.2)
    assert data()['runtime']['replay']['elapsed_ms']==elapsed,'Paused replay clock renewed'
    capture('recording-05-replay-paused')
    click('Stop REPLAY',1000,565)
    data(lambda d:d['data_mode']=='DEMO' and not d['runtime']['replay']['active'])
    report['checks'].append('Rewind/pause/stop return to labelled demo without profile restart')
    if windows:
        # Export through the real themed parameter sheet and native file picker.
        click('Export calibration observations...',0,0)
        file_dialog('Recording to export',files[0],'Open')
        ui.set_dialog_fields(app.pid,'Calibration observations',['STW','whole-pack','DEMO / pack-1'])
        ui.click_text(app.pid,'Export...')
        output=profile/'calibration.csv';file_dialog('Save reviewed calibration observations',output,'Save')
        deadline=time.monotonic()+5
        while not output.exists() and time.monotonic()<deadline:time.sleep(.1)
        report['calibration_export']={'requested':str(output),'files':[str(p.relative_to(profile)) for p in profile.rglob('*.csv')],
                                      'captions':[c for _,c in ui.children(handle)]}
        capture('recording-calibration-export-result')
        assert output.exists(),report['calibration_export']
        assert output.read_text().startswith('OpenNavXCalibration,1\nreference,STW\nbasis,whole-pack')
        assert 'DEMO' in output.read_text();shutil.copy2(output,evidence/'recording-calibration.csv')
        report['checks'].append('Native calibration export saved finite source-labelled demo pairs')
    page('Diagnostics','i') if not windows else page('System diagnostics','i')
    capture('recording-06-returned-diagnostics')
    if windows:
        monitor=ui.monitor_process(app.pid);ui.close(handle);ui.wait_clean_exit(monitor)
    else:subprocess.run([str(exe),'--configdir',str(profile),'--remote','--quit'],env=env,check=True,capture_output=True)
    assert app.wait(timeout=30)==0
    report['result']='passed; screenshot review required'
except BaseException as error:
    report['error']=str(error)
    if app and app.poll() is None and 'handle' in globals():
        try:capture('recording-failure')
        except Exception:pass
    raise
finally:
    if app and app.poll() is None:app.terminate();app.wait(timeout=20)
    if xserver:xserver.terminate();xserver.wait(timeout=10)
    for name in ['opencpn.log','opennav-diagnostics.json']:
        if (profile/name).exists():shutil.copy2(profile/name,evidence/('recording-'+name))
    (evidence/'recording-results.json').write_text(json.dumps(report,indent=2)+'\n')
    temporary.cleanup()
print(json.dumps(report,indent=2))

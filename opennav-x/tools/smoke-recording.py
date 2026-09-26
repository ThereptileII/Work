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
import zipfile
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
def click(label):
    if windows:
        ui.click_text(app.pid,label)
        return
    # Use actual owner-drawn control bounds. Alerts now share the status row,
    # and replay/recording actions can move after state or viewport changes.
    deadline=time.monotonic()+20
    while time.monotonic()<deadline:
        display=data()['runtime']['display']
        controls=display.get('interaction_controls',display.get('product_controls',[]))
        choices={tuple(c[k] for k in ('x','y','width','height')):c
                 for c in controls if c['label']==label}
        visible=[c for c in choices.values() if c['visible'] and c['enabled']]
        if visible:
            assert len(visible)==1,('Ambiguous control',label,visible)
            control=visible[0]
            xdo('mousemove',control['x']+control['width']//2,
                control['y']+control['height']//2);xdo('click',1);time.sleep(.4)
            return
        # Scroll through the same visible Up/Down controls used by a person;
        # do not inject clicks into clipped/offscreen children.
        enabled=[c for c in choices.values() if c['enabled']]
        if len(enabled)==1:
            top=max((c['y']+c['height'] for c in controls
                     if c['label'] in ('Day','Dusk','Night','Menu') and c['visible']),default=56)
            direction='Up' if enabled[0]['y']<top else 'Down'
            scroll=[c for c in controls if c['label']==direction and c['visible'] and c['enabled']]
            if len(scroll)==1:
                before=display['page_scroll_px'];c=scroll[0]
                xdo('mousemove',c['x']+c['width']//2,c['y']+c['height']//2);xdo('click',1)
                data(lambda d:d['runtime']['display']['page_scroll_px']!=before)
                continue
        time.sleep(.2)
    raise AssertionError(('Recording control unavailable',label,data()['ui_page'],choices))
def page(label):
    if label!='Energy':
        click('System')
        data(lambda d:d['ui_page']=='System')
    click({'Field diagnostic bundle':'Export diagnostic bundle',
           'System diagnostics':'Diagnostics'}.get(label,label))
    expected={'Energy':'Energy','Commissioning & recordings':'Commissioning & recordings','Diagnostics':'Diagnostics','System diagnostics':'Diagnostics','Field diagnostic bundle':'Field diagnostic bundle'}[label]
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
        time.sleep(.1)
        # Synchronous EM_SETSEL avoids a queued Ctrl+A arriving halfway through
        # the synchronous WM_CHAR stream and deleting the filename prefix.
        # WM_CHAR still exercises the Common Item Dialog's EN_CHANGE/model path.
        ui.SendMessageW(edit,0x00B1,0,-1)
        encoded=str(path).encode('utf-16-le')
        for i in range(0,len(encoded),2):
            ui.SendMessageW(edit,0x0102,int.from_bytes(encoded[i:i+2],'little'),0)
        report.setdefault('file_dialogs',[]).append({'title':title,'requested':str(path),'observed':ui.control_text(edit)})
        assert ui.control_text(edit)==str(path),(title,ui.control_text(edit),str(path))
        if accept=='Save':ui.capture(dialog,evidence/'recording-calibration-file-dialog.png',resize=False)
        ui.dismiss_native_dialog(dialog,accept)
    else:
        dialog=xdo('search','--onlyvisible','--pid',app.pid,'--name','^'+title+'$').splitlines()[-1]
        xdo('windowraise',dialog);xdo('windowfocus',dialog)
        if accept=='Save':xdo('key','alt+n','ctrl+a')
        else:xdo('key','ctrl+l')
        time.sleep(.2);xdo('type','--clearmodifiers','--',str(path))
        if accept=='Save':subprocess.run(['import','-window','root',str(evidence/'field-report-save-dialog-linux.png')],env=env,check=True)
        xdo('key','Return');time.sleep(.6)

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
    if windows:
        handle,_=ui.wait_window('OpenNav X / OpenCPN',app.pid)
        ui.size_window(handle)
    else:
        handle=xdo('search','--onlyvisible','--pid',app.pid,'--name','^OpenNav X / OpenCPN$').splitlines()[0]
        xdo('windowsize',handle,1280,800);xdo('windowmove',handle,0,0);xdo('windowfocus',handle)
    # Wait for the application layout, not merely the asynchronous native size
    # request. Otherwise a cached small-window button rectangle can be clicked
    # after the window has already expanded.
    data(lambda d:any(c['label']=='System' and c['visible'] and c['x']>1100 and c['y']>700
                      for c in d['runtime']['display'].get('interaction_controls',[])))
    assert not data()['runtime']['recording']['active']
    page('Commissioning & recordings');capture('recording-01-commissioning')
    click('Record instruments only')
    data(lambda d:d['runtime']['recording']['active'])
    time.sleep(3.3)
    capture('recording-02-active')
    page('Commissioning & recordings')
    click('Stop & save recording')
    stopped=data(lambda d:not d['runtime']['recording']['active'] and d['runtime']['recording']['published']>=3)
    files=list((profile/'recordings').glob('session-*/*.onxr'));assert len(files)==1,files
    count=recording_frames(files[0]);assert count>=3
    shutil.copy2(files[0],evidence/'recording-private.onxr')
    report['checks'].append(f'Actual UI captured/saved {count} normalized frames without positions/route')
    page('Commissioning & recordings')
    click('Open recording for REPLAY...')
    file_dialog('Open normalized recording',files[0],'Open')
    replay=data(lambda d:d['data_mode']=='REPLAY')
    assert replay['runtime']['replay']['active'] and not replay['runtime']['replay']['allows_hardware_control']
    assert not replay.get('source_candidates'),'Live source candidates blended into replay'
    page('Energy');capture('recording-03-replay-energy')
    time.sleep(count+6)
    stale=data(lambda d:next(x for x in d['data'] if x['name']=='Battery SOC')['quality']=='STALE')
    capture('recording-04-replay-ended-stale')
    assert 'arrival_soc' not in stale['energy'],stale['energy']
    report['checks'].append('Replay is identified; separate source state; hardware control blocked; end ages into stale')
    page('Commissioning & recordings')
    click('Rewind REPLAY')
    data(lambda d:next(x for x in d['data'] if x['name']=='Battery SOC')['quality']!='STALE')
    click('Pause / resume REPLAY')
    paused=data(lambda d:d['runtime']['replay'].get('paused'))
    elapsed=paused['runtime']['replay']['elapsed_ms'];time.sleep(1.2)
    assert data()['runtime']['replay']['elapsed_ms']==elapsed,'Paused replay clock renewed'
    capture('recording-05-replay-paused')
    click('Stop REPLAY')
    data(lambda d:d['data_mode']=='DEMO' and not d['runtime']['replay']['active'])
    report['checks'].append('Rewind/pause/stop return to labelled demo without profile restart')
    if windows:
        # Export through the real themed parameter sheet and native file picker.
        click('Export calibration observations...')
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
    page('Field diagnostic bundle');capture('field-report-01-export-page')
    click('Export Diagnostic Bundle')
    bundle=profile/'field-report.zip';file_dialog('Export Diagnostic Bundle',bundle,'Save')
    deadline=time.monotonic()+5
    while not bundle.exists() and time.monotonic()<deadline:time.sleep(.1)
    capture('field-report-02-export-result')
    report['bundle_files']=[str(p.relative_to(profile)) for p in profile.rglob('*.zip')]
    assert bundle.exists(),('Diagnostic ZIP not published',report['bundle_files'])
    with zipfile.ZipFile(bundle) as z:
        assert z.testzip() is None
        assert set(z.namelist())=={'READ_ME.txt','build-and-recovery.txt','source-health.txt','energy-assumptions.txt','adapters.txt','smartnav-events.txt','recent-transitions.log'}
        report_bytes=b''.join(z.read(n) for n in z.namelist())
        assert str(profile).encode() not in report_bytes
        assert b'Latitude / deg / withheld' in report_bytes
        assert b'DEMO' in report_bytes and b'Build:' in report_bytes
    shutil.copy2(bundle,evidence/('field-report.zip' if windows else 'field-report-linux.zip'))
    report['checks'].append('Actual diagnostic ZIP export: integrity, whitelist, profile/position privacy and DEMO provenance')
    if windows:
        click('Export with selected recording...')
        file_dialog('Explicitly select recording to share',files[0],'Open')
        ui.click_text(app.pid,'Include selected recording')
        selected=profile/'field-report-selected.zip';file_dialog('Export Diagnostic Bundle',selected,'Save')
        deadline=time.monotonic()+5
        while not selected.exists() and time.monotonic()<deadline:time.sleep(.1)
        assert selected.exists(),'Explicit recording bundle absent'
        with zipfile.ZipFile(selected) as z:
            assert z.testzip() is None
            assert z.read('selected-recording.onxr')==files[0].read_bytes()
            assert b'Navigation included: NO' in z.read('recording-consent.txt')
        shutil.copy2(selected,evidence/'field-report-selected.zip')
        report['checks'].append('Native explicit recording selection, consent, ZIP content equality')
    page('Diagnostics') if not windows else page('System diagnostics')
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

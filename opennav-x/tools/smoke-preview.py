#!/usr/bin/env python3
"""Exercise preview pages/scenarios; on Windows run the extracted ZIP itself."""
import argparse
import ctypes
import hashlib
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
parser=argparse.ArgumentParser();parser.add_argument('--install',type=Path);parser.add_argument('--runtime',type=Path)
args=parser.parse_args();windows=sys.platform=='win32'
if windows != bool(args.install):raise SystemExit('Native fixture tests require --install; Linux uses its development install')
evidence=root/'evidence/local';evidence.mkdir(parents=True,exist_ok=True)
temporary=tempfile.TemporaryDirectory(prefix='OpenNav preview ',dir=None if windows else '/tmp')
temp=Path(temporary.name)
env=dict(os.environ);ui=None;xserver=None;app=None;handle=None;pid=None
normal_locations=[];normal_before={}
report={'authority':'native Windows fixture-enabled disposable test tree' if windows else 'Linux development',
        'checks':[],'screenshots':[],'higher_dpi':'Not exercised by this hosted desktop; manual validation remains open'}
phase='initial setup';launches=[];observed_processes={};failure_processes=[]

def event(kind,**details):
    # Stage-bound observations only; never inspect environment or sensor input.
    rows=report.setdefault('process_events',[])
    if len(rows)<256:
        row=dict(event=kind,phase=phase,monotonic_ms=int(time.monotonic()*1000));row.update(details);rows.append(row)

def proc_stat(target):
    fields=(Path('/proc')/str(target)/'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=target,state=fields[0],ppid=int(fields[1]),start_ticks=int(fields[19]))

def observe_processes():
    """Snapshot only this subreaper's OpenCPN descendants, without reaping."""
    if windows:return []
    rows=[];pending=[os.getpid()];seen=set()
    while pending and len(seen)<256:
        parent=pending.pop()
        if parent in seen:continue
        seen.add(parent)
        try:children=(Path('/proc')/str(parent)/'task'/str(parent)/'children').read_text().split()
        except OSError:continue
        for child in children[:256-len(seen)]:
            target=int(child);pending.append(target)
            try:
                before=proc_stat(target);base=Path('/proc')/child
                if (base/'comm').read_text().strip()!='opencpn':continue
                # Birth identity is checked after every multi-file observation.
                key=(target,before['start_ticks']);previous=observed_processes.get(key)
                if before['state']=='Z':
                    row=dict(before,argv=previous.get('argv',[]) if previous else [],
                             argv_state='retained before exit' if previous else 'unavailable: exited before observation')
                    if all(hasattr(os,name) for name in ('waitid','WNOWAIT','WEXITED','WNOHANG','P_PID')):
                        try:
                            status=os.waitid(os.P_PID,target,os.WEXITED|os.WNOHANG|os.WNOWAIT)
                            if status:row['unreaped_exit']=dict(pid=status.si_pid,code=status.si_code,status=status.si_status)
                        except (OSError,ChildProcessError) as error:row['exit_observation_error']=str(error)
                    else:row['exit_observation_error']='waitid/WNOWAIT unavailable'
                else:
                    if (base/'exe').resolve()!=exe.resolve():continue
                    with (base/'cmdline').open('rb') as stream:raw=stream.read(8192)
                    row=dict(before,argv=[v.decode(errors='replace')[:2048] for v in raw.split(b'\0') if v][:32])
                after=proc_stat(target)
                if (after['pid'],after['start_ticks'])!=key:continue
                observed_processes[key]=row;rows.append(row)
            except (OSError,ValueError,IndexError):continue
    return rows

def process_stage(kind,**details):
    try:event(kind,processes=observe_processes(),**details)
    except Exception as error:event('observation-error',stage=kind,error=repr(error))

def wait_launch(process,timeout):
    try:code=process.wait(timeout=timeout)
    except Exception as error:
        event('popen-wait-error',pid=process.pid,error=repr(error));raise
    event('popen-exit',pid=process.pid,exit_code=code)
    return code

def wait_pid(target,options):
    child,status=os.waitpid(target,options)
    if child:
        birth=max((start for (number,start) in observed_processes if number==child),default=None)
        event('waitpid-exit',pid=child,start_ticks=birth,raw_wait_status=status,exit_code=os.waitstatus_to_exitcode(status))
    return child,status

def wait_native(process_handle,target):
    # The same helper deadline and zero-exit assertion, retaining its result.
    try:code=ui.wait_exit_code(process_handle)
    except Exception as error:
        event('native-wait-error',pid=target,error=repr(error));raise
    event('native-exit',pid=target,exit_code=code)
    assert code==0,f'Native process exited with code {code}'

def module(name):
    spec=importlib.util.spec_from_file_location(name,root/'tools'/f'{name}.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod

def normal_snapshot():
    result={}
    for location in normal_locations:
        candidates=location.rglob('*') if location.is_dir() else [location]
        for file in candidates:
            if file.is_file(): result[str(file)]=hashlib.sha256(file.read_bytes()).hexdigest()
    return result

if windows:
    # Read-only audit of existing Windows OpenCPN config/install locations.
    # APPDATA alone is insufficient: upstream also uses common application data.
    for key in ['APPDATA','LOCALAPPDATA','PROGRAMDATA']:
        if os.environ.get(key):
            base=Path(os.environ[key])
            normal_locations += [base/'opencpn',base/'opencpn.ini',base/'opencpn.log']
    for key in ['ProgramFiles','ProgramFiles(x86)']:
        if os.environ.get(key): normal_locations.append(Path(os.environ[key])/'OpenCPN')
    normal_before=normal_snapshot()
    # The application capture remains 1280x800. Leave room for its native
    # caption/frame and the runner taskbar so real pointer input cannot hit
    # Windows' clock over the application's footer.
    ui=module('windows-ui');report['display']=ui.ensure_desktop(1440,900)
    package=temp/'OpenNavX-CI-Fixtures';profile=package/'profile';logs=package/'logs';exe=package/'app/opencpn.exe'
    shutil.copytree(args.install,package/'app')
    if not args.runtime or not args.runtime.is_dir():raise SystemExit('Native app-local runtime directory required')
    for dll in args.runtime.glob('*.dll'):shutil.copy2(dll,package/'app'/dll.name)
    (package/'app/OPENNAV_PORTABLE_PREVIEW').write_text('Internal fixture regression only; never distribute\n')
    subprocess.run([sys.executable,str(root/'tools/prepare-test-profile.py'),'--build',str(root/'build/xnav-windows'),'--profile',str(profile)],check=True)
    shutil.copytree(package/'app/plugins',profile/'plugins',dirs_exist_ok=True)
    logs.mkdir()
    with (profile/'opencpn.conf').open('a') as f:f.write('\n[Settings/GlobalState]\nVPLatLon=59.0800,18.5000\nVPScale=0.003\n')
    for name,mode in {'Run-XNav':'--xnav','Run-XNav-Demo':'--xnav --xnav-demo','Run-Legacy':'--legacy','Run-Safe':'--safe-mode'}.items():
        (package/(name+'.cmd')).write_text('@echo off\n"%~dp0app\\opencpn.exe" --portable --configdir "%~dp0profile" --no_opengl '+mode+' %*\nexit /b %errorlevel%\n')
    report['checks'].append('Disposable fixture tree uses exact tested native install; never included in product package')
    # A fake normal roaming profile is an isolation canary, never the runner's
    # actual user profile. DLL lookup gets no compiler/dependency PATH entries.
    fake=temp/'normal user data';normal=fake/'opencpn';normal.mkdir(parents=True)
    (normal/'opencpn.ini').write_text('NORMAL PROFILE MUST NOT CHANGE\n')
    env['APPDATA']=str(fake);env['PATH']=os.environ['SystemRoot']+'\\System32;'+os.environ['SystemRoot']
    report['runtime_path']=env['PATH']
else:
    if ctypes.CDLL(None).prctl(36,1,0,0,0)!=0:raise RuntimeError('Cannot track restarted processes')
    # Exercise the same portable resource save/reload path as Windows. Ordinary
    # non-portable mode/input coverage remains in the existing smoke scripts.
    package=temp/'OpenNavX-Beta1-Portable';app_dir=package/'app';app_dir.mkdir(parents=True)
    for resource in (root/'build/xnav-install/share/opencpn').iterdir():
        (app_dir/resource.name).symlink_to(resource,target_is_directory=resource.is_dir())
    exe=app_dir/'opencpn';shutil.copy2(root/'build/xnav-install/bin/opencpn',exe)
    (app_dir/'OPENNAV_PORTABLE_PREVIEW').write_text('Isolated Linux portable regression fixture\n')
    profile=package/'profile';logs=package/'logs'
    subprocess.run([sys.executable,str(root/'tools/prepare-test-profile.py'),'--build',str(root/'build/xnav-linux'),'--profile',str(profile)],check=True)
    with (profile/'opencpn.conf').open('a') as f:f.write('\n[Settings/GlobalState]\nVPLatLon=59.0800,18.5000\nVPScale=0.003\n')
    number=111
    while Path(f'/tmp/.X{number}-lock').exists():number+=1
    env['DISPLAY']=f':{number}'
    xserver=subprocess.Popen(['Xvfb',env['DISPLAY'],'-screen','0','1280x800x24','-nolisten','tcp'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    time.sleep(1)
# Only this freshly extracted disposable test copy receives the fixture marker.
# The downloadable profile remains a clean user preview, without test hooks.
report['executable_sha256']=hashlib.sha256(exe.read_bytes()).hexdigest()
(profile/'OPENNAV_TEST_PROFILE').write_text('CI disposable extracted preview only\n')
if env.get('OPENNAV_TEST_UI_TRACE'):
    env['OPENNAV_TEST_UI_TRACE_FILE']=str(profile/'opennav-ui-trace.log')
fixtures=module('profile-fixtures');fixtures.seed(profile);expected_profile=fixtures.snapshot(profile)
chartcheck=module('chart-render-check')

def xdo(*arguments):
    return subprocess.check_output(['xdotool',*map(str,arguments)],env=env,text=True,timeout=10).strip()
def window(title,expected_pid=None):
    process_stage('await-window',title=title,expected_pid=expected_pid)
    if windows:
        try:h,found_pid=ui.wait_window(title,expected_pid)
        except Exception:
            process_stage('window-timeout',title=title,expected_pid=expected_pid);raise
        report['last_observed_window']=dict(handle=int(h),pid=found_pid,title=title,phase=phase)
        event('window-found',**report['last_observed_window'])
        return h,found_pid
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        r=subprocess.run(['xdotool','search','--onlyvisible','--name','^'+title+'$'],env=env,capture_output=True,text=True)
        if r.returncode==0 and r.stdout.strip():
            h=r.stdout.splitlines()[0];xdo('windowsize',h,1280,800);xdo('windowmove',h,0,0);xdo('windowfocus',h)
            found_pid=int(xdo('getwindowpid',h))
            report['last_observed_window']=dict(handle=h,pid=found_pid,title=title,phase=phase)
            process_stage('window-found',handle=h,pid=found_pid,title=title)
            return h,found_pid
        time.sleep(.1)
    process_stage('window-timeout',title=title,expected_pid=expected_pid)
    raise RuntimeError('Window not found: '+title)
def ready(count):
    deadline=time.monotonic()+60
    while time.monotonic()<deadline:
        log=profile/'opencpn.log'
        if log.exists() and log.read_text(errors='replace').count('OnInitTimer...Finalize Canvases')>=count:
            # The pinned frame schedules a one-second recapture/Raise after
            # its deferred SendSizeEvent. A bare Xvfb has no window manager to
            # keep transient popups above that parent raise.
            time.sleep(1.5);return
        time.sleep(.2)
    raise RuntimeError('Initialization incomplete')
def data(predicate=lambda d:True,timeout=12):
    d={}
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:
            d=read_json_snapshot(logs/'opennav-diagnostics.json')
            if predicate(d):return d
        except (FileNotFoundError,json.JSONDecodeError,PermissionError):pass
        time.sleep(.2)
    raise AssertionError('Diagnostic predicate did not become true: '+json.dumps(d.get('runtime',{}).get('display',{})))
def light(expected):
    # Distinct single clicks. GTK coalesces rapid physical clicks into a
    # double-click event; that is not two independent button activations.
    time.sleep(.65)
    shell_click(data()['runtime']['display']['light'],in_status=True)
    data(lambda d:d['runtime']['display']['light']==expected)
interaction=module('product-interaction')
def pointer_click(target):
    x=target['x']+target['width']//2;y=target['y']+target['height']//2
    assert target['visible'] and target['enabled'],target
    if windows:
        point=ui.W.POINT(x,y);hit=ui.WindowFromPoint(point);owner=ui.W.DWORD()
        ui.GetWindowThreadProcessId(hit,ui.C.byref(owner))
        assert owner.value==pid and ui.IsWindowEnabled(hit) and ui.text(hit)==target['label'],('Pointer target mismatch',target,ui.text(hit))
        foreground=ui.declare(ui.user,'GetForegroundWindow',ui.W.HWND)
        front=foreground();front_pid=ui.W.DWORD()
        ui.GetWindowThreadProcessId(front,ui.C.byref(front_pid))
        # Activate the target's own surface, never its owner frame. Reopening
        # an owned drawer leaves the frame active; sending down/up across that
        # activation transition can lose the first press. Match the production
        # capture harness and recheck the exact HWND after activation.
        assert front_pid.value==pid, ('Foreign foreground before input',ui.text(front),front_pid.value)
        ancestor=ui.declare(ui.user,'GetAncestor',ui.W.HWND,ui.W.HWND,ui.W.UINT)
        surface=ancestor(hit,2)
        ui.SetForegroundWindow(surface)
        deadline=time.monotonic()+3
        while foreground()!=surface and time.monotonic()<deadline:time.sleep(.05)
        assert foreground()==surface, ('Input surface did not activate',target)
        assert ui.SetCursorPos(x,y)
        actual=ui.WindowFromPoint(point)
        assert actual==hit and ui.IsWindowEnabled(hit), ('Pointer target changed before input',target,ui.text(actual),ui.text(front))
        ui.MouseEvent(2,0,0,0,0);time.sleep(.05);ui.MouseEvent(4,0,0,0,0)
    else:
        xdo('mousemove',x,y)
        xdo('mousedown',1,'sleep','0.05','mouseup',1)
    time.sleep(.4)
def shell_click(label,outside_drawer=False,in_status=False,in_drawer=False,settled=lambda d:True):
    record=data(lambda d:any(r['label']==label and r['visible'] and r['enabled'] for r in d['runtime']['display']['interaction_controls']))
    display=record['runtime']['display'];bounds=display.get('drawer',{})
    targets=[r for r in display['interaction_controls'] if r['label']==label and r['visible'] and r['enabled']]
    if in_status:
        alerts=[c for c in display['interaction_controls'] if (c['label']=='Alerts' or c['label'].startswith('Alerts ')) and c['visible']]
        assert len(alerts)==1,('Unique status alert required',alerts)
        targets=[r for r in targets if r['y']==alerts[0]['y'] and r['height']==alerts[0]['height']]
    if outside_drawer and bounds:
        targets=[r for r in targets if not (bounds['x']<=r['x']<bounds['x']+bounds['width'] and bounds['y']<=r['y']<bounds['y']+bounds['height'])]
    if in_drawer:
        assert bounds,'Expected an open prototype drawer'
        targets=[r for r in targets if bounds['x']<=r['x'] and r['x']+r['width']<=bounds['x']+bounds['width'] and bounds['y']<=r['y'] and r['y']+r['height']<=bounds['y']+bounds['height']]
    assert len(targets)==1,('Unique visible control required',label,targets)
    ticks=int(record['runtime']['ui_update']['ticks'])
    report.setdefault('pointer_actions',[]).append(dict(label=label,page=record.get('ui_page'),target=targets[0]))
    pointer_click(targets[0])
    data(lambda d:int(d['runtime']['ui_update']['ticks'])>=ticks+3 and settled(d))
def product_scroll(direction):
    if windows:ui.click_text(pid,'Down' if direction>0 else 'Up')
    else:xdo('mousemove',700,430,'click',5 if direction>0 else 4)
    time.sleep(.4)

VESSEL_FIELD_LABELS=['Field: Vessel name','Field: Draft · metres','Field: Safety depth · metres',
                     'Field: Usable battery capacity · kWh','Field: Minimum reserve · %']
def vessel_form_ready(record):
    if record.get('ui_page')!='Settings' or not record['runtime']['display'].get('drawer'):
        return False
    controls=record['runtime']['display']['interaction_controls']
    return (all(sum(c['label']==label for c in controls)==1 for label in VESSEL_FIELD_LABELS) and
            sum(c['label']=='Save vessel profile' for c in controls)==1)
def vessel_form_contract(record):
    assert vessel_form_ready(record),'Vessel Preferences form did not settle with five fields and Save identity'
    controls=record['runtime']['display']['interaction_controls']
    save=next(c for c in controls if c['label']=='Save vessel profile')
    assert save['enabled'],'Vessel profile Save identity must remain available'
    return {'fields':VESSEL_FIELD_LABELS,'save':{'label':save['label'],'enabled':save['enabled']}}

def preferences_target(record,label):
    display=record['runtime']['display'];drawer=display.get('drawer',{})
    assert record.get('ui_page')=='Settings' and drawer, 'Preferences drawer closed during selection'
    # The diagnostic walk also includes hidden controls on previous pages.
    # A usable target must be visible, enabled and wholly inside this drawer.
    candidates=[r for r in display['interaction_controls'] if r['label']==label and r['enabled'] and
                drawer['x']<=r['x'] and r['x']+r['width']<=drawer['x']+drawer['width']]
    visible=[r for r in candidates if r['visible'] and drawer['y']<=r['y'] and
             r['y']+r['height']<=drawer['y']+drawer['height']]
    assert len(visible)<=1,(label,'unique visible Preferences action',visible)
    if visible:return visible[0],True
    # Below-fold rows remain in the copied geometry. Use one horizontally
    # contained candidate only for deciding a scroll, never for clicking.
    assert len(candidates)==1,(label,'unique Preferences scroll target',candidates)
    return candidates[0],False

def preferences_entry(section,entry):
    command('Settings','g')
    destinations={'Advanced vessel model':'Vessel safety settings',
                  'Advanced battery model':'Energy configuration'}
    if section=='Vessel':
        section_ready=vessel_form_ready
    elif entry in destinations:
        raise AssertionError('Advanced model destinations must be reached from the Vessel section')
    else:
        def section_ready(d):
            display=d['runtime']['display'];bounds=display.get('drawer',{})
            return d.get('ui_page')=='Settings' and bool(bounds) and any(
                r['label']==entry and r['enabled'] and
                bounds['x']<=r['x'] and r['x']+r['width']<=bounds['x']+bounds['width'] and
                (section=='System' or (r['visible'] and
                 bounds['y']<=r['y'] and r['y']+r['height']<=bounds['y']+bounds['height']))
                for r in display['interaction_controls'])
    shell_click(section,in_drawer=True,settled=section_ready)
    if section=='Vessel':
        form=vessel_form_contract(data())
        report['vessel_preferences_form']=form
        if not report.get('vessel_form_captured'):
            capture('beta-vessel-preferences-form')
            report['vessel_form_captured']=True
    if entry not in destinations and section!='System':
        shell_click(entry,in_drawer=True)
        return
    # Section readiness means the controls exist; the destination can be
    # below the viewport. Scroll before requiring a visible pointer target.
    if windows:
        # windows-ui's native pointer path scrolls clipped drawer actions
        # only after checking their actual HWND and containing viewport.
        ticks=int(data()['runtime']['ui_update']['ticks'])
        ui.pointer_text(pid,entry)
        data(lambda d:int(d['runtime']['ui_update']['ticks'])>=ticks+3)
    else:
        last_y=None
        for _ in range(32):
            current=data();display=current['runtime']['display'];drawer=display.get('drawer',{})
            target,usable=preferences_target(current,entry)
            if usable:
                shell_click(entry,in_drawer=True)
                break
            assert current.get('ui_page')=='Settings' and drawer,(entry,'Preferences drawer closed during scroll')
            assert target['y']!=last_y,(entry,'Preferences scroll did not move the action')
            last_y=target['y'];ticks=int(current['runtime']['ui_update']['ticks'])
            x=drawer['x']+drawer['width']//2;y=drawer['y']+drawer['height']//2
            xdo('mousemove',x,y,'click',5)
            data(lambda d:int(d['runtime']['ui_update']['ticks'])>=ticks+3)
        else:raise AssertionError(entry+': bounded drawer scroll could not reach action')
    if entry in destinations:
        data(lambda d:d.get('ui_page')==destinations[entry])
def display_preferences():
    # Chart layers and palette choice have separate real destinations.
    preferences_entry('Navigation','Chart presentation')
    data(lambda d:d.get('ui_page')=='Chart presentation')
    ui.pointer_text(pid,'Chart palette preferences',scroll_surface='Chart presentation')
    data(lambda d:d.get('ui_page')=='Display')

def product_click(label,enabled=True):
    target=interaction.control(data,label,product_scroll,enabled=enabled)
    pointer_click(target)
    return target
def item(d,name):return next(i for i in d['data'] if i['name']==name)
def command(label,shortcut):
    # Primary workflows exercise visible prototype controls on both platforms.
    direct={'n':'Chart','r':'Passage','e':'Energy','m':'Settings','g':'Settings',
            'v':'Instruments','a':'Traffic','h':'Anchor','z':'Radar'}
    if shortcut in direct:
        pages={'n':'Navigation','r':'Route','e':'Energy','m':'Settings','g':'Settings',
               'v':'Vessel instruments','a':'AIS targets','h':'Anchor watch','z':'Radar status'}
        shell_click(direct[shortcut],outside_drawer=True,settled=lambda d:d.get('ui_page')==pages[shortcut])
    elif shortcut=='s':
        preferences_entry('System','Interface & recovery')
        data(lambda d:d['ui_page']=='System')
    elif shortcut=='i':preferences_entry('System','Diagnostics')
    else:accelerator(shortcut)
def accelerator(key):
    if not windows:xdo('windowfocus',handle);xdo('key','ctrl+shift+'+key)
    else:ui.accelerator(handle,key)
    time.sleep(.5)
def scenario(label,index):
    # Fixture-only accelerators never exist in the installed product.
    accelerator('F'+str(index+1))
def capture(name):
    path=evidence/(name+('.png' if windows else '-linux.png'))
    if windows:rgb=ui.capture(handle,path,resize=False,screen_pixels=True)
    else:
        time.sleep(.4);subprocess.run(['import','-window','root',str(path)],env=env,check=True)
        rgb=subprocess.check_output(['convert',str(path),'-depth','8','rgb:-'],env=env)
    report['screenshots'].append(path.name)
    return rgb
def chart_capture(name,phase):
    title=ui.text(handle) if windows else xdo('getwindowname',handle)
    style='XNav' if title=='SKAGER / OpenCPN' else 'Standard'
    report.setdefault('chart_rendering',[]).append((chartcheck.presentation if os.environ.get('SKAGER_DESIGN_VALIDATION') == 'true' else chartcheck.functional)(capture(name),style,'Day',phase))
def page_capture(name, page):
    capture(name)
    if windows:
        report.setdefault('page_visibility', []).append(ui.assert_preview_page(handle, page))
def close_current():
    process_stage('close-request',pid=pid)
    if windows:
        h=ui.monitor_process(pid);ui.close(handle);wait_native(h,pid)
    else:
        # Portable previews intentionally refuse remote commands. Send the
        # ordinary window-manager close event to this test window instead.
        xlib=ctypes.CDLL('libX11.so.6')
        xlib.XOpenDisplay.argtypes=[ctypes.c_char_p];xlib.XOpenDisplay.restype=ctypes.c_void_p
        display=xlib.XOpenDisplay(env['DISPLAY'].encode());assert display
        xlib.XInternAtom.argtypes=[ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int];xlib.XInternAtom.restype=ctypes.c_ulong
        class Event(ctypes.Structure):
            _fields_=[('type',ctypes.c_int),('serial',ctypes.c_ulong),('send',ctypes.c_int),
                      ('display',ctypes.c_void_p),('window',ctypes.c_ulong),('message',ctypes.c_ulong),
                      ('format',ctypes.c_int),('data',ctypes.c_long*5)]
        event=Event();event.type=33;event.display=display;event.window=int(handle);event.format=32
        event.message=xlib.XInternAtom(display,b'WM_PROTOCOLS',0)
        event.data[0]=xlib.XInternAtom(display,b'WM_DELETE_WINDOW',0)
        xlib.XSendEvent.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_long,ctypes.c_void_p]
        xlib.XFlush.argtypes=[ctypes.c_void_p];xlib.XCloseDisplay.argtypes=[ctypes.c_void_p]
        xlib.XSendEvent(display,int(handle),0,0,ctypes.byref(event));xlib.XFlush(display);xlib.XCloseDisplay(display)
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            child,status=wait_pid(pid,os.WNOHANG)
            if child:
                assert os.waitstatus_to_exitcode(status)==0;return
            time.sleep(.1)
        raise RuntimeError('Preview did not exit cleanly')
def preserved():
    assert fixtures.snapshot(profile)==expected_profile,'Preview changed seeded navigation/configuration fixtures'
    if windows:
        assert (normal/'opencpn.ini').read_text()=='NORMAL PROFILE MUST NOT CHANGE\n'
        assert normal_snapshot()==normal_before,'Existing normal OpenCPN files changed'
def switch_to_xnav(count, transition, name):
    global handle, pid, phase
    phase=transition
    process_stage('switch-to-xnav-request',pid=pid)
    if windows:
        old=ui.monitor_process(pid);ui.click_menu(handle,'Switch to SKAGER');wait_native(old,pid)
    else:
        old=pid;xdo('mousemove',600,400,'click',3);time.sleep(.4);xdo('key','End','Return')
        _,status=wait_pid(old,0);assert os.waitstatus_to_exitcode(status)==0
    handle,pid=window('SKAGER / OpenCPN');ready(count);preserved()
    chart_capture(name,transition)
    if windows:
        saved=data(lambda d:d['settings']['capacity_kwh']=='24' and d['settings']['reserve_percent']=='20')
        assert saved['settings']['battery_device']=='', 'No battery identity may be fabricated'
        report['checks'].append('Explicit live energy configuration survived mode restart')
def launch(mode,demo=False,launcher=None,direct=False):
    global phase
    phase=f'launch {len(launches)+1}: '+(launcher or ('direct executable' if direct else mode+(' demo' if demo else '')))
    number=len(launches)+1
    stdout_name=f'preview-launch-{number:02d}.stdout.log';stderr_name=f'preview-launch-{number:02d}.stderr.log'
    arguments=([os.environ['COMSPEC'],'/d','/c',str(package/launcher)] if windows and launcher else
               [str(exe)]+([] if direct else ['--configdir',str(profile),'--no_opengl','--'+mode]+(['--xnav-demo'] if demo else [])))
    # Children inherit these descriptors across controlled restarts. The
    # launch PID identifies the original parent (cmd.exe on Windows), not an
    # unseen replacement process that may fail before creating its window.
    with (evidence/stdout_name).open('w') as stdout,(evidence/stderr_name).open('w') as stderr:
        process=subprocess.Popen(arguments,env=env,cwd=temp,stdout=stdout,stderr=stderr)
    record=dict(phase=phase,pid=process.pid,arguments=arguments,stdout=stdout_name,stderr=stderr_name)
    launches.append((process,record));report.setdefault('launches',[]).append(record)
    process_stage('direct-launch',pid=process.pid)
    return process

def failure_inventory(error):
    # This precedes teardown and Popen.poll(): WNOWAIT must observe adopted
    # zombie children while the original wait contracts still own reaping.
    inventory=dict(error=repr(error),phase=phase,last_observed_window=report.get('last_observed_window'),
                   processes=observe_processes(),windows=[])
    report['failure_inventory']='preview-failure-inventory.json'
    try:
        if windows:
            query=ui.declare(ui.kernel,'QueryFullProcessImageNameW',ui.W.BOOL,ui.W.HANDLE,ui.W.DWORD,ui.W.LPWSTR,ui.C.POINTER(ui.W.DWORD))
            for h,process,title in ui.windows()[:64]:
                ph=ui.OpenProcess(0x1000,False,process)
                if not ph:continue
                try:
                    size=ui.W.DWORD(32768);name=ui.C.create_unicode_buffer(size.value)
                    if query(ph,0,name,ui.C.byref(size)) and str(Path(name.value).resolve()).casefold()==str(exe.resolve()).casefold():
                        rect=ui.W.RECT();ui.GetWindowRect(h,ui.C.byref(rect))
                        inventory['windows'].append(dict(handle=int(h),pid=process,title=title,
                            bounds=[rect.left,rect.top,rect.right-rect.left,rect.bottom-rect.top],
                            children=[caption for _,caption in ui.children(h)[:128]]))
                        if process not in failure_processes:failure_processes.append(process)
                finally:ui.CloseHandle(ph)
            desktop=ui.declare(ui.user,'GetDesktopWindow',ui.W.HWND)()
            path=evidence/'preview-failure.png'
            ui.capture(desktop,path,resize=False,screen_pixels=True)
            inventory['screenshot']=path.name
        else:
            found=subprocess.run(['xdotool','search','--all','--onlyvisible','--name','.*'],env=env,capture_output=True,text=True,timeout=5)
            inventory['window_search']=dict(return_code=found.returncode,stderr=found.stderr[:2048])
            for window_id in found.stdout.splitlines()[:32]:
                row=dict(handle=window_id)
                for key,command in [('title','getwindowname'),('pid','getwindowpid'),('geometry','getwindowgeometry')]:
                    result=subprocess.run(['xdotool',command,window_id],env=env,capture_output=True,text=True,timeout=3)
                    row[key]=dict(return_code=result.returncode,stdout=result.stdout[:4096],stderr=result.stderr[:2048])
                inventory['windows'].append(row)
            path=evidence/'preview-failure-linux.png'
            result=subprocess.run(['import','-window','root',str(path)],env=env,capture_output=True,text=True,timeout=10)
            inventory['screenshot']=dict(path=path.name,return_code=result.returncode,stderr=result.stderr[:2048])
    except Exception as observed_error:inventory['observation_error']=repr(observed_error)
    finally:
        (evidence/report['failure_inventory']).write_text(json.dumps(inventory,indent=2)+'\n')

try:
    app=launch('xnav',True,'Run-XNav-Demo.cmd' if windows else None)
    handle,pid=window('SKAGER / OpenCPN');ready(1)
    if windows:
        rect=ui.W.RECT();ui.GetWindowRect(handle,ui.C.byref(rect))
        report['initial_outer_pixels']=[rect.right-rect.left,rect.bottom-rect.top]
        assert rect.right-rect.left>=1280 and rect.bottom-rect.top>=740,report['initial_outer_pixels']
        native_log=(profile/'opencpn.log').read_text(errors='replace')
        assert 'TC_FILE_NOT_FOUND' not in native_log,'Portable resource paths do not resolve'
        assert 'Using portable plugin dir:' in native_log
        assert any('PluginLoader: Loading PlugIn:' in line and line.endswith('\\profile\\plugins\\dashboard_pi.dll')
                   for line in native_log.splitlines()), 'Bundled Dashboard was not discovered by the portable loader'
        # Exercise bottom-pane reflow after a narrow viewport, not just a wide
        # first launch. Keep the destination summary clear of adjacent controls.
        assert ui.SetWindowPos(handle,None,0,0,960,800,4)
        time.sleep(.6)
        ui.size_window(handle)
        ui.assert_route_summary_layout(handle)
        report['checks'].append('Bottom route summary lays out after narrow-to-wide resize')
    if not windows:
        command('System','s')
        data(lambda d:d['ui_page']=='System')
        capture('beta-system-page')
        xdo('key','Escape');time.sleep(.4)
        data(lambda d:d['ui_page']=='Settings')
        command('Navigation','n')
        data(lambda d:d['ui_page']=='Navigation')
    first=data(lambda d:d['data_mode']=='DEMO' and 'arrival_soc' in d['energy'])
    assert first['route']['source'].startswith('DEMO')
    chart_colors=chartcheck.reference(capture('preview-01-navigation-day'))
    chart_capture('preview-11-startup-xnav','Direct XNav startup')
    light('Dusk');light('Night')
    report['chart_rendering'].append((chartcheck.presentation if os.environ.get('SKAGER_DESIGN_VALIDATION') == 'true' else chartcheck.functional)(capture('preview-02-navigation-night'),'XNav','Night','Night world-chart land/water palette'))
    light('Day')
    command('Route','r');page_capture('preview-03-route','Route')
    command('Energy','e');page_capture('preview-04-energy','Energy')
    command('Diagnostics','i')
    page_capture('preview-05-diagnostics','Diagnostics')
    data(lambda d:d['runtime']['display']['can_scroll_down'])
    if windows:ui.click_text(pid,'Down')
    else:shell_click('Down')
    data(lambda d:d['runtime']['display']['page_scroll_px']>0)
    if windows:ui.click_text(pid,'Up')
    else:shell_click('Up')
    data(lambda d:d['runtime']['display']['page_scroll_px']==0)
    report['checks'].append('Persistent page controls scroll diagnostics and return to top without native scrollbars')
    # Alpha product pages use real touch-button actions on Windows and public
    # frame shortcuts on Linux. Existing preview regression assertions remain.
    for title,key,name in [('Vessel instruments','v','instruments'),
                           ('AIS targets','a','ais'),('SmartNav advisories','j','smartnav'),
                           ('Manual autopilot','y','autopilot'),('Anchor watch','h','anchor'),
                           ('Settings','g','settings'),('Routes','b','routes'),
                           ('Waypoints','w','waypoints')]:
        command(title,key)
        expected_page='SmartNav' if name=='smartnav' else title
        data(lambda d:d.get('ui_page')==expected_page)
        if name == 'instruments':
            data(lambda d:d['runtime']['display']['minimum_value_height_dip']>=120)
            report.setdefault('grouped_regions',[]).append(interaction.grouped_regions(data))
        if name=='autopilot':
            drawer=data()['runtime']['display']['drawer']
            if windows:
                client=ui.W.RECT();assert ui.GetClientRect(handle,ui.C.byref(client))
                origin=ui.W.POINT(0,0)
                to_screen=ui.declare(ui.user,'ClientToScreen',ui.W.BOOL,ui.W.HWND,ui.C.POINTER(ui.W.POINT))
                assert to_screen(handle,ui.C.byref(origin))
                scale=ui.GetDpiForWindow(handle)/96
                expected=ui.prototype_drawer_bounds(client.right,client.bottom,scale,(origin.x,origin.y))
            else:
                scale=1;expected=dict(x=682,y=80,width=398,height=674)
            assert all(abs(drawer[k]-v)<=1 for k,v in expected.items()), ('Pilot drawer differs from actual client prototype',drawer,expected)
            controls=data()['runtime']['display']['interaction_controls']
            for label in ('−10°','−1°','+1°','+10°','Standby','Auto','Track','Wind'):
                found=[c for c in controls if c['label']==label and c['visible']]
                assert len(found)==1 and abs(found[0]['height']-48*scale)<=1, (label,'pilot touch control missing')
            report.setdefault('pilot_drawer',[]).append(dict(bounds=drawer,course_controls=8))
        capture('alpha-'+name)
        light('Dusk');light('Night')
        report.setdefault('night_surfaces',[]).append(chartcheck.dark_surface(capture('beta-night-'+name),expected_page))
        light('Day')
        if windows:ui.assert_product_page(handle,expected_page)
        if windows and name=='autopilot':
            shell_click('Enable control',in_drawer=True);ui.click_text(pid,'Enable DEMO')
            shell_click('Auto',in_drawer=True);ui.click_text(pid,'Request AUTO')
            data(lambda d:d['runtime']['pilot']['mode']=='AUTO' and d['runtime']['pilot']['fresh'] and d['runtime']['pilot']['command_state']=='Confirmed')
            previous=data()['runtime']['pilot']['command_id']
            shell_click('+1°',in_drawer=True)
            data(lambda d:d['runtime']['pilot']['command_state']=='Confirmed' and d['runtime']['pilot']['command_id']!=previous)
            capture('alpha-autopilot-confirmed')
            shell_click('Standby',in_drawer=True)
            data(lambda d:d['runtime']['pilot']['mode']=='STANDBY' and d['runtime']['pilot']['command_state']=='Confirmed')
            shell_click('Enable control',in_drawer=True)
            data(lambda d:not d['runtime']['pilot']['enabled'])
            assert any(c['label']=='Auto' and not c['enabled'] for c in data()['runtime']['display']['interaction_controls'])
            report['checks'].append('Native manual test enable/AUTO/+1/STANDBY/disable with fresh feedback and disabled OFF controls')

        if windows:
            assert not any(caption.startswith('SKAGER page:') for _,caption in ui.children(handle)), 'Preview pane covers product page'
    report['checks'].append('Prototype navigation and eight retained product views captured; advanced frame accelerators remain covered')
    for title,key,name in [('Energy configuration','k','energy-settings'),
                           ('Data Sources','o','sources'),
                           ('Vessel safety settings','q','vessel-settings'),
                           ('Radar status','z','radar-status'),
                           ('Display & layout','f','display')]:
        if windows:
            if name=='energy-settings':
                preferences_entry('Vessel','Advanced battery model')
            elif name=='display':
                display_preferences()
            else:
                section,entry={'sources':('Sensors','Manage sensors'),'vessel-settings':('Vessel','Advanced vessel model'),
                               'radar-status':('Radar','Radar status')}[name]
                preferences_entry(section,entry)
        else:xdo('key','ctrl+shift+'+key);time.sleep(.6)
        expected_page='Display' if name=='display' else title
        data(lambda d:d.get('ui_page')==expected_page)
        if name == 'instruments':
            data(lambda d:d['runtime']['display']['minimum_value_height_dip']>=120)
            report.setdefault('grouped_regions',[]).append(interaction.grouped_regions(data))
        if name=='autopilot':
            drawer=data()['runtime']['display']['drawer']
            if windows:
                client=ui.W.RECT();assert ui.GetClientRect(handle,ui.C.byref(client))
                origin=ui.W.POINT(0,0)
                to_screen=ui.declare(ui.user,'ClientToScreen',ui.W.BOOL,ui.W.HWND,ui.C.POINTER(ui.W.POINT))
                assert to_screen(handle,ui.C.byref(origin))
                scale=ui.GetDpiForWindow(handle)/96
                expected=ui.prototype_drawer_bounds(client.right,client.bottom,scale,(origin.x,origin.y))
            else:
                scale=1;expected=dict(x=682,y=80,width=398,height=674)
            assert all(abs(drawer[k]-v)<=1 for k,v in expected.items()), ('Pilot drawer differs from actual client prototype',drawer,expected)
            controls=data()['runtime']['display']['interaction_controls']
            for label in ('−10°','−1°','+1°','+10°','Standby','Auto','Track','Wind'):
                found=[c for c in controls if c['label']==label and c['visible']]
                assert len(found)==1 and abs(found[0]['height']-48*scale)<=1, (label,'pilot touch control missing')
            report.setdefault('pilot_drawer',[]).append(dict(bounds=drawer,course_controls=8))
        capture('alpha-'+name)
        light('Dusk');light('Night')
        report.setdefault('night_surfaces',[]).append(chartcheck.dark_surface(capture('beta-night-'+name),expected_page))
        light('Day')
        if windows:ui.assert_product_page(handle,expected_page)
        if windows and name=='energy-settings':
            ui.click_text(pid,'Configure battery & reserve')
            ui.set_dialog_fields(pid,'Battery assumptions',['24','20','0.5'])
            ui.click_text(pid,'Save')
            data(lambda d:d['settings']['capacity_kwh']=='24' and d['settings']['reserve_percent']=='20')
            assert data()['data_mode']=='DEMO', 'Live configuration must not silently disable/replace DEMO'
            capture('alpha-energy-settings-saved')
            report['checks'].append('Native battery assumption sheet saves explicit live configuration; DEMO remains separate')
        if windows and name=='display':
            product_click('Dusk');data(lambda d:d['runtime']['display']['light']=='Dusk');capture('alpha-display-dusk')
            product_click('Night');data(lambda d:d['runtime']['display']['light']=='Night');capture('alpha-display-night')
            product_click('Day');data(lambda d:d['runtime']['display']['light']=='Day')
            product_click('Configure data rail');product_click('Energy rail')
            data(lambda d:d['settings']['data_rail']==['soc','pack_power','sog','depth'])
            command('Navigation','n');capture('alpha-energy-rail')
            display_preferences()
            product_click('Configure data rail');product_click('Navigation rail')
            data(lambda d:d['settings']['data_rail']==['sog','depth','aws','heading'])
            product_click('Back to Display')
            data(lambda d:d['ui_page']=='Display')
            product_click('Configure instruments')
            product_click('Shown / PRESSURE')
            data(lambda d:'pressure' not in d['settings']['instruments'])
            product_click('Add / PRESSURE')
            data(lambda d:'pressure' in d['settings']['instruments'])
            report['checks'].append('Native palettes, data-rail presets and instrument selection preserve telemetry provenance')
    report['checks'].append('Energy, source, vessel-safety and radar settings pages captured')
    # A genuine waypoint transition intentionally withholds route/arrival
    # values. Wait for one coherent valid snapshot before comparing progress.
    later=data(lambda d:d['route']['state']=='Valid' and
               'remaining_nm' in d['route'] and 'arrival_soc' in d['energy'] and
               item(d,'Battery SOC')['value']<item(first,'Battery SOC')['value'])
    assert item(later,'Latitude')['value']!=item(first,'Latitude')['value']
    assert later['route']['remaining_nm']<first['route']['remaining_nm']
    assert later['energy']['arrival_soc']!=first['energy']['arrival_soc']
    report['checks'].append('Moving position, SOC, route distance and advisory destination SOC change')
    command('Energy','e');scenario('Sensors stale',1)
    stale=data(lambda d:item(d,'Battery SOC')['quality']=='STALE',timeout=12)
    assert 'arrival_soc' not in stale['energy'] and 'remaining_nm' not in stale['route']
    assert any(a['id'].startswith('position-') and a['level']=='CRITICAL' for a in stale['runtime']['alerts'])
    page_capture('preview-06-stale','Energy')
    # Global strip survives center-page changes. Acknowledgement cannot resolve
    # the fault and recovery followed by another dropout creates a new episode.
    if windows:
        alert_caption=next(c for _,c in ui.children(handle) if c.startswith('Alerts '))
        shell_click(alert_caption)
    else:xdo('key','ctrl+shift+F9');time.sleep(.5)
    alert_data=data(lambda d:d.get('ui_page')=='Alerts')
    gps=next(a for a in alert_data['runtime']['alerts'] if a['id'].startswith('position-'))
    labels={c['label'] for c in alert_data['runtime']['display']['product_controls']}
    assert all('Acknowledge '+a['id'] not in labels for a in alert_data['runtime']['alerts']), 'Normal alert actions must not expose internal identifiers'
    previous_ack={(a['id'],a['episode']):a['acknowledged'] for a in alert_data['runtime']['alerts']}
    capture('beta-alerts-active')
    for attempt in range(20):
        current=data(lambda d:d.get('ui_page')=='Alerts')
        assert any(a['id']==gps['id'] and a['episode']==gps['episode'] for a in current['runtime']['alerts']), 'GPS episode changed before manual acknowledgement'
        drawer=current['runtime']['display']['drawer']
        matches=[c for c in current['runtime']['display']['interaction_controls']
            if c['label']=='Acknowledge' and c.get('accessible_name')=='Acknowledge Position unavailable or stale' and c['enabled']]
        assert len(matches)==1,'Exactly one readable GPS acknowledgement required'
        if matches[0]['visible']:
            pointer_click(matches[0]);break
        x=drawer['x']+drawer['width']//2;y=drawer['y']+drawer['height']//2
        if windows:
            ui.SetCursorPos(x,y);ui.MouseEvent(0x0800,0,0,(-120)&0xffffffff,0)
        else:xdo('mousemove',x,y,'click',5)
        time.sleep(.5)
    else:raise AssertionError('GPS acknowledgement could not be reached by scrolling the drawer')
    acknowledged=data(lambda d:any(a['id']==gps['id'] and a['episode']==gps['episode'] and a['acknowledged'] for a in d['runtime']['alerts']))
    for a in acknowledged['runtime']['alerts']:
        key=(a['id'],a['episode'])
        if key in previous_ack and key!=(gps['id'],gps['episode']):
            assert a['acknowledged']==previous_ack[key], 'Readable action must acknowledge only its original alert episode'
    capture('beta-alerts-acknowledged')
    command('Energy','e')
    assert data()['runtime']['alerts'],'Alert must remain after acknowledgement/page change'
    scenario('Sensors unavailable',2)
    missing=data(lambda d:item(d,'Depth below transducer')['quality']=='UNAVAILABLE')
    assert 'arrival_soc' not in missing['energy'];page_capture('preview-06b-unavailable','Energy')
    scenario('Route inactive',3)
    inactive=data(lambda d:d['route']['state']=='NoActiveRoute')
    assert 'arrival_soc' not in inactive['energy'] and 'range_nm' in inactive['energy']
    scenario('Route ending',4)
    data(lambda d:d['route']['state']=='Valid' and d['route'].get('remaining_nm',1)<1)
    ended=data(lambda d:d['route']['state']=='NoActiveRoute',timeout=10)
    assert 'arrival_soc' not in ended['energy']
    scenario('Low battery',5);data(lambda d:item(d,'Battery SOC').get('value')==12)
    scenario('High power',6);data(lambda d:item(d,'Motor electrical power').get('value')==18)
    scenario('Energy shortfall',7)
    shortfall=data(lambda d:d['energy'].get('shortfall_kwh',0)>0)
    assert 'arrival_soc' not in shortfall['energy'];page_capture('preview-06c-shortfall','Energy')
    assert any(a['id']=='energy-shortfall' for a in shortfall['runtime']['alerts'])
    scenario('Cruising',0)
    data(lambda d:not any(a['id'].startswith('position-') or a['id']=='energy-shortfall' for a in d['runtime'].get('alerts',[])))
    scenario('Sensors stale',1)
    recurrence=data(lambda d:any(a['id']==gps['id'] and a['episode']!=gps['episode'] and not a['acknowledged'] for a in d['runtime']['alerts']))
    scenario('Cruising',0)
    data(lambda d:not any(a['id'].startswith('position-') or a['id']=='energy-shortfall' for a in d['runtime'].get('alerts',[])))
    report['checks'].append('Global alerts persist across pages/acknowledgement, recover, and recur as new episodes')
    report['checks'].append('All eight fixture-accelerator scenarios pass validity/shortfall assertions')
    command('Navigation','n')
    if windows:
        assert not any(caption.startswith(('SKAGER page:', 'SKAGER product page:')) for _, caption in ui.children(handle))
        ui.click_text(pid,'+')
        shell_click('Passage')
        ui.assert_preview_page(handle,'Route')
        report['checks'].append('Page resize/visibility and Navigation return with chart zoom passed')
    phase='XNav to Legacy'
    process_stage('switch-to-legacy-request',pid=pid)
    if windows:
        ui.open_system(pid);ui.click_text(pid,'Open Legacy OpenCPN')
    else:xdo('key','ctrl+shift+l')
    assert wait_launch(app,35)==0
    handle,pid=window('SKAGER Legacy / OpenCPN');ready(2);preserved();chart_capture('preview-07-legacy','XNav to Legacy')
    switch_to_xnav(3,'XNav to Legacy to XNav','preview-09-returned-xnav')
    live=data(lambda d:d['data_mode']!='DEMO')
    assert 'arrival_soc' not in live['energy'];preserved()
    if windows:
        phase='XNav to Safe'
        process_stage('switch-to-safe-request',pid=pid)
        old=ui.monitor_process(pid);ui.open_system(pid);ui.click_text(pid,'Safe Mode');wait_native(old,pid)
        handle,pid=window('SKAGER Safe Mode / OpenCPN');ready(4);chart_capture('preview-08-safe','XNav to Safe')
        switch_to_xnav(5,'Safe to XNav','preview-12-safe-to-xnav');close_current();preserved()
        count=5
        for launcher,title in [('Run-XNav.cmd','SKAGER / OpenCPN'),('Run-Legacy.cmd','SKAGER Legacy / OpenCPN'),('Run-Safe.cmd','SKAGER Safe Mode / OpenCPN')]:
            app=launch('',launcher=launcher);handle,pid=window(title);count+=1;ready(count)
            chart_capture('preview-13-'+launcher[4:-4].lower(),'Direct '+title+' launcher startup')
            close_current();assert wait_launch(app,15)==0;preserved()
        # Reproduce the persisted empty-path artifact from Preview 0.1. Direct
        # startup must repair it without importing or rewriting chart choices.
        with (profile/'opencpn.conf').open('a') as stream:
            stream.write('\n[Directories]\nBaseShapefileDir=./\n')
        app=launch('',direct=True);handle,pid=window('SKAGER / OpenCPN');count+=1;ready(count)
        chart_capture('preview-10-repaired-basemap','Direct startup with old Preview 0.1 basemap setting')
        close_current();assert wait_launch(app,15)==0;preserved()
        refused=subprocess.run([str(exe),'--xnav','--configdir',str(normal)],env=env,capture_output=True,timeout=20)
        assert b'refuses a profile outside' in refused.stderr,refused.stderr
        assert (normal/'opencpn.ini').read_text()=='NORMAL PROFILE MUST NOT CHANGE\n'
        assert sorted(p.name for p in normal.iterdir())==['opencpn.ini']
        report['checks'].append('All four launchers, direct executable launch and external-profile refusal passed with no development PATH')
        report['checks'].append('Normal profile canary and existing OpenCPN files under APPDATA, LOCALAPPDATA, PROGRAMDATA and Program Files remain unchanged')
        report['normal_files_audited']=len(normal_before)
        # Deactivation is logged only for a successfully initialized plugin.
        # Verify all completed normal launches, and no activation in Safe Mode,
        # using the upstream lifecycle rather than only the saved preference.
        sessions=(profile/'opencpn.log').read_text(errors='replace').split('SKAGER startup: ')[1:]
        normal_plugins=safe_plugins=0
        for session in sessions:
            initialized=any('PluginLoader: Deactivating PlugIn:' in line and line.endswith('\\profile\\plugins\\dashboard_pi.dll')
                            for line in session.splitlines())
            if session.startswith('safe'):
                assert not initialized, 'Dashboard initialized during Safe Mode'
                safe_plugins+=1
            else:
                assert initialized, 'Dashboard did not initialize/cleanly unload in a normal mode'
                normal_plugins+=1
        assert normal_plugins==7 and safe_plugins==2,(normal_plugins,safe_plugins)
        report['checks'].append('Bundled Dashboard initialized and cleanly unloaded in seven normal launches; inactive in both Safe launches')
    else:
        close_current();app=launch('safe-mode');handle,pid=window('SKAGER Safe Mode / OpenCPN');ready(4);chart_capture('preview-08-safe','XNav to Safe')
        switch_to_xnav(5,'Safe to XNav','preview-12-safe-to-xnav');close_current();preserved()
        app=launch('legacy');handle,pid=window('SKAGER Legacy / OpenCPN');ready(6)
        chart_capture('preview-13-legacy','Direct Legacy startup')
        switch_to_xnav(7,'Direct Legacy to XNav','preview-14-legacy-to-xnav');close_current();preserved()
    handle=None
    report['checks'].append('XNav / Legacy / Safe clean lifecycle and shared navigation/config persistence passed; mode switch stops Demo')
    report['checks'].append('Real bundled coastline remains rendered after Legacy return and Safe restart')
    report['result']='passed; screenshot review required'
except BaseException as error:
    report['error']=repr(error)
    try:failure_inventory(error)
    except Exception as observed_error:report['failure_inventory_error']=repr(observed_error)
    raise
finally:
    # Record direct parents by their actual PID/launch phase; a previous
    # parent's exit is never the status of the awaited replacement window.
    for process,record in launches:
        record['return_code_before_cleanup']=process.poll()
    if windows and 'result' not in report:
        terminate=ui.declare(ui.kernel,'TerminateProcess',ui.W.BOOL,ui.W.HANDLE,ui.W.UINT)
        for process in failure_processes:
            ph=ui.OpenProcess(1,False,process)
            if ph:
                try:
                    event('failure-cleanup-terminate',pid=process)
                    terminate(ph,1)
                finally:ui.CloseHandle(ph)
        time.sleep(.5)
    if handle and windows:
        try:ui.close(handle)
        except Exception:pass
    if app and app.poll() is None:
        event('cleanup-terminate',pid=app.pid)
        app.terminate()
        try:wait_launch(app,10)
        except subprocess.TimeoutExpired:
            event('cleanup-kill',pid=app.pid);app.kill()
    if xserver:
        xserver.terminate();code=xserver.wait(timeout=10)
        event('xserver-exit',pid=xserver.pid,exit_code=code)
    for folder,name in [(profile,'preview-profile'),(logs,'preview-logs')]:
        shutil.copytree(folder,evidence/name,dirs_exist_ok=True,ignore=shutil.ignore_patterns('*.pem','opencpn-ipc'))
    (evidence/('preview-results.json' if windows else 'preview-linux-results.json')).write_text(json.dumps(report,indent=2)+'\n')
    temporary.cleanup()
print(report['result'])

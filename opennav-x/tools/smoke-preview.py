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
if windows: (profile/'OPENNAV_TEST_PROFILE').write_text('CI disposable extracted preview only\n')
fixtures=module('profile-fixtures');fixtures.seed(profile);expected_profile=fixtures.snapshot(profile)
chartcheck=module('chart-render-check')

def xdo(*arguments):
    return subprocess.check_output(['xdotool',*map(str,arguments)],env=env,text=True).strip()
def window(title,expected_pid=None):
    if windows:return ui.wait_window(title,expected_pid)
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        r=subprocess.run(['xdotool','search','--onlyvisible','--name','^'+title+'$'],env=env,capture_output=True,text=True)
        if r.returncode==0 and r.stdout.strip():
            h=r.stdout.splitlines()[0];xdo('windowsize',h,1280,800);xdo('windowmove',h,0,0);xdo('windowfocus',h)
            return h,int(xdo('getwindowpid',h))
        time.sleep(.1)
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
        ancestor=ui.declare(ui.user,'GetAncestor',ui.W.HWND,ui.W.HWND,ui.W.UINT)
        foreground=ui.declare(ui.user,'GetForegroundWindow',ui.W.HWND)
        surface=ancestor(hit,2);ui.SetForegroundWindow(surface)
        deadline=time.monotonic()+3
        while foreground()!=surface and time.monotonic()<deadline:time.sleep(.05)
        assert foreground()==surface and ui.WindowFromPoint(point)==hit
        ui.SetCursorPos(x,y);ui.MouseEvent(2,0,0,0,0);time.sleep(.05);ui.MouseEvent(4,0,0,0,0)
    else:xdo('mousemove',x,y,'click',1)
    time.sleep(.4)
def shell_click(label,outside_drawer=False,in_status=False,in_drawer=False):
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
    ticks=int(record['runtime']['ui_update']['ticks']);pointer_click(targets[0])
    data(lambda d:int(d['runtime']['ui_update']['ticks'])>=ticks+3)
def product_scroll(direction):
    if windows:ui.click_text(pid,'Down' if direction>0 else 'Up')
    else:xdo('mousemove',700,430,'click',5 if direction>0 else 4)
    time.sleep(.4)
def product_click(label,enabled=True):
    target=interaction.control(data,label,product_scroll,enabled=enabled)
    pointer_click(target)
    return target
def item(d,name):return next(i for i in d['data'] if i['name']==name)
def command(label,shortcut):
    # Primary workflows exercise visible prototype controls on both platforms.
    direct={'n':'Chart','r':'Passage','e':'Energy','m':'Settings','g':'Settings',
            'v':'Instruments','a':'Traffic','h':'Anchor','z':'Radar','s':'System'}
    if shortcut in direct:shell_click(direct[shortcut],outside_drawer=True)
    elif shortcut=='i':shell_click('System',outside_drawer=True);product_click('Diagnostics')
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
    style='XNav' if title=='OpenNav X / OpenCPN' else 'Standard'
    report.setdefault('chart_rendering',[]).append(chartcheck.presentation(capture(name),style,'Day',phase))
def page_capture(name, page):
    capture(name)
    if windows:
        report.setdefault('page_visibility', []).append(ui.assert_preview_page(handle, page))
def close_current():
    if windows:
        h=ui.monitor_process(pid);ui.close(handle);ui.wait_clean_exit(h)
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
            child,status=os.waitpid(pid,os.WNOHANG)
            if child:
                assert os.waitstatus_to_exitcode(status)==0;return
            time.sleep(.1)
        raise RuntimeError('Preview did not exit cleanly')
def preserved():
    assert fixtures.snapshot(profile)==expected_profile,'Preview changed seeded navigation/configuration fixtures'
    if windows:
        assert (normal/'opencpn.ini').read_text()=='NORMAL PROFILE MUST NOT CHANGE\n'
        assert normal_snapshot()==normal_before,'Existing normal OpenCPN files changed'
def switch_to_xnav(count, phase, name):
    global handle, pid
    if windows:
        old=ui.monitor_process(pid);ui.click_menu(handle,'Switch to XNav');ui.wait_clean_exit(old)
    else:
        old=pid;xdo('mousemove',600,400,'click',3);time.sleep(.4);xdo('key','End','Return')
        _,status=os.waitpid(old,0);assert os.waitstatus_to_exitcode(status)==0
    handle,pid=window('OpenNav X / OpenCPN');ready(count);preserved()
    chart_capture(name,phase)
    if windows:
        saved=data(lambda d:d['settings']['capacity_kwh']=='24' and d['settings']['reserve_percent']=='20')
        assert saved['settings']['battery_device']=='', 'No battery identity may be fabricated'
        report['checks'].append('Explicit live energy configuration survived mode restart')
def launch(mode,demo=False,launcher=None,direct=False):
    if windows and launcher:
        return subprocess.Popen([os.environ['COMSPEC'],'/d','/c',str(package/launcher)],cwd=temp,env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    with (evidence/'preview-launch.log').open('a') as out:
        return subprocess.Popen([str(exe)]+([] if direct else ['--configdir',str(profile),'--no_opengl','--'+mode]+(['--xnav-demo'] if demo else [])),env=env,cwd=temp,stdout=out,stderr=out)

try:
    app=launch('xnav',True,'Run-XNav-Demo.cmd' if windows else None)
    handle,pid=window('OpenNav X / OpenCPN');ready(1)
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
        xdo('key','ctrl+shift+s');time.sleep(.5)
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
    report['chart_rendering'].append(chartcheck.presentation(capture('preview-02-navigation-night'),'XNav','Night','Night world-chart land/water palette'))
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
            assert not any(caption.startswith('OpenNav page:') for _,caption in ui.children(handle)), 'Preview pane covers product page'
    report['checks'].append('Prototype navigation and eight retained product views captured; advanced frame accelerators remain covered')
    for title,key,name in [('Energy configuration','k','energy-settings'),
                           ('Data Sources','o','sources'),
                           ('Vessel safety settings','q','vessel-settings'),
                           ('Radar status','z','radar-status'),
                           ('Display & layout','f','display')]:
        if windows:
            command('Settings','g')
            if name=='energy-settings':
                shell_click('Vessel',in_drawer=True);shell_click('Battery & reserve',in_drawer=True)
            else:
                section,entry={'sources':('Sensors','Manage sensors'),'vessel-settings':('Vessel','Vessel dimensions'),
                               'radar-status':('Radar','Radar status'),'display':('Display','Chart presentation')}[name]
                shell_click(section,in_drawer=True);shell_click(entry,in_drawer=True)
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
            ui.click_text(pid,'Dusk');time.sleep(.5);capture('alpha-display-dusk')
            ui.click_text(pid,'Night');time.sleep(.5);capture('alpha-display-night')
            ui.click_text(pid,'Day');time.sleep(.5)
            ui.click_text(pid,'Configure data rail');ui.click_text(pid,'Energy rail')
            data(lambda d:d['settings']['data_rail']==['soc','pack_power','sog','depth'])
            command('Navigation','n');capture('alpha-energy-rail')
            command('Settings','g');shell_click('Display',in_drawer=True);shell_click('Chart presentation',in_drawer=True)
            ui.click_text(pid,'Configure data rail');ui.click_text(pid,'Navigation rail')
            data(lambda d:d['settings']['data_rail']==['sog','depth','aws','heading'])
            ui.click_text(pid,'Back to Display');ui.click_text(pid,'Configure instruments')
            product_click('Shown / PRESSURE')
            data(lambda d:'pressure' not in d['settings']['instruments'])
            product_click('Add / PRESSURE')
            data(lambda d:'pressure' in d['settings']['instruments'])
            report['checks'].append('Native palettes, data-rail presets and instrument selection preserve telemetry provenance')
    report['checks'].append('Energy, source, vessel-safety and radar settings pages captured')
    later=data(lambda d:item(d,'Battery SOC')['value']<item(first,'Battery SOC')['value'])
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
        assert not any(caption.startswith(('OpenNav page:', 'OpenNav product page:')) for _, caption in ui.children(handle))
        ui.click_text(pid,'+')
        shell_click('Passage')
        ui.assert_preview_page(handle,'Route')
        report['checks'].append('Page resize/visibility and Navigation return with chart zoom passed')
    if windows:
        ui.click_text(pid,'System');ui.click_text(pid,'Open Legacy OpenCPN')
    else:xdo('key','ctrl+shift+l')
    assert app.wait(timeout=35)==0
    handle,pid=window('OpenCPN / Legacy');ready(2);preserved();chart_capture('preview-07-legacy','XNav to Legacy')
    switch_to_xnav(3,'XNav to Legacy to XNav','preview-09-returned-xnav')
    live=data(lambda d:d['data_mode']!='DEMO')
    assert 'arrival_soc' not in live['energy'];preserved()
    if windows:
        old=ui.monitor_process(pid);ui.click_text(pid,'System');ui.click_text(pid,'Safe Mode');ui.wait_clean_exit(old)
        handle,pid=window('OpenNav Safe Mode / OpenCPN');ready(4);chart_capture('preview-08-safe','XNav to Safe')
        switch_to_xnav(5,'Safe to XNav','preview-12-safe-to-xnav');close_current();preserved()
        count=5
        for launcher,title in [('Run-XNav.cmd','OpenNav X / OpenCPN'),('Run-Legacy.cmd','OpenCPN / Legacy'),('Run-Safe.cmd','OpenNav Safe Mode / OpenCPN')]:
            app=launch('',launcher=launcher);handle,pid=window(title);count+=1;ready(count)
            chart_capture('preview-13-'+launcher[4:-4].lower(),'Direct '+title+' launcher startup')
            close_current();assert app.wait(timeout=15)==0;preserved()
        # Reproduce the persisted empty-path artifact from Preview 0.1. Direct
        # startup must repair it without importing or rewriting chart choices.
        with (profile/'opencpn.conf').open('a') as stream:
            stream.write('\n[Directories]\nBaseShapefileDir=./\n')
        app=launch('',direct=True);handle,pid=window('OpenNav X / OpenCPN');count+=1;ready(count)
        chart_capture('preview-10-repaired-basemap','Direct startup with old Preview 0.1 basemap setting')
        close_current();assert app.wait(timeout=15)==0;preserved()
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
        sessions=(profile/'opencpn.log').read_text(errors='replace').split('OpenNav startup: ')[1:]
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
        close_current();app=launch('safe-mode');handle,pid=window('OpenNav Safe Mode / OpenCPN');ready(4);chart_capture('preview-08-safe','XNav to Safe')
        switch_to_xnav(5,'Safe to XNav','preview-12-safe-to-xnav');close_current();preserved()
        app=launch('legacy');handle,pid=window('OpenCPN / Legacy');ready(6)
        chart_capture('preview-13-legacy','Direct Legacy startup')
        switch_to_xnav(7,'Direct Legacy to XNav','preview-14-legacy-to-xnav');close_current();preserved()
    handle=None
    report['checks'].append('XNav / Legacy / Safe clean lifecycle and shared navigation/config persistence passed; mode switch stops Demo')
    report['checks'].append('Real bundled coastline remains rendered after Legacy return and Safe restart')
    report['result']='passed; screenshot review required'
finally:
    if not windows and 'result' not in report:
        report['failure_process_exit']=app.poll() if app else None
        found=subprocess.run(['xdotool','search','--onlyvisible','--name','.*'],env=env,capture_output=True,text=True)
        report['failure_windows']=[]
        for window_id in found.stdout.splitlines():
            title=subprocess.run(['xdotool','getwindowname',window_id],env=env,capture_output=True,text=True)
            report['failure_windows'].append({'id':window_id,'title':title.stdout.strip()})
    if windows and 'result' not in report:
        # Preserve the actual startup dialog instead of dismissing it. Terminate
        # only an executable belonging to this freshly extracted test package.
        query=ui.declare(ui.kernel,'QueryFullProcessImageNameW',ui.W.BOOL,ui.W.HANDLE,ui.W.DWORD,ui.W.LPWSTR,ui.C.POINTER(ui.W.DWORD))
        terminate=ui.declare(ui.kernel,'TerminateProcess',ui.W.BOOL,ui.W.HANDLE,ui.W.UINT)
        seen=set();report['failure_windows']=[]
        for h,process,title in ui.windows():
            ph=ui.OpenProcess(0x1000|1,False,process)
            if not ph:continue
            try:
                size=ui.W.DWORD(32768);name=ui.C.create_unicode_buffer(size.value)
                if query(ph,0,name,ui.C.byref(size)) and str(Path(name.value).resolve()).casefold()==str(exe.resolve()).casefold():
                    report['failure_windows'].append({'title':title,'children':[caption for _,caption in ui.children(h)]})
                    if process not in seen:terminate(ph,1);seen.add(process)
            finally:ui.CloseHandle(ph)
        time.sleep(.5)
    if handle and windows:
        try:ui.close(handle)
        except Exception:pass
    if app and app.poll() is None:
        app.terminate()
        try:app.wait(timeout=10)
        except subprocess.TimeoutExpired:app.kill()
    if xserver:xserver.terminate();xserver.wait(timeout=10)
    for folder,name in [(profile,'preview-profile'),(logs,'preview-logs')]:
        shutil.copytree(folder,evidence/name,dirs_exist_ok=True,ignore=shutil.ignore_patterns('*.pem','opencpn-ipc'))
    (evidence/('preview-results.json' if windows else 'preview-linux-results.json')).write_text(json.dumps(report,indent=2)+'\n')
    temporary.cleanup()
print(report['result'])

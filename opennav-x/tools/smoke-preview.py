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

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--package',type=Path)
args=parser.parse_args();windows=sys.platform=='win32'
if windows != bool(args.package):raise SystemExit('Native Windows requires --package; Linux uses its development install')
evidence=root/'evidence/local';evidence.mkdir(parents=True,exist_ok=True)
temporary=tempfile.TemporaryDirectory(prefix='OpenNav preview ',dir=None if windows else '/tmp')
temp=Path(temporary.name)
env=dict(os.environ);ui=None;xserver=None;app=None;handle=None;pid=None
normal_locations=[];normal_before={}
report={'authority':'native Windows extracted ZIP' if windows else 'Linux development',
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
    ui=module('windows-ui');report['display']=ui.ensure_desktop()
    with zipfile.ZipFile(args.package) as z:z.extractall(temp)
    package=temp/'OpenNavX-Alpha1-Portable';profile=package/'profile';logs=package/'logs';exe=package/'app/opencpn.exe'
    manifest=json.loads((package/'FILE_SHA256.json').read_text())
    for name,expected in manifest.items():assert hashlib.sha256((package/name).read_bytes()).hexdigest()==expected,name
    report['checks'].append('All extracted package file hashes match')
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
    package=temp/'OpenNavX-Alpha1-Portable';app_dir=package/'app';app_dir.mkdir(parents=True)
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
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:
            d=json.loads((logs/'opennav-diagnostics.json').read_text())
            if predicate(d):return d
        except (FileNotFoundError,json.JSONDecodeError,PermissionError):pass
        time.sleep(.2)
    raise AssertionError('Diagnostic predicate did not become true: '+json.dumps(d.get('runtime',{}).get('display',{})))
def item(d,name):return next(i for i in d['data'] if i['name']==name)
def command(label,shortcut):
    if windows:ui.click_text(pid,label)
    else:xdo('key','ctrl+shift+'+shortcut);time.sleep(.5)
def scenario(label,index):
    if windows:ui.click_text(pid,'Demo');ui.click_text(pid,label)
    else:xdo('key','ctrl+shift+F'+str(index+1));time.sleep(.4)
def capture(name):
    path=evidence/(name+('.png' if windows else '-linux.png'))
    if windows:rgb=ui.capture(handle,path)
    else:
        time.sleep(.4);subprocess.run(['import','-window','root',str(path)],env=env,check=True)
        rgb=subprocess.check_output(['convert',str(path),'-depth','8','rgb:-'],env=env)
    report['screenshots'].append(path.name)
    return rgb
def chart_capture(name,phase):
    report.setdefault('chart_rendering',[]).append(chartcheck.check(capture(name),chart_colors,phase))
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
        # Exercise the label's hidden-to-visible transition, not just a wide
        # first launch. This exposed an overlap in the live navigation captures.
        assert ui.SetWindowPos(handle,None,0,0,960,800,4)
        time.sleep(.6)
        ui.size_window(handle)
        ui.assert_route_summary_layout(handle)
        report['checks'].append('Bottom route summary lays out after narrow-to-wide resize')
    if not windows:
        xdo('mousemove',1220,772,'click',1);time.sleep(.4);capture('beta-system-popup')
        # Bare Xvfb has no window manager to move X input focus to a transient.
        popup=xdo('search','--onlyvisible','--pid',pid,'--name','^opencpn$').splitlines()[-1]
        xdo('windowfocus',popup)
        xdo('key','Escape');xdo('windowfocus',handle);time.sleep(.3)
    first=data(lambda d:d['data_mode']=='DEMO' and 'arrival_soc' in d['energy'])
    assert first['route']['source'].startswith('DEMO')
    chart_colors=chartcheck.reference(capture('preview-01-navigation-day'))
    chart_capture('preview-11-startup-xnav','Direct XNav startup')
    if windows:
        ui.click_text(pid,'Light');ui.click_text(pid,'Light')
    else:
        xdo('mousemove',1240,28,'click',1)
        data(lambda d:d['runtime']['display']['light']=='Dusk')
        xdo('mousemove',1240,28,'click',1)
        data(lambda d:d['runtime']['display']['light']=='Night')
    report['chart_rendering'].append(chartcheck.night(capture('preview-02-navigation-night'),chart_colors,'Night world-chart land/water palette'))
    if windows:ui.click_text(pid,'Light')
    else:xdo('click',1);time.sleep(.5)
    command('Route','r');page_capture('preview-03-route','Route')
    command('Energy','e');page_capture('preview-04-energy','Energy')
    if windows:ui.click_text(pid,'System');ui.click_text(pid,'Diagnostics')
    else:xdo('key','ctrl+shift+i');time.sleep(.5)
    page_capture('preview-05-diagnostics','Diagnostics')
    data(lambda d:d['runtime']['display']['can_scroll_down'])
    if windows:ui.click_text(pid,'Down')
    else:xdo('mousemove',1164,28,'click',1)
    data(lambda d:d['runtime']['display']['page_scroll_px']>0)
    if windows:ui.click_text(pid,'Up')
    else:xdo('mousemove',1092,28,'click',1)
    data(lambda d:d['runtime']['display']['page_scroll_px']==0)
    report['checks'].append('Persistent page controls scroll diagnostics and return to top without native scrollbars')
    # Alpha product pages use real touch-button actions on Windows and public
    # frame shortcuts on Linux. Existing preview regression assertions remain.
    for title,key,name in [('Vessel instruments','v','instruments'),
                           ('AIS targets','a','ais'),('SmartNav advisories','j','smartnav'),
                           ('Manual autopilot','y','autopilot'),('Anchor watch','h','anchor'),
                           ('Settings','g','settings'),('Routes','b','routes'),
                           ('Waypoints','w','waypoints')]:
        command('Menu','m')
        if windows: ui.click_text(pid,title)
        else: xdo('key','ctrl+shift+'+key);time.sleep(.6)
        expected_page='SmartNav' if name=='smartnav' else title
        data(lambda d:d.get('ui_page')==expected_page)
        if name in ('instruments','autopilot'):
            data(lambda d:d['runtime']['display']['minimum_value_height_dip']>=120)
        capture('alpha-'+name)
        if windows:ui.click_text(pid,'Light');ui.click_text(pid,'Light')
        else:
            xdo('mousemove',1240,28,'click',1);time.sleep(.4);xdo('click',1)
        data(lambda d:d['runtime']['display']['light']=='Night')
        report.setdefault('night_surfaces',[]).append(chartcheck.dark_surface(capture('beta-night-'+name),expected_page))
        if windows:ui.click_text(pid,'Light')
        else:xdo('click',1)
        data(lambda d:d['runtime']['display']['light']=='Day')
        if windows:ui.assert_product_page(handle,expected_page)
        if windows and name=='autopilot':
            ui.click_text(pid,'Enable / disable DEMO manual control');ui.click_text(pid,'Enable DEMO')
            ui.click_text(pid,'AUTO');ui.click_text(pid,'Request AUTO');time.sleep(1)
            captions=[caption for _,caption in ui.children(handle)]
            assert any(c.startswith('AUTO / Feedback current') for c in captions),captions
            assert any(c.startswith('Command: Confirmed') for c in captions),captions
            ui.click_text(pid,'+1° magnetic course');time.sleep(1)
            assert any(c.startswith('Command: Confirmed') for _,c in ui.children(handle))
            capture('alpha-autopilot-confirmed')
            ui.click_text(pid,'STBY');time.sleep(1)
            assert any(c.startswith('STANDBY / Feedback current') for _,c in ui.children(handle))
            ui.click_text(pid,'Enable / disable DEMO manual control')
            report['checks'].append('Native manual DEMO enable/AUTO/+1/STANDBY/disable with new-feedback confirmation')

        if windows:
            assert not any(caption.startswith('OpenNav page:') for _,caption in ui.children(handle)), 'Preview pane covers Alpha page'
    report['checks'].append('Alpha menu and eight product page interactions captured')
    for title,key,name in [('Energy configuration','k','energy-settings'),
                           ('Data Sources','o','sources'),
                           ('Vessel safety settings','q','vessel-settings'),
                           ('Radar status','z','radar-status'),
                           ('Display & layout','f','display')]:
        if windows:
            command('Menu','m');ui.click_text(pid,'Settings');ui.click_text(pid,title)
        else:xdo('key','ctrl+shift+'+key);time.sleep(.6)
        expected_page='Display' if name=='display' else title
        data(lambda d:d.get('ui_page')==expected_page)
        if name in ('instruments','autopilot'):
            data(lambda d:d['runtime']['display']['minimum_value_height_dip']>=120)
        capture('alpha-'+name)
        if windows:ui.click_text(pid,'Light');ui.click_text(pid,'Light')
        else:
            xdo('mousemove',1240,28,'click',1);time.sleep(.4);xdo('click',1)
        data(lambda d:d['runtime']['display']['light']=='Night')
        report.setdefault('night_surfaces',[]).append(chartcheck.dark_surface(capture('beta-night-'+name),expected_page))
        if windows:ui.click_text(pid,'Light')
        else:xdo('click',1)
        data(lambda d:d['runtime']['display']['light']=='Day')
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
            data(lambda d:d['settings']['data_rail']==['soc','pack_power','rpm','sog','depth'])
            command('Navigation','n');capture('alpha-energy-rail')
            command('Menu','m');ui.click_text(pid,'Settings');ui.click_text(pid,'Display & layout')
            ui.click_text(pid,'Configure data rail');ui.click_text(pid,'Navigation rail')
            data(lambda d:d['settings']['data_rail']==['sog','cog','heading','depth','aws'])
            ui.click_text(pid,'Back to Display');ui.click_text(pid,'Configure instruments')
            ui.click_text(pid,'Shown / PRESSURE')
            data(lambda d:'pressure' not in d['settings']['instruments'])
            ui.click_text(pid,'Add / PRESSURE')
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
        alert_caption=next(c for _,c in ui.children(handle) if c.startswith('Alerts / '))
        ui.click_text(pid,alert_caption)
    else:xdo('key','ctrl+shift+F9');time.sleep(.5)
    alert_data=data(lambda d:d.get('ui_page')=='Alerts')
    gps=next(a for a in alert_data['runtime']['alerts'] if a['id'].startswith('position-'))
    capture('beta-alerts-active')
    if windows:ui.click_text(pid,'Acknowledge '+gps['id'])
    else:
        # First condition row: caption/action/source then two 52-DIP buttons.
        gps_index=next(i for i,a in enumerate(alert_data['runtime']['alerts']) if a['id']==gps['id'])
        xdo('mousemove',900,368+177*gps_index,'click',1)
    acknowledged=data(lambda d:any(a['id']==gps['id'] and a['acknowledged'] for a in d['runtime']['alerts']))
    assert any(a['episode']==gps['episode'] for a in acknowledged['runtime']['alerts'])
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
    report['checks'].append('All eight GUI-selected scenarios pass validity/shortfall assertions')
    command('Navigation','n')
    if windows:
        assert not any(caption.startswith('OpenNav page:') for _, caption in ui.children(handle))
        ui.click_text(pid,'+')
        ui.click_text(pid,'Route')
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

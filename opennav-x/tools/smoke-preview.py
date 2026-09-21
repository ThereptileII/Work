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
    package=temp/'OpenNavX-DeveloperPreview';profile=package/'profile';logs=package/'logs';exe=package/'app/opencpn.exe'
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
    profile=temp/'profile';logs=profile;exe=root/'build/xnav-install/bin/opencpn'
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
            time.sleep(.6);return
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
    raise AssertionError('Diagnostic predicate did not become true')
def item(d,name):return next(i for i in d['data'] if i['name']==name)
def command(label,shortcut):
    if windows:ui.click_text(pid,label)
    else:xdo('key','ctrl+shift+'+shortcut);time.sleep(.5)
def scenario(label,index):
    if windows:ui.click_text(pid,'Demo');ui.click_text(pid,label)
    else:xdo('key','ctrl+shift+F'+str(index+1));time.sleep(.4)
def capture(name):
    path=evidence/(name+('.png' if windows else '-linux.png'))
    if windows:ui.capture(handle,path)
    else:
        time.sleep(.4);subprocess.run(['import','-window','root',str(path)],env=env,check=True)
    report['screenshots'].append(path.name)
def close_current():
    if windows:
        h=ui.monitor_process(pid);ui.close(handle);ui.wait_clean_exit(h)
    else:
        subprocess.run([str(exe),'--configdir',str(profile),'--remote','--quit'],env=env,check=True,timeout=15)
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
    first=data(lambda d:d['data_mode']=='DEMO' and 'arrival_soc' in d['energy'])
    assert first['route']['source'].startswith('DEMO')
    capture('preview-01-navigation-day')
    if windows:
        ui.click_text(pid,'Light');ui.click_text(pid,'Light')
    else:
        xdo('mousemove',1240,28,'click',1);time.sleep(.4);xdo('click',1);time.sleep(.4)
    capture('preview-02-navigation-night')
    if windows:ui.click_text(pid,'Light')
    else:xdo('click',1);time.sleep(.5)
    command('Route','r');capture('preview-03-route')
    command('Energy','e');capture('preview-04-energy')
    if windows:ui.click_text(pid,'System');ui.click_text(pid,'Diagnostics')
    else:xdo('key','ctrl+shift+i');time.sleep(.5)
    capture('preview-05-diagnostics')
    later=data(lambda d:item(d,'Battery SOC')['value']<item(first,'Battery SOC')['value'])
    assert item(later,'Latitude')['value']!=item(first,'Latitude')['value']
    assert later['route']['remaining_nm']<first['route']['remaining_nm']
    assert later['energy']['arrival_soc']!=first['energy']['arrival_soc']
    report['checks'].append('Moving position, SOC, route distance and advisory destination SOC change')
    command('Energy','e');scenario('Sensors stale',1)
    stale=data(lambda d:item(d,'Battery SOC')['quality']=='STALE',timeout=12)
    assert 'arrival_soc' not in stale['energy'] and 'remaining_nm' not in stale['route']
    capture('preview-06-stale')
    scenario('Sensors unavailable',2)
    missing=data(lambda d:item(d,'Depth below transducer')['quality']=='UNAVAILABLE')
    assert 'arrival_soc' not in missing['energy'];capture('preview-06b-unavailable')
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
    assert 'arrival_soc' not in shortfall['energy'];capture('preview-06c-shortfall')
    report['checks'].append('All eight GUI-selected scenarios pass validity/shortfall assertions')
    if windows:
        ui.click_text(pid,'System');ui.click_text(pid,'Open Legacy OpenCPN')
    else:xdo('key','ctrl+shift+l')
    assert app.wait(timeout=35)==0
    handle,pid=window('OpenCPN / Legacy');ready(2);preserved();capture('preview-07-legacy')
    if windows:
        old=ui.monitor_process(pid);ui.click_menu(handle,'Switch to XNav');ui.wait_clean_exit(old)
    else:
        old=pid;xdo('mousemove',600,400,'click',3);time.sleep(.4);xdo('key','End','Return')
        _,status=os.waitpid(old,0);assert os.waitstatus_to_exitcode(status)==0
    handle,pid=window('OpenNav X / OpenCPN');ready(3)
    live=data(lambda d:d['data_mode']!='DEMO')
    assert 'arrival_soc' not in live['energy'];preserved()
    if windows:
        old=ui.monitor_process(pid);ui.click_text(pid,'System');ui.click_text(pid,'Safe Mode');ui.wait_clean_exit(old)
        handle,pid=window('OpenNav Safe Mode / OpenCPN');ready(4);capture('preview-08-safe');close_current();preserved()
        count=4
        for launcher,title in [('Run-XNav.cmd','OpenNav X / OpenCPN'),('Run-Legacy.cmd','OpenCPN / Legacy'),('Run-Safe.cmd','OpenNav Safe Mode / OpenCPN')]:
            app=launch('',launcher=launcher);handle,pid=window(title);count+=1;ready(count);close_current();assert app.wait(timeout=15)==0;preserved()
        # Direct executable launch must also remain inside the package.
        app=launch('',direct=True);handle,pid=window('OpenNav X / OpenCPN');count+=1;ready(count);close_current();assert app.wait(timeout=15)==0;preserved()
        refused=subprocess.run([str(exe),'--xnav','--configdir',str(normal)],env=env,capture_output=True,timeout=20)
        assert b'refuses a profile outside' in refused.stderr,refused.stderr
        assert (normal/'opencpn.ini').read_text()=='NORMAL PROFILE MUST NOT CHANGE\n'
        assert sorted(p.name for p in normal.iterdir())==['opencpn.ini']
        report['checks'].append('All four launchers, direct executable launch and external-profile refusal passed with no development PATH')
        report['checks'].append('Normal profile canary and existing OpenCPN files under APPDATA, LOCALAPPDATA, PROGRAMDATA and Program Files remain unchanged')
        report['normal_files_audited']=len(normal_before)
    else:
        close_current();app=launch('safe-mode');handle,pid=window('OpenNav Safe Mode / OpenCPN');ready(4);capture('preview-08-safe');close_current();preserved()
    handle=None
    report['checks'].append('XNav / Legacy / Safe clean lifecycle and shared navigation/config persistence passed; mode switch stops Demo')
    report['result']='passed; screenshot review required'
finally:
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

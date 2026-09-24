#!/usr/bin/env python3
"""Terminate only owned disposable test processes; exercise real startup recovery."""
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
root=Path(__file__).resolve().parents[1]
windows=sys.platform=='win32'
evidence=root/'evidence/local';evidence.mkdir(parents=True,exist_ok=True)
def module(name):
    spec=importlib.util.spec_from_file_location(name,root/'tools'/f'{name}.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
fixtures=module('profile-fixtures');chart=module('chart-render-check')
ui=module('windows-ui') if windows else None
temporary=tempfile.TemporaryDirectory(prefix='opennav recovery ',dir=None if windows else '/tmp')
profile=Path(temporary.name)/'profile'
subprocess.run([sys.executable,str(root/'tools/prepare-test-profile.py'),'--build',str(root/('build/xnav-windows' if windows else 'build/xnav-linux')),'--profile',str(profile)],check=True)
fixtures.seed(profile);expected=fixtures.snapshot(profile)
with (profile/'opencpn.conf').open('a') as f:f.write('\n[Settings/GlobalState]\nVPLatLon=59.0800,18.5000\nVPScale=0.003\n')
env=dict(os.environ);xserver=None;app=None;pid=None;handle=None;owned=set()
report={'authority':'native Windows' if windows else 'Linux development','checks':[],'screenshots':[],'chart_rendering':[]}
exe=root/('build/xnav-install/opencpn.exe' if windows else 'build/xnav-install/bin/opencpn')
if windows:report['display']=ui.ensure_desktop()
else:
    assert ctypes.CDLL(None).prctl(36,1,0,0,0)==0
    number=122
    while Path(f'/tmp/.X{number}-lock').exists():number+=1
    env['DISPLAY']=f':{number}'
    xserver=subprocess.Popen(['Xvfb',env['DISPLAY'],'-screen','0','1280x800x24','-nolisten','tcp'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    time.sleep(1)
def xdo(*args):return subprocess.check_output(['xdotool',*map(str,args)],env=env,text=True).strip()
def window(title,timeout=45):
    if windows:
        h,p=ui.wait_window(title,timeout=timeout);owned.add(p);return h,p
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        p=subprocess.run(['xdotool','search','--onlyvisible','--name','^'+title+'$'],env=env,capture_output=True,text=True)
        if p.returncode==0 and p.stdout.strip():
            h=p.stdout.splitlines()[0];pid=int(xdo('getwindowpid',h));owned.add(pid)
            return h,pid
        time.sleep(.1)
    raise RuntimeError('Missing window: '+title)
def ready(count):
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        log=profile/'opencpn.log'
        if log.exists() and log.read_text(errors='replace').count('OnInitTimer...Finalize Canvases')>=count:
            time.sleep(.5);return
        time.sleep(.1)
    raise RuntimeError('Deferred startup incomplete')
def capture(name):
    path=evidence/(name+('.png' if windows else '-linux.png'))
    if windows:ui.size_window(handle);rgb=ui.capture(handle,path)
    else:
        xdo('windowsize',handle,1280,800,'windowmove',handle,0,0);time.sleep(.5)
        subprocess.run(['import','-window','root',str(path)],env=env,check=True)
        rgb=subprocess.check_output(['convert',str(path),'-depth','8','rgb:-'],env=env)
    report['screenshots'].append(path.name);return rgb
def launch():
    with (evidence/'recovery-launch.log').open('a') as out:
        a=subprocess.Popen([str(exe),'--configdir',str(profile),'--no_opengl','--xnav'],env=env,stdout=out,stderr=out)
    owned.add(a.pid);return a
try:
    for count in (1,2):
        app=launch()
        if count==2:
            dialog,_=window('Safe Restart')
            if windows:ui.click_text(app.pid,'Normal start')
            else:xdo('windowfocus',dialog,'key','Return')
        handle,pid=window('OpenNav X / OpenCPN');ready(count)
        state=(profile/'opennav-startup.state').read_text()
        assert 'pending 1' in state and f'failures {count-1}' in state,state
        if count==1:colors=chart.reference(capture('recovery-01-xnav'))
        app.kill();app.wait(timeout=15);owned.discard(app.pid)
        assert fixtures.snapshot(profile)==expected,'Abrupt stop lost navigation/configuration'
        report['checks'].append(f'Owned XNav process terminated before healthy startup {count}; navigation fixtures preserved')
    app=launch()
    dialog,_=window('OpenNav startup recovery')
    if windows:ui.click_text(app.pid,'OK')
    else:xdo('windowfocus',dialog,'key','Return')
    handle,pid=window('OpenNav Safe Mode / OpenCPN');ready(3)
    log=(profile/'opencpn.log').read_text(errors='replace')
    assert 'OpenNav automatic Safe Mode' in log
    safe=log.rsplit('OpenNav startup: safe',1)[-1]
    assert 'Initializing PlugIn: Dashboard' not in safe, 'Safe Mode loaded Dashboard'
    state=(profile/'opennav-startup.state').read_text()
    assert 'failures 2' in state and 'pending 0' in state,state
    report['chart_rendering'].append(chart.check(capture('recovery-02-safe'),colors,'Automatic Safe recovery'))
    assert fixtures.snapshot(profile)==expected
    report['checks'].append('Third explicit XNav launch automatically starts Safe; recovery notice and disabled plugin verified')
    if windows:ui.click_menu(handle,'Switch to XNav')
    else:
        xdo('windowfocus',handle,'mousemove',600,400,'click',3);time.sleep(.3);xdo('key','End','Return')
    assert app.wait(timeout=30)==0;owned.discard(app.pid)
    handle,pid=window('OpenNav X / OpenCPN');ready(4)
    state=(profile/'opennav-startup.state').read_text()
    assert 'failures 0' in state and 'pending 1' in state,state
    assert list(profile.glob('opennav-startup.state.retry-*')),'Retry evidence not retained'
    report['chart_rendering'].append(chart.check(capture('recovery-03-retry'),colors,'Human-requested XNav retry'))
    if windows:
        process=ui.monitor_process(pid);ui.close(handle);ui.wait_clean_exit(process)
    else:
        subprocess.run([str(exe),'--configdir',str(profile),'--remote','--quit'],env=env,check=True,timeout=15)
        child,status=os.waitpid(pid,0);assert os.waitstatus_to_exitcode(status)==0
    owned.discard(pid)
    assert fixtures.snapshot(profile)==expected
    state=(profile/'opennav-startup.state').read_text();assert 'failures 0' in state and 'pending 0' in state,state
    report['checks'].append('Human menu retry resets guard with saved evidence; clean close and navigation/config/plugin persistence passed')
    report['result']='passed; native screenshot review required'
except Exception as error:
    report['result']='failed';report['error']=repr(error)
    if windows:
        report['visible_windows']=[]
        for owner in owned:
            for h,process,title in ui.windows(owner):
                report['visible_windows'].append({'pid':process,'title':title})
                try:
                    name='recovery-failed-'+str(len(report['visible_windows']))+'.png'
                    ui.capture(h,evidence/name,resize=False,screen_pixels=True)
                    report['screenshots'].append(name)
                except Exception as capture_error:
                    report.setdefault('capture_errors',[]).append(repr(capture_error))
    raise
finally:
    for child in owned:
        if windows:subprocess.run(['taskkill','/PID',str(child),'/F'],capture_output=True)
        else:
            try:os.kill(child,9);os.waitpid(child,0)
            except (ProcessLookupError,ChildProcessError):pass
    if xserver:xserver.terminate();xserver.wait(timeout=10)
    (evidence/('recovery-results.json' if windows else 'recovery-linux-results.json')).write_text(json.dumps(report,indent=2))
    shutil.copytree(profile,evidence/('recovery-profile' if windows else 'recovery-linux-profile'),dirs_exist_ok=True,ignore=shutil.ignore_patterns('opencpn-ipc','*.pem'))
    temporary.cleanup()
print(report['result'])

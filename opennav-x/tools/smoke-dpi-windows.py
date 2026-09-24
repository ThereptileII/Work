#!/usr/bin/env python3
"""Actual native Windows DPI; no bitmap rescaling or fake DPI acceptance."""
import ctypes as C
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
assert sys.platform=='win32','Native Windows required'
assert os.environ.get('GITHUB_ACTIONS')=='true','Disposable GitHub desktop only'
root=Path(__file__).resolve().parents[1]
def module(name):
    s=importlib.util.spec_from_file_location(name,root/'tools'/f'{name}.py')
    m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
ui=module('windows-ui');chart=module('chart-render-check');fixtures=module('profile-fixtures')
helper=root/'build/xnav-windows/Release/opennav-test-dpi.exe'
exe=root/'build/xnav-install/opencpn.exe'
env=dict(os.environ,OPENNAV_DISPOSABLE_DESKTOP='1')
evidence=root/'evidence/local';evidence.mkdir(parents=True,exist_ok=True)
report={'authority':'native Windows monitor DPI and GetDpiForWindow','scales':[],'screenshots':[]}
report['desktop']=ui.ensure_desktop(1920,1080)
def dpi(*args):
    result=subprocess.run([str(helper),*map(str,args)],env=env,capture_output=True,text=True,timeout=15)
    if result.returncode:raise RuntimeError(result.stderr.strip())
    return json.loads(result.stdout)
original=dpi();report['original']=original
assert original['percent'] in (100,125,150), 'Unexpected desktop scale; refuse a change that cannot be restored'
temporary=tempfile.TemporaryDirectory(prefix='opennav dpi ')
profile=Path(temporary.name)/'profile'
subprocess.run([sys.executable,str(root/'tools/prepare-test-profile.py'),'--build',str(root/'build/xnav-windows'),'--profile',str(profile)],check=True)
fixtures.seed(profile);expected=fixtures.snapshot(profile)
with (profile/'opencpn.conf').open('a') as f:f.write('\n[Settings/GlobalState]\nVPLatLon=59.0800,18.5000\nVPScale=0.003\n')
app=None;handle=None;pid=None;owned=set();count=0;colors=None

def data(predicate,timeout=15):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:
            d=json.loads((profile/'opennav-diagnostics.json').read_text())
            if predicate(d):return d
        except (OSError,json.JSONDecodeError):pass
        time.sleep(.15)
    raise RuntimeError('DPI interaction diagnostic condition failed')
def ready():
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        log=profile/'opencpn.log'
        if log.exists() and log.read_text(errors='replace').count('OnInitTimer...Finalize Canvases')>=count:
            time.sleep(.7);return
        time.sleep(.2)
    raise RuntimeError('DPI startup did not finish')
def capture(name,window=None):
    path=evidence/(name+'.png');rgb=ui.capture(handle if window is None else window,path,resize=window is None)
    report['screenshots'].append(path.name);return rgb

def main_buttons(scale):
    frame=ui.W.RECT();ui.GetWindowRect(handle,C.byref(frame));sizes={}
    for label in ['Navigation','Route','Energy','Demo','Menu','System','Light','+','−','GPS']:
        found=[h for h,t in ui.children(handle) if t==label]
        assert len(found)==1,(label,found)
        r=ui.W.RECT();assert ui.GetWindowRect(found[0],C.byref(r))
        assert r.bottom-r.top>=44*scale/100,(label,'touch height',r.bottom-r.top)
        assert frame.left<=r.left<r.right<=frame.right and frame.top<=r.top<r.bottom<=frame.bottom,(label,'clipped control')
        sizes[label]=[r.right-r.left,r.bottom-r.top]
    return sizes

def close_current():
    global app
    process=ui.monitor_process(pid);ui.close(handle);ui.wait_clean_exit(process);owned.discard(pid)
    if app and app.pid==pid:assert app.wait(timeout=5)==0
    app=None
try:
    for scale in (100,125,150):
        applied=dpi(scale);assert applied['percent']==scale;time.sleep(1)
        with (evidence/'dpi-launch.log').open('a') as out:
            app=subprocess.Popen([str(exe),'--configdir',str(profile),'--no_opengl','--xnav','--xnav-demo'],env=env,stdout=out,stderr=out)
        count+=1;owned.add(app.pid);handle,pid=ui.wait_window('OpenNav X / OpenCPN',app.pid);ready();ui.size_window(handle)
        observed=ui.GetDpiForWindow(handle);assert observed==96*scale//100,(scale,observed,'Actual application DPI must match request')
        d=data(lambda d:d['data_mode']=='DEMO' and any(f'DPI: {observed}' in s for s in d['build_info']))
        entry={'percent':scale,'GetDpiForWindow':observed,'wxDpi':observed,'buttons':main_buttons(scale),'chart_rendering':[]}
        rgb=capture(f'dpi-{scale}-01-navigation-day')
        if colors is None:colors=chart.reference(rgb)
        entry['chart_rendering'].append(chart.check(rgb,colors,f'{scale}% Day'))
        ui.click_text(pid,'Light');ui.click_text(pid,'Light');capture(f'dpi-{scale}-02-navigation-night');ui.click_text(pid,'Light')
        for label,page in [('Route','Route'),('Energy','Energy')]:
            ui.click_text(pid,label);data(lambda d:d['ui_page']==page);ui.assert_preview_page(handle,page);capture(f'dpi-{scale}-{page.lower()}')
        ui.click_text(pid,'Menu');ui.click_text(pid,'Vessel instruments');data(lambda d:d['ui_page']=='Vessel instruments');ui.assert_product_page(handle,'Vessel instruments');capture(f'dpi-{scale}-instruments')
        ui.click_text(pid,'Menu');ui.click_text(pid,'Settings');ui.click_text(pid,'Energy configuration')
        data(lambda d:d['ui_page']=='Energy configuration');capture(f'dpi-{scale}-settings')
        ui.click_text(pid,'Configure battery & reserve');dialog,_=ui.wait_window('Battery assumptions',pid)
        r=ui.W.RECT();f=ui.W.RECT();ui.GetWindowRect(dialog,C.byref(r));ui.GetWindowRect(handle,C.byref(f))
        assert f.left<=r.left<r.right<=f.right and f.top<=r.top<r.bottom<=f.bottom,'DPI sheet clipped outside window'
        capture(f'dpi-{scale}-sheet',dialog);ui.click_text(pid,'Cancel')
        ui.click_text(pid,'Navigation');data(lambda d:d['ui_page']=='Navigation')
        # A real Windows touch-injection sequence, checked through resulting UI state.
        menu=[h for h,t in ui.children(handle) if t=='Menu'];assert len(menu)==1
        rect=ui.W.RECT();ui.GetWindowRect(menu[0],C.byref(rect))
        ui.SetForegroundWindow(handle)
        touch=dpi('--tap',(rect.left+rect.right)//2,(rect.top+rect.bottom)//2)
        assert touch['touch_injected'];data(lambda d:d['ui_page']=='Menu')
        entry['touch']='Injected native down/up on Menu changed the page; physical touch remains untested'
        ui.click_text(pid,'System');ui.click_text(pid,'Open Legacy OpenCPN')
        assert app.wait(timeout=30)==0;owned.discard(pid);count+=1
        handle,pid=ui.wait_window('OpenCPN / Legacy');owned.add(pid);ready();ui.size_window(handle)
        assert ui.GetDpiForWindow(handle)==observed
        entry['chart_rendering'].append(chart.check(capture(f'dpi-{scale}-legacy'),colors,f'{scale}% Legacy'))
        process=ui.monitor_process(pid);ui.click_menu(handle,'Switch to XNav');ui.wait_clean_exit(process);owned.discard(pid);count+=1
        handle,pid=ui.wait_window('OpenNav X / OpenCPN');owned.add(pid);ready();ui.size_window(handle)
        assert ui.GetDpiForWindow(handle)==observed
        entry['chart_rendering'].append(chart.check(capture(f'dpi-{scale}-returned-xnav'),colors,f'{scale}% returned XNav'))
        old=ui.monitor_process(pid);ui.click_text(pid,'System');ui.click_text(pid,'Safe Mode')
        ui.wait_clean_exit(old);owned.discard(pid);count+=1
        handle,pid=ui.wait_window('OpenNav Safe Mode / OpenCPN');owned.add(pid);ready();ui.size_window(handle)
        assert ui.GetDpiForWindow(handle)==observed
        entry['chart_rendering'].append(chart.check(capture(f'dpi-{scale}-safe'),colors,f'{scale}% Safe'))
        old=ui.monitor_process(pid);ui.click_menu(handle,'Switch to XNav');ui.wait_clean_exit(old);owned.discard(pid);count+=1
        handle,pid=ui.wait_window('OpenNav X / OpenCPN');owned.add(pid);ready();ui.size_window(handle)
        assert ui.GetDpiForWindow(handle)==observed
        entry['chart_rendering'].append(chart.check(capture(f'dpi-{scale}-safe-to-xnav'),colors,f'{scale}% Safe to XNav'))
        close_current();assert fixtures.snapshot(profile)==expected
        report['scales'].append(entry)
    report['result']='passed; native visual review required'
finally:
    for p in owned:subprocess.run(['taskkill','/PID',str(p),'/F'],capture_output=True)
    report['restored']=dpi(original['percent'])
    (evidence/'dpi-results.json').write_text(json.dumps(report,indent=2))
    shutil.copytree(profile,evidence/'dpi-profile',dirs_exist_ok=True,ignore=shutil.ignore_patterns('opencpn-ipc','*.pem'))
    temporary.cleanup()
print(report['result'])

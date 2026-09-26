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

def data(predicate=lambda d:True,timeout=15):
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
    light=data()['runtime']['display']['light']
    for label in ['Navigation','Route','Energy','Pilot','STBY','Demo','Menu','System',light,'+','−','Center']:
        found=[h for h,t in ui.children(handle) if t==label]
        assert len(found)==1,(label,found)
        r=ui.W.RECT();assert ui.GetWindowRect(found[0],C.byref(r))
        assert r.bottom-r.top>=48*scale/100,(label,'touch height',r.bottom-r.top)
        assert frame.left<=r.left<r.right<=frame.right and frame.top<=r.top<r.bottom<=frame.bottom,(label,'clipped control')
        sizes[label]=[r.right-r.left,r.bottom-r.top]
    return sizes

def bounds(window):
    rect=ui.W.RECT();assert ui.GetWindowRect(window,C.byref(rect));return rect

def chrome_bounds():
    labels=ui.children(handle)
    brand=[h for h,t in labels if t=='OpenNav X']
    navigation=[h for h,t in labels if t=='Navigation']
    assert len(brand)==len(navigation)==1
    return bounds(ui.GetParent(brand[0])).bottom,bounds(ui.GetParent(navigation[0])).top

def rail_geometry(scale):
    d=data(lambda d:len(d['runtime']['display']['rail_regions'])==4)
    regions=d['runtime']['display']['rail_regions'];frame=bounds(handle)
    top,bottom=chrome_bounds();previous=top
    for region in regions:
        x,y,w,h=(region[k] for k in ('x','y','width','height'))
        assert region['visible'],('A primary rail value is clipped',region)
        assert frame.left<=x<x+w<=frame.right and previous<=y<y+h<=bottom,region
        assert h>=80*scale/100,('Primary value too small',region)
        previous=y+h
    return [{key:region[key] for key in ('label','x','y','width','height','visible')} for region in regions]

def critical_alert_accessible(scale):
    data(lambda d:any(a['level']=='CRITICAL' for a in d['runtime']['alerts']))
    labels=ui.children(handle)
    alert=[h for h,t in labels if t.startswith('Alerts ') and not t.startswith('Alerts /')]
    titles=[t for _,t in labels if t.startswith(('CRITICAL /','DEMO / CRITICAL /'))]
    assert len(alert)==1 and titles,('Critical alert and its action must remain visible',labels)
    area=bounds(alert[0]);top,_=chrome_bounds()
    assert area.bottom<=top and area.bottom-area.top>=48*scale/100,'Alert action clipped'
    ui.SetForegroundWindow(handle)
    hit=ui.WindowFromPoint(ui.W.POINT((area.left+area.right)//2,(area.top+area.bottom)//2))
    while hit and hit!=alert[0]:hit=ui.GetParent(hit)
    assert hit==alert[0],'A page or popup obscures the critical alert action'
    return {'title':titles[0],'accessible':True,'bounds':[area.left,area.top,area.right,area.bottom]}

def instrument_geometry():
    display=data()['runtime']['display'];regions=display['product_regions']
    frame=bounds(handle);top,bottom=chrome_bounds()
    assert regions and regions[0]['visible'],'First instrument group must be fully visible'
    for region in regions:
        x,y,w,h=(region[k] for k in ('x','y','width','height'))
        assert frame.left<=x<x+w<=frame.right and w>0 and h>0,region
        if region['visible']:assert top<=y<y+h<=bottom,region
    assert display['minimum_value_height_dip']>=120
    return regions

def system_geometry(scale):
    required={'Open Legacy OpenCPN','Restart XNav','Safe Mode','Diagnostics',
              'Open diagnostics folder','Commissioning & recordings',
              'Export diagnostic bundle','Advanced / Legacy Settings'}
    seen=set();checked=[]
    for _ in range(30):
        d=data(lambda d:d['ui_page']=='System');display=d['runtime']['display']
        frame=bounds(handle);top,bottom=chrome_bounds()
        for control in display['product_controls']:
            if control['label'] not in required:continue
            x,y,w,h=(control[k] for k in ('x','y','width','height'))
            assert frame.left<=x<x+w<=frame.right and h>=48*scale/100,control
            if control['visible']:
                assert top<=y<y+h<=bottom,('System action overlaps global bars',control)
                seen.add(control['label']);checked.append(control)
        critical_alert_accessible(scale)
        if seen==required:break
        assert display['can_scroll_down'],('System actions inaccessible',required-seen)
        previous=display['page_scroll_px'];ui.click_text(pid,'Down')
        data(lambda d:d['runtime']['display']['page_scroll_px']>previous)
    assert seen==required,('System actions missing',required-seen)
    for _ in range(30):
        if not data()['runtime']['display']['can_scroll_up']:break
        previous=data()['runtime']['display']['page_scroll_px'];ui.click_text(pid,'Up')
        data(lambda d:d['runtime']['display']['page_scroll_px']<previous)
    return checked

def chart_context_geometry(scale):
    display=data()['runtime']['display'];chart_area=display['chart_region']
    assert ui.SetCursorPos(chart_area['x']+chart_area['width']//2,
                           chart_area['y']+chart_area['height']//2)
    ui.MouseEvent(8,0,0,0,0);time.sleep(.08);ui.MouseEvent(16,0,0,0,0)
    labels={'Go to','Waypoint','Measure','Info'}
    def controls(record):
        return [c for c in record['runtime']['display'].get('interaction_controls',[])
                if c['visible'] and c['label'] in labels]
    record=data(lambda d:len(controls(d))==4)
    assert record['ui_page']=='Navigation','Context replaced the chart'
    checked=controls(record)
    for c in checked:
        assert chart_area['x']<=c['x']<c['x']+c['width']<=chart_area['x']+chart_area['width'],c
        assert chart_area['y']<=c['y']<c['y']+c['height']<=chart_area['y']+chart_area['height'],c
        assert c['height']>=48*scale/100,c
    assert not next(c['enabled'] for c in checked if c['label']=='Go to'),'GPS-free Go To must be disabled'
    assert next(c['enabled'] for c in checked if c['label']=='Waypoint'),'Chart waypoint creation requires no vessel fix'
    path=evidence/f'dpi-{scale}-chart-context.png';ui.capture(handle,path,screen_pixels=True)
    report['screenshots'].append(path.name)
    close=[c for c in record['runtime']['display']['interaction_controls']
           if c['visible'] and c['label']=='Close']
    assert len(close)==1,close
    c=close[0]
    assert dpi('--tap',c['x']+c['width']//2,c['y']+c['height']//2)['touch_injected']
    data(lambda d:not controls(d))
    return {'controls':checked,'touch_close':'Native injected touch dismissed modeless card'}

def close_current():
    global app
    process=ui.monitor_process(pid);ui.close(handle);ui.wait_clean_exit(process);owned.discard(pid)
    if app and app.pid==pid:assert app.wait(timeout=5)==0
    app=None
try:
    for scale in (100,125,150):
        applied=dpi(scale);assert applied['percent']==scale;time.sleep(1)
        with (evidence/'dpi-launch.log').open('a') as out:
            app=subprocess.Popen([str(exe),'--configdir',str(profile),'--no_opengl','--xnav'],env=env,stdout=out,stderr=out)
        count+=1;owned.add(app.pid);handle,pid=ui.wait_window('OpenNav X / OpenCPN',app.pid);ready();ui.size_window(handle)
        observed=ui.GetDpiForWindow(handle);assert observed==96*scale//100,(scale,observed,'Actual application DPI must match request')
        d=data(lambda d:d['data_mode']=='OPENCPN selected navigation' and any(f'DPI: {observed}' in s for s in d['build_info']))
        entry={'percent':scale,'GetDpiForWindow':observed,'wxDpi':observed,'buttons':main_buttons(scale),'chart_rendering':[]}
        assert not d['runtime']['alerts'],'Clean isolated input-free startup must have no inherited alert'
        entry['rail_without_alert']=rail_geometry(scale)
        capture(f'dpi-{scale}-00-navigation-no-input')
        entry['chart_context']=chart_context_geometry(scale)
        ui.click_text(pid,'Demo');ui.click_text(pid,'Cruising')
        data(lambda d:d['data_mode']=='DEMO' and any(a['level']=='CRITICAL' for a in d['runtime']['alerts']))
        entry['rail_with_critical_alert']=rail_geometry(scale)
        assert entry['rail_without_alert']==entry['rail_with_critical_alert'],'Alert changed or hid a primary rail value'
        entry['critical_alert']=critical_alert_accessible(scale)
        rgb=capture(f'dpi-{scale}-01-navigation-day')
        if colors is None:colors=chart.reference(rgb)
        entry['chart_rendering'].append(chart.check(rgb,colors,f'{scale}% Day'))
        ui.cycle_light(pid);data(lambda d:d['runtime']['display']['light']=='Dusk')
        capture(f'dpi-{scale}-navigation-dusk')
        ui.cycle_light(pid)
        data(lambda d:d['runtime']['display']['light']=='Night')
        night=capture(f'dpi-{scale}-02-navigation-night')
        entry['chart_rendering'].append(chart.night(night,colors,f'{scale}% Night'))
        entry['night_surfaces']=[chart.dark_surface(night,f'{scale}% Night navigation')]
        entry['native_caption_themed']=data()['runtime']['display']['native_caption_themed']
        # Reaching the endpoint must not focus a hidden first child and jump
        # back up. Require the final menu action fully visible after settling.
        ui.click_text(pid,'Menu');data(lambda d:d['ui_page']=='Menu')
        for _ in range(40):
            if not data()['runtime']['display']['can_scroll_down']:break
            ui.click_text(pid,'Down')
        else:raise AssertionError('Menu cannot retain its bottom scroll endpoint')
        bottom=data()['runtime']['display']['page_scroll_px'];time.sleep(1.2)
        assert data()['runtime']['display']['page_scroll_px']==bottom,'Menu jumped after disabling Down'
        last=[h for h,t in ui.children(handle) if t=='System & diagnostics'];assert len(last)==1
        area=ui.W.RECT();item=ui.W.RECT()
        ui.GetWindowRect(ui.GetParent(last[0]),C.byref(area));ui.GetWindowRect(last[0],C.byref(item))
        assert area.top<=item.top<item.bottom<=area.bottom,'Final menu action clipped at scroll endpoint'
        capture(f'dpi-{scale}-menu-bottom')
        entry['menu_endpoint']='Last action fully visible; settled endpoint retained after Down disables'
        # Review every main workflow at each scale in the actual night palette.
        for label,page in [('Route','Route'),('Energy','Energy'),('Pilot','Manual autopilot')]:
            ui.click_text(pid,label);data(lambda d:d['ui_page']==page)
            entry['night_surfaces'].append(chart.dark_surface(capture(f'dpi-{scale}-night-{label.lower()}'),page))
        for label,page in [('Vessel instruments','Vessel instruments'),('AIS targets','AIS targets'),
                           ('SmartNav advisories','SmartNav'),('Anchor watch','Anchor watch'),
                           ('Settings','Settings')]:
            ui.click_text(pid,'Menu');ui.click_text(pid,label);data(lambda d:d['ui_page']==page)
            entry['night_surfaces'].append(chart.dark_surface(capture(f'dpi-{scale}-night-'+page.lower().replace(' ','-').replace('&','and')),page))
        for label,page in [('Commissioning & recordings','Commissioning & recordings'),
                           ('Export diagnostic bundle','Field diagnostic bundle')]:
            ui.click_text(pid,'System');ui.click_text(pid,label);data(lambda d:d['ui_page']==page)
            entry['night_surfaces'].append(chart.dark_surface(capture(f'dpi-{scale}-night-'+page.lower().replace(' ','-').replace('&','and')),page))
        ui.click_text(pid,'System');ui.click_text(pid,'Diagnostics');data(lambda d:d['ui_page']=='Diagnostics')
        entry['night_surfaces'].append(chart.dark_surface(capture(f'dpi-{scale}-night-diagnostics'),'Diagnostics'))
        # The data-heavy diagnostics page must be navigable by actual touch.
        assert data()['runtime']['display']['can_scroll_down']
        def touch_label(label):
            matches=[h for h,t in ui.children(handle) if t==label];assert len(matches)==1,(label,matches)
            rect=ui.W.RECT();ui.GetWindowRect(matches[0],C.byref(rect));ui.SetForegroundWindow(handle)
            assert dpi('--tap',(rect.left+rect.right)//2,(rect.top+rect.bottom)//2)['touch_injected']
        touch_label('Down');data(lambda d:d['runtime']['display']['page_scroll_px']>0)
        touch_label('Up');data(lambda d:d['runtime']['display']['page_scroll_px']==0)
        ui.SetForegroundWindow(handle)
        # Painted diagnostics content is a direct, unambiguous pan surface.
        assert dpi('--pan',650,550,650,300)['touch_injected']
        data(lambda d:d['runtime']['display']['page_scroll_px']>0)
        capture(f'dpi-{scale}-touch-scrolled-diagnostics')
        entry['touch_scroll']='Native touch Up/Down and vertical pan move the actual diagnostics viewport'
        ui.cycle_light(pid)
        for label,page in [('Route','Route'),('Energy','Energy')]:
            ui.click_text(pid,label);data(lambda d:d['ui_page']==page);ui.assert_preview_page(handle,page);capture(f'dpi-{scale}-{page.lower()}')
        ui.click_text(pid,'Menu');ui.click_text(pid,'Vessel instruments');data(lambda d:d['ui_page']=='Vessel instruments' and d['runtime']['display']['minimum_value_height_dip']>=120);ui.assert_product_page(handle,'Vessel instruments');entry['instrument_groups']=instrument_geometry();capture(f'dpi-{scale}-instruments')
        ui.click_text(pid,'Menu');ui.click_text(pid,'Settings');ui.click_text(pid,'VESSEL');ui.click_text(pid,'Energy configuration')
        data(lambda d:d['ui_page']=='Energy configuration');capture(f'dpi-{scale}-settings')
        ui.cycle_light(pid);ui.cycle_light(pid);data(lambda d:d['runtime']['display']['light']=='Night')
        ui.click_text(pid,'Configure battery & reserve');dialog,_=ui.wait_window('Battery assumptions',pid)
        r=ui.W.RECT();f=ui.W.RECT();ui.GetWindowRect(dialog,C.byref(r));ui.GetWindowRect(handle,C.byref(f))
        assert f.left<=r.left<r.right<=f.right and f.top<=r.top<r.bottom<=f.bottom,'DPI sheet clipped outside window'
        top,bottom=chrome_bounds()
        assert r.top>=top and r.bottom<=bottom,'Sheet overlaps global alerts/navigation'
        capture(f'dpi-{scale}-night-sheet',dialog);ui.click_text(pid,'Cancel');ui.cycle_light(pid)
        ui.click_text(pid,'Navigation');data(lambda d:d['ui_page']=='Navigation')
        # A real Windows touch-injection sequence, checked through resulting UI state.
        menu=[h for h,t in ui.children(handle) if t=='Menu'];assert len(menu)==1
        rect=ui.W.RECT();ui.GetWindowRect(menu[0],C.byref(rect))
        ui.SetForegroundWindow(handle)
        touch=dpi('--tap',(rect.left+rect.right)//2,(rect.top+rect.bottom)//2)
        assert touch['touch_injected'];data(lambda d:d['ui_page']=='Menu')
        entry['touch']='Injected native down/up on Menu changed the page; physical touch remains untested'
        ui.click_text(pid,'Settings');ui.click_text(pid,'DISPLAY')
        ui.click_text(pid,'Fullscreen / window');time.sleep(.7)
        full=ui.W.RECT();ui.GetWindowRect(handle,C.byref(full))
        assert (full.left,full.top,full.right,full.bottom)==(0,0,1920,1080),'Fullscreen did not cover the disposable desktop'
        path=evidence/f'dpi-{scale}-fullscreen.png';ui.capture(handle,path,resize=False)
        report['screenshots'].append(path.name)
        assert ui.GetDpiForWindow(handle)==observed
        ui.click_text(pid,'Fullscreen / window');time.sleep(.7);ui.size_window(handle)
        ui.click_text(pid,'Navigation');data(lambda d:d['ui_page']=='Navigation')
        entry['restored_buttons']=main_buttons(scale)
        entry['restored_rail']=rail_geometry(scale)
        assert entry['restored_rail']==entry['rail_with_critical_alert'],'Fullscreen return changed primary rail visibility'
        entry['fullscreen']='1920x1080 physical desktop; returned to 1280x800 with original DPI and controls'
        ui.click_text(pid,'System')
        data(lambda d:d['ui_page']=='System');ui.assert_product_page(handle,'System')
        entry['system_controls']=system_geometry(scale)
        entry['system_critical_alert']=critical_alert_accessible(scale)
        path=evidence/f'dpi-{scale}-system-page.png';ui.capture(handle,path,screen_pixels=True)
        report['screenshots'].append(path.name)
        ui.click_text(pid,'Open Legacy OpenCPN')
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
except Exception as error:
    report['result']='failed';report['error']=repr(error)
    try:
        capture(f'dpi-{scale}-failure')
        report['failure_diagnostics']=data(timeout=2)
        geometry=[]
        for control,label in ui.children(handle):
            rect=ui.W.RECT();ui.GetWindowRect(control,C.byref(rect))
            geometry.append({'label':label,'rect':[rect.left,rect.top,rect.right,rect.bottom],
                             'enabled':bool(ui.IsWindowEnabled(control))})
        report['failure_controls']=geometry
    except Exception as capture_error:report['capture_error']=repr(capture_error)
    raise
finally:
    for p in owned:subprocess.run(['taskkill','/PID',str(p),'/F'],capture_output=True)
    report['restored']=dpi(original['percent'])
    (evidence/'dpi-results.json').write_text(json.dumps(report,indent=2))
    shutil.copytree(profile,evidence/'dpi-profile',dirs_exist_ok=True,ignore=shutil.ignore_patterns('opencpn-ipc','*.pem'))
    temporary.cleanup()
print(report['result'])

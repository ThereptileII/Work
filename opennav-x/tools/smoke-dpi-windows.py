#!/usr/bin/env python3
"""Actual native Windows DPI; no bitmap rescaling or fake DPI acceptance."""
import ctypes as C
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
from diagnostic_snapshot import read_json_snapshot
assert sys.platform=='win32','Native Windows required'
assert os.environ.get('GITHUB_ACTIONS')=='true','Disposable GitHub desktop only'
root=Path(__file__).resolve().parents[1]
def module(name):
    s=importlib.util.spec_from_file_location(name,root/'tools'/f'{name}.py')
    m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
ui=module('windows-ui');chart=module('chart-render-check');fixtures=module('profile-fixtures')
geometry_observation=module('diagnostic-geometry')
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
            d=read_json_snapshot(profile/'opennav-diagnostics.json')
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
    path=evidence/(name+'.png');rgb=ui.capture(handle if window is None else window,path,resize=False,screen_pixels=True)
    report['screenshots'].append(path.name);return rgb

def main_buttons(scale):
    frame=ui.W.RECT();ui.GetWindowRect(handle,C.byref(frame));sizes={}
    display=current_layout_observation()['runtime']['display'];light=display['light']
    client=ui.W.RECT();assert ui.GetClientRect(handle,C.byref(client))
    logical_height=client.bottom*100/scale
    nav=43 if logical_height<=600 else 51 if logical_height<=740 else 61
    pilot=66 if logical_height<=600 else 74 if logical_height<=740 else 87
    heights={label:nav for label in ('Chart','Passage','Traffic','Energy','Instruments','Anchor','Radar','Settings')}
    heights.update({'Autopilot':pilot,light:44,'+':44,'−':44,'Follow boat':44})
    for label,height in heights.items():
        found=[c for c in display['interaction_controls'] if c['label']==label and c['visible']]
        assert len(found)==1,(label,found)
        c=found[0];x,y,w,h=[c[k] for k in ('x','y','width','height')]
        assert abs(h-height*scale/100)<=1,(label,'exact prototype height',h,height)
        assert frame.left<=x<x+w<=frame.right and frame.top<=y<y+h<=frame.bottom,(label,'clipped control')
        sizes[label]=[w,h]
    footer=display['footer_region']
    footer_handles=[h for h,t in ui.children(handle) if t=='OpenNav status footer']
    assert len(footer_handles)==1
    native=bounds(footer_handles[0])
    assert footer==dict(x=native.left,y=native.top,width=native.right-native.left,height=native.bottom-native.top)
    assert abs(footer['height']-34*scale/100)<=1 and footer['width']==client.right, 'Exact full-width prototype footer required'
    assert display['footer_middle_visible']==(client.right*100/scale>1100), 'Middle footer must follow exact prototype breakpoint'
    health=[c for c in display['interaction_controls'] if c['label']=='Source health' and c['visible']]
    assert len(health)==1 and health[0]['enabled']
    h=health[0]
    assert native.left<=h['x']<h['x']+h['width']<=native.right and native.top<=h['y']<h['y']+h['height']<=native.bottom, 'Footer source health must remain visible at this DPI'
    assert not any(c['label']=='System' and c['visible'] for c in display['interaction_controls']), 'Obsolete footer System action remains'
    sizes['footer']=[footer['width'],footer['height']]
    return sizes

def bounds(window):
    rect=ui.W.RECT();assert ui.GetWindowRect(window,C.byref(rect));return rect

def primary_hint_hover(label, should_show, settle=1.5):
    """Hover actual controls, observe native tooltip HWNDs and visible pixels."""
    d=data()
    if label=='Settings':
        found=[h for h,t in ui.children(handle) if t==label];assert len(found)==1
        r=bounds(found[0]);point=ui.W.POINT((r.left+r.right)//2,(r.top+r.bottom)//2)
    else:
        found=[r for r in d['runtime']['display']['rail_regions'] if r['label']=='sog' and r['visible']]
        assert label=='SOG' and len(found)==1
        r=found[0];point=ui.W.POINT(r['x']+r['width']//2,r['y']+r['height']//2)
    foreground=ui.declare(ui.user,'GetForegroundWindow',ui.W.HWND)
    ui.SetForegroundWindow(handle);assert foreground()==handle
    # Enter from the chart, not from another help window or a retained hover.
    chart_rect=d['runtime']['display']['chart_region']
    assert ui.SetCursorPos(chart_rect['x']+40,chart_rect['y']+40)
    time.sleep(.15)
    hit=ui.WindowFromPoint(point);owner=ui.W.DWORD()
    ui.GetWindowThreadProcessId(hit,C.byref(owner));assert owner.value==pid
    ancestor=hit
    while ancestor and ancestor!=handle:ancestor=ui.GetParent(ancestor)
    assert ancestor==handle,'Hover target belongs to another top-level window'
    if label=='Settings':assert hit==found[0]
    assert ui.SetCursorPos(point.x,point.y)
    def native_hints():
        hints=[]
        for h,_,text in ui.windows(pid):
            name=C.create_unicode_buffer(256);ui.GetClassNameW(h,name,len(name))
            if name.value=='tooltips_class32':hints.append({'handle':int(h),'text':text})
        return hints
    started=time.monotonic();observed=[]
    deadline=started+(5 if should_show else settle)
    while time.monotonic()<deadline:
        assert foreground()==handle,'Another window interrupted the hover observation'
        observed=native_hints()
        if should_show and observed:break
        assert should_show or not observed,(label,'Native tooltip visible in dim palette',observed)
        time.sleep(.1)
    assert bool(observed)==should_show,(label,'Day hover must prove the native tooltip path works')
    name=f"dpi-{scale}-{d['runtime']['display']['light'].lower()}-{label.lower()}-hover.png"
    rgb=ui.capture(handle,evidence/name,resize=False,screen_pixels=True)
    report['screenshots'].append(name)
    assert foreground()==handle and bool(native_hints())==should_show
    result={'label':label,'palette':d['runtime']['display']['light'],
            'native_tooltip_visible':bool(observed),'observed_seconds':round(time.monotonic()-started,3),
            'screen_capture':name}
    if not should_show:result['surface']=chart.dark_surface(rgb,result['palette']+' '+label+' hover')
    return result

def chrome_bounds():
    labels=ui.children(handle)
    alerts=[h for h,t in labels if t=='Alerts' or t.startswith('Alerts ')]
    footer=[h for h,t in labels if t=='OpenNav status footer']
    assert len(alerts)==len(footer)==1
    return bounds(ui.GetParent(alerts[0])).bottom,bounds(footer[0]).top

def current_layout_observation():
    # Diagnostics publishes at 1 Hz; size_window's native 0.5 s settle can still
    # leave a pre-resize observation on disk. Pair its geometry with the actual
    # HWND chrome, not with a fixed sleep or a predicate that waits for the rail
    # to fit (which could conceal real clipping).
    previous=int(data()['runtime']['ui_update']['ticks'])
    def native_controls():
        children=ui.children(handle);result={}
        for label in ('Settings','Chart','Source health'):
            found=[control for control,text in children if text==label]
            assert len(found)==1,(label,found)
            rect=bounds(found[0])
            result[label]=(rect.left,rect.top,rect.right,rect.bottom)
        return result
    expected=native_controls()
    observed=data(lambda d:geometry_observation.matches_native_controls(d,expected,previous))
    assert native_controls()==expected,'Native frame moved while pairing diagnostic geometry'
    return observed

def preferences_observation(label='Chart safety depth'):
    # A pan can finish before the 1Hz diagnostic publication. Pair the actual
    # row position even while clipped; never poll for visibility or good layout.
    previous=int(data()['runtime']['ui_update']['ticks'])
    popup,_=ui.wait_window('OpenNav preferences',pid)
    matches=[h for h,t in ui.children(popup) if t==label]
    assert len(matches)==1, ('One native Preferences action required',label)
    target=matches[0]
    rect=bounds(target);expected={label:(rect.left,rect.top,rect.right,rect.bottom)}
    observed=data(lambda d:geometry_observation.matches_native_controls(d,expected,previous,require_visible=False))
    actual=bounds(target)
    assert (actual.left,actual.top,actual.right,actual.bottom)==expected[label], 'Native Preferences moved while pairing its published geometry'
    return observed,target

def touch_preferences_action(label,scale):
    """Reach a real drawer action before tapping; never message a clipped HWND."""
    popup,_=ui.wait_window('OpenNav preferences',pid)
    foreground=ui.declare(ui.user,'GetForegroundWindow',ui.W.HWND)
    ui.SetForegroundWindow(popup)
    attempts=[];last_rect=None
    for _ in range(40):
        assert foreground()==popup,'Another window interrupted the Preferences gesture'
        observed,target=preferences_observation(label)
        controls=observed['runtime']['display']['interaction_controls']
        found=[c for c in controls if c['label']==label]
        assert len(found)==1,(label,'Missing or duplicate paired action')
        control=found[0]
        assert control['enabled'] and ui.IsWindowEnabled(target),(label,'Preferences action disabled')
        rect=bounds(target);native=(rect.left,rect.top,rect.right,rect.bottom)
        assert native==(control['x'],control['y'],control['x']+control['width'],control['y']+control['height']), 'Preferences moved after its observation'
        body=ui.GetParent(target)
        assert ui.GetParent(body)==popup,'Preferences target is not in this drawer body'
        viewport=bounds(body)
        assert viewport.left<=rect.left<rect.right<=viewport.right,(label,'Action horizontally clipped')
        visible=viewport.top<=rect.top<rect.bottom<=viewport.bottom
        attempts.append({'native_bounds':list(native),'visible':control['visible'],
                         'tick':observed['runtime']['ui_update']['ticks']})
        assert bool(control['visible'])==visible,(label,'Native and diagnostic visibility disagree')
        if visible:break
        assert native!=last_rect,(label,'Touch pan did not move the clipped action')
        last_rect=native
        padding=round(24*scale/100);distance=round(160*scale/100)
        x=(viewport.left+viewport.right)//2
        below=rect.bottom>viewport.bottom
        start=viewport.bottom-padding if below else viewport.top+padding
        end=max(viewport.top+padding,start-distance) if below else min(viewport.bottom-padding,start+distance)
        assert start!=end,(label,'No usable Preferences pan area')
        hit=ui.WindowFromPoint(ui.W.POINT(x,start))
        assert hit==body or ui.IsChild(body,hit),'Preferences pan would touch another surface'
        assert dpi('--pan',x,start,x,end)['touch_injected']
        time.sleep(.4)
    else:raise AssertionError(label+': Preferences action cannot be reached by bounded touch scroll')
    assert foreground()==popup and ui.IsWindowEnabled(target)
    current=bounds(target)
    assert (current.left,current.top,current.right,current.bottom)==native,'Preferences moved before the tap'
    point=ui.W.POINT((rect.left+rect.right)//2,(rect.top+rect.bottom)//2)
    assert ui.WindowFromPoint(point)==target,'Another surface covers the Preferences action'
    result=dpi('--tap',point.x,point.y)
    assert result['touch_injected']
    return {'label':label,'observations':attempts,'fully_visible':True,
            'native_hit_target_verified':True,'tap':[point.x,point.y],
            'touch_injected':True}

def rail_geometry(scale):
    d=current_layout_observation()
    regions=d['runtime']['display']['rail_regions'];frame=bounds(handle)
    assert len(regions)==4,('Four primary rail values required',regions)
    client=ui.W.RECT();assert ui.GetClientRect(handle,C.byref(client))
    logical_width=client.right*100/scale;logical_height=client.bottom*100/scale
    # The immutable prototype has an explicit compact desktop override:
    # @media(max-height:600px) and (min-width:761px) .data-rail .metric
    # min-height:72px. The former unconditional 80px oracle contradicted this
    # at native150%, where a 72-DIP row is108 physical pixels. This is a measured
    # CSS constraint, not a relaxed clipping or touch-target tolerance.
    original=(root/'docs/design/prototype/index.html').read_bytes()
    assert hashlib.sha256(original).hexdigest()=='b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447'
    compact=logical_width>=761 and logical_height<=600
    minimum=72 if compact else 80
    report.setdefault('rail_oracle',[]).append(dict(percent=scale,
        logical_client=[logical_width,logical_height],minimum_height_dip=minimum,
        source='Immutable prototype compact desktop CSS' if compact else 'Retained normal-workspace minimum'))
    top,bottom=chrome_bounds();previous=top
    for region in regions:
        x,y,w,h=(region[k] for k in ('x','y','width','height'))
        assert region['visible'],('A primary rail value is clipped',region)
        assert frame.left<=x<x+w<=frame.right and previous<=y<y+h<=bottom,region
        assert h>=minimum*scale/100,('Primary value too small',region,minimum)
        previous=y+h
    return [{key:region[key] for key in ('label','x','y','width','height','visible')} for region in regions]

def critical_alert_accessible(scale):
    data(lambda d:any(a['level']=='CRITICAL' for a in d['runtime']['alerts']))
    labels=ui.children(handle)
    alert=[h for h,t in labels if t.startswith('Alerts ') and not t.startswith('Alerts /')]
    titles=[t for _,t in labels if t.startswith(('CRITICAL /','DEMO / CRITICAL /'))]
    assert len(alert)==1 and titles,('Critical alert and its action must remain visible',labels)
    area=bounds(alert[0]);top,_=chrome_bounds()
    assert area.bottom<=top and abs(area.bottom-area.top-44*scale/100)<=1,'Prototype alert action clipped'
    ui.SetForegroundWindow(handle)
    hit=ui.WindowFromPoint(ui.W.POINT((area.left+area.right)//2,(area.top+area.bottom)//2))
    while hit and hit!=alert[0]:hit=ui.GetParent(hit)
    assert hit==alert[0],'A page or popup obscures the critical alert action'
    return {'title':titles[0],'accessible':True,'bounds':[area.left,area.top,area.right,area.bottom]}

def instrument_geometry():
    display=data()['runtime']['display'];regions=display['product_regions']
    frame=bounds(handle);top,bottom=chrome_bounds()
    assert regions,'Instrument regions missing'
    if regions[0]['label']=='Wind and heading':
        # At higher DPI the actual prototype layout stacks its columns. Verify
        # scrolling reaches a fully visible reading; a clipped wind card is
        # intentional in the reference, not permission to hide numeric tiles.
        for _ in range(30):
            if any(r['visible'] for r in regions[1:]):break
            assert display['can_scroll_down'],'Instrument readings inaccessible'
            previous=display['page_scroll_px']
            # Prototype Instruments has no permanent scroll toolbar. Exercise
            # the real vertical touch gesture on its painted content instead.
            ui.SetForegroundWindow(handle)
            assert dpi('--pan',650,500,650,250)['touch_injected']
            display=data(lambda d:d['runtime']['display']['page_scroll_px']>previous)['runtime']['display']
            regions=display['product_regions']
        assert any(r['visible'] for r in regions[1:]),'No readable instrument tile reached'
    else:
        assert regions[0]['visible'],'First instrument group must be fully visible'
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
    display=current_layout_observation()['runtime']['display'];chart_area=display['chart_region']
    # Native mouse input targets the foreground desktop, unlike PrintWindow
    # and SendMessage-based controls. After installer maintenance, the first
    # chart gesture must establish and verify its actual destination. Never
    # retry a click to conceal a missing card or route input to another window.
    foreground=ui.declare(ui.user,'GetForegroundWindow',ui.W.HWND)
    ui.SetForegroundWindow(handle)
    deadline=time.monotonic()+2
    while foreground()!=handle and time.monotonic()<deadline:time.sleep(.05)
    assert foreground()==handle,'Chart context requires the actual main foreground window'
    point=ui.W.POINT(chart_area['x']+chart_area['width']//2,
                     chart_area['y']+chart_area['height']//2)
    assert ui.SetCursorPos(point.x,point.y)
    time.sleep(.15)
    hit=ui.WindowFromPoint(point);owner=ui.W.DWORD()
    ui.GetWindowThreadProcessId(hit,C.byref(owner));rect=bounds(hit)
    expected=(chart_area['x'],chart_area['y'],chart_area['x']+chart_area['width'],
              chart_area['y']+chart_area['height'])
    observed=(rect.left,rect.top,rect.right,rect.bottom)
    input_evidence={'percent':scale,'point':[point.x,point.y],
              'main_is_foreground':foreground()==handle,'target_pid':owner.value,
              'expected_pid':pid,'chart_bounds':expected,'native_hit_bounds':observed}
    report.setdefault('chart_context_inputs',[]).append(input_evidence)
    assert input_evidence['main_is_foreground'] and owner.value==pid and observed==expected, \
        ('Native chart gesture is obscured or its geometry changed',input_evidence)
    ui.MouseEvent(8,0,0,0,0);time.sleep(.08);ui.MouseEvent(16,0,0,0,0)
    labels={'Go to','Waypoint','Measure','Info'}
    def controls(record):
        return [c for c in record['runtime']['display'].get('interaction_controls',[])
                if c['visible'] and c['label'] in labels
                and c.get('accessible_name')==c['label']]
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
        ui.accelerator(handle,'T');ui.click_text(pid,'Cruising')
        data(lambda d:d['data_mode']=='DEMO' and any(a['level']=='CRITICAL' for a in d['runtime']['alerts']))
        entry['rail_with_critical_alert']=rail_geometry(scale)
        assert entry['rail_without_alert']==entry['rail_with_critical_alert'],'Alert changed or hid a primary rail value'
        entry['critical_alert']=critical_alert_accessible(scale)
        rgb=capture(f'dpi-{scale}-01-navigation-day')
        if colors is None:colors=chart.reference(rgb)
        entry['chart_rendering'].append(chart.presentation(rgb,'XNav','Day',f'{scale}% Day'))
        day_hint=primary_hint_hover('Settings',True)
        hover_settle=max(1.5,day_hint['observed_seconds']+.5)
        entry['primary_hints']=[day_hint]
        ui.cycle_light(pid);data(lambda d:d['runtime']['display']['light']=='Dusk')
        capture(f'dpi-{scale}-navigation-dusk')
        entry['primary_hints'].append(primary_hint_hover('Settings',False,hover_settle))
        ui.cycle_light(pid)
        data(lambda d:d['runtime']['display']['light']=='Night')
        night=capture(f'dpi-{scale}-02-navigation-night')
        entry['chart_rendering'].append(chart.presentation(night,'XNav','Night',f'{scale}% Night'))
        entry['night_surfaces']=[chart.dark_surface(night,f'{scale}% Night navigation')]
        for hint_label in ('Settings','SOG'):
            entry['primary_hints'].append(primary_hint_hover(hint_label,False,hover_settle))
        entry['native_caption_themed']=data()['runtime']['display']['native_caption_themed']
        # The old menu is now the prototype Preferences drawer. Preserve its
        # settled endpoint/accessibility regression with the real lower action.
        ui.click_text(pid,'Settings');data(lambda d:d['ui_page']=='Settings')
        ui.click_text(pid,'Vessel')
        for _ in range(40):
            current,_=preferences_observation();display=current['runtime']['display'];drawer=display['drawer']
            last=[c for c in display['interaction_controls'] if c['label']=='Chart safety depth' and c['visible']]
            if len(last)==1:break
            popup,_=ui.wait_window('OpenNav preferences',pid);ui.SetForegroundWindow(popup)
            x=drawer['x']+drawer['width']//2;end=drawer['y']+drawer['height']-50
            assert dpi('--pan',x,end,x,end-int(160*scale/100))['touch_injected']
            time.sleep(.4)
        else:raise AssertionError('Lower Preferences action cannot be reached by touch')
        endpoint=last[0];time.sleep(1.2)
        after,_=preferences_observation()
        report.setdefault('preferences_endpoint_observations',[]).append({'scale':scale,'before':endpoint,
            'after':[c for c in after['runtime']['display']['interaction_controls'] if c['label']=='Chart safety depth']})
        assert endpoint in after['runtime']['display']['interaction_controls'],'Preferences jumped after its lower action became visible'
        capture(f'dpi-{scale}-preferences-bottom')
        entry['menu_endpoint']='Replacement Preferences lower action fully visible and stable after native touch scroll'
        # Review every main workflow at each scale in the actual night palette.
        for label,page in [('Passage','Route'),('Energy','Energy'),('Autopilot','Manual autopilot')]:
            ui.click_text(pid,label);data(lambda d:d['ui_page']==page)
            entry['night_surfaces'].append(chart.dark_surface(capture(f'dpi-{scale}-night-{label.lower()}'),page))
        for label,page in [('Instruments','Vessel instruments'),('Traffic','AIS targets'),
                           ('SmartNav advisories','SmartNav'),('Anchor','Anchor watch'),
                           ('Settings','Settings')]:
            if page=='SmartNav':ui.accelerator(handle,'J')
            else:ui.click_text(pid,label)
            data(lambda d:d['ui_page']==page)
            entry['night_surfaces'].append(chart.dark_surface(capture(f'dpi-{scale}-night-'+page.lower().replace(' ','-').replace('&','and')),page))
        # These actions belong to the prototype Preferences drawer. Re-enter
        # it for each action: the destination page dismisses the drawer, so
        # a bare second "System" click has no visible Settings section target.
        # Visible captions changed; diagnostic destination identities did not.
        for label,page in [('Recordings & commissioning','Commissioning & recordings'),
                           ('Export diagnostics','Field diagnostic bundle')]:
            ui.click_text(pid,'Settings');data(lambda d:d['ui_page']=='Settings')
            ui.click_text(pid,'System');ui.click_text(pid,label);data(lambda d:d['ui_page']==page)
            entry['night_surfaces'].append(chart.dark_surface(capture(f'dpi-{scale}-night-'+page.lower().replace(' ','-').replace('&','and')),page))
        ui.open_system(pid);ui.click_text(pid,'Diagnostics')
        # The page identity is published before its first native paint computes
        # the scroll extent. Wait for that same page's settled geometry; the
        # later touch assertions still require actual viewport movement.
        data(lambda d:d['ui_page']=='Diagnostics' and
             d['runtime']['display']['page_scroll_px']==0 and
             d['runtime']['display']['can_scroll_down'])
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
        for label,page in [('Passage','Route'),('Energy','Energy')]:
            ui.click_text(pid,label);data(lambda d:d['ui_page']==page);ui.assert_preview_page(handle,page);capture(f'dpi-{scale}-{page.lower()}')
        ui.click_text(pid,'Instruments');data(lambda d:d['ui_page']=='Vessel instruments' and d['runtime']['display']['minimum_value_height_dip']>=120);ui.assert_product_page(handle,'Vessel instruments');entry['instrument_groups']=instrument_geometry();capture(f'dpi-{scale}-instruments')
        ui.click_text(pid,'Settings');ui.click_text(pid,'Vessel')
        entry['battery_preferences_touch']=touch_preferences_action('Battery & reserve',scale)
        data(lambda d:d['ui_page']=='Energy configuration');capture(f'dpi-{scale}-settings')
        ui.cycle_light(pid);ui.cycle_light(pid);data(lambda d:d['runtime']['display']['light']=='Night')
        ui.click_text(pid,'Configure battery & reserve');dialog,_=ui.wait_window('Battery assumptions',pid)
        r=ui.W.RECT();f=ui.W.RECT();ui.GetWindowRect(dialog,C.byref(r));ui.GetWindowRect(handle,C.byref(f))
        assert f.left<=r.left<r.right<=f.right and f.top<=r.top<r.bottom<=f.bottom,'DPI sheet clipped outside window'
        top,bottom=chrome_bounds()
        assert r.top>=top and r.bottom<=bottom,'Sheet overlaps global alerts/navigation'
        capture(f'dpi-{scale}-night-sheet',dialog);ui.click_text(pid,'Cancel');ui.cycle_light(pid)
        ui.click_text(pid,'Chart');data(lambda d:d['ui_page']=='Navigation')
        # A real Windows touch-injection sequence, checked through resulting UI state.
        menu=[h for h,t in ui.children(handle) if t=='Settings'];assert len(menu)==1
        rect=ui.W.RECT();ui.GetWindowRect(menu[0],C.byref(rect))
        ui.SetForegroundWindow(handle)
        touch=dpi('--tap',(rect.left+rect.right)//2,(rect.top+rect.bottom)//2)
        assert touch['touch_injected'];data(lambda d:d['ui_page']=='Settings')
        entry['touch']='Injected native down/up on Settings opened Preferences; physical touch remains untested'
        ui.click_text(pid,'Display')
        ui.click_text(pid,'Toggle fullscreen');time.sleep(.7)
        full=ui.W.RECT();ui.GetWindowRect(handle,C.byref(full))
        assert (full.left,full.top,full.right,full.bottom)==(0,0,1920,1080),'Fullscreen did not cover the disposable desktop'
        path=evidence/f'dpi-{scale}-fullscreen.png';ui.capture(handle,path,resize=False)
        report['screenshots'].append(path.name)
        assert ui.GetDpiForWindow(handle)==observed
        ui.click_text(pid,'Toggle fullscreen');time.sleep(.7);ui.size_window(handle)
        ui.click_text(pid,'Chart');data(lambda d:d['ui_page']=='Navigation')
        entry['restored_buttons']=main_buttons(scale)
        entry['restored_rail']=rail_geometry(scale)
        assert entry['restored_rail']==entry['rail_with_critical_alert'],'Fullscreen return changed primary rail visibility'
        entry['fullscreen']='1920x1080 physical desktop; returned to 1280x800 with original DPI and controls'
        ui.pointer_text(pid,'Settings');data(lambda d:d['ui_page']=='Settings')
        ui.pointer_text(pid,'System')
        entry['recovery_preferences_touch']=touch_preferences_action('Interface & recovery',scale)
        data(lambda d:d['ui_page']=='System');ui.assert_product_page(handle,'System')
        entry['system_controls']=system_geometry(scale)
        entry['system_critical_alert']=critical_alert_accessible(scale)
        path=evidence/f'dpi-{scale}-system-page.png';ui.capture(handle,path,screen_pixels=True)
        report['screenshots'].append(path.name)
        ui.click_text(pid,'Open Legacy OpenCPN')
        assert app.wait(timeout=30)==0;owned.discard(pid);count+=1
        handle,pid=ui.wait_window('OpenCPN / Legacy');owned.add(pid);ready();ui.size_window(handle)
        assert ui.GetDpiForWindow(handle)==observed
        entry['chart_rendering'].append(chart.presentation(capture(f'dpi-{scale}-legacy'),'Standard','Day',f'{scale}% Legacy'))
        process=ui.monitor_process(pid);ui.click_menu(handle,'Switch to XNav');ui.wait_clean_exit(process);owned.discard(pid);count+=1
        handle,pid=ui.wait_window('OpenNav X / OpenCPN');owned.add(pid);ready();ui.size_window(handle)
        assert ui.GetDpiForWindow(handle)==observed
        entry['chart_rendering'].append(chart.presentation(capture(f'dpi-{scale}-returned-xnav'),'XNav','Day',f'{scale}% returned XNav'))
        old=ui.monitor_process(pid);ui.open_system(pid);ui.click_text(pid,'Safe Mode')
        ui.wait_clean_exit(old);owned.discard(pid);count+=1
        handle,pid=ui.wait_window('OpenNav Safe Mode / OpenCPN');owned.add(pid);ready();ui.size_window(handle)
        assert ui.GetDpiForWindow(handle)==observed
        entry['chart_rendering'].append(chart.presentation(capture(f'dpi-{scale}-safe'),'Standard','Day',f'{scale}% Safe'))
        old=ui.monitor_process(pid);ui.click_menu(handle,'Switch to XNav');ui.wait_clean_exit(old);owned.discard(pid);count+=1
        handle,pid=ui.wait_window('OpenNav X / OpenCPN');owned.add(pid);ready();ui.size_window(handle)
        assert ui.GetDpiForWindow(handle)==observed
        entry['chart_rendering'].append(chart.presentation(capture(f'dpi-{scale}-safe-to-xnav'),'XNav','Day',f'{scale}% Safe to XNav'))
        close_current();assert fixtures.snapshot(profile)==expected
        report['scales'].append(entry)
    report['result']='passed; native visual review required'
except Exception as error:
    report['result']='failed';report['error']=repr(error)
    try:
        foreground=ui.declare(ui.user,'GetForegroundWindow',ui.W.HWND)()
        foreground_owner=ui.W.DWORD()
        ui.GetWindowThreadProcessId(foreground,C.byref(foreground_owner))
        report['failure_foreground']={'is_main':foreground==handle,
            'pid':foreground_owner.value,'title':ui.text(foreground)}
        capture(f'dpi-{scale}-failure')
        screen=evidence/f'dpi-{scale}-failure-visible.png'
        ui.capture(handle,screen,resize=False,screen_pixels=True)
        report['screenshots'].append(screen.name)
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

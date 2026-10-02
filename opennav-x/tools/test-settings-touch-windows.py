#!/usr/bin/env python3
"""Bounded disposable 125% Settings widget touch proof, not application acceptance."""
import argparse
import ctypes as C
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from diagnostic_snapshot import read_json_snapshot

ROOT=Path(__file__).resolve().parents[1]
def module(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'tools'/f'{name}.py')
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--client',type=Path,required=True)
    parser.add_argument('--dpi-helper',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if sys.platform!='win32' or os.environ.get('GITHUB_ACTIONS')!='true' or os.environ.get('RUNNER_ENVIRONMENT')!='github-hosted':
        raise SystemExit('Disposable hosted native Windows only')
    args.client=args.client.resolve();args.dpi_helper=args.dpi_helper.resolve();args.output=args.output.resolve()
    args.output.mkdir(parents=True,exist_ok=False)
    report={'passed':False,'scope':'Actual SettingsDrawer component, synthetic state; no OpenCPN, profile or devices',
            'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
            'github_sha':os.environ.get('GITHUB_SHA'),'executable_sha256':sha(args.client),
            'dpi_helper_sha256':sha(args.dpi_helper),'sources':{p:sha(ROOT/p) for p in (
                'tests/settings_touch_test.cpp','tests/WindowsDpi.cpp','tools/preferences-touch.py',
                'tools/test-settings-touch-windows.py','tools/diagnostic-geometry.py','tools/windows-ui.py')}}
    ui=module('windows-ui');touch=module('preferences-touch');geometry=module('diagnostic-geometry')
    env=dict(os.environ,OPENNAV_DISPOSABLE_DESKTOP='1')
    original=None;child=None;frame=None;success=False;log=None;restoring=False
    deadline=time.monotonic()+65
    def dpi(*values):
        if not restoring:assert time.monotonic()<deadline,'Touch proof deadline'
        run=subprocess.run([str(args.dpi_helper),*map(str,values)],env=env,capture_output=True,text=True,timeout=10)
        if run.returncode:raise RuntimeError(run.stderr.strip())
        return json.loads(run.stdout)
    def bounds(hwnd):
        rect=ui.W.RECT();assert ui.GetWindowRect(hwnd,C.byref(rect));return rect
    def data(predicate=lambda r:True):
        observation_deadline=min(deadline,time.monotonic()+8)
        while time.monotonic()<observation_deadline:
            assert child.poll() is None,'Touch component exited before observation'
            try:
                record=read_json_snapshot(args.output/'observation.json')
                if predicate(record):return record
            except (OSError,ValueError):pass
            time.sleep(.05)
        raise AssertionError('Native component observation deadline')
    def observe(label='Advanced battery model'):
        previous=data()['runtime']['ui_update']['ticks']
        popup,_=ui.wait_window('OpenNav preferences',child.pid,timeout=5)
        found=[h for h,t in ui.children(popup) if t==label];assert len(found)==1,(label,found)
        target=found[0];r=bounds(target);native=(r.left,r.top,r.right,r.bottom)
        record=data(lambda d:geometry.matches_native_controls(d,{label:native},previous,require_visible=False))
        r=bounds(target);assert (r.left,r.top,r.right,r.bottom)==native,'Action moved during observation pairing'
        return record,target
    def capture(name):
        ui.capture(frame,args.output/(name+'.png'),resize=False,screen_pixels=True)
    try:
        report['desktop']=ui.ensure_desktop(1440,900)
        original=dpi();report['original_dpi']=original
        assert original['percent'] in (100,125,150),'Unexpected DPI restoration target'
        report['applied_dpi']=dpi(125);assert report['applied_dpi']['percent']==125
        log=(args.output/'host.log').open('wb')
        child=subprocess.Popen([str(args.client),str(args.output)],env=env,stdout=log,stderr=subprocess.STDOUT)
        frame,_=ui.wait_window('TEST ONLY - Preferences touch',child.pid,timeout=10)
        ui.size_window(frame)
        before,target=observe();popup,_=ui.wait_window('OpenNav preferences',child.pid,timeout=5)
        ui.SetForegroundWindow(popup);time.sleep(.2)
        assert ui.GetDpiForWindow(frame)==120 and ui.GetDpiForWindow(popup)==120
        assert before['native_dpi']==before['wx_dpi']==120,'Actual native and wx DPI must both be 120'
        assert before['pid']==child.pid and before['fixture_only'] and before['body_scroll_px']==0
        report['initial']=before;capture('preferences-125-before')
        # Reproduce the retained old formula on actual widgets, never synthetic hit geometry.
        client=ui.W.RECT();assert ui.GetClientRect(frame,C.byref(client))
        assert (client.right,client.bottom)==(1262,753),'Host client must match retained FFE125% frame'
        assert before['runtime']['display']['drawer']=={'x':545,'y':128,'width':513,'height':605},'Drawer geometry differs from retained FFE'
        report['frame_client_pixels']=[client.right,client.bottom]
        drawer=before['runtime']['display']['drawer'];x=drawer['x']+drawer['width']//2;y=drawer['y']+drawer['height']-50
        field=next(c for c in before['runtime']['display']['interaction_controls'] if c['label']=='Field: Usable battery capacity · kWh')
        assert tuple(field[k] for k in ('x','y','width','height'))==(590,683,422,28),'Battery field geometry differs from retained FFE'
        hit=ui.WindowFromPoint(ui.W.POINT(x,y));native_class=C.create_unicode_buffer(256)
        assert ui.GetClassNameW(hit,native_class,len(native_class))
        report['negative']={'start':[x,y],'end':[x,y-200],'hit_hwnd':int(hit),
                            'hit_class':native_class.value,'expected_field':field}
        assert field['visible'] and hit==field['hwnd'] and native_class.value=='Edit','Old gesture must hit actual battery Edit'
        assert dpi('--pan',x,y,x,y-200)['touch_injected']
        time.sleep(.5);after,_=observe();report['negative']['after']=after;capture('preferences-125-old-target')
        assert after['body_scroll_px']==0,'Old field-targeted pan did not reproduce retained failure'
        assert after['body_pan_messages']==before['body_pan_messages'],'Old field pan unexpectedly reached body'
        assert after['child_pan_messages']>before['child_pan_messages'] or after['child_mouse_downs']>before['child_mouse_downs'],'No native field input was observed'
        def fields(record):return {c['label']:c['value'] for c in record['runtime']['display']['interaction_controls'] if c['label'].startswith('Field: ')}
        assert fields(after)==fields(before),'Negative gesture modified vessel fields'
        reached,_target,reach=touch.reach_preferences_action('Advanced battery model',125,ui=ui,pid=child.pid,
            observe=observe,bounds=bounds,dpi=dpi,report=report)
        endpoint=next(c for c in reached['runtime']['display']['interaction_controls'] if c['label']=='Advanced battery model')
        time.sleep(1.2);settled,_=observe()
        assert endpoint in settled['runtime']['display']['interaction_controls'],'Preferences jumped at the lower endpoint'
        report['vessel_form']=touch.vessel_form_contract(settled,require_save_visible=True)
        assert settled['body_scroll_px']>0 and settled['body_pan_messages']>after['body_pan_messages'],'No actual body pan/displacement'
        assert fields(settled)==fields(before) and settled['saves']==settled['actions']==0,'Reach must not edit/save/activate'
        report['positive']={'reach':reach,'settled':settled};capture('preferences-125-body-target')
        report['tap']=touch.touch_preferences_action('Advanced battery model',125,ui=ui,pid=child.pid,
            observe=observe,bounds=bounds,dpi=dpi,report=report)
        activated=data(lambda d:d['actions']==1)
        assert activated['battery_action'] and activated['saves']==0 and fields(activated)==fields(before)
        report['activated']=activated
        assert sha(args.client)==report['executable_sha256'] and sha(args.dpi_helper)==report['dpi_helper_sha256']
        assert all(sha(ROOT/p)==h for p,h in report['sources'].items())
        success=True
    except Exception as exc:
        report['error']=repr(exc)
        if frame:
            try:capture('preferences-125-failure')
            except Exception as capture_error:report['capture_error']=repr(capture_error)
    finally:
        # Independently attempt both process cleanup and DPI restoration.
        if child:
            try:
                (args.output/'stop').write_text('stop\n')
                assert child.wait(timeout=5)==0,'Touch host failed'
                final=json.loads((args.output/'host-result.json').read_text())
                report['host_result']=final
                assert final['completed'] and final['saves']==0 and (not success or final['actions']==1)
            except Exception as exc:
                report['cleanup_error']=repr(exc)
                if child.poll() is None:
                    try:child.kill();child.wait(timeout=5)
                    except Exception as kill_error:report['kill_error']=repr(kill_error)
        if log:log.close()
        if original:
            restoring=True
            try:
                report['restored_dpi']=dpi(original['percent'])
                assert report['restored_dpi']['percent']==original['percent'],'DPI restoration failed'
            except Exception as exc:report['restore_error']=repr(exc)
        report['passed']=success and not any(key.endswith('error') for key in report)
        (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())

#!/usr/bin/env python3
"""Elapsed-time endurance of the actual application; isolated, no boat output.

Short runs validate this harness. Release qualification requires >=10800 actual
seconds, independently of the deterministic trip's accelerated scenario clock.
"""
import argparse
import ctypes as C
import importlib.util
import hashlib
import json
import os
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
import tempfile
import time
from diagnostic_snapshot import read_json_snapshot

root = Path(__file__).resolve().parents[1]
windows = sys.platform == 'win32'
parser = argparse.ArgumentParser()
parser.add_argument('--seconds', type=int, default=120)
parser.add_argument('--evidence', type=Path, default=root/'evidence/local/soak')
parser.add_argument('--viewport-only', action='store_true',
                    help='observe one zoom pair and palette action only; never an endurance pass')
args = parser.parse_args()
if not 120 <= args.seconds <= 21600:
    raise SystemExit('Choose 120..21600 actual elapsed seconds')
args.evidence.mkdir(parents=True, exist_ok=True)
report = {'status': 'running', 'requested_seconds': None if args.viewport_only else args.seconds,
          'release_duration': not args.viewport_only and args.seconds >= 10800,
          'authority': 'native Windows' if windows else 'Linux development',
          'scope': 'Actual process, DEMO vessel/route/AIS, software chart, repeated UI and dropout recovery; no hardware commands',
          'samples': 0, 'actions': [], 'screenshots': []}
if args.viewport_only:
    report['scope'] = 'Diagnostic-only three chart inputs on disposable DEMO; no endurance, release, native Windows or boat qualification'
report['harness_commit'] = subprocess.check_output(
    ['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
report['harness_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
report['viewport_observations'] = []

def module(name):
    spec = importlib.util.spec_from_file_location(name, root/'tools'/f'{name}.py')
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    return mod

ui = module('windows-ui') if windows else None
temp = tempfile.TemporaryDirectory(prefix='OpenNav endurance ')
profile = Path(temp.name).resolve()/'profile'
variant = 'xnav-windows' if windows else 'xnav-linux'
subprocess.run([sys.executable, str(root/'tools/prepare-test-profile.py'),
                '--build', str(root/'build'/variant), '--profile', str(profile)], check=True)
fixtures = module('profile-fixtures'); fixtures.seed(profile)
expected_profile = fixtures.snapshot(profile)
with (profile/'opencpn.conf').open('a') as f:
    f.write('\n[Settings/GlobalState]\nVPLatLon=59.0800,18.5000\nVPScale=0.003\n')
exe = root/('build/xnav-install/opencpn.exe' if windows else 'build/xnav-install/bin/opencpn')
report['executable'] = {'path': str(exe.resolve()), 'bytes': exe.stat().st_size,
                        'mtime_ns': exe.stat().st_mtime_ns,
                        'sha256': hashlib.sha256(exe.read_bytes()).hexdigest()}
env = dict(os.environ)
trace = profile/'opennav-ui-trace.log'
env['OPENNAV_TEST_UI_TRACE'] = 'pointer'
env['OPENNAV_TEST_UI_TRACE_FILE'] = str(trace)
xserver = app = process_handle = None
samples = []

def persist():
    target = args.evidence/'results.json'
    staging = target.with_suffix('.tmp')
    staging.write_text(json.dumps(report, indent=2)+'\n')
    staging.replace(target)

def xdo(*values):
    return subprocess.check_output(['xdotool', *map(str, values)], env=env, text=True, timeout=5).strip()

def data(predicate=lambda d: True, timeout=8):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        assert app.poll() is None, ('Unexpected process exit', app.returncode)
        d = read_json_snapshot(profile/'opennav-diagnostics.json')
        if predicate(d): return d
        time.sleep(.1)
    raise AssertionError('Application diagnostic state did not advance')

def viewport(d):
    return {'observed_monotonic': time.monotonic(), 'observed_unix_ns': time.time_ns(),
            'diagnostic_mtime_ns': (profile/'opennav-diagnostics.json').stat().st_mtime_ns,
            'diagnostic_clock': d.get('clock'), 'ui_update': d['runtime']['ui_update'],
            'page': d['ui_page'], 'light': d['runtime']['display']['light'],
            'chart': d['runtime']['chart']}

def observe_chart_action(label, send, selected_control=None):
    """Observe input and a later snapshot; never infer activation from fresh ticks.

    Keep the existing click methods and their delays. The Linux caller retains
    its original control snapshot so this diagnostic does not silently repair
    a stale-coordinate issue. Record fresh advertised bounds alongside it.
    """
    before = data()
    controls = [c for c in before['runtime']['display']['interaction_controls']
                if c['label'] == label and c['visible'] and c['enabled']]
    item = {'label': label, 'before': viewport(before), 'input_returned': False,
            'fresh_diagnostic_controls': controls, 'selected_control': selected_control,
            'input_method': 'native HWND messages' if windows else 'X11 pointer click',
            'activation_scope': 'Pointer trace, when present, is evidence; fresh diagnostics alone are not activation acknowledgement'}
    if selected_control:
        item['sent_screen_point'] = [selected_control['x']+selected_control['width']//2,
                                     selected_control['y']+selected_control['height']//2]
    if windows:
        # The existing helpers choose the target. Retain the actual native
        # candidates separately from the application-advertised coordinates.
        item['native_candidates'] = []
        for parent, _, _ in ui.windows(app.pid):
            for child, caption in ui.children(parent):
                if caption == label:
                    rect = ui.W.RECT()
                    assert ui.GetWindowRect(child, C.byref(rect))
                    item['native_candidates'].append({'hwnd': int(child),
                        'screen_bounds': [rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top]})
    offset = trace.stat().st_size if trace.exists() else 0
    report['viewport_observations'].append(item)
    if args.viewport_only:
        name = 'viewport-' + str(len(report['viewport_observations'])) + '-before'
        capture(name)
        item['before_screenshot'] = name + '.png'
    persist()
    try:
        item['input_started_monotonic'] = time.monotonic()
        send()
        item['input_returned'] = True
        item['input_finished_monotonic'] = time.monotonic()
        after = data(lambda d: int(d['runtime']['ui_update']['ticks']) >
                     int(before['runtime']['ui_update']['ticks']))
        item['after'] = viewport(after)
        item['scale_ratio'] = after['runtime']['chart']['scale_ppm'] / before['runtime']['chart']['scale_ppm']
    finally:
        lines = []
        if trace.exists():
            with trace.open('rb') as source:
                source.seek(offset)
                lines = source.read(65536).decode('utf-8', errors='replace').splitlines()
        item['pointer_trace'] = lines
        item['activation_observed'] = any(' pointer.activate ' in line and line.endswith(' detail=1')
                                          for line in lines)
        item['trace_limit'] = 'Existing fixture trace is bounded to 2048 records; absence is not proof of failed activation'
        persist()

def page(label, shortcut, expected):
    start = time.monotonic()
    if windows:
        if shortcut=='j':ui.accelerator(handle,shortcut)
        else:ui.click_text(app.pid, label)
    else:
        xdo('windowfocus', handle); xdo('key', 'ctrl+shift+'+shortcut)
    data(lambda d: d['ui_page'] == expected)
    return time.monotonic()-start

def scenario(label, index, predicate):
    if windows:
        ui.accelerator(handle,'F'+str(index+1))
    else:
        xdo('windowfocus', handle); xdo('key', 'ctrl+shift+F'+str(index+1))
    return data(predicate)

def capture(name):
    path = args.evidence/(name+'.png')
    if windows: ui.capture(handle, path)
    else: subprocess.run(['import', '-window', 'root', str(path)], env=env, check=True, timeout=10)
    report['screenshots'].append(path.name)

def resources():
    if not windows:
        base = Path('/proc')/str(app.pid)
        fields = (base/'stat').read_text().rsplit(')', 1)[1].split()
        return {'cpu_seconds': (int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK'),
                'resident_bytes': int(fields[21])*os.sysconf('SC_PAGE_SIZE'),
                'handles': len(list((base/'fd').iterdir())), 'threads': int(fields[17])}
    kernel = C.WinDLL('kernel32', use_last_error=True)
    psapi = C.WinDLL('psapi', use_last_error=True)
    class Memory(C.Structure):
        _fields_ = [('cb', C.c_ulong), ('faults', C.c_ulong)] + [(name, C.c_size_t) for name in
                    ('peak_ws', 'ws', 'peak_paged', 'paged', 'peak_nonpaged', 'nonpaged', 'pagefile', 'peak_pagefile', 'private')]
    memory = Memory(); memory.cb = C.sizeof(memory)
    get_memory = ui.declare(psapi, 'GetProcessMemoryInfo', C.c_int, C.c_void_p, C.POINTER(Memory), C.c_ulong)
    assert get_memory(process_handle, C.byref(memory), memory.cb)
    times = [C.c_ulonglong() for _ in range(4)]
    get_times = ui.declare(kernel, 'GetProcessTimes', C.c_int, C.c_void_p, *([C.POINTER(C.c_ulonglong)]*4))
    assert get_times(process_handle, *[C.byref(t) for t in times])
    handles = C.c_ulong()
    get_handles = ui.declare(kernel, 'GetProcessHandleCount', C.c_int, C.c_void_p, C.POINTER(C.c_ulong))
    assert get_handles(process_handle, C.byref(handles))
    get_gui = ui.declare(ui.user, 'GetGuiResources', C.c_ulong, C.c_void_p, C.c_ulong)
    return {'cpu_seconds': (times[2].value+times[3].value)/1e7,
            'resident_bytes': memory.ws, 'private_bytes': memory.private,
            'handles': handles.value, 'gdi_objects': get_gui(process_handle, 0),
            'user_objects': get_gui(process_handle, 1)}

try:
    if windows:
        report['display'] = ui.ensure_desktop()
    else:
        number = 171
        while Path(f'/tmp/.X{number}-lock').exists(): number += 1
        env['DISPLAY'] = f':{number}'
        xserver = subprocess.Popen(['Xvfb', env['DISPLAY'], '-screen', '0', '1280x800x24', '-nolisten', 'tcp'],
                                   env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1)
    launched = time.monotonic()
    with (args.evidence/'launch.log').open('w') as log:
        app = subprocess.Popen([str(exe), '--configdir', str(profile), '--no_opengl', '--xnav', '--xnav-demo'],
                               env=env, stdout=log, stderr=log)
    if windows:
        handle, _ = ui.wait_window('SKAGER / OpenCPN', app.pid)
        open_process = ui.declare(C.WinDLL('kernel32'), 'OpenProcess', C.c_void_p, C.c_ulong, C.c_int, C.c_ulong)
        process_handle = open_process(0x410, 0, app.pid)
        assert process_handle
    else:
        deadline = time.monotonic()+60
        while time.monotonic() < deadline:
            found = subprocess.run(['xdotool', 'search', '--all', '--onlyvisible', '--pid', str(app.pid), '--name', '^SKAGER / OpenCPN$'],
                                   env=env, capture_output=True, text=True)
            if found.returncode == 0 and found.stdout.strip():
                handle = found.stdout.splitlines()[0]; break
            time.sleep(.2)
        else: raise AssertionError('XNav did not open')
        xdo('windowsize', handle, 1280, 800); xdo('windowmove', handle, 0, 0); xdo('windowfocus', handle)
    deadline = time.monotonic()+60
    while time.monotonic() < deadline:
        path = profile/'opencpn.log'
        if path.exists() and 'OnInitTimer...Finalize Canvases' in path.read_text(errors='replace'): break
        time.sleep(.2)
    else: raise AssertionError('Deferred startup did not finish')
    time.sleep(1.5)
    first = data(lambda d: d['data_mode'] == 'DEMO' and 'arrival_soc' in d['energy'])
    report['build_commit'] = first['build_commit']
    report['binary_matches_harness_commit'] = first['build_commit'] == report['harness_commit']
    report['initial_viewport'] = viewport(first)
    report['startup_seconds'] = time.monotonic()-launched
    chart = module('chart-render-check'); colors = chart.presentation(
        ui.capture(handle, args.evidence/'start.png') if windows else
        subprocess.check_output(['import', '-window', 'root', '-depth', '8', 'rgb:-'], env=env),
        'XNav', first['runtime']['display']['light'], 'Endurance startup')
    capture('start')
    start = time.monotonic(); next_action = start; next_sample = start
    sequence = 0; last_ticks = -1; last_distance = None; progress_changes = 0
    pages = [('Chart','n','Navigation'), ('Passage','r','Route'),
             ('Energy','e','Energy'), ('Instruments','v','Vessel instruments'),
             ('Traffic','a','AIS targets'), ('SmartNav advisories','j','SmartNav')]
    while time.monotonic()-start < args.seconds:
        now = time.monotonic()
        if now >= next_action:
            slot = sequence % 6
            action = {'elapsed': round(now-start, 3), 'slot': slot}
            action['page_observation_seconds'] = page(*pages[slot])
            if slot == 0:
                scenario('Cruising', 0, lambda d: 'arrival_soc' in d['energy'] and d['runtime']['smartnav']['route_valid'])
                last_distance = None
                # Opposite zoom inputs are observed, not assumed to be exact inverses.
                if windows:
                    for label in ('+', '−'):
                        observe_chart_action(label, lambda label=label: ui.click_text(app.pid, label))
                    before_light = data()['runtime']['display']['light']
                    expected_light = {'Day': 'Dusk', 'Dusk': 'Night', 'Night': 'Day'}[before_light]
                    observe_chart_action(before_light, lambda: ui.cycle_light(app.pid))
                    data(lambda d: d['runtime']['display']['light'] == expected_light)
                    action['palette'] = expected_light
                else:
                    controls=data()['runtime']['display']['interaction_controls']
                    for label in ['+','−',data()['runtime']['display']['light']]:
                        found=[c for c in controls if c['label']==label and c['visible'] and c['enabled']]
                        assert len(found)==1,(label,'unique visible prototype action required')
                        c=found[0]
                        def click(c=c):
                            xdo('mousemove',c['x']+c['width']//2,c['y']+c['height']//2,'click',1)
                            time.sleep(.5)
                        observe_chart_action(label, click, c)
            elif slot == 4:
                missing = (sequence//6) % 2
                # Unavailable removes selected instruments/power, deliberately
                # retaining GPS and route. Stale freezes the entire source.
                # Require the appropriate dependency loss, not blanket advice loss.
                def dropout(d):
                    if 'arrival_soc' in d['energy']:
                        return False
                    if not missing:
                        return not d['runtime']['smartnav']['route_valid']
                    values = {item['name']: item for item in d['data']}
                    return d['runtime']['smartnav']['route_valid'] and all(
                        values[name]['quality'] == 'UNAVAILABLE' for name in
                        ['Depth below transducer', 'Motor speed', 'Battery current (+ discharge)'])
                scenario('Sensors unavailable' if missing else 'Sensors stale', 2 if missing else 1, dropout)
                action['dropout'] = 'unavailable' if missing else 'stale'
            elif slot == 5:
                scenario('Cruising', 0, lambda d: 'arrival_soc' in d['energy'] and d['runtime']['smartnav']['route_valid'])
                action['recovered'] = True; last_distance = None
            report['actions'].append(action)
            sequence += 1; next_action = start+sequence*20
        if now >= next_sample:
            d = data(); ticks = int(d['runtime']['ui_update']['ticks'])
            assert ticks > last_ticks, 'UI update loop stopped'
            last_ticks = ticks
            distance = d.get('route', {}).get('remaining_nm')
            if distance is not None and last_distance is not None and distance < last_distance:
                progress_changes += 1
            last_distance = distance
            sample = resources()
            sample.update(elapsed=round(time.monotonic()-start, 3), ui_update=d['runtime']['ui_update'],
                          chart=d['runtime']['chart'], light=d['runtime']['display']['light'],
                          advice_events=d['runtime']['smartnav']['event_count'],
                          ais_events=d['runtime']['smartnav']['ais_event_count'],
                          energy_available='arrival_soc' in d['energy'], remaining_nm=distance)
            samples.append(sample); report['samples'] = len(samples)
            with (args.evidence/'metrics.jsonl').open('a') as log: log.write(json.dumps(sample)+'\n')
            report['elapsed_seconds'] = sample['elapsed']; persist()
            next_sample += 10
        if args.viewport_only:
            break
        time.sleep(.2)
    report['elapsed_seconds'] = time.monotonic()-start
    if not args.viewport_only:
        assert report['elapsed_seconds'] >= args.seconds
        assert progress_changes >= 2, 'Route progress did not change'
        assert any(s['ais_events'] > 0 for s in samples), 'No AIS encounter context exercised'
        assert any(not s['energy_available'] for s in samples), 'Dropout did not suppress energy'
        assert any(a.get('recovered') for a in report['actions']), 'No dropout recovery'
    page(*pages[0]); capture('end')
    report['final_viewport'] = viewport(data())
    # Compare medians after warm-up, not peak allocator/cache growth during startup.
    if not args.viewport_only:
        stable = [s for s in samples if s['elapsed'] >= min(300, args.seconds/4)]
        span = max(2, len(stable)//5)
        growth = {}
        limits = {'resident_bytes': 128*1024*1024, 'private_bytes': 128*1024*1024,
                  'handles': 128 if windows else 32, 'gdi_objects': 64, 'user_objects': 64, 'threads': 8}
        for key, limit in limits.items():
            if key not in stable[0]: continue
            growth[key] = statistics.median(s[key] for s in stable[-span:])-statistics.median(s[key] for s in stable[:span])
            assert growth[key] <= limit, (key, 'sustained resource growth', growth[key], limit)
        report['resource_growth'] = growth; report['growth_limits'] = limits
        report['cpu_percent_one_core'] = 100*(samples[-1]['cpu_seconds']-samples[0]['cpu_seconds'])/(samples[-1]['elapsed']-samples[0]['elapsed'])
        report['maximum_page_observation_seconds'] = max(a['page_observation_seconds'] for a in report['actions'])
        report['ui_observation_limit_seconds'] = 8
        report['route_progress_samples'] = progress_changes
    if windows:
        monitor = ui.monitor_process(app.pid); ui.close(handle); ui.wait_clean_exit(monitor)
    else:
        subprocess.run([str(exe), '--configdir', str(profile), '--remote', '--quit'], env=env, check=True, capture_output=True, timeout=15)
    assert app.wait(timeout=30) == 0
    assert fixtures.snapshot(profile) == expected_profile, 'Navigation/profile fixtures changed'
    report['status'] = 'observed' if args.viewport_only else 'passed'
except BaseException as error:
    report['status'] = 'failed'; report['error'] = repr(error)
    if app and app.poll() is None and 'handle' in globals():
        try: capture('failure')
        except Exception: pass
    raise
finally:
    if app and app.poll() is None: app.terminate(); app.wait(timeout=20)
    if process_handle:
        ui.declare(C.WinDLL('kernel32'), 'CloseHandle', C.c_int, C.c_void_p)(process_handle)
    if xserver: xserver.terminate(); xserver.wait(timeout=10)
    for name in ('opencpn.log','opennav-diagnostics.json','opencpn.conf','opennav-ui-trace.log'):
        if (profile/name).exists(): shutil.copy2(profile/name, args.evidence/name)
    persist(); temp.cleanup()
print(json.dumps(report, indent=2))

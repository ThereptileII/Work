#!/usr/bin/env python3
"""Elapsed-time endurance of the actual application; isolated, no boat output.

Short runs validate this harness. Release qualification requires >=10800 actual
seconds, independently of the deterministic trip's accelerated scenario clock.
"""
import argparse
import ctypes as C
import importlib.util
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
args = parser.parse_args()
if not 120 <= args.seconds <= 21600:
    raise SystemExit('Choose 120..21600 actual elapsed seconds')
args.evidence.mkdir(parents=True, exist_ok=True)
report = {'status': 'running', 'requested_seconds': args.seconds,
          'release_duration': args.seconds >= 10800,
          'authority': 'native Windows' if windows else 'Linux development',
          'scope': 'Actual process, DEMO vessel/route/AIS, software chart, repeated UI and dropout recovery; no hardware commands',
          'samples': 0, 'actions': [], 'screenshots': []}

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
env = dict(os.environ)
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

def page(label, shortcut, expected, menu=False):
    start = time.monotonic()
    if windows:
        if menu: ui.click_text(app.pid, 'Menu')
        ui.click_text(app.pid, label)
    else:
        xdo('windowfocus', handle); xdo('key', 'ctrl+shift+'+shortcut)
    data(lambda d: d['ui_page'] == expected)
    return time.monotonic()-start

def scenario(label, index, predicate):
    if windows:
        ui.click_text(app.pid, 'Demo'); ui.click_text(app.pid, label)
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
        handle, _ = ui.wait_window('OpenNav X / OpenCPN', app.pid)
        open_process = ui.declare(C.WinDLL('kernel32'), 'OpenProcess', C.c_void_p, C.c_ulong, C.c_int, C.c_ulong)
        process_handle = open_process(0x410, 0, app.pid)
        assert process_handle
    else:
        deadline = time.monotonic()+60
        while time.monotonic() < deadline:
            found = subprocess.run(['xdotool', 'search', '--onlyvisible', '--pid', str(app.pid), '--name', '^OpenNav X / OpenCPN$'],
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
    report['startup_seconds'] = time.monotonic()-launched
    chart = module('chart-render-check'); colors = chart.reference(
        ui.capture(handle, args.evidence/'start.png') if windows else
        subprocess.check_output(['import', '-window', 'root', '-depth', '8', 'rgb:-'], env=env))
    capture('start')
    start = time.monotonic(); next_action = start; next_sample = start
    sequence = 0; last_ticks = -1; last_distance = None; progress_changes = 0
    pages = [('Navigation','n','Navigation',False), ('Route','r','Route',False),
             ('Energy','e','Energy',False), ('Vessel instruments','v','Vessel instruments',True),
             ('AIS targets','a','AIS targets',True), ('SmartNav advisories','j','SmartNav',True)]
    while time.monotonic()-start < args.seconds:
        now = time.monotonic()
        if now >= next_action:
            slot = sequence % 6
            action = {'elapsed': round(now-start, 3), 'slot': slot}
            action['page_observation_seconds'] = page(*pages[slot])
            if slot == 0:
                scenario('Cruising', 0, lambda d: 'arrival_soc' in d['energy'] and d['runtime']['smartnav']['route_valid'])
                last_distance = None
                # Chart input uses actual controls, with net-zero zoom change.
                if windows:
                    ui.click_text(app.pid, '+'); ui.click_text(app.pid, '−')
                    ui.click_text(app.pid, 'Light')
                else:
                    xdo('mousemove', 27, 140, 'click', 1); xdo('mousemove', 27, 196, 'click', 1)
                    xdo('mousemove', 1240, 28, 'click', 1)
            elif slot == 4:
                missing = (sequence//6) % 2
                scenario('Sensors unavailable' if missing else 'Sensors stale', 2 if missing else 1,
                         lambda d: 'arrival_soc' not in d['energy'] and not d['runtime']['smartnav']['route_valid'])
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
                          advice_events=d['runtime']['smartnav']['event_count'],
                          ais_events=d['runtime']['smartnav']['ais_event_count'],
                          energy_available='arrival_soc' in d['energy'], remaining_nm=distance)
            samples.append(sample); report['samples'] = len(samples)
            with (args.evidence/'metrics.jsonl').open('a') as log: log.write(json.dumps(sample)+'\n')
            report['elapsed_seconds'] = sample['elapsed']; persist()
            next_sample += 10
        time.sleep(.2)
    report['elapsed_seconds'] = time.monotonic()-start
    assert report['elapsed_seconds'] >= args.seconds
    assert progress_changes >= 2, 'Route progress did not change'
    assert any(s['ais_events'] > 0 for s in samples), 'No AIS encounter context exercised'
    assert any(not s['energy_available'] for s in samples), 'Dropout did not suppress energy'
    assert any(a.get('recovered') for a in report['actions']), 'No dropout recovery'
    page(*pages[0]); capture('end')
    # Compare medians after warm-up, not peak allocator/cache growth during startup.
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
    report['status'] = 'passed'
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
    for name in ('opencpn.log','opennav-diagnostics.json'):
        if (profile/name).exists(): shutil.copy2(profile/name, args.evidence/name)
    persist(); temp.cleanup()
print(json.dumps(report, indent=2))

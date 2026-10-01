"""Real native Windows mode-cycle interaction against a disposable shared profile."""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import uuid
from diagnostic_snapshot import read_json_snapshot
from peer_boundary import PeerBoundary

def module(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result

ui = module('windows-ui')
display = ui.ensure_desktop()
fixtures = module('profile-fixtures')
charts = module('chart-render-check')
root = Path(__file__).resolve().parents[1]
profile = root / 'build/profiles' / ('mode cycle ' + str(uuid.uuid4()))
evidence = root / 'evidence/local'
evidence.mkdir(parents=True, exist_ok=True)
subprocess.run([sys.executable, str(root / 'tools/prepare-test-profile.py'), '--build',
                str(root / 'build/xnav-windows'), '--profile', str(profile)], check=True)
fixtures.seed(profile)
peer_boundary = PeerBoundary(profile)
expected = fixtures.snapshot(profile)
exe = root / 'build/xnav-install/opencpn.exe'
logfile = profile / 'opencpn.log'
process = subprocess.Popen([str(exe), '--configdir', str(profile), '--no_opengl', '--xnav'])
pid = process.pid
handle = None
report = {'display': display, 'profile': str(profile), 'steps': [], 'expected': expected}

def data(predicate=lambda d:True):
    deadline=time.monotonic()+12
    while time.monotonic()<deadline:
        d=read_json_snapshot(profile/'opennav-diagnostics.json')
        if predicate(d):return d
        time.sleep(.15)
    raise AssertionError('Mode-cycle diagnostic state did not arrive')

def capture_chart(name,style='XNav',light='Day'):
    rgb=ui.capture(handle,evidence/name)
    report.setdefault('chart_rendering',[]).append(charts.presentation(rgb,style,light,name))

def ready(expected_count):
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        log = logfile.read_text(errors='replace') if logfile.exists() else ''
        if log.count('OnInitTimer...Finalize Canvases') >= expected_count:
            time.sleep(1)
            report.setdefault('peer_boundary', []).append(peer_boundary.observe(pid))
            return
        time.sleep(.2)
    raise RuntimeError('Deferred initialization did not finish')

def saved(step):
    peer_boundary.assert_preserved()
    actual = fixtures.snapshot(profile)
    assert actual == expected, f'{step}: persisted fixtures changed: {actual}'
    report['steps'].append({'step': step, 'persistence': 'pass'})

try:
    handle, pid = ui.wait_window('OpenNav X / OpenCPN', pid)
    ready(1)
    capture_chart('01-xnav-unavailable.png')
    ui.cycle_light(pid)
    data(lambda d:d['runtime']['display']['light']=='Dusk')
    capture_chart('02-xnav-dusk.png',light='Dusk')
    ui.cycle_light(pid)
    data(lambda d:d['runtime']['display']['light']=='Night')
    capture_chart('03-xnav-night.png',light='Night')
    ui.cycle_light(pid)
    data(lambda d:d['runtime']['display']['light']=='Day')
    ui.click_text(pid, '+')
    ui.click_text(pid, '−')
    ui.accelerator(handle, 'T')
    ui.click_text(pid, 'Cruising')
    data(lambda d:d['data_mode']=='DEMO' and d['route']['state']=='Valid')
    ui.capture(handle, evidence / '04-xnav-simulation.png')
    ui.accelerator(handle, 'T')
    ui.click_text(pid, 'Pause simulation')
    time.sleep(6)
    data(lambda d:next(i for i in d['data'] if i['name']=='Speed over ground')['quality']=='STALE')
    ui.capture(handle, evidence / '05-xnav-stale.png')
    report['steps'].append({'step': 'theme / zoom / simulator start and pause', 'interaction': 'pass',
                            'visual_review': 'required; stale labels and fixture values must be checked'})
    ui.open_system(pid)
    ui.click_text(pid, 'Open Legacy OpenCPN')
    assert process.wait(timeout=40) == 0, 'XNav did not exit cleanly'
    handle, pid = ui.wait_window('OpenCPN / Legacy')
    assert pid != process.pid, 'Mode change must use a new process'
    ready(2)
    saved('XNav to Legacy')
    capture_chart('11-legacy-after-xnav.png','Standard')
    old_process_handle = ui.monitor_process(pid)
    ui.click_menu(handle, 'Switch to XNav')
    ui.wait_clean_exit(old_process_handle)
    next_handle, next_pid = ui.wait_window('OpenNav X / OpenCPN')
    assert next_pid != pid
    handle, pid = next_handle, next_pid
    ready(3)
    saved('Legacy to XNav')
    data(lambda d:d['data_mode']!='DEMO')
    capture_chart('06-xnav-after-legacy.png')
    last_process_handle = ui.monitor_process(pid)
    ui.close(handle)
    ui.wait_clean_exit(last_process_handle)
    saved('Final XNav close')
    # Safe Mode must win over both conflicting normal flags, preserve the saved
    # normal preference, and use the exact same navigation/configuration store.
    safe = subprocess.Popen([str(exe), '--configdir', str(profile), '--no_opengl',
                             '--xnav', '--legacy', '--safe-mode'])
    handle, pid = ui.wait_window('OpenNav Safe Mode / OpenCPN', safe.pid)
    ready(4)
    capture_chart('12-safe-shared-profile.png','Standard')
    ui.close(handle)
    assert safe.wait(timeout=30) == 0
    saved('Safe override and close')
    normal = subprocess.Popen([str(exe), '--configdir', str(profile), '--no_opengl'])
    handle, pid = ui.wait_window('OpenNav X / OpenCPN', normal.pid)
    ready(5)
    ui.close(handle)
    assert normal.wait(timeout=30) == 0
    saved('Persisted XNav after Safe Mode')
    report['result'] = 'interaction and fixture persistence passed; visual review required'
finally:
    if handle and ui.windows(pid):
        ui.close(handle)
    # Never target a user's process; the initial pid belongs to this invocation.
    if process.poll() is None:
        process.terminate()
        process.wait(timeout=10)
    shutil.copytree(profile, evidence / 'mode-cycle-profile', dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('*.pem'))
    (evidence / 'mode-cycle-results.json').write_text(json.dumps(report, indent=2))

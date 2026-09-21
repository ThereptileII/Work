"""Real native Windows mode-cycle interaction against a disposable shared profile."""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import uuid

def module(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result

ui = module('windows-ui')
display = ui.ensure_desktop()
fixtures = module('profile-fixtures')
root = Path(__file__).resolve().parents[1]
profile = root / 'build/profiles' / ('mode cycle ' + str(uuid.uuid4()))
evidence = root / 'evidence/local'
evidence.mkdir(parents=True, exist_ok=True)
subprocess.run([sys.executable, str(root / 'tools/prepare-test-profile.py'), '--build',
                str(root / 'build/xnav-windows'), '--profile', str(profile)], check=True)
fixtures.seed(profile)
expected = fixtures.snapshot(profile)
exe = root / 'build/xnav-install/opencpn.exe'
logfile = profile / 'opencpn.log'
process = subprocess.Popen([str(exe), '--configdir', str(profile), '--no_opengl', '--xnav'])
pid = process.pid
handle = None
report = {'display': display, 'profile': str(profile), 'steps': [], 'expected': expected}

def ready(expected_count):
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        log = logfile.read_text(errors='replace') if logfile.exists() else ''
        if log.count('OnInitTimer...Finalize Canvases') >= expected_count:
            time.sleep(1)
            return
        time.sleep(.2)
    raise RuntimeError('Deferred initialization did not finish')

def saved(step):
    actual = fixtures.snapshot(profile)
    assert actual == expected, f'{step}: persisted fixtures changed: {actual}'
    report['steps'].append({'step': step, 'persistence': 'pass'})

try:
    handle, pid = ui.wait_window('OpenNav X / OpenCPN', pid)
    ready(1)
    ui.capture(handle, evidence / '01-xnav-unavailable.png')
    ui.click_text(pid, 'Light')
    ui.capture(handle, evidence / '02-xnav-dusk.png')
    ui.click_text(pid, 'Light')
    ui.capture(handle, evidence / '03-xnav-night.png')
    ui.click_text(pid, 'Light')
    ui.click_text(pid, '+')
    ui.click_text(pid, '−')
    ui.click_text(pid, 'System')
    ui.click_text(pid, 'Start labelled simulation')
    ui.capture(handle, evidence / '04-xnav-simulation.png')
    ui.click_text(pid, 'System')
    ui.click_text(pid, 'Pause simulation')
    time.sleep(6)
    ui.capture(handle, evidence / '05-xnav-stale.png')
    report['steps'].append({'step': 'theme / zoom / simulator start and pause', 'interaction': 'pass',
                            'visual_review': 'required; stale labels and fixture values must be checked'})
    ui.click_text(pid, 'System')
    ui.click_text(pid, 'Open Legacy OpenCPN')
    assert process.wait(timeout=40) == 0, 'XNav did not exit cleanly'
    handle, pid = ui.wait_window('OpenCPN / Legacy')
    assert pid != process.pid, 'Mode change must use a new process'
    ready(2)
    saved('XNav to Legacy')
    ui.capture(handle, evidence / '11-legacy-after-xnav.png')
    old_process_handle = ui.monitor_process(pid)
    ui.click_menu(handle, 'Switch to XNav')
    ui.wait_clean_exit(old_process_handle)
    next_handle, next_pid = ui.wait_window('OpenNav X / OpenCPN')
    assert next_pid != pid
    handle, pid = next_handle, next_pid
    ready(3)
    saved('Legacy to XNav')
    ui.capture(handle, evidence / '06-xnav-after-legacy.png')
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
    ui.capture(handle, evidence / '12-safe-shared-profile.png')
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

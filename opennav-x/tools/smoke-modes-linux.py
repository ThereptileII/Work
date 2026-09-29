#!/usr/bin/env python3
"""Private X server interaction gate; native Windows remains visual authority."""
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
from diagnostic_snapshot import read_json_snapshot

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('fixtures', root / 'tools/profile-fixtures.py')
fixtures = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixtures)
evidence = root / 'evidence/local'
evidence.mkdir(parents=True, exist_ok=True)
# wx IPC uses an AF_UNIX pathname, limited to 107 bytes on Linux. Keep the
# disposable profile short even when the checkout lives in a long CI path.
temporary = tempfile.TemporaryDirectory(prefix='opennav cycle ', dir='/tmp')
profile = Path(temporary.name) / 'profile with spaces'
subprocess.run([sys.executable, str(root / 'tools/prepare-test-profile.py'), '--build',
                str(root / 'build/xnav-linux'), '--profile', str(profile)], check=True)
fixtures.seed(profile)
expected = fixtures.snapshot(profile)
# Adopt the restart helper, so every process exit can be checked, including
# the children that outlive their original OpenCPN parent.
if ctypes.CDLL(None).prctl(36, 1, 0, 0, 0) != 0:
    raise RuntimeError('Cannot become test-process subreaper')
display_number = 98
while Path(f'/tmp/.X{display_number}-lock').exists():
    display_number += 1
env = dict(os.environ, DISPLAY=f':{display_number}')
xserver = subprocess.Popen(['Xvfb', env['DISPLAY'], '-screen', '0', '1280x800x24', '-nolisten', 'tcp'],
                           env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
exe = root / 'build/xnav-install/bin/opencpn'
app = None
owned_pids = set()
report = {'profile': str(profile), 'authority': 'Linux development only', 'steps': [], 'expected': expected}

def xdo(*args):
    return subprocess.check_output(['xdotool', *map(str, args)], env=env, text=True).strip()

def window(title):
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        r = subprocess.run(['xdotool', 'search', '--onlyvisible', '--name', '^' + title + '$'],
                           env=env, text=True, capture_output=True)
        if r.returncode == 0 and r.stdout.strip():
            handle = r.stdout.splitlines()[0]
            pid = int(xdo('getwindowpid', handle))
            owned_pids.add(pid)
            xdo('windowsize', handle, 1280, 800)
            xdo('windowmove', handle, 0, 0)
            return handle, pid
        time.sleep(.1)
    raise RuntimeError(f'Window not found: {title}')

def ready(count):
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        p = profile / 'opencpn.log'
        if p.exists() and p.read_text(errors='replace').count('OnInitTimer...Finalize Canvases') >= count:
            # Pinned deferred resize schedules the frame's focus/raise a second
            # later. Capture after that startup callback, as the preview gate
            # does, so owned chart surfaces are not sampled mid-initialization.
            time.sleep(1.5)
            return
        time.sleep(.2)
    raise RuntimeError('Initialization did not finish')

def capture(name):
    time.sleep(.35)
    subprocess.run(['import', '-window', 'root', str(evidence / (name + '-linux.png'))], env=env, check=True)

def click(x, y, button=1):
    xdo('mousemove', x, y, 'click', button)
    time.sleep(.4)

def observe(predicate=lambda d: True):
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        d=read_json_snapshot(profile/'opennav-diagnostics.json')
        if predicate(d):return d
        time.sleep(.1)
    raise AssertionError('Current mode-cycle UI state did not arrive')

def action(label, light=None, page=None):
    before=observe();ticks=int(before['runtime']['ui_update']['ticks'])
    matches=[c for c in before['runtime']['display']['interaction_controls']
             if c['label']==label and c['visible'] and c['enabled']]
    assert len(matches)==1,('Unique visible mode-cycle control required',label,matches)
    c=matches[0]
    assert 0<=c['x']<c['x']+c['width']<=1280 and 0<=c['y']<c['y']+c['height']<=800
    click(c['x']+c['width']//2,c['y']+c['height']//2)
    observe(lambda d:int(d['runtime']['ui_update']['ticks'])>=ticks+3
            and (light is None or d['runtime']['display']['light']==light)
            and (page is None or d['ui_page']==page))

def saved(step):
    actual = fixtures.snapshot(profile)
    assert actual == expected, f'{step}: fixture data changed: {actual}'
    report['steps'].append({'step': step, 'persistence': 'pass'})

def wait_exit(pid):
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        child, status = os.waitpid(pid, os.WNOHANG)
        if child:
            owned_pids.discard(pid)
            assert os.waitstatus_to_exitcode(status) == 0, f'Process {pid} exit status {status}'
            return
        time.sleep(.1)
    raise RuntimeError(f'Process {pid} did not exit')

try:
    time.sleep(1)
    with (evidence / 'linux-mode-cycle-launch.log').open('w') as output:
        app = subprocess.Popen([str(exe), '--configdir', str(profile), '--no_opengl', '--xnav'],
                               env=env, stdout=output, stderr=output)
    owned_pids.add(app.pid)
    handle, pid = window('OpenNav X / OpenCPN')
    ready(1)
    capture('01-xnav-unavailable')
    action('Day',light='Dusk')
    capture('02-xnav-dusk')
    action('Dusk',light='Night')
    capture('03-xnav-night')
    action('Night',light='Day')
    action('+')
    action('−')
    action('Settings',page='Settings')
    action('System',page='Settings')
    action('Interface & recovery',page='System')
    capture('07-system')
    xdo('key', 'Escape')
    action('Chart',page='Navigation')
    xdo('windowfocus', handle)
    xdo('key', 'ctrl+shift+d')
    time.sleep(.5)
    capture('04-xnav-simulation')
    xdo('key', 'ctrl+shift+p')
    time.sleep(6)
    capture('05-xnav-stale')
    xdo('key', 'ctrl+shift+l')
    assert app.wait(timeout=30) == 0, 'Initial XNav exit failed'
    owned_pids.discard(app.pid)
    handle, pid = window('OpenCPN / Legacy')
    assert pid != app.pid
    ready(2)
    saved('XNav to Legacy')
    capture('11-legacy-after-xnav')
    # The final canvas context-menu item is the integration's mode fallback.
    click(600, 400, 3)
    xdo('key', 'End', 'Return')
    wait_exit(pid)
    handle, pid = window('OpenNav X / OpenCPN')
    ready(3)
    saved('Legacy to XNav')
    capture('06-xnav-after-legacy')
    assert (profile / 'opencpn-ipc').is_socket(), 'OpenCPN IPC socket was not created'
    subprocess.run([str(exe), '--configdir', str(profile), '--remote', '--quit'], env=env, check=True, timeout=15)
    wait_exit(pid)
    saved('Final IPC close')
    safe = subprocess.Popen([str(exe), '--configdir', str(profile), '--no_opengl',
                             '--xnav', '--legacy', '--safe-mode'], env=env,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    owned_pids.add(safe.pid)
    handle, pid = window('OpenNav Safe Mode / OpenCPN')
    ready(4)
    capture('12-safe-shared-profile')
    subprocess.run([str(exe), '--configdir', str(profile), '--remote', '--quit'], env=env, check=True, timeout=15)
    assert safe.wait(timeout=30) == 0
    owned_pids.discard(safe.pid)
    saved('Safe override and close')
    normal = subprocess.Popen([str(exe), '--configdir', str(profile), '--no_opengl'], env=env,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    owned_pids.add(normal.pid)
    handle, pid = window('OpenNav X / OpenCPN')
    ready(5)
    subprocess.run([str(exe), '--configdir', str(profile), '--remote', '--quit'], env=env, check=True, timeout=15)
    assert normal.wait(timeout=30) == 0
    owned_pids.discard(normal.pid)
    saved('Persisted XNav after Safe Mode')
    report['result'] = 'interaction and persistence passed; screenshot review required'
    print(report['result'])
finally:
    for pid in owned_pids:
        try:
            os.kill(pid, 15)
        except ProcessLookupError:
            pass
    xserver.terminate()
    xserver.wait(timeout=10)
    (evidence / 'linux-mode-cycle-results.json').write_text(json.dumps(report, indent=2))
    shutil.copytree(profile, evidence / 'linux-mode-cycle-profile', dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('opencpn-ipc', '*.pem'))
    temporary.cleanup()

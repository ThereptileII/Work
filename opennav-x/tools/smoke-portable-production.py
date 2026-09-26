#!/usr/bin/env python3
"""Native OFF-build recovery ZIP gate. No simulated inputs or test-mode switches.

Only disposable extracted files are used. The normal installation/profile is
hash-audited, and real bundled coastline pixels must survive every mode restart.
The separate fixture-enabled integration suite remains mandatory.
"""
import argparse
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
import zipfile

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--package', required=True, type=Path)
args = parser.parse_args()
if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
    raise SystemExit('Production ZIP smoke requires disposable native Windows CI')
EVIDENCE = ROOT / 'evidence/local'
EVIDENCE.mkdir(parents=True, exist_ok=True)
report = {'status': 'running', 'authority': 'Native Windows; exact extracted product ZIP; fixtures OFF',
          'checks': [], 'screenshots': [], 'chart_rendering': []}

def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / (name + '.py'))
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value

ui = module('windows-ui')
charts = module('chart-render-check')
report['display'] = ui.ensure_desktop()
normal_locations = []
for key in ('APPDATA', 'LOCALAPPDATA', 'PROGRAMDATA'):
    if os.environ.get(key):
        base = Path(os.environ[key])
        normal_locations.extend((base / 'opencpn', base / 'opencpn.ini', base / 'opencpn.log'))
for key in ('ProgramFiles', 'ProgramFiles(x86)'):
    if os.environ.get(key):
        normal_locations.append(Path(os.environ[key]) / 'OpenCPN')

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def inventory():
    return {str(file): sha(file) for location in normal_locations
            for file in (location.rglob('*') if location.is_dir() else [location]) if file.is_file()}

normal_before = inventory()
temporary = tempfile.TemporaryDirectory(prefix='OpenNav product recovery ')
temp = Path(temporary.name)
package = temp / 'OpenNavX-Beta2-Portable-Recovery'
profile = package / 'profile'
logs = package / 'logs'
exe = package / 'app/opencpn.exe'
process = None
handle = None
pid = None
starts = 0
owned = set()
env = dict(os.environ)
env['PATH'] = os.environ['SystemRoot'] + '\\System32;' + os.environ['SystemRoot']
fake = temp / 'unrelated normal profile'
fake.mkdir()
(fake / 'opencpn.ini').write_text('NORMAL PROFILE MUST REMAIN UNCHANGED\n')
env['APPDATA'] = str(fake)

def check(text):
    report['checks'].append(text)
    print(text, flush=True)

def ready():
    global starts
    starts += 1
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        log = profile / 'opencpn.log'
        if log.exists() and log.read_text(errors='replace').count('OnInitTimer...Finalize Canvases') >= starts:
            time.sleep(1.5)
            assert ui.IsWindowEnabled(handle), 'Application blocked by a modal dialog'
            return
        time.sleep(.2)
    raise AssertionError('Normal chart initialization did not complete')

def data(predicate=lambda value: True):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        try:
            value = json.loads((logs / 'opennav-diagnostics.json').read_text())
            if predicate(value):
                assert value['test_fixtures'] is False
                assert value['build_purpose'] == 'INSTALLED PRODUCT'
                assert value['data_mode'] == 'OPENCPN selected navigation'
                assert all(not str(item.get('source', '')).upper().startswith('DEMO') for item in value['data'])
                return value
        except (FileNotFoundError, PermissionError, json.JSONDecodeError):
            pass
        time.sleep(.2)
    raise AssertionError('Product diagnostics predicate did not become true')

def capture(name, chart=True):
    path = EVIDENCE / ('recovery-' + name + '.png')
    rgb = ui.capture(handle, path)
    report['screenshots'].append(path.name)
    if chart and 'colors' in globals():
        report['chart_rendering'].append(charts.check(rgb, colors, name))
    return rgb

def no_demo_controls():
    for _, caption in ui.children(handle):
        assert not any(word in caption.casefold() for word in ('demo', 'cruising', 'sensors stale', 'energy shortfall scenario')), caption


def preserved():
    assert inventory() == normal_before, 'Portable build changed a normal OpenCPN file'
    assert (fake / 'opencpn.ini').read_text() == 'NORMAL PROFILE MUST REMAIN UNCHANGED\n'
    assert sorted(file.name for file in fake.iterdir()) == ['opencpn.ini']


def launch(launcher, title):
    global process, handle, pid
    process = subprocess.Popen([os.environ['COMSPEC'], '/d', '/c', str(package / launcher)],
                               cwd=temp, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    handle, pid = ui.wait_window(title)
    owned.add(pid)
    ready()


def close():
    global handle, pid
    monitor = ui.monitor_process(pid)
    ui.close(handle)
    ui.wait_clean_exit(monitor)
    owned.discard(pid)
    handle = None
    pid = None
    if process and process.poll() is None:
        assert process.wait(timeout=15) == 0
    preserved()


def restart_xnav_from_legacy(name):
    global handle, pid
    previous = pid
    monitor = ui.monitor_process(pid)
    ui.click_menu(handle, 'Switch to XNav')
    ui.wait_clean_exit(monitor)
    owned.discard(previous)
    handle, pid = ui.wait_window('OpenNav X / OpenCPN')
    owned.add(pid)
    ready()
    data()
    no_demo_controls()
    capture(name)
    preserved()

try:
    with zipfile.ZipFile(args.package) as archive:
        for item in archive.infolist():
            parts = Path(item.filename).parts
            assert parts and parts[0] == package.name and '..' not in parts and not Path(item.filename).is_absolute()
            assert '\\' not in item.filename and ':' not in item.filename
        assert archive.testzip() is None
        archive.extractall(temp)
    manifest = json.loads((package / 'FILE_SHA256.json').read_text())
    assert {p.relative_to(package).as_posix() for p in package.rglob('*') if p.is_file()} == set(manifest) | {'FILE_SHA256.json'}
    for name, expected in manifest.items():
        assert sha(package / name) == expected, name
    assert not (package / 'Run-XNav-Demo.cmd').exists() and not (package / 'demo').exists()
    forbidden = {'OPENNAV_TEST_PROFILE', 'OPENNAV_ROUTE_FIXTURE', 'OPENNAV_OBJECT_FIXTURE', 'scenarios.json'}
    assert not any(file.name in forbidden or 'demo' in (part.casefold() for part in file.relative_to(package).parts) for file in package.rglob('*'))
    build = json.loads((package / 'docs/PRODUCT_BUILD.json').read_text())
    assert build['test_fixtures'] is False and build['build_purpose'] == 'INSTALLED PRODUCT'
    assert build['executable_sha256'] == sha(exe)
    selftest = temp / 'product-loader.json'
    tested = subprocess.run([str(exe), '--opennav-self-test', str(selftest)], env=env,
                            capture_output=True, timeout=30)
    assert tested.returncode == 0
    identity = json.loads(selftest.read_text())
    assert identity['test_fixtures'] is False and identity['build_purpose'] == 'INSTALLED PRODUCT'
    assert identity['version'] == '0.4.0-beta2' and identity['commit'] == os.environ['GITHUB_SHA']
    assert not identity['profile_initialized'] and not identity['plugins_loaded']
    report['build'] = build
    report['package_sha256'] = sha(args.package)
    report['files_verified'] = len(manifest)
    check('Exact extracted package inventory/hash and actual fixture-free executable identity verified')

    launch('Run-XNav.cmd', 'OpenNav X / OpenCPN')
    live = data()
    no_demo_controls()
    for name in ('Latitude', 'Battery SOC', 'Motor electrical power', 'Depth below transducer'):
        item = next(item for item in live['data'] if item['name'] == name)
        assert 'value' not in item, name + ' invented a sensor value without input'
    assert 'remaining_nm' not in live.get('route', {}) and 'arrival_soc' not in live.get('energy', {})
    colors = charts.reference(capture('navigation-day', chart=False))
    for mode in ('Dusk', 'Night', 'Day'):
        previous = data()['runtime']['display']['light']
        ui.click_text(pid, previous)
        data(lambda value: value['runtime']['display']['light'] == mode)
        rgb = capture('navigation-' + ('day-restored' if mode == 'Day' else mode.lower()), chart=mode == 'Day')
        if mode == 'Night':
            report['chart_rendering'].append(charts.night(rgb, colors, 'Product Night coastline'))
    ui.click_text(pid, 'Energy')
    data(lambda value: value['ui_page'] == 'Energy')
    capture('energy-unavailable', chart=False)
    no_demo_controls()
    ui.click_text(pid, 'Menu')
    capture('menu', chart=False)
    no_demo_controls()
    ui.click_text(pid, 'System')
    capture('system', chart=False)
    no_demo_controls()
    ui.click_text(pid, 'Diagnostics')
    data(lambda value: value['ui_page'] == 'Diagnostics')
    capture('diagnostics', chart=False)
    no_demo_controls()
    assert not any('ALPHA 1 / NOT FOR NAVIGATION' in caption for _, caption in ui.children(handle))
    ui.click_text(pid, 'Navigation')
    data(lambda value: value['ui_page'] == 'Navigation')
    capture('navigation-return')
    close()
    check('Out-of-box product shows real unavailable state, no Demo controls and Day/Dusk/Night chart content')

    launch('Run-Legacy.cmd', 'OpenCPN / Legacy')
    capture('legacy-start')
    restart_xnav_from_legacy('legacy-to-xnav')
    close()
    check('Legacy launcher and Legacy-to-XNav retain real coastline and isolated profile')

    launch('Run-XNav.cmd', 'OpenNav X / OpenCPN')
    capture('xnav-before-legacy')
    monitor = ui.monitor_process(pid)
    previous = pid
    ui.click_text(pid, 'System')
    ui.click_text(pid, 'Open Legacy OpenCPN')
    ui.wait_clean_exit(monitor)
    owned.discard(previous)
    handle, pid = ui.wait_window('OpenCPN / Legacy')
    owned.add(pid)
    ready()
    capture('xnav-to-legacy')
    restart_xnav_from_legacy('xnav-legacy-xnav')
    close()
    check('Product XNav-to-Legacy-to-XNav controlled restart preserves coastline')

    launch('Run-Safe.cmd', 'OpenNav Safe Mode / OpenCPN')
    capture('safe')
    restart_xnav_from_legacy('safe-to-xnav')
    close()
    check('Safe launcher and return to XNav render real chart content with clean process exits')
    refused = subprocess.run([str(exe), '--xnav', '--configdir', str(fake)], env=env,
                             capture_output=True, timeout=20)
    assert b'refuses a profile outside' in refused.stderr, refused.stderr
    preserved()
    check('External-profile override refused; normal installed files and profile canary unchanged')
    report['normal_files_audited'] = len(normal_before)
    report['status'] = 'passed'
except Exception as error:
    report['status'] = 'failed'
    report['error'] = repr(error)
    raise
finally:
    # Disposable CI only: clean up owned processes after preserving failure UI.
    for window, owner, title in ui.windows():
        if owner in owned:
            report.setdefault('failure_windows', []).append({'title': title, 'captions': [text for _, text in ui.children(window)]})
            try:
                ui.capture(window, EVIDENCE / ('recovery-failure-' + str(owner) + '.png'))
                ui.close(window)
            except Exception:
                pass
    if process and process.poll() is None:
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
    for source, name in ((profile, 'recovery-profile'), (logs, 'recovery-logs')):
        if source.exists():
            shutil.copytree(source, EVIDENCE / name, dirs_exist_ok=True,
                            ignore=shutil.ignore_patterns('*.pem', 'opencpn-ipc'))
    (EVIDENCE / 'production-recovery-results.json').write_text(json.dumps(report, indent=2) + '\n')
    temporary.cleanup()
print('Production recovery ZIP gate passed; native screenshot review remains required')

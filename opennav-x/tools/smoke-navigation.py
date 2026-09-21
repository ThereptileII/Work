#!/usr/bin/env python3
"""Loopback-only synthetic NMEA through OpenCPN's real input and XNav bridge."""
import datetime
import importlib.util
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time

route_fixture = sys.argv[1:] == ['--route-fixture']
if sys.argv[1:] and not route_fixture:
    raise SystemExit('Usage: smoke-navigation.py [--route-fixture]')
prefix = 'route' if route_fixture else 'navigation'
root = Path(__file__).resolve().parents[1]
windows = sys.platform == 'win32'
evidence = root / 'evidence/local'
evidence.mkdir(parents=True, exist_ok=True)
temporary = tempfile.TemporaryDirectory(prefix='opennav input ', dir=None if windows else '/tmp')
profile = Path(temporary.name) / 'profile'
variant = 'xnav-windows' if windows else 'xnav-linux'
subprocess.run([sys.executable, str(root / 'tools/prepare-test-profile.py'),
                '--build', str(root / 'build' / variant), '--profile', str(profile)], check=True)
if route_fixture:
    (profile / 'OPENNAV_ROUTE_FIXTURE').write_text('Explicit isolated integration-test driver.\n')
server = socket.socket()
server.bind(('127.0.0.1', 0))
server.listen(1)
server.settimeout(.5)
port = server.getsockname()[1]
with (profile / 'opencpn.conf').open('a') as stream:
    # TCP client, checksums required, input only, enabled, loopback peer only.
    stream.write('\n[Settings/NMEADataSource]\nDataConnections='
                 f'1;0;127.0.0.1;{port};0;;4800;1;0;0;;0;;0;0;0;0;1;'
                 'SIMULATED loopback navigation fixture;0;;0;1;\n')
stop = threading.Event()
connected = threading.Event()
phase = ['none']
counts = {'rmc': 0, 'gga': 0}
failures = []

def sentence(body):
    checksum = 0
    for byte in body.encode('ascii'):
        checksum ^= byte
    return f'${body}*{checksum:02X}\r\n'.encode('ascii')

def transmit():
    peer = None
    try:
        while not stop.is_set():
            try:
                peer, address = server.accept()
                assert address[0] == '127.0.0.1'
                break
            except TimeoutError:
                pass
        if peer is None:
            return
        peer.settimeout(2)
        connected.set()
        while not stop.wait(.3):
            mode = phase[0]
            if mode == 'none':
                continue
            now = datetime.datetime.now(datetime.timezone.utc)
            utc = now.strftime('%H%M%S')
            # Fixed synthetic position. Sent only into this disposable profile.
            peer.sendall(sentence(f'GPGGA,{utc},5642.000,N,01236.000,E,1,08,1.0,0.0,M,0.0,M,,'))
            counts['gga'] += 1
            if mode == 'rmc':
                peer.sendall(sentence(f'GPRMC,{utc},A,5642.000,N,01236.000,E,6.3,147.0,{now:%d%m%y},,,A'))
                counts['rmc'] += 1
    except Exception as error:
        if not stop.is_set():
            failures.append(str(error))
    finally:
        if peer:
            peer.close()

thread = threading.Thread(target=transmit, daemon=True)
thread.start()
env = dict(os.environ)
xserver = app = None
report = {'authority': 'native Windows' if windows else 'Linux development',
          'fixture': 'Synthetic NMEA over loopback; no external devices or production profile',
          'expected': {'sog_kn': 6.3, 'cog_deg': 147, 'wind': 'unavailable', 'depth': 'unavailable'},
          'screenshots': [], 'visual_review': 'required'}

try:
    if windows:
        spec = importlib.util.spec_from_file_location('ui', root / 'tools/windows-ui.py')
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
        report['display'] = ui.ensure_desktop()
        exe = root / 'build/xnav-install/opencpn.exe'
    else:
        display = 101
        while Path(f'/tmp/.X{display}-lock').exists():
            display += 1
        env['DISPLAY'] = f':{display}'
        xserver = subprocess.Popen(['Xvfb', env['DISPLAY'], '-screen', '0', '1280x800x24', '-nolisten', 'tcp'],
                                   env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1)
        exe = root / 'build/xnav-install/bin/opencpn'
    with (evidence / f'{prefix}-input-launch.log').open('w') as output:
        app = subprocess.Popen([str(exe), '--configdir', str(profile), '--no_opengl', '--xnav'] + (['--xnav-route-fixture'] if route_fixture else []),
                               env=env, stdout=output, stderr=output)
    deadline = time.monotonic() + 60
    while True:
        assert app.poll() is None, 'Application exited during navigation test'
        log = profile / 'opencpn.log'
        if log.exists() and 'OnInitTimer...Finalize Canvases' in log.read_text(errors='replace'):
            break
        if time.monotonic() >= deadline:
            raise RuntimeError('Deferred initialization did not finish')
        time.sleep(.2)
    assert connected.wait(10), 'OpenCPN did not connect to loopback fixture'
    if windows:
        handle, _ = ui.wait_window('OpenNav X / OpenCPN', app.pid)
    else:
        handle = subprocess.check_output(['xdotool', 'search', '--onlyvisible', '--pid', str(app.pid),
                    '--name', '^OpenNav X / OpenCPN$'], env=env, text=True).splitlines()[0]
        subprocess.run(['xdotool', 'windowsize', handle, '1280', '800', 'windowmove', handle, '0', '0'], env=env, check=True)

    def capture(name):
        path = evidence / f'{prefix}-{name}.png'
        if windows:
            ui.capture(handle, path)
        else:
            subprocess.run(['import', '-window', 'root', str(path)], env=env, check=True)
        report['screenshots'].append(path.name)

    if route_fixture:
        phase[0] = 'rmc'
        deadline = time.monotonic() + 100
        seen_live = seen_stale = False
        while time.monotonic() < deadline:
            assert app.poll() is None, 'Route fixture process exited'
            result_file = profile / 'route-fixture-results.json'
            if result_file.exists():
                result = json.loads(result_file.read_text())
                assert result['result'] != 'failed', result
                checks = result.get('checks', [])
                if any(x['check'] == 'middle point real upstream progress' for x in checks) and not seen_live:
                    capture('01-active-route')
                    seen_live = True
                if result.get('phase') == 'stop-input':
                    phase[0] = 'none'
                elif result.get('phase') == 'resume-input':
                    if not seen_stale:
                        capture('02-stale-position')
                        seen_stale = True
                    phase[0] = 'rmc'
                if result['result'] == 'passed':
                    assert seen_live and seen_stale, 'Missing route scenario captures'
                    report['route_contract'] = result
                    (evidence / 'route-progress-results.json').write_text(json.dumps(result, indent=2))
                    break
            time.sleep(.2)
        else:
            raise RuntimeError('Route fixture did not finish normal navigation passes')
        assert not failures, failures
    else:
        capture('01-unavailable')
        phase[0] = 'rmc'
        time.sleep(3)
        assert counts['rmc'] >= 5 and not failures
        if windows:
            assert any(caption == 'OpenCPN navigation' for _, caption in ui.children(handle)), 'UI did not receive selected data'
        capture('02-live')
        phase[0] = 'gga'
        time.sleep(6.2)
        capture('03-position-only-velocity-stale')
        phase[0] = 'none'
        time.sleep(6.2)
        if windows:
            assert any(caption == 'Navigation stale' for _, caption in ui.children(handle)), 'UI did not age stopped data'
        capture('04-all-stale')
        assert not failures, failures
    stop.set()
    thread.join(timeout=3)
    if windows:
        ui.close(handle)
    else:
        subprocess.run([str(exe), '--configdir', str(profile), '--remote', '--quit'], env=env, check=True, timeout=15)
    assert app.wait(timeout=30) == 0, 'Navigation test did not close cleanly'
    report['result'] = 'loopback transport and lifecycle passed; numeric and stale screenshot review required'
finally:
    stop.set()
    thread.join(timeout=3)
    server.close()
    if app and app.poll() is None:
        app.terminate()
        app.wait(timeout=10)
    if xserver:
        xserver.terminate()
        xserver.wait(timeout=10)
    report['sent'] = counts
    report['transport_errors'] = failures
    (evidence / f'{prefix}-input-results.json').write_text(json.dumps(report, indent=2))
    shutil.copytree(profile, evidence / f'{prefix}-input-profile', dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('opencpn-ipc', '*.pem'))
    temporary.cleanup()

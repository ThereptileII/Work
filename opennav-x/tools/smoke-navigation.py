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
instruments = sys.argv[1:] == ['--instruments']
objects = sys.argv[1:] == ['--objects']
if sys.argv[1:] and not (route_fixture or instruments or objects):
    raise SystemExit('Usage: smoke-navigation.py [--route-fixture|--instruments|--objects]')
prefix = 'objects' if objects else 'route' if route_fixture else 'instruments' if instruments else 'navigation'
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
if objects:
    (profile / 'OPENNAV_OBJECT_FIXTURE').write_text('Explicit isolated object integration test.\n')
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
            if mode in ('rmc', 'invalid'):
                peer.sendall(sentence(f'GPRMC,{utc},A,5642.000,N,01236.000,E,6.3,147.0,{now:%d%m%y},,,A'))
                counts['rmc'] += 1
                if instruments:
                    bodies = ['IIHDT,149,T','IIVHW,149,T,145,M,6,N,11.1,K','IIDPT,8.4,-2',
                              'IIMWV,72,R,16.2,N,A','IIMWV,94,T,12.8,N,A',
                              'IIMTW,15.4,C','IIRSA,-4,A,,V']
                    if mode == 'invalid':
                        bodies = ['IIHDT,,T','IIVHW,,T,,M,,N,,K','IIDPT,,-2',
                                  'IIMWV,72,R,16.2,N,V','IIMWV,94,T,12.8,N,V',
                                  'IIMTW,,C','IIRSA,-4,V,,V']
                    for body in bodies: peer.sendall(sentence(body))
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
if instruments:
    report['expected'] = {'selected_sog_kn': 6.3, 'selected_cog_deg': 147,
                          'depth_below_transducer_m': 8.4, 'heading_true_deg': 149,
                          'stw_kn': 6, 'aws_kn': 16.2, 'awa_deg': 72,
                          'tws_kn': 12.8, 'twa_deg': 94, 'rudder_deg': -4,
                          'water_temperature_c': 15.4}

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
        app = subprocess.Popen([str(exe), '--configdir', str(profile), '--no_opengl', '--xnav'] + (['--xnav-route-fixture'] if route_fixture else ['--xnav-object-fixture'] if objects else []),
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

    if objects:
        phase[0]='rmc';deadline=time.monotonic()+70;seen=set()
        while time.monotonic()<deadline:
            assert app.poll() is None,'Object fixture exited'
            path=profile/'objects-fixture-results.json'
            if path.exists():
                result=json.loads(path.read_text());assert result['result']!='failed',result
                current=result.get('phase','')
                if current in ['route-card','waypoint-card','ais-card'] and current not in seen:
                    time.sleep(.6);capture(current);seen.add(current)
                if result['result']=='passed':
                    assert len(seen)==3,seen
                    report['object_contract']=result;break
            time.sleep(.2)
        else:raise RuntimeError('Object contract fixture timed out')
        assert not failures,failures
        if windows:
            ui.click_text(app.pid,'Menu');ui.click_text(app.pid,'Waypoints')
            ui.click_text(app.pid,'ALPHA TEST edited / mark');ui.click_text(app.pid,'Edit waypoint')
            ui.set_text_in_dialog(app.pid,'Edit waypoint','ALPHA TEST edited','ALPHA TEST UI edited')
            ui.click_text(app.pid,'Save');time.sleep(.6)
            assert any(c=='ALPHA TEST UI edited' for _,c in ui.children(handle))
            capture('waypoint-edit-sheet-result')
            ui.click_text(app.pid,'Delete isolated waypoint');ui.click_text(app.pid,'Cancel')
            assert any(c=='ALPHA TEST UI edited' for _,c in ui.children(handle)), 'Cancel changed the mark'
            ui.click_text(app.pid,'Edit waypoint')
            ui.set_text_in_dialog(app.pid,'Edit waypoint','ALPHA TEST UI edited','ALPHA TEST edited')
            ui.click_text(app.pid,'Save')
            report['native_edit_confirmation']='Themed property sheet saves and refreshes; delete cancellation preserves mark'
    elif instruments:
        def snapshot(name):
            record=json.loads((profile/'opennav-diagnostics.json').read_text())
            (evidence/f'instruments-{name}.json').write_text(json.dumps(record,indent=2))
            return {item['name']:item for item in record['data']}
        capture('01-unavailable')
        phase[0]='rmc';time.sleep(3)
        values=snapshot('02-live')
        expected={'Heading':149,'Speed through water':6,'Depth below transducer':8.4,
                  'Apparent wind speed':16.2,'Apparent wind angle':72,
                  'True wind speed':12.8,'True wind angle':94,'Rudder':-4,'Water temperature':15.4}
        for name,value in expected.items():
            assert abs(values[name]['value']-value)<.01,(name,values[name])
            assert 'NMEA0183' in values[name]['source'],values[name]
            assert values[name]['quality'] in ('LIVE','ESTIMATED'),values[name]
        capture('02-live')
        phase[0]='gga';time.sleep(6.2);values=snapshot('03-stale')
        for name in expected: assert values[name]['quality']=='STALE',(name,values[name])
        capture('03-stale')
        phase[0]='invalid';time.sleep(3);values=snapshot('04-unavailable')
        for name in expected: assert 'value' not in values[name],(name,values[name])
        capture('04-unavailable')
        report['checks']=['Upstream loopback -> marine bridge -> live UI snapshots',
                          'Depth offset not applied; sensor meanings and source preserved',
                          'Position-only updates cannot refresh instruments',
                          'Invalid sensor status and empty fields suppress retained values']
        assert not failures,failures
    elif route_fixture:
        phase[0] = 'rmc'
        deadline = time.monotonic() + 100
        seen_live = seen_stale = False
        while time.monotonic() < deadline:
            assert app.poll() is None, 'Route fixture process exited'
            result_file = profile / 'route-fixture-results.json'
            if result_file.exists():
                result = json.loads(result_file.read_text())
                assert result['result'] in ('running', 'failed', 'passed'), 'Invalid route fixture report state'
                assert result['result'] != 'failed', result
                checks = result.get('checks', [])
                if any(x['check'] == 'middle point real upstream progress' for x in checks) and not seen_live:
                    capture('01-active-route')
                    seen_live = True
                if result.get('phase') == 'stop-input':
                    phase[0] = 'none'
                elif result.get('phase') == 'resume-input':
                    if not seen_stale:
                        # Route observation and the shell's 250 ms repaint have
                        # independent schedules. Keep input stopped while the
                        # visible rail catches up to the proven stale contract.
                        time.sleep(.6)
                        if windows:
                            assert any(caption == 'Navigation stale' for _, caption in ui.children(handle))
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
    if objects:
        import sqlite3
        from contextlib import closing
        with closing(sqlite3.connect(profile/'navobj.db')) as db:
            assert db.execute('select name from routes where guid=?',('OPENNAV-ALPHA-OBJECT-ROUTE',)).fetchone()==('ALPHA TEST renamed route',)
            assert db.execute('select count(*) from routepoints where Name=?',('ALPHA TEST edited',)).fetchone()==(1,)
        report['checks']=['Navigation object and AIS contracts through actual integrated executable',
                          'Deferred chart-selection cards and clean close',
                          'Route and restored waypoint persisted in existing OpenCPN navobj.db']
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

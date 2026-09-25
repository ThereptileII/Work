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
from diagnostic_snapshot import read_json_snapshot

route_fixture = sys.argv[1:] == ['--route-fixture']
instruments = sys.argv[1:] == ['--instruments']
objects = sys.argv[1:] == ['--objects']
boat = sys.argv[1:] == ['--boat']
n2k = sys.argv[1:] == ['--n2k'] or boat
if sys.argv[1:] and not (route_fixture or instruments or objects or n2k):
    raise SystemExit('Usage: smoke-navigation.py [--route-fixture|--instruments|--objects|--n2k|--boat]')
prefix = 'boat' if boat else 'n2k' if n2k else 'objects' if objects else 'route' if route_fixture else 'instruments' if instruments else 'navigation'
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
                 f'1;0;127.0.0.1;{port};{1 if n2k else 0};;4800;1;0;0;;0;;0;0;0;0;1;'
                 'SIMULATED loopback navigation fixture;0;;0;1;\n')
    if boat:
        iface=f'TCP:127.0.0.1:{port}'
        settings={k:'' for k in ['corridor','draft','efficiency','hotel','margin']}
        settings.update({'capacity':'24','reserve':'20','minimum_speed':'0.5',
            'battery':f'NMEA2000/{iface}/NAME-40328200ffd23456/source-35/instance-0',
            'current':'charge','consumption':'measured','model_source':'Explicit isolated commissioning test',
            'boat_bridge.interface':iface,'boat_bridge.name':'40328200ffd23456'})
        record='OpenNavXSettings 1\n'+''.join(json.dumps(k)+' '+json.dumps(v)+'\n' for k,v in sorted(settings.items()))
        stream.write('\n[OpenNav]\nAlphaSettings='+record.replace('\\','\\\\').replace('\n','\\n')+'\n')
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
    n2k_name = bytes.fromhex('5634d2ff00823240' if boat else '4523c1ff008750c0')
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
        while not stop.wait(.1 if boat else .3):
            mode = phase[0]
            if mode == 'none':
                continue
            if n2k:
                if mode == 'gga':
                    continue  # Keep socket alive, stop every marine source.
                payloads = {
                    127751: bytes.fromhex('0700660dccf7ffff'),
                    127506: bytes.fromhex('07000044ffffffffffffff'),
                    127489: bytes.fromhex('00ffffffffeb82' + 'ff' * 19),
                    127488: bytes.fromhex('00d00cffffffffff'),
                    127493: bytes.fromhex('00fcffffffffffff')}
                if mode == 'invalid':
                    payloads = {p: bytes([0] + [255] * (len(b) - 1)) for p, b in payloads.items()}
                    payloads[127751] = bytes.fromhex('0700ffffffff7fff')
                    payloads[127506] = bytes.fromhex('070000ffffffffffffffff')
                    payloads[127493] = bytes.fromhex('00ffffffffffffff')
                if mode == 'reidentified':
                    n2k_name = bytes.fromhex('4623c1ff008750c0')
                if boat:
                    payloads[127505]=bytes.fromhex('006842e8030000ff') # Virtual 68% / 100 L
                    if mode != 'noheartbeat':
                        version=1 if mode=='oldproducer' else 2
                        flags=1 if version==1 else 0x81 if mode=='expired' else 0xf1
                        payloads={61184:bytes([7,version,2,flags,255,255,255,255]),**payloads}
                payloads = {60928: n2k_name, **payloads}
                # Actisense complete-PGN ASCII, source 35, destination 255,
                # priority 6. Existing OpenCPN network driver owns framing.
                data = ''.join(f'A001001.732 23FF6 {p:05X} {b.hex().upper()}\r\n'
                               for p, b in payloads.items())
                if mode == 'conflict':
                    data += f'A001001.732 24FF6 0EE00 {n2k_name.hex().upper()}\r\n'
                peer.sendall(data.encode('ascii'))
                counts['rmc'] += 1  # Legacy counter name: one synthetic batch.
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
if n2k:
    report['expected'] = {'battery_voltage_v': 343, 'battery_current_source_a': -21,
                          'soc_percent': 68, 'motor_rpm': 820, 'coolant_c': 62, 'gear': 'Forward'}
if boat:
    report['expected'].pop('coolant_c')
    report['expected'].update({'motor_c':62,'virtual_fuel':'suppressed','regeneration':'Two bars',
                               'v1_quality':'UNCERTAIN','v2_expiry':'per sensor group'})
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
                result=read_json_snapshot(path);assert result['result']!='failed',result
                current=result.get('phase','')
                if current in ['route-card','waypoint-card','ais-card'] and current not in seen:
                    time.sleep(.6);capture(current);seen.add(current)
                if current=='ais-advice' and not report.get('live_ais_advice'):
                    sample=read_json_snapshot(profile/'opennav-diagnostics.json')
                    if sample['runtime'].get('smartnav',{}).get('ais_event_count',0)>0:
                        report['live_ais_advice']='Copied actual AIS alarm reaches shell SmartNav at a coherent observation epoch'
                if result['result']=='passed':
                    assert report.get('live_ais_advice'),'No actual AIS advisory observed'
                    assert len(seen)==3,seen
                    report['object_contract']=result;break
            time.sleep(.2)
        else:raise RuntimeError('Object contract fixture timed out')
        assert not failures,failures
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            ready=read_json_snapshot(profile/'opennav-diagnostics.json')
            if ready['ui_page']=='AIS target' and not ready['runtime'].get('alerts'):break
            time.sleep(.15)
        else:raise AssertionError('AIS fixture alarm did not resolve before card interaction')
        if windows:ui.click_text(app.pid,'Select target on chart')
        else:subprocess.run(['xdotool','mousemove','600','230','click','1'],env=env,check=True)
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            selected=read_json_snapshot(profile/'opennav-diagnostics.json')
            if selected['ui_page']=='Navigation' and selected['runtime'].get('ais_selected_mmsi')==990000001:break
            time.sleep(.2)
        else:raise AssertionError(('AIS chart selection did not become current',selected))
        capture('ais-selected-chart')
        report['ais_selection']='Actual target card to existing chart target/frame; copied selection identity reported'
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
    elif boat:
        def snapshot(name):
            record=read_json_snapshot(profile/'opennav-diagnostics.json')
            (evidence/f'boat-{name}.json').write_text(json.dumps(record,indent=2))
            return record,{v['name']:v for v in record['data']}
        phase[0]='oldproducer';time.sleep(3)
        record,values=snapshot('v1-uncertain')
        assert values['Battery SOC']['quality']=='UNCERTAIN',values['Battery SOC']
        assert values['Motor temperature']['quality']=='UNCERTAIN',values['Motor temperature']
        assert 'value' not in values['Whole-pack net discharge']
        assert 'value' not in values['Fuel tank']
        if windows:ui.click_text(app.pid,'Energy')
        else:subprocess.run(['xdotool','mousemove','275','764','click','1'],env=env,check=True)
        capture('01-producer-unverified')
        phase[0]='rmc';time.sleep(3)
        record,values=snapshot('v2-live')
        for name,value in {'Battery SOC':68,'Battery voltage':343,'Motor speed':820,'Motor temperature':62}.items():
            assert abs(values[name]['value']-value)<.01 and values[name]['quality']=='LIVE',(name,values[name])
        assert abs(values['Whole-pack net discharge']['value']-7.203)<.001
        assert 'value' not in values['Engine coolant temperature'] and 'value' not in values['Fuel tank']
        regen=next(v for v in record['text_data'] if v['name']=='Regeneration')
        assert regen['value']=='Two bars' and regen['quality']=='LIVE',regen
        capture('02-verified-marine-fields')
        phase[0]='expired';time.sleep(2)
        record,values=snapshot('producer-expired')
        for name in ['Battery SOC','Battery voltage','Motor speed','Motor temperature','Whole-pack net discharge']:
            assert 'value' not in values[name],(name,values[name])
        assert next(v for v in record['text_data'] if v['name']=='Gear')['value']=='Forward'
        capture('03-sensor-expiry-with-network-live')
        phase[0]='noheartbeat';time.sleep(2)
        record,values=snapshot('heartbeat-lost')
        assert values['Battery SOC']['quality']=='UNCERTAIN' and 'value' not in values['Whole-pack net discharge']
        phase[0]='rmc';time.sleep(2)
        record,values=snapshot('recovered')
        assert values['Battery SOC']['quality']=='LIVE'
        report['checks']=['Actual OpenCPN bus with explicit NAME-bound boat mapping',
            'Legacy producer values uncertain; no net power or fictitious fuel',
            'v2 per-group expiry, real motor field, regeneration and coherent V x I',
            'Continuing network traffic cannot mask expired EV sensor groups',
            'Heartbeat loss suppresses dependent estimates; new sensor input recovers']
        assert not failures,failures
    elif n2k:
        def n2k_snapshot(name):
            record = read_json_snapshot(profile/'opennav-diagnostics.json')
            (evidence/f'n2k-{name}.json').write_text(json.dumps(record, indent=2))
            return record, {item['name']: item for item in record['data']}
        phase[0] = 'rmc'
        time.sleep(4)
        record, values = n2k_snapshot('live')
        expected = {'Battery voltage': 343, 'Battery current (source convention)': -21,
                    'Battery SOC': 68, 'Motor speed': 820, 'Engine coolant temperature': 62}
        for name, value in expected.items():
            assert abs(values[name]['value'] - value) < .01, (name, values[name])
            assert values[name]['quality'] == 'LIVE', values[name]
            assert 'NMEA2000' in values[name]['source'], values[name]
        pack_names=['Battery SOC','Battery voltage','Battery current (source convention)']
        first_pack=values['Battery SOC']['device_id']
        assert '/NAME-c0508700ffc12345/' in first_pack,first_pack
        assert all(values[n]['device_id']==first_pack for n in pack_names),values
        assert 'value' not in values['Motor temperature']
        assert 'value' not in values['Whole-pack net discharge'], 'Current sign must be configured'
        assert 'value' not in values['Latitude'], 'Instrument source cannot fabricate GPS'
        gear = next(v for v in record['text_data'] if v['name'] == 'Gear')
        assert gear['value'] == 'Forward' and gear['quality'] == 'LIVE', gear
        assert any(x.get('frequency_hz', 0) > 1 and int(x['observations']) > 3
                   for x in record['source_candidates']), record.get('source_candidates')
        if windows:
            assert any(c == 'GPS unavailable / Marine input' for _, c in ui.children(handle))
            ui.click_text(app.pid, 'Energy')
        else:
            subprocess.run(['xdotool', 'mousemove', '275', '764', 'click', '1'], env=env, check=True)
        time.sleep(.5);capture('01-live-energy')
        phase[0]='reidentified';time.sleep(2)
        record,values=n2k_snapshot('reidentified')
        second_pack=values['Battery SOC']['device_id']
        assert second_pack!=first_pack and '/NAME-c0508700ffc12346/' in second_pack
        assert all(values[n]['device_id']==second_pack for n in pack_names),values
        assert all(first_pack not in s['device_id'] for s in record['source_candidates'])
        phase[0] = 'gga';time.sleep(6.2)
        record, values = n2k_snapshot('stale')
        for name in expected:
            assert values[name]['quality'] == 'STALE', (name, values[name])
        assert all('frequency_hz' not in v for v in record['source_candidates'])
        gear = next(v for v in record['text_data'] if v['name'] == 'Gear')
        assert gear['quality'] == 'STALE', gear
        capture('02-stale-energy')
        phase[0] = 'invalid';time.sleep(3)
        record, values = n2k_snapshot('invalid')
        for name in expected:
            assert 'value' not in values[name], (name, values[name])
        assert all(v['state'] == 'INVALID' and int(v['invalid_observations']) > 0
                   for v in record['source_candidates']), record['source_candidates']
        assert 'value' not in next(v for v in record['text_data'] if v['name'] == 'Gear')
        capture('03-invalid-energy')
        phase[0]='conflict';time.sleep(2)
        record,values=n2k_snapshot('conflict')
        assert all('value' not in values[n] for n in expected),values
        assert not record.get('source_candidates'),record.get('source_candidates')
        report['checks'] = ['Actual OpenCPN TCP N2K driver -> NavMsgBus -> MarineBridge -> owned snapshots',
                            'HV voltage/current/SOC/RPM/coolant/gear byte fixtures and provenance',
                            'Real address claim unifies SOC and voltage/current pack identity; reassignment removes old samples',
                            'Duplicate NAME at another address suppresses ambiguous instrument data',
                            'No fabricated GPS, motor-temperature mapping or configured power',
                            'Source cadence and invalid counts; dropout suppresses live rate',
                            'N2K sensor loss/NA invalidates energy inputs and discrete gear']
        assert not failures, failures
    elif instruments:
        def snapshot(name):
            record=read_json_snapshot(profile/'opennav-diagnostics.json')
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
                result = read_json_snapshot(result_file)
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

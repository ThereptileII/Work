#!/usr/bin/env python3
"""Loopback-only synthetic NMEA through OpenCPN's real input and XNav bridge."""
import argparse
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


def exact_native_reference_desktop(display):
    """Use product fullscreen only for the authoritative 1280x800 desktop."""
    after = display.get('after', ())
    return len(after) >= 2 and tuple(after[:2]) == (1280, 800)


parser = argparse.ArgumentParser(description=__doc__)
mode = parser.add_mutually_exclusive_group()
for flag in ('route-fixture', 'route-fixture-standard', 'instruments', 'objects', 'n2k', 'boat'):
    mode.add_argument('--'+flag, action='store_true')
parser.add_argument('--theme', choices=('Day', 'Dusk', 'Night'), default='Day')
parser.add_argument('--renderer', choices=('software', 'opengl'), default='software')
args = parser.parse_args()
route_standard = args.route_fixture_standard
route_fixture = args.route_fixture or route_standard
instruments, objects, boat = args.instruments, args.objects, args.boat
n2k = args.n2k or boat
if not route_fixture and (args.theme != 'Day' or args.renderer != 'software'):
    parser.error('Theme/renderer variants are restricted to the isolated route fixture')
prefix = 'boat' if boat else 'n2k' if n2k else 'objects' if objects else 'route-standard' if route_standard else 'route' if route_fixture else 'instruments' if instruments else 'navigation'
if args.theme != 'Day': prefix += '-'+args.theme.lower()
if args.renderer != 'software': prefix += '-'+args.renderer
root = Path(__file__).resolve().parents[1]
windows = sys.platform == 'win32'
design_validation = os.environ.get('SKAGER_DESIGN_VALIDATION') == 'true'
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
    if route_fixture:
        # Fixed viewport contains the actual upstream-created route around the
        # loopback vessel. No private chart material or independent geometry.
        stream.write('\n[Settings]\nOpenGL=' + str(int(args.renderer == 'opengl')) + '\n'
                     '[Settings/GlobalState]\nVPLatLon=56.9000,12.8000\nVPScale=0.009\n'
                     'nColorScheme=' + str(('Day','Dusk','Night').index(args.theme)+1) + '\n'
                     '[OpenNav]\nChartPresentationV1=' + ('Standard' if route_standard else 'XNav') + '\n')
    # TCP client, checksums required, input only, enabled, loopback peer only.
    if objects:
        # The marked scenario adds this real input connection AFTER startup,
        # using the same UpdateDatastreams path as OpenCPN's connection editor.
        # This deliberately tests the reported add-GPS-without-restart path.
        stream.write('\n[Settings/NMEADataSource]\nDataConnections=\n'
                     '[Settings/GlobalState]\nVPLatLon=59.0800,18.5000\nVPScale=0.003\n')
        (profile / 'OPENNAV_OBJECT_INPUT_PORT').write_text(str(port) + '\n')
    else:
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
counts = {'rmc': 0, 'gga': 0, 'ais': 0}
failures = []

def sentence(body, delimiter='$'):
    checksum = 0
    for byte in body.encode('ascii'):
        checksum ^= byte
    return f'{delimiter}{body}*{checksum:02X}\r\n'.encode('ascii')


def ais_position(mmsi):
    # Type 1 positions match the pinned ais_decoder.cpp::Parse_VDXBitstring
    # one-based fields (MMSI 9/30, SOG 51/10, lon 62/28, lat 90/27,
    # COG 117/12, HDG 129/9). No copied model object substitutes for decoding.
    bits = ['0'] * 168
    def field(first, width, value):
        assert 0 <= value < 1 << width
        bits[first - 1:first - 1 + width] = f'{value:0{width}b}'
    field(1, 6, 1)
    field(9, 30, mmsi)
    field(43, 8, 128)  # turn rate unavailable
    field(51, 10, 70)  # 7 kn
    field(62, 28, round(12.9 * 600000))
    field(90, 27, round(56.82 * 600000))
    field(138, 6, 60)  # no fabricated UTC second
    armored = ''
    for i in range(0, len(bits), 6):
        value = int(''.join(bits[i:i + 6]), 2)
        armored += chr(value + 48 + (8 if value >= 40 else 0))
    return sentence(f'AIVDM,1,1,,A,{armored},0', '!')

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
                if objects:
                    peer.sendall(ais_position(990000002))
                    malformed = bytearray(ais_position(990000003))
                    malformed[-4] = ord('0') if malformed[-4] != ord('0') else ord('1')
                    peer.sendall(malformed)  # wrong checksum must not create a target
                    counts['ais'] += 1
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
          'screenshots': [], 'visual_review': 'requested' if design_validation else 'not requested'}
if objects:
    report['time_environment'] = {'TZ': os.environ.get('TZ', '(system)'),
                                  'names': list(time.tzname),
                                  'utc_offset_seconds': datetime.datetime.now().astimezone().utcoffset().total_seconds()}
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
    native_reference_fullscreen = False
    if windows:
        spec = importlib.util.spec_from_file_location('ui', root / 'tools/windows-ui.py')
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
        report['display'] = ui.ensure_desktop()
        native_reference_fullscreen = exact_native_reference_desktop(report['display'])
        exe = root / 'build/xnav-install/opencpn.exe'
    else:
        env['GDK_BACKEND'] = 'x11'
        env.pop('WAYLAND_DISPLAY', None)
        display = 101
        while Path(f'/tmp/.X{display}-lock').exists():
            display += 1
        env['DISPLAY'] = f':{display}'
        xserver = subprocess.Popen(['Xvfb', env['DISPLAY'], '-screen', '0', '1280x800x24', '-nolisten', 'tcp'],
                                   env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1)
        exe = root / 'build/xnav-install/bin/opencpn'
    launch = [str(exe), '--configdir', str(profile), '--xnav']
    if native_reference_fullscreen:
        # A decorated 1280x800 outer window extends into the taskbar's reserved
        # area on an exact 1280x800 desktop. Exercise OpenCPN's supported native
        # fullscreen path so visible-pointer checks address the product surface.
        launch.append('--fullscreen')
    launch += (['--no_opengl'] if args.renderer == 'software' else [])
    launch += (['--xnav-route-fixture'] if route_fixture else
               ['--xnav-object-fixture'] if objects else [])
    with (evidence / f'{prefix}-input-launch.log').open('w') as output:
        app = subprocess.Popen(launch,
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
    resize_tick=int(read_json_snapshot(profile/'opennav-diagnostics.json')['runtime']['ui_update']['ticks'])
    if windows:
        handle, _ = ui.wait_window('SKAGER / OpenCPN', app.pid)
        if native_reference_fullscreen:
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                outer=ui.W.RECT();client=ui.W.RECT();origin=ui.W.POINT(0,0)
                assert ui.GetWindowRect(handle,ui.C.byref(outer))
                assert ui.GetClientRect(handle,ui.C.byref(client))
                assert ui.ScreenToClient(handle,ui.C.byref(origin))
                native_surface={
                    'desktop_pixels':[1280,800],
                    'outer':{'x':outer.left,'y':outer.top,
                             'width':outer.right-outer.left,'height':outer.bottom-outer.top},
                    'client':{'x':-origin.x,'y':-origin.y,
                              'width':client.right-client.left,'height':client.bottom-client.top},
                    'launch_fullscreen':True,
                    'api':'OpenCPN --fullscreen -> MyFrame::ToggleFullScreen -> wxFrame::ShowFullScreen'}
                outer_geometry=native_surface['outer'];client_geometry=native_surface['client']
                client_contained=(client_geometry['width']>0 and client_geometry['height']>0 and
                    outer_geometry['x']<=client_geometry['x'] and
                    outer_geometry['y']<=client_geometry['y'] and
                    client_geometry['x']+client_geometry['width']<=outer_geometry['x']+outer_geometry['width'] and
                    client_geometry['y']+client_geometry['height']<=outer_geometry['y']+outer_geometry['height'])
                if (outer_geometry=={'x':0,'y':0,'width':1280,'height':800} and client_contained):
                    break
                time.sleep(.1)
            else:
                raise AssertionError(('Native reference fullscreen did not cover the exact desktop',native_surface))
            report['native_test_surface']=native_surface
        else:
            # capture() normally resizes, but the object-layout gate runs before
            # its first capture. Establish the native size before inspecting it.
            ui.size_window(handle)
    else:
        handle = subprocess.check_output(['xdotool', 'search', '--all', '--onlyvisible', '--pid', str(app.pid),
                    '--name', '^SKAGER / OpenCPN$'], env=env, text=True).splitlines()[0]
        subprocess.run(['xdotool', 'windowsize', handle, '1280', '800', 'windowmove', handle, '0', '0'], env=env, check=True)

    # Unavailable values already match before resize. Do not accept the old
    # small-window geometry merely because the input state is still unchanged.
    # This is a publication barrier, not a wait for geometry to pass its check.
    resized_at=time.time_ns()
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        current=read_json_snapshot(profile/'opennav-diagnostics.json')
        if ((profile/'opennav-diagnostics.json').stat().st_mtime_ns>resized_at and
                int(current['runtime']['ui_update']['ticks'])>=resize_tick+3):break
        time.sleep(.15)
    else:raise AssertionError('Fresh navigation publication did not follow initial resize')
    report['initial_resize_publication']={'before_ticks':resize_tick,
        'after_ticks':int(current['runtime']['ui_update']['ticks']),
        'footer':current['runtime']['display']['footer_region']}

    def capture(name):
        path = evidence / f'{prefix}-{name}.png'
        if windows:
            rgb = ui.capture(handle, path,
                resize=not (native_reference_fullscreen or objects or route_fixture),
                screen_pixels=objects or route_fixture)
        else:
            subprocess.run(['import', '-window', 'root', str(path)], env=env, check=True)
            rgb = subprocess.check_output(['convert', str(path), '-depth', '8', 'rgb:-'], env=env)
        report['screenshots'].append(path.name)
        return rgb

    def assert_stale_navigation():
        # The reserved status slot presents the critical GPS-loss alert instead
        # of the ordinary source caption. Check that real safety presentation,
        # retained observation age and the still-visible rail, not the obsolete
        # pre-alert caption which is intentionally hidden.
        deadline=time.monotonic()+5
        latest={}
        while time.monotonic()<deadline:
            latest=read_json_snapshot(profile/'opennav-diagnostics.json')
            values={v['name']:v for v in latest['data']}
            names=('Latitude','Longitude','Speed over ground','Course over ground')
            stale=all(values[n]['quality']=='STALE' and
                      values[n].get('age_ms',-1)>=values[n]['stale_after_ms'] and
                      'value' in values[n] for n in names)
            alerts=latest['runtime'].get('alerts',[])
            critical=any(a['id'] in ('position-lat','position-lon') and
                         a['level']=='CRITICAL' and not a['acknowledged'] for a in alerts)
            rail=latest['runtime']['display'].get('rail_regions',[])
            visible=len(rail)==4 and all(r['visible'] for r in rail) and any(r['label']=='sog' for r in rail)
            caption=(not windows or any(c.startswith('CRITICAL / Position unavailable or stale')
                                       for _,c in ui.children(handle)))
            if stale and critical and visible and caption:
                report.setdefault('navigation_loss_checks',[]).append({
                    'critical_alert_visible':True,'rail_visible':True,
                    'stale_sources':{n:values[n] for n in names}})
                return
            time.sleep(.15)
        raise AssertionError(('Stopped navigation not visibly degraded',latest))

    def footer_observation(position_state, cog_state):
        """Check the painted footer's owned state against actual decoder input."""
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            record=read_json_snapshot(profile/'opennav-diagnostics.json')
            footer=record['runtime'].get('navigation_footer',{})
            if (footer.get('position_state')==position_state and
                    footer.get('cog_state')==cog_state):
                break
            time.sleep(.15)
        else:raise AssertionError(('Selected navigation did not reach footer',position_state,cog_state,footer))
        assert not footer['historical'] and footer['health_source']=='Vessel data', footer
        assert footer['xte']=='—', 'XTE has no owned observation contract; it must remain unavailable'
        if position_state=='Current':
            assert footer['navigation_state']=='EXPLORING', footer
            assert footer['position']=='56° 42.000′ N   012° 36.000′ E', footer
            assert footer['health_summary']=='1 live signal' and footer['health_state']=='Current', footer
        else:
            assert footer['navigation_state']=='NO POSITION', footer
            assert footer['position']==('GPS POSITION STALE' if position_state=='Stale' else 'GPS POSITION UNAVAILABLE'), footer
            assert footer['health_summary']==('0 live signals, 1 stale' if position_state=='Stale' else '0 live signals'), footer
        assert footer['cog']=={'Current':'147°','Stale':'STALE','Unavailable':'—'}[cog_state], footer
        if windows:
            native=[h for h,caption in ui.children(handle) if caption=='SKAGER status footer']
            assert len(native)==1 and ui.IsWindowVisible(native[0]), 'Current native footer must be visible'
            bounds=ui.W.RECT()
            assert ui.GetWindowRect(native[0],ui.C.byref(bounds))
            measured=dict(x=bounds.left,y=bounds.top,width=bounds.right-bounds.left,height=bounds.bottom-bounds.top)
            comparison={'position_state':position_state,'cog_state':cog_state,
                'ticks':int(record['runtime']['ui_update']['ticks']),
                'native':measured,'published':record['runtime']['display']['footer_region']}
            report.setdefault('native_footer_geometry',[]).append(comparison)
            (evidence/f'{prefix}-native-footer-geometry.json').write_text(json.dumps(report['native_footer_geometry'],indent=2))
            if record['runtime']['display']['footer_region']!=measured:
                failed=evidence/f'{prefix}-footer-geometry-failed.png'
                try:
                    ui.capture(handle,failed,resize=False,screen_pixels=True)
                    report['screenshots'].append(failed.name)
                except Exception as error:
                    report['footer_failure_capture_error']=str(error)
            assert record['runtime']['display']['footer_region']==measured, 'Footer observation must match actual visible native surface'
            report.setdefault('native_footer_regions',[]).append(measured)
        report.setdefault('navigation_footer',[]).append(footer)
        return record

    def footer_health_action():
        if windows:
            ui.pointer_text(app.pid,'Source health')
        else:
            record=read_json_snapshot(profile/'opennav-diagnostics.json')
            controls=[c for c in record['runtime']['display']['interaction_controls']
                      if c['label']=='Source health' and c['visible'] and c['enabled']]
            assert len(controls)==1, ('Source health footer action must be visible',controls)
            target=controls[0]
            subprocess.run(['xdotool','mousemove',str(target['x']+target['width']//2),
                            str(target['y']+target['height']//2),'click','1'],env=env,check=True)
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            record=read_json_snapshot(profile/'opennav-diagnostics.json')
            if record['ui_page']=='Source health':break
            time.sleep(.15)
        else:raise AssertionError('Visible footer action did not open Source health')
        capture('02-live-footer-source-health')
        if windows:
            ui.pointer_text(app.pid,'Chart')
        else:
            controls=[c for c in record['runtime']['display']['interaction_controls']
                      if c['label']=='Chart' and c['visible'] and c['enabled']]
            assert len(controls)==1
            target=controls[0]
            subprocess.run(['xdotool','mousemove',str(target['x']+target['width']//2),
                            str(target['y']+target['height']//2),'click','1'],env=env,check=True)
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            if read_json_snapshot(profile/'opennav-diagnostics.json')['ui_page']=='Navigation':break
            time.sleep(.15)
        else:raise AssertionError('Source health did not return to the chart')
        report['footer_source_health_action']='Visible pointer opens actual Health drawer and returns to chart'

    def horizon_observation(follow_enabled):
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            record=read_json_snapshot(profile/'opennav-diagnostics.json')
            display=record['runtime']['display']
            controls=[c for c in display['interaction_controls']
                      if c['label']=='Horizon now' and c['visible']]
            if len(controls)==1 and controls[0]['enabled']==follow_enabled:break
            time.sleep(.15)
        else:raise AssertionError(('Horizon freshness did not follow selected navigation',follow_enabled,controls))
        bounds=display['horizon_region'];control=controls[0]
        assert bounds['x']<=control['x'] and bounds['y']<=control['y']
        assert control['x']+control['width']<=bounds['x']+bounds['width']
        assert control['y']+control['height']<=bounds['y']+bounds['height']
        report.setdefault('horizon_navigation',[]).append(dict(now=control,follow=record['runtime']['chart']['follow']))
        return record,control

    def horizon_follow_action():
        before,control=horizon_observation(True)
        original=before['runtime']['chart']['follow']
        # Both activations use the actual native control. Verify the existing
        # upstream canvas flag, not a view-only selected state, then restore it.
        for expected in (not original,original):
            if windows:ui.pointer_text(app.pid,'Horizon now')
            else:
                subprocess.run(['xdotool','mousemove',str(control['x']+control['width']//2),
                                str(control['y']+control['height']//2),'mousedown','1',
                                'sleep','0.05','mouseup','1'],env=env,check=True)
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                record=read_json_snapshot(profile/'opennav-diagnostics.json')
                if record['runtime']['chart']['follow']==expected:break
                time.sleep(.15)
            else:raise AssertionError(('Horizon NOW did not reach upstream follow',expected))
            assert record['ui_page']=='Navigation'
            time.sleep(.65)  # distinct presses, never a double-click retry
        capture('02-live-horizon-follow')
        report['horizon_follow_action']='Actual pointer input toggles upstream follow and restores it; no route or hardware command'

    if objects:
        spec = importlib.util.spec_from_file_location('chartcheck', root / 'tools/chart-render-check.py')
        chartcheck = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(chartcheck)
        # Native resizing returns before wx necessarily finishes its layout.
        # Verify exact frame size and actual client-relative geometry before
        # learning colors. Win32 decorations legitimately leave a shorter chart
        # than bare Xvfb; a small startup frame is never an acceptable reference.
        deadline=time.monotonic()+12
        layout_error=None
        while time.monotonic()<deadline:
            sized=read_json_snapshot(profile/'opennav-diagnostics.json')
            display=sized['runtime']['display']
            if windows:
                outer=ui.W.RECT();inner=ui.W.RECT();origin=ui.W.POINT(0,0)
                assert ui.GetWindowRect(handle,ui.C.byref(outer)) and ui.GetClientRect(handle,ui.C.byref(inner))
                assert ui.ScreenToClient(handle,ui.C.byref(origin))
                frame={'x':outer.left,'y':outer.top,'width':outer.right-outer.left,'height':outer.bottom-outer.top}
                client={'x':-origin.x,'y':-origin.y,'width':inner.right-inner.left,'height':inner.bottom-inner.top}
            else:
                geometry=subprocess.check_output(['xdotool','getwindowgeometry','--shell',handle],env=env,text=True)
                values=dict(line.split('=',1) for line in geometry.splitlines() if '=' in line)
                frame={k:int(values[n]) for k,n in [('x','X'),('y','Y'),('width','WIDTH'),('height','HEIGHT')]}
                client=dict(frame)  # This isolated Xvfb run has no window manager/decorations.
            try:
                report['startup_layout']=(chartcheck.navigation_layout if design_validation else chartcheck.functional_layout)(display,frame,client)
                break
            except AssertionError as error:
                layout_error=str(error)
                report['startup_layout_failure']={'reason':layout_error,'frame':frame,'client':client,'display':display}
            time.sleep(.2)
        else:
            path=evidence/f'{prefix}-startup-layout-failure.png'
            if windows:ui.capture(handle,path,resize=False,screen_pixels=True)
            else:subprocess.run(['import','-window','root',str(path)],env=env,check=True)
            report['screenshots'].append(path.name)
            raise AssertionError(('Actual 1280x800 navigation layout failed',layout_error,report['startup_layout_failure']))
        report.pop('startup_layout_failure',None)
        def object_snapshot():
            return read_json_snapshot(profile/'opennav-diagnostics.json')
        def wait_object(predicate, description, timeout=10):
            expires=time.monotonic()+timeout
            latest={}
            while time.monotonic()<expires:
                assert app.poll() is None, 'Application exited during context interaction'
                latest=object_snapshot()
                if predicate(latest):return latest
                time.sleep(.15)
            raise AssertionError((description,latest))
        def context_controls(sample,label):
            return [c for c in sample['runtime']['display'].get('interaction_controls',[])
                    if c['label']==label and c['visible']]
        def focus_context():
            if windows:return
            # Bare Xvfb has no window manager to enforce transient-owner
            # stacking after OpenCPN's deferred frame Raise(). Model that normal
            # desktop ownership explicitly; mouse down/up still hit the real
            # card controls. Native Windows receives no focus workaround.
            found=subprocess.run(['xdotool','search','--all','--onlyvisible','--pid',str(app.pid),
                                  '--name','^SKAGER ((AIS|waypoint) context|vessel traffic)$'],env=env,
                                 capture_output=True,text=True)
            cards=found.stdout.splitlines()
            assert len(cards)<=1,('Multiple compact contexts',cards)
            if cards:
                subprocess.run(['xdotool','windowraise',cards[0],'windowfocus',cards[0]],env=env,check=True)
                report['linux_context_stacking']='Bare Xvfb has no WM; explicit raise/focus models transient ownership, native pointer events remain real'
                return int(cards[0])
            return None
        def linux_pointer_top_level():
            # Ask the X server which actual top-level window is under the
            # pointer. wx geometry alone cannot prove hit testing/stacking on
            # bare Xvfb, especially after GTK processes queued motion/focus.
            import ctypes
            import ctypes.util
            x11=ctypes.CDLL(ctypes.util.find_library('X11'))
            x11.XOpenDisplay.argtypes=[ctypes.c_char_p];x11.XOpenDisplay.restype=ctypes.c_void_p
            x11.XDefaultRootWindow.argtypes=[ctypes.c_void_p];x11.XDefaultRootWindow.restype=ctypes.c_ulong
            x11.XQueryPointer.argtypes=[ctypes.c_void_p,ctypes.c_ulong,
                ctypes.POINTER(ctypes.c_ulong),ctypes.POINTER(ctypes.c_ulong),
                ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_int),
                ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_uint)]
            x11.XQueryPointer.restype=ctypes.c_int
            x11.XCloseDisplay.argtypes=[ctypes.c_void_p]
            display=x11.XOpenDisplay(env['DISPLAY'].encode())
            assert display,'Unable to inspect native X11 pointer target'
            try:
                root_window=ctypes.c_ulong();child=ctypes.c_ulong()
                rx=ctypes.c_int();ry=ctypes.c_int();wx=ctypes.c_int();wy=ctypes.c_int();mask=ctypes.c_uint()
                assert x11.XQueryPointer(display,x11.XDefaultRootWindow(display),
                    ctypes.byref(root_window),ctypes.byref(child),ctypes.byref(rx),ctypes.byref(ry),
                    ctypes.byref(wx),ctypes.byref(wy),ctypes.byref(mask)),'Pointer left the test display'
                return {'window':child.value,'x':rx.value,'y':ry.value,'buttons':mask.value}
            finally:x11.XCloseDisplay(display)
        def click_object(label):
            latest=wait_object(lambda s:any(c['enabled'] for c in context_controls(s,label)),
                               'Visible enabled action '+label)
            choices=context_controls(latest,label)
            unique={(c['x'],c['y'],c['width'],c['height']):c for c in choices if c['enabled']}
            assert len(unique)==1,('Ambiguous context action',label,choices)
            choice=next(iter(unique.values()))
            x,y=choice['x']+choice['width']//2,choice['y']+choice['height']//2
            if windows:
                assert ui.SetCursorPos(x,y)
                ui.MouseEvent(2,0,0,0,0);time.sleep(.05);ui.MouseEvent(4,0,0,0,0)
            else:
                # Motion can queue GTK focus/raise work. Settle that first,
                # then model transient ownership and verify the native target
                # before sending exactly one physical press/release sequence.
                subprocess.run(['xdotool','mousemove','--sync',str(x),str(y)],env=env,check=True)
                time.sleep(.15)
                card=focus_context()
                pointer=linux_pointer_top_level()
                if latest['ui_page']=='Navigation':
                    assert card is not None,('Visible context action has no native owned window',label)
                if card:
                    deadline=time.monotonic()+2;stable=0
                    while time.monotonic()<deadline:
                        pointer=linux_pointer_top_level()
                        stable=stable+1 if pointer['window']==card and pointer['x']==x and pointer['y']==y else 0
                        if stable==2:break
                        focus_context();time.sleep(.08)
                    assert stable==2,('Native context hit target did not settle',label,card,pointer)
                assert not pointer['buttons'] & (256|512|1024),'Unexpected held mouse button'
                report.setdefault('native_pointer_checks',[]).append({'action':label,'expected_context':card,'pointer':pointer})
                subprocess.run(['xdotool','mousedown','1'],env=env,check=True)
                try:time.sleep(.06)
                finally:subprocess.run(['xdotool','mouseup','1'],env=env,check=True)
        def chart_bounded_context(labels):
            latest=wait_object(lambda s:all(context_controls(s,label) for label in labels),
                               'Context actions visible over chart')
            assert latest['ui_page']=='Navigation',latest['ui_page']
            rect=latest['runtime']['display']['chart_region']
            for label in labels:
                for control in context_controls(latest,label):
                    assert (rect['x']<=control['x'] and rect['y']<=control['y'] and
                            control['x']+control['width']<=rect['x']+rect['width'] and
                            control['y']+control['height']<=rect['y']+rect['height']),control
            focus_context()
            return latest
        def ais_drawer():
            latest=wait_object(lambda s:s['ui_page']=='AIS target' and
                               context_controls(s,'Back') and
                               'drawer' in s['runtime']['display'],
                               'Selected target opens the AIS drawer')
            drawer=latest['runtime']['display']['drawer']
            chart=latest['runtime']['display']['chart_region']
            expected={'x':chart['x']+chart['width']-14-398,'y':chart['y']+12,
                      'width':398,'height':client['y']+client['height']-34-12-(chart['y']+12)}
            if design_validation:
                assert all(abs(drawer[k]-v)<=1 for k,v in expected.items()),(drawer,expected)
            else:
                assert drawer['width']>0 and drawer['height']>0 and client['x']<=drawer['x'] and client['y']<=drawer['y']
                assert drawer['x']+drawer['width']<=client['x']+client['width'] and drawer['y']+drawer['height']<=client['y']+client['height'], 'AIS drawer must fit the actual client'
            report.setdefault('ais_drawer_geometry',[]).append(drawer)
            focus_context()
            return latest
        def scroll_drawer_to(label):
            # Native wheel input traverses the actual scroll viewport. Do not
            # invoke a hidden callback or click an off-screen rectangle.
            for _ in range(12):
                latest=object_snapshot()
                if any(c['enabled'] for c in context_controls(latest,label)):return
                drawer=latest['runtime']['display']['drawer']
                x,y=drawer['x']+drawer['width']//2,drawer['y']+drawer['height']-60
                if windows:
                    assert ui.SetCursorPos(x,y)
                    ui.MouseEvent(0x0800,0,0,ui.C.c_uint32(-360).value,0)
                else:
                    focus_context()
                    subprocess.run(['xdotool','mousemove','--sync',str(x),str(y),
                                    'click','--repeat','3','--delay','50','5'],env=env,check=True)
                # Diagnostics are sampled by the application timer, not by
                # this reader. Wait for a post-wheel sample before deciding
                # whether to scroll again; otherwise queued wheel input can
                # move the button after we selected its previous rectangle.
                geometry_file=profile/'opennav-diagnostics.json'
                previous=geometry_file.stat().st_mtime_ns
                expires=time.monotonic()+3
                while geometry_file.stat().st_mtime_ns==previous:
                    assert time.monotonic()<expires,'No post-scroll diagnostic update'
                    time.sleep(.1)
            raise AssertionError(('Native drawer scrolling did not reveal action',label))
        chart_colors = chartcheck.reference(capture('initial-no-input-chart'))
        assert all(min(c)>0 for c in chart_colors),'Black desktop is not a chart color'
        phase[0]='rmc';deadline=time.monotonic()+150;seen=set()
        route_phases = ['route-card','route-detail-renamed','route-detail-active',
                        'route-detail-advanced','route-detail-completed',
                        'route-detail-delete-selected','route-detail-deleted']
        waypoint_phases = ['waypoint-detail-selected','waypoint-detail-renamed',
                           'waypoint-detail-protected','waypoint-detail-invalid',
                           'waypoint-detail-ambiguous','waypoint-detail-deleted']
        def route_controls(sample,label):
            return [c for c in sample['runtime']['display'].get('product_controls',[])
                    if c['label']==label]
        while time.monotonic()<deadline:
            assert app.poll() is None,'Object fixture exited'
            path=profile/'objects-fixture-results.json'
            if path.exists():
                result=read_json_snapshot(path);assert result['result']!='failed',result
                current=result.get('phase','')
                if current in waypoint_phases and current not in seen:
                    if current=='waypoint-detail-selected':
                        chart_bounded_context(['GO TO','Details','Edit waypoint','Remove'])
                        click_object('Details')
                    def waypoint_reconciled(s):
                        if s['ui_page']!='Waypoint detail':return False
                        controls={c['label']:c['enabled'] for c in s['runtime']['display']['product_controls']}
                        if current in ('waypoint-detail-ambiguous','waypoint-detail-deleted'):
                            return 'Back to waypoints' in controls and not any(
                                name in controls for name in ('GO TO','Edit waypoint','Delete waypoint','View on chart'))
                        protected=current in ('waypoint-detail-protected','waypoint-detail-invalid')
                        return all(controls.get(name)==(not protected) for name in
                                   ('GO TO','Edit waypoint','Delete waypoint')) and controls.get('View on chart')==(current!='waypoint-detail-invalid')
                    ready=wait_object(waypoint_reconciled,'Selected waypoint follows '+current)
                    capture(current)
                    report.setdefault('waypoint_detail_lifecycle',[]).append({
                        'phase':current,'controls':ready['runtime']['display']['product_controls']})
                    (profile/(current+'-observed')).write_text('Current selected waypoint control state verified.\n')
                    seen.add(current)
                if current in route_phases+['settings-return','settings-return-navigation','waypoint-card','ais-card'] and current not in seen:
                    time.sleep(.6)
                    if current in route_phases:
                        active=current in ('route-detail-active','route-detail-advanced')
                        removed=current=='route-detail-deleted'
                        def reconciled(s):
                            if s['ui_page']!='Route detail':return False
                            stop=route_controls(s,'Stop navigation')
                            activate=route_controls(s,'Activate route')
                            if removed:
                                mutations=['Stop navigation','Activate route','Edit route name / description',
                                           'Edit route points on chart','Reverse route']
                                return all(not route_controls(s,label) for label in mutations) and bool(route_controls(s,'Back to routes'))
                            expected=stop if active else activate
                            edits=route_controls(s,'Edit route name / description')
                            return bool(expected and expected[0]['enabled'] and edits and
                                        edits[0]['enabled']!=active and not (activate if active else stop))
                        ready=wait_object(reconciled,'Open route detail follows '+current)
                        report.setdefault('route_detail_lifecycle',[]).append({
                            'phase':current,'controls':ready['runtime']['display']['product_controls']})
                    if current=='waypoint-card':chart_bounded_context(['GO TO','Details','Edit waypoint','Remove'])
                    if current=='ais-card':ais_drawer()
                    if current in ('settings-return','settings-return-navigation'):
                        # The fixture publishes its phase immediately after
                        # synchronous pane restoration. Diagnostics publish on
                        # the next timer tick; a fixed .6s sleep can still read
                        # the old hidden Route detail controls on Windows.
                        # Wait for a new Navigation publication, not for good
                        # geometry: missing/clipped controls must still fail.
                        previous_tick=int(object_snapshot()['runtime']['ui_update']['ticks'])
                        ready=wait_object(lambda s:s['ui_page']=='Navigation' and
                            int(s['runtime']['ui_update']['ticks'])>previous_tick,
                            'Fresh Navigation publication after settings restoration')
                    rgb = capture(current)
                    if current=='route-detail-renamed':
                        click_object('Activate route')
                        wait_object(lambda s:context_controls(s,'Cancel') and context_controls(s,'Activate'),
                                    'Actual activation confirmation is open')
                        (profile/'route-modal-opened').write_text('Owned activation confirmation visible.\n')
                        modal_deadline=time.monotonic()+8
                        while time.monotonic()<modal_deadline:
                            modal_result=read_json_snapshot(path)
                            assert modal_result['result']!='failed',modal_result
                            if modal_result.get('phase')=='route-detail-modal-changed':break
                            time.sleep(.15)
                        else:raise AssertionError('Fixture did not change route behind confirmation')
                        # Two refresh intervals while the dialog is alive: an
                        # Update-driven DestroyChildren would invalidate it.
                        time.sleep(2.2)
                        wait_object(lambda s:context_controls(s,'Cancel') and context_controls(s,'Activate'),
                                    'Route change must not rebuild a live modal sheet')
                        capture('route-detail-stale-confirmation')
                        click_object('Activate')
                        wait_object(lambda s:not context_controls(s,'Cancel'),
                                    'Stale confirmation closes normally')
                        (profile/'route-modal-confirmed').write_text('Confirmed original activation intent after external edit.\n')
                    if current in route_phases:
                        (profile/(current+'-observed')).write_text('Open detail native actions and enabled state verified.\n')
                    if current in ('settings-return','settings-return-navigation'):
                        # Reconfiguration must restore visible chart, primary
                        # values and recovery controls after an object page.
                        layout=ready['runtime']['display']
                        report.setdefault('settings_return_layout',[]).append(
                            (chartcheck.navigation_layout if design_validation else chartcheck.functional_layout)(layout,frame,client))
                        report.setdefault('chart_rendering', []).append(chartcheck.check(
                            rgb, chart_colors, 'Actual settings reconfiguration returns coastline without restart / '+current))
                        (profile / (current+'-observed')).write_text('Native chart land and water verified.\n')
                    seen.add(current)
                    if current=='waypoint-card':
                        phase[0]='none'
                        wait_object(lambda s:any(not c['enabled'] for c in context_controls(s,'GO TO')),
                                    'Stale GPS must disable compact waypoint Go To',timeout=10)
                        capture('waypoint-stale-position')
                        phase[0]='rmc'
                        wait_object(lambda s:any(c['enabled'] for c in context_controls(s,'GO TO')),
                                    'Fresh selected GPS restores range action')
                        click_object('Details')
                        wait_object(lambda s:s['ui_page']=='Waypoint detail','Compact waypoint opens full Details')
                        capture('waypoint-details')
                        phase[0]='none'
                        wait_object(lambda s:route_controls(s,'GO TO') and not route_controls(s,'GO TO')[0]['enabled'],
                                    'Stopped GPS disables detail Go To without reopening',timeout=10)
                        capture('waypoint-detail-stale-position')
                        phase[0]='rmc'
                        wait_object(lambda s:route_controls(s,'GO TO') and route_controls(s,'GO TO')[0]['enabled'],
                                    'Fresh GPS restores selected detail Go To')
                        click_object('View on chart')
                        wait_object(lambda s:s['ui_page']=='Navigation','Waypoint Details returns to chart')
                        (profile/'waypoint-context-observed').write_text('Compact, stale GPS guard, Details and chart return verified.\n')
                    if current=='ais-card':
                        scroll_drawer_to('Show on chart')
                        capture('ais-target-action-visible')
                        click_object('Show on chart')
                        wait_object(lambda s:s['ui_page']=='Navigation' and
                                    s['runtime'].get('ais_selected_mmsi')==990000001,
                                    'Prototype AIS drawer selects existing chart target')
                        capture('ais-context-selected-chart')
                        (profile/'ais-context-observed').write_text('Prototype AIS drawer, native scroll and chart selection verified.\n')
                if current=='ais-advice' and not report.get('live_ais_advice'):
                    sample=read_json_snapshot(profile/'opennav-diagnostics.json')
                    if sample['runtime'].get('smartnav',{}).get('ais_event_count',0)>0:
                        report['live_ais_advice']='Copied actual AIS alarm reaches shell SmartNav at a coherent observation epoch'
                        (profile/'ais-advice-observed').write_text('Observed actual shell diagnostic AIS event\n')
                if result['result']=='passed':
                    assert report.get('live_ais_advice'),'No actual AIS advisory observed'
                    assert len(seen)==17,seen
                    assert result.get('late_connection_added_after_deferred') and counts['ais'] >= 3,result
                    report['object_contract']=result;break
            time.sleep(.2)
        else:raise RuntimeError('Object contract fixture timed out')
        assert not failures,failures
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            ready=read_json_snapshot(profile/'opennav-diagnostics.json')
            if ready['ui_page']=='AIS target' and context_controls(ready,'Back') and not ready['runtime'].get('alerts'):break
            time.sleep(.15)
        else:raise AssertionError('AIS fixture alarm did not resolve before card interaction')
        ais_drawer()
        capture('ais-details')
        scroll_drawer_to('Show on chart')
        click_object('Show on chart')
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            selected=read_json_snapshot(profile/'opennav-diagnostics.json')
            if selected['ui_page']=='Navigation' and selected['runtime'].get('ais_selected_mmsi')==990000001:break
            time.sleep(.2)
        else:raise AssertionError(('AIS chart selection did not become current',selected))
        capture('ais-selected-chart')
        report['ais_selection']='Actual target card to existing chart target/frame; copied selection identity reported'
        if windows:
            ui.click_text(app.pid,'Settings');ui.click_text(app.pid,'Navigation');ui.click_text(app.pid,'Waypoints')
            ui.click_text(app.pid,'ALPHA TEST edited / mark');ui.click_text(app.pid,'Edit waypoint')
            ui.set_text_in_dialog(app.pid,'Edit waypoint','ALPHA TEST edited','ALPHA TEST UI edited')
            ui.click_text(app.pid,'Save');time.sleep(.6)
            assert any(c=='ALPHA TEST UI edited' for _,c in ui.children(handle))
            capture('waypoint-edit-sheet-result')
            ui.click_text(app.pid,'Delete waypoint');ui.click_text(app.pid,'Cancel')
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
                    rgb = capture('01-active-route')
                    entry = next(x for x in checks if x['check'] == 'middle point real upstream progress')
                    points = entry['route_pixels']
                    live_display = read_json_snapshot(profile/'opennav-diagnostics.json')['runtime']
                    chart_region = live_display['display']['chart_region']
                    assert all(chart_region['x']+16 < p['x'] < chart_region['x']+chart_region['width']-16 and
                               chart_region['y']+16 < p['y'] < chart_region['y']+chart_region['height']-16
                               for p in points), 'All test waypoints must fit the actual chart viewport'
                    assert live_display['display']['light'] == args.theme
                    assert live_display['chart']['opengl_enabled'] == (args.renderer == 'opengl')
                    assert len(points) == 3, 'Upstream route projection missing'
                    if not route_standard and not design_validation:
                        spec=importlib.util.spec_from_file_location('routecheck',root/'tools/chart-render-check.py')
                        routecheck=importlib.util.module_from_spec(spec);spec.loader.exec_module(routecheck)
                        left=top=0
                        if windows:
                            outer=ui.W.RECT();assert ui.GetWindowRect(handle,ui.C.byref(outer))
                            left,top=outer.left,outer.top
                        report['active_route_paint']=dict(routecheck.route_visibility(rgb,points,(left,top)),
                            style='XNav',theme=args.theme,renderer=args.renderer)
                    else:
                        if route_standard:
                            ink = tuple(entry['stock_active_ink'])
                        else:
                            tokens = json.loads((root/'docs/design/prototype-tokens.json').read_text())
                            ink = tuple(bytes.fromhex(tokens['themes'][args.theme.lower()]['--route'].lstrip('#')))
                            if args.theme == 'Night':
                                # Immutable chart-canvas brightness, before the
                                # separate inspected GL framebuffer conversion.
                                ink = tuple(round(channel*.78) for channel in ink)
                        requested_ink = ink
                        if args.renderer == 'opengl':
                            # Pinned ocpnDC shader uniforms divide RGB by 256,
                            # then the normalized framebuffer quantizes to 255.
                            # Model that inspected conversion exactly, not an
                            # image tolerance that could hide a different palette.
                            ink = tuple(round(v*255/256) for v in requested_ink)
                        left = top = 0
                        if windows:
                            outer=ui.W.RECT();assert ui.GetWindowRect(handle,ui.C.byref(outer))
                            left,top=outer.left,outer.top
                        assert len(rgb) == 1280*800*3
                        samples=[]
                        for a,b in zip(points,points[1:]):
                            for fraction in (.2,.35,.5,.65,.8):
                                x=round(a['x']+(b['x']-a['x'])*fraction)-left
                                y=round(a['y']+(b['y']-a['y'])*fraction)-top
                                assert 4<=x<1276 and 4<=y<796, 'Projected route sample offscreen'
                                hits=sum(tuple(rgb[(py*1280+px)*3:(py*1280+px)*3+3]) == ink
                                         for py in range(y-3,y+4) for px in range(x-3,x+4))
                                assert hits>=2, ('Actual upstream route stroke has wrong/missing ink',x,y,ink,hits)
                                samples.append(dict(x=x,y=y,exact_ink_pixels=hits))
                        report['active_route_paint'] = dict(style='Standard' if route_standard else 'XNav',
                                                           requested_ink=requested_ink,ink=ink,theme=args.theme,renderer=args.renderer,projection='pinned ChartCanvas::GetCanvasPointPix',samples=samples)
                    seen_live = True
                if result.get('phase') == 'stop-input':
                    phase[0] = 'none'
                elif result.get('phase') == 'resume-input':
                    if not seen_stale:
                        # Route observation and the shell's 250 ms repaint have
                        # independent schedules. Keep input stopped while the
                        # visible rail catches up to the proven stale contract.
                        time.sleep(.6)
                        assert_stale_navigation()
                        capture('02-stale-position')
                        seen_stale = True
                    phase[0] = 'rmc'
                if result['result'] == 'passed':
                    assert seen_live and seen_stale, 'Missing route scenario captures'
                    report['route_contract'] = result
                    (evidence / f'{prefix}-progress-results.json').write_text(json.dumps(result, indent=2))
                    break
            time.sleep(.2)
        else:
            raise RuntimeError('Route fixture did not finish normal navigation passes')
        assert not failures, failures
    else:
        footer_observation('Unavailable','Unavailable')
        horizon_observation(False)
        capture('01-unavailable')
        phase[0] = 'rmc'
        time.sleep(3)
        assert counts['rmc'] >= 5 and not failures
        footer_observation('Current','Current')
        capture('02-live')
        footer_health_action()
        horizon_follow_action()
        phase[0] = 'gga'
        time.sleep(6.2)
        footer_observation('Current','Stale')
        horizon_observation(True)  # Current position permits follow despite stale velocity.
        capture('03-position-only-velocity-stale')
        phase[0] = 'none'
        time.sleep(6.2)
        assert_stale_navigation()
        footer_observation('Stale','Stale')
        horizon_observation(False)
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
            assert db.execute('select name from routes where guid=?',('OPENNAV-ALPHA-OBJECT-ROUTE',)).fetchone()==('BETA TEST externally renamed route',)
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
        try:app.wait(timeout=10)
        except subprocess.TimeoutExpired:
            # Preserve the original test failure and its evidence if a modal
            # upstream dialog prevents normal termination in this fixture.
            app.kill();app.wait(timeout=5)
    if xserver:
        xserver.terminate()
        xserver.wait(timeout=10)
    report['sent'] = counts
    report['transport_errors'] = failures
    (evidence / f'{prefix}-input-results.json').write_text(json.dumps(report, indent=2))
    shutil.copytree(profile, evidence / f'{prefix}-input-profile', dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('opencpn-ipc', '*.pem'))
    temporary.cleanup()

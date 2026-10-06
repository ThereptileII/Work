#!/usr/bin/env python3
"""Native chart/waypoint/route gestures in a disposable input-only profile.

The only marine connection is an input-only 127.0.0.1 GPS server. The test
neither selects a real profile nor exposes a physical command transport.
Buttons are located from copied native geometry, never wx IDs or action calls.
"""
import datetime
import importlib.util
import json
import os
from pathlib import Path
import shutil
import socket
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time
from contextlib import closing
from diagnostic_snapshot import read_json_snapshot
from smoke_startup import defer_boat_setup, native_setup_window

root = Path(__file__).resolve().parents[1]
windows = sys.platform == 'win32'
evidence = root / 'evidence/local'
evidence.mkdir(parents=True, exist_ok=True)
temporary = tempfile.TemporaryDirectory(prefix='OpenNav isolated user flows ')
profile = Path(temporary.name) / 'profile'
variant = 'xnav-windows' if windows else 'xnav-linux'
subprocess.run([sys.executable, str(root / 'tools/prepare-test-profile.py'),
                '--build', str(root / 'build' / variant), '--profile', str(profile)], check=True)
server = socket.socket()
server.bind(('127.0.0.1', 0))
server.listen(1)
server.settimeout(.25)
port = server.getsockname()[1]
with (profile / 'opencpn.conf').open('a') as stream:
    stream.write('\n[Settings/NMEADataSource]\nDataConnections='
                 f'1;0;127.0.0.1;{port};0;;4800;1;0;0;;0;;0;0;0;0;1;'
                 'ISOLATED loopback GPS test;0;;0;1;\n'
                 '[Settings/GlobalState]\nVPLatLon=59.0800,18.5000\nVPScale=0.003\n')
stop = threading.Event()
connected = threading.Event()
failures = []
batches = [0]


def sentence(body):
    checksum = 0
    for byte in body.encode('ascii'):
        checksum ^= byte
    return f'${body}*{checksum:02X}\r\n'.encode('ascii')


def feed():
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
        peer.settimeout(.01)
        connected.set()
        while not stop.wait(.2):
            now = datetime.datetime.now(datetime.timezone.utc)
            utc = now.strftime('%H%M%S')
            peer.sendall(sentence(f'GPGGA,{utc},5904.800,N,01830.000,E,1,08,1.0,0.0,M,0.0,M,,') +
                         sentence(f'GPRMC,{utc},A,5904.800,N,01830.000,E,6.3,147.0,{now:%d%m%y},,,A'))
            batches[0] += 1
            try:
                received = peer.recv(4096)
                assert not received, 'An input-only test connection emitted navigation output'
            except TimeoutError:
                pass
    except BaseException as error:
        if not stop.is_set():
            failures.append(repr(error))
    finally:
        if peer:
            peer.close()


thread = threading.Thread(target=feed, daemon=True)
thread.start()
env = dict(os.environ)
env['OPENNAV_TEST_UI_TRACE'] = '1'  # Compiled out of the installed product.
env['OPENNAV_TEST_UI_TRACE_FILE'] = str(profile / 'opennav-ui-trace.log')
app = xserver = handle = ui = None
report = {'authority': 'native Windows' if windows else 'Linux development',
          'profile': 'new disposable, input-only loopback GPS',
          'physical_output': 'No serial/CAN/device transport; output disabled',
          'checks': [], 'screenshots': []}
observed_at = [0.0]


def file_observation(path):
    try:
        stat = path.stat()
        return {'bytes': stat.st_size, 'modified_ns': stat.st_mtime_ns}
    except FileNotFoundError:
        return {'exists': False}


def observe_diagnostics(record):
    at = time.monotonic()
    if at - observed_at[0] < 1:
        return
    observed_at[0] = at
    display = record.get('runtime', {}).get('display', {})
    rows = report.setdefault('diagnostic_observations', [])
    rows.append({'at': at, 'file': file_observation(profile / 'opennav-diagnostics.json'),
                 'pending': file_observation(profile / 'opennav-diagnostics.json.pending'),
                 'ui_update': record.get('runtime', {}).get('ui_update', {}),
                 'orientation': [r['label'] for r in display.get('interaction_controls', [])
                                 if r['label'] in ('North', 'Course', 'Heading')],
                 'page': record.get('ui_page')})
    del rows[:-128]


def native_timeout_observation():
    """Read native state after failure; WM_NULL cannot invoke a UI command."""
    if not windows or not handle:
        return
    C, W = ui.C, ui.W
    send = ui.declare(ui.user, 'SendMessageTimeoutW', C.c_ssize_t,
                      W.HWND, W.UINT, W.WPARAM, W.LPARAM, W.UINT, W.UINT,
                      C.POINTER(C.c_size_t))
    foreground = ui.declare(ui.user, 'GetForegroundWindow', W.HWND)
    result = C.c_size_t()
    C.set_last_error(0)
    response = send(handle, 0, 0, 0, 3, 500, C.byref(result))
    observed = {'wm_null_responded': bool(response), 'win32_error': C.get_last_error(),
                'main_is_foreground': foreground() == handle,
                'native_orientation': [label for _, label in ui.children(handle)
                                       if label in ('North', 'Course', 'Heading')]}
    class GuiInfo(C.Structure):
        _fields_ = [('size', W.DWORD), ('flags', W.DWORD), ('active', W.HWND),
                    ('focus', W.HWND), ('capture', W.HWND), ('menu_owner', W.HWND),
                    ('move_size', W.HWND), ('caret', W.HWND), ('caret_rect', W.RECT)]
    info = GuiInfo()
    info.size = C.sizeof(info)
    query = ui.declare(ui.user, 'GetGUIThreadInfo', W.BOOL, W.DWORD, C.POINTER(GuiInfo))
    thread_id = ui.GetWindowThreadProcessId(handle, None)
    if query(thread_id, C.byref(info)):
        observed['gui_thread'] = {key: getattr(info, key) for key in
                                  ('flags', 'active', 'focus', 'capture', 'menu_owner', 'move_size')}
    report['native_timeout'] = observed


def xdo(*args):
    return subprocess.check_output(['xdotool', *map(str, args)], env=env, text=True, timeout=10).strip()


def data(predicate=lambda d: True, timeout=20):
    deadline = time.monotonic() + timeout
    last = {}
    while time.monotonic() < deadline:
        assert not failures, failures
        if app:
            assert app.poll() is None, 'Application exited during UI test'
        try:
            last = read_json_snapshot(profile / 'opennav-diagnostics.json')
            observe_diagnostics(last)
            if predicate(last):
                return last
        except (FileNotFoundError, ValueError, PermissionError):
            pass
        time.sleep(.15)
    raise AssertionError(('Diagnostic condition timed out', last.get('ui_page'),
                          last.get('runtime', {}).get('display', {})))


def control(label, enabled=True, accessible_name=None):
    selected = []
    def ready(record):
        selected[:] = [row for row in record.get('runtime', {}).get('display', {}).get('interaction_controls', [])
                       if row['label'] == label and row['visible'] and row['enabled']==enabled
                       and (accessible_name is None or row.get('accessible_name')==accessible_name)]
        # The owned modal is traversed after its underlying page. A confirmed
        # action can intentionally share its caption with that page action.
        # Inline name-editor Cancel remains visible even with no modal open;
        # only the sheet's exact accessible name establishes this exception.
        return len(selected) == 1 or (len(selected) > 1 and any(
            row['label'] == 'Cancel' and row.get('accessible_name') == 'Cancel' and row['visible']
            for row in record['runtime']['display']['interaction_controls']))
    data(ready)
    rectangle = selected[-1]
    assert rectangle['width'] >= 48 and rectangle['height'] >= 48, rectangle
    return rectangle


def physical_click(x, y, right=False):
    if windows:
        assert ui.SetCursorPos(int(x), int(y))
        time.sleep(.2)
        hit = ui.WindowFromPoint(ui.W.POINT(int(x), int(y)))
        owner = ui.W.DWORD()
        ui.GetWindowThreadProcessId(hit, ui.C.byref(owner))
        assert owner.value == app.pid, ('Native pointer is obscured by another process',
                                        int(x), int(y), int(hit or 0), owner.value)
        ui.MouseEvent(0x0008 if right else 0x0002, 0, 0, 0, 0)
        time.sleep(.08)
        ui.MouseEvent(0x0010 if right else 0x0004, 0, 0, 0, 0)
    else:
        xdo('mousemove', int(x), int(y))
        time.sleep(.2)
        xdo('mousedown', 3 if right else 1)
        time.sleep(.08)
        xdo('mouseup', 3 if right else 1)
    time.sleep(.65)


def click(label, accessible_name=None):
    rectangle = control(label, accessible_name=accessible_name)
    report.setdefault('interactions', []).append({'action': label, 'bounds': rectangle})
    physical_click(rectangle['x'] + rectangle['width'] // 2,
                   rectangle['y'] + rectangle['height'] // 2)


def chart_click(x_fraction, y_fraction, right=False):
    record = data(lambda d: d['ui_page'] == 'Navigation')
    rectangle = record['runtime']['display']['chart_region']
    assert rectangle['width'] > 600 and rectangle['height'] > 400, rectangle
    physical_click(rectangle['x'] + rectangle['width'] * x_fraction,
                   rectangle['y'] + rectangle['height'] * y_fraction, right)


def escape():
    if windows:
        key = ui.declare(ui.user, 'keybd_event', None, ui.W.BYTE, ui.W.BYTE, ui.W.DWORD, ui.C.c_size_t)
        key(0x1b, 0, 0, 0)
        key(0x1b, 0, 2, 0)
    else:
        xdo('key', 'Escape')
    time.sleep(.4)


def no_context(record):
    return not any(r['label'] == 'Waypoint' and r.get('accessible_name')=='Waypoint' and r['visible']
                   for r in record['runtime']['display']['interaction_controls'])


def type_name(title, value):
    if windows:
        # Both creation paths now use the shared Name + Description sheet.
        ui.set_dialog_fields(app.pid, title, [value, ''])
    else:
        # Focusing the top-level X11 window can move keyboard focus away from
        # GTK's edit widget. Use its actual native field rectangle instead.
        click('Field: Name')
        xdo('key', 'ctrl+a')
        xdo('type', '--clearmodifiers', '--delay', 25, value)


def capture(name):
    path = evidence / ('flow-' + name + ('.png' if windows else '-linux.png'))
    if windows:
        # Owned context/edit windows are separate native top-level surfaces;
        # PrintWindow(main) would silently omit them. Do not resize underneath
        # an open sheet while capturing its visible screen pixels.
        ui.capture(handle, path, resize=False, screen_pixels=True)
    else:
        subprocess.run(['import', '-window', 'root', str(path)], env=env, check=True)
    report['screenshots'].append(path.name)


def catalog(page):
    click('Passage')
    data(lambda d:d['ui_page']=='Route' and 'drawer' in d['runtime']['display'])
    for _ in range(12):
        current=data()['runtime']['display']
        if any(c['label']=='Passage library' and c['visible'] and c['enabled'] for c in current['interaction_controls']):
            break
        drawer=current['drawer'];x=drawer['x']+drawer['width']//2;y=drawer['y']+drawer['height']//2
        if windows:
            assert ui.SetCursorPos(x,y)
            target=ui.WindowFromPoint(ui.W.POINT(x,y));owner=ui.W.DWORD()
            ui.GetWindowThreadProcessId(target,ui.C.byref(owner));assert owner.value==app.pid
            ui.MouseEvent(0x0800,0,0,(-240)&0xffffffff,0)
        else:xdo('mousemove',x,y,'click',5,'click',5)
        time.sleep(.4)
    else:raise AssertionError('Passage library cannot be reached by ordinary drawer scrolling')
    click('Passage library')
    data(lambda d:d['ui_page']=='Routes')
    if page=='Waypoints':click('Waypoints')
    data(lambda d: d['ui_page'] == page)


try:
    if windows:
        spec = importlib.util.spec_from_file_location('ui', root / 'tools/windows-ui.py')
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
        # Keep the actual 1280x800 test frame above an unobscured desktop work
        # area. No taskbar hiding/topmost window or target-local click bypass.
        report['display'] = ui.ensure_desktop(1920, 1080)
        exe = root / 'build/xnav-install/opencpn.exe'
    else:
        number = 131
        while Path(f'/tmp/.X{number}-lock').exists():
            number += 1
        env['DISPLAY'] = f':{number}'
        xserver = subprocess.Popen(['Xvfb', env['DISPLAY'], '-screen', '0', '1280x800x24', '-nolisten', 'tcp'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1)
        exe = root / 'build/xnav-install/bin/opencpn'
    with (evidence / 'user-flows-launch.log').open('w') as output:
        app = subprocess.Popen([str(exe), '--xnav', '--no_opengl', '--configdir', str(profile)],
                               env=env, stdout=output, stderr=output)
    assert connected.wait(45), 'Input-only GPS connection not established'
    deadline = time.monotonic() + 60
    while not ((profile / 'opencpn.log').exists() and 'OnInitTimer...Finalize Canvases' in
               (profile / 'opencpn.log').read_text(errors='replace')):
        assert app.poll() is None and time.monotonic() < deadline, 'Deferred initialization did not finish'
        time.sleep(.2)
    data(lambda d: any(v['name'] == 'Latitude' and v['quality'] == 'LIVE' for v in d['data']), 45)
    trace = profile / 'opennav-ui-trace.log'
    assert trace.exists() and 'tick.end' in trace.read_text(), 'Test-only native trace is unavailable'
    if windows:
        handle, _ = ui.wait_window('SKAGER / OpenCPN', app.pid)
        ui.size_window(handle)
        frame = ui.W.RECT()
        assert ui.GetWindowRect(handle, ui.C.byref(frame))
        report['frame_pixels'] = [frame.right - frame.left, frame.bottom - frame.top]
        assert report['frame_pixels'] == [1280, 800]
    else:
        handle = xdo('search', '--all', '--onlyvisible', '--pid', app.pid, '--name', '^SKAGER / OpenCPN$').splitlines()[0]
        xdo('windowsize', handle, 1280, 800, 'windowmove', handle, 0, 0, 'windowfocus', handle)
    time.sleep(2)
    def defer_setup_click(target):
        if windows:
            ui.pointer_text(app.pid, 'Later')
        else:
            dialogs=xdo('search','--all','--onlyvisible','--pid',app.pid,
                        '--name','^Boat Setup & Sensor Check$').splitlines()
            assert len(dialogs)==1, ('Expected one owned first-start setup sheet',dialogs)
            xdo('windowraise',dialogs[0])
            physical_click(target['x']+target['width']//2,target['y']+target['height']//2)
    report['first_start_setup']=defer_boat_setup(data,defer_setup_click,
        native_window=(lambda:native_setup_window(ui,app.pid)) if windows else None)
    click('North')
    control('Course')
    course_before = data()
    course_tick = int(course_before['runtime']['ui_update']['ticks'])
    position_at = next(int(v['observed_monotonic_ms']) for v in course_before['data']
                       if v['name'] == 'Latitude' and v['quality'] == 'LIVE')
    course_after = data(lambda d: int(d['runtime']['ui_update']['ticks']) >= course_tick + 4 and
                        any(v['name'] == 'Latitude' and v['quality'] == 'LIVE' and
                            int(v['observed_monotonic_ms']) > position_at for v in d['data']))
    report['course_up_updates'] = {'before_tick': course_tick,
                                   'after_tick': int(course_after['runtime']['ui_update']['ticks']),
                                   'fresh_position_advanced': True}
    capture('00-course-up')
    click('Course')
    control('North')
    report['checks'].append('Touch chart orientation toggles upstream North/Course state and restores North')
    capture('01-navigation')
    initial_chart = data()['runtime']['display']['chart_region']
    chart_click(.73, .22, right=True)
    control('Waypoint', accessible_name='Waypoint')
    chart_click(.72, .74)
    data(no_context)
    chart_click(.73, .22, right=True)
    control('Waypoint', accessible_name='Waypoint')
    escape()
    data(no_context)
    report['checks'].append('Chart card dismisses by outside pointer press and Escape without changing navigation')
    chart_click(.73, .22, right=True)
    control('Waypoint', accessible_name='Waypoint')
    capture('02-chart-context')
    click('Waypoint', accessible_name='Waypoint')
    capture('02b-waypoint-sheet')
    type_name('Create waypoint', 'UI passage destination')
    click('Save')
    catalog('Waypoints')
    click('UI passage destination / mark')
    data(lambda d: d['ui_page'] == 'Waypoint detail')
    capture('03-waypoint')
    click('GO TO')
    click('START')
    active = data(lambda d: d.get('route', {}).get('state') == 'Valid' and d['route'].get('remaining_nm', 0) > 0)
    destination_id = active['route']['waypoint']
    capture('04-go-to')
    catalog('Routes')
    record = data(lambda d: any(r['label'].endswith(' / ACTIVE') for r in d['runtime']['display']['product_controls']))
    active_label = next(r['label'] for r in record['runtime']['display']['product_controls'] if r['label'].endswith(' / ACTIVE'))
    click(active_label)
    click('Stop navigation')
    # Wait for the confirmation sheet; an old pre-modal record must not
    # redirect a click back onto the underlying identically named action.
    control('Cancel')
    click('Stop navigation')
    data(lambda d: d.get('route', {}).get('state') == 'NoActiveRoute')
    report['checks'].append('Chart context saves copied chart position; selected waypoint Go To uses real selected GPS and upstream route progress; stop deactivates')

    catalog('Routes')
    click('Create route on chart')
    data(lambda d: d['runtime']['display']['route_creation_active'])
    draft_controls = {name: control(name, enabled=name!='Undo') for name in ('Cancel', 'Undo', 'Done')}
    assert all(bounds['width'] >= 88 and bounds['height'] >= 48 for bounds in draft_controls.values())
    draft_chart = data()['runtime']['display']['chart_region']
    assert draft_chart == initial_chart, (initial_chart, draft_chart)
    report['draft_controls'] = draft_controls
    report['checks'].append('Draft actions have full-width touch labels without changing chart or data-rail viewport')
    chart_click(.22, .62)
    control('Undo', enabled=False)
    chart_click(.45, .42)
    control('Undo')
    click('Undo')
    control('Undo', enabled=False)
    chart_click(.45, .42)
    chart_click(.68, .64)
    click('Undo')
    control('Undo')
    data(lambda d: d['runtime']['display']['route_creation_active'])
    click('Done')
    control('Save route')
    capture('05-route-name')
    click('Cancel')
    data(lambda d: d['runtime']['display']['route_creation_active'])
    click('Done')
    type_name('Save route', 'UI coastal passage')
    click('Save route')
    data(lambda d: d['ui_page'] == 'Route detail' and not d['runtime']['display']['route_creation_active'])
    capture('06-saved-route')
    report['checks'].append('Three chart taps, Undo, naming Cancel retaining draft, and named Save open the real route detail')
    report['checks'].append('Native draft Undo disabled with zero/one point, enabled with two, disabled after undo to one, and usable again after another point')
    catalog('Routes')
    click('Create route on chart')
    chart_click(.30, .75)
    chart_click(.62, .77)
    click('Cancel')
    click('Discard route')
    data(lambda d: not d['runtime']['display']['route_creation_active'])
    report['checks'].append('Whole-route Cancel discards only the unfinished new route and resets native creation state')
    catalog('Routes')
    click('UI coastal passage / 2 points')
    data(lambda d: d['ui_page'] == 'Route detail')
    capture('07-preserved-route')
    stop.set()
    thread.join(timeout=3)
    if windows:
        ui.close(handle)
    else:
        subprocess.run([str(exe), '--configdir', str(profile), '--remote', '--quit'], env=env, check=True, timeout=15)
    assert app.wait(timeout=30) == 0, 'Flow test did not close cleanly'
    with closing(sqlite3.connect((profile / 'navobj.db').as_uri() + '?mode=ro', uri=True)) as database:
        saved = database.execute('SELECT guid FROM routes WHERE name=?', ('UI coastal passage',)).fetchall()
        assert len(saved) == 1, saved
        points = database.execute('SELECT point_guid FROM routepoints_link WHERE route_guid=? ORDER BY point_order', saved[0]).fetchall()
        assert len(points) == 2, points
        assert database.execute('SELECT COUNT(*) FROM routes').fetchone() == (1,)
        mark = database.execute('SELECT guid,lat,lon FROM routepoints WHERE Name=?', ('UI passage destination',)).fetchall()
        assert len(mark) == 1 and mark[0][0] == destination_id, mark
        assert abs(mark[0][1] - 59.08) > .001 or abs(mark[0][2] - 18.5) > .001, mark
        report['database'] = {'saved_routes': 1, 'saved_route_points': len(points), 'original_waypoint_preserved': True,
                              'waypoint_is_clicked_position_not_view_center': True}
    report['checks'].append('Read-only navobj.db audit proves two-point saved route, no cancelled draft, original waypoint identity and clicked position preserved')
    assert not failures, failures
    report['result'] = 'passed; native screenshots require review'
except Exception as error:
    report['failure'] = repr(error)
    try:
        native_timeout_observation()
    except Exception as probe_error:
        report['native_timeout_error'] = repr(probe_error)
    raise
finally:
    if app and app.poll() is None and handle and 'result' not in report:
        try:
            capture('failure')
        except Exception as error:
            report['failure_capture'] = repr(error)
    stop.set()
    thread.join(timeout=3)
    server.close()
    if app and app.poll() is None:
        app.terminate()
        try:
            app.wait(timeout=10)
        except subprocess.TimeoutExpired:
            app.kill()
            app.wait(timeout=5)
    if xserver:
        xserver.terminate()
        xserver.wait(timeout=10)
    report['gps_batches'] = batches[0]
    report['transport_errors'] = failures
    report['test_ui_trace'] = file_observation(profile / 'opennav-ui-trace.log')
    shutil.copytree(profile, evidence / 'user-flows-profile', dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('opencpn-ipc', '*.pem'))
    (evidence / 'user-flows-results.json').write_text(json.dumps(report, indent=2) + '\n')
    temporary.cleanup()
print(report['result'])

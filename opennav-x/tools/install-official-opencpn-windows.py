"""Install the exact stock prerequisite through its visible wizard in native CI."""
import ctypes
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

assert sys.platform == "win32" and os.environ.get("GITHUB_ACTIONS") == "true"
setup, directory, output = map(lambda p: Path(p).resolve(), sys.argv[1:])
assert hashlib.sha256(setup.read_bytes()).hexdigest() == "e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa"
assert '"' not in str(directory) and not (directory / 'opencpn.exe').exists()
assert ctypes.windll.shell32.IsUserAnAdmin(), "Stock prerequisite requires an elevated disposable runner"
spec = importlib.util.spec_from_file_location("ui", Path(__file__).with_name("windows-ui.py"))
ui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ui)
enabled = ui.declare(ui.user, 'IsWindowEnabled', ui.W.BOOL, ui.W.HWND)
record = {"status": "running", "invocation": "Unmodified official stock interactive wizard", "callerAdministrator": True, "pages": []}
record['desktop'] = ui.ensure_desktop()
output.parent.mkdir(parents=True, exist_ok=True)
process = subprocess.Popen(subprocess.list2cmdline([str(setup)]) + ' /D=' + str(directory))
directory_set = installed = finished = False
acted = set()

def native_class(handle):
    name = ctypes.create_unicode_buffer(128)
    ui.GetClassNameW(handle, name, len(name))
    return name.value

def capture(handle, title, controls, action):
    name = 'official-wizard-' + str(len(record['pages'])) + '.png'
    ui.SetForegroundWindow(handle)
    time.sleep(.2)
    ui.capture(handle, output.parent / name, resize=False, screen_pixels=True)
    record['pages'].append({'title': title, 'controls': controls, 'action': action, 'screenshot': name})

try:
    deadline = time.monotonic() + 180
    while process.poll() is None and time.monotonic() < deadline:
        for handle, pid, title in ui.windows(process.pid):
            children = ui.children(handle)
            controls = [ui.control_text(h) for h, _ in children]
            signature = (title, tuple(controls))
            if signature in acted:
                continue
            text = '\n'.join(controls)
            buttons = {ui.control_text(h).replace('&', '').strip(): h for h, _ in children if native_class(h) == 'Button' and enabled(h)}
            choice = None
            if 'Please select a language:' in text:
                assert 'English' in controls
                choice = 'OK'
            elif 'Destination Folder' in text:
                edits = [h for h, _ in children if native_class(h) == 'Edit']
                assert len(edits) == 1, controls
                value = ctypes.create_unicode_buffer(str(directory))
                ui.SendMessageW(edits[0], 0x000C, 0, ctypes.cast(value, ctypes.c_void_p).value)
                assert ui.control_text(edits[0]) == str(directory)
                directory_set = True
                choice = 'Next >'
            elif 'Install' in buttons:
                assert directory_set and str(directory) in text, ('Unexpected installation destination', controls)
                installed = True
                choice = 'Install'
            elif 'Finish' in buttons:
                assert installed and (directory / 'opencpn.exe').exists()
                # Normal upstream Finish offers to launch the app/readme.
                # Leave subsequent stock/profile launch to the explicit test.
                for child, _ in children:
                    label = ui.control_text(child).replace('&', '').strip().lower()
                    if native_class(child) == 'Button' and (label.startswith('run ') or label.startswith('show ')):
                        if ui.SendMessageW(child, 0x00F0, 0, 0) == 1:
                            ui.SendMessageW(child, 0x00F1, 0, 0)
                choice = 'Finish'
                finished = True
            elif 'Next >' in buttons:
                choice = 'Next >'
            if choice:
                assert choice in buttons, (choice, controls)
                capture(handle, title, controls, choice)
                acted.add(signature)
                assert len(acted) <= 16, 'Unexpected stock wizard page loop'
                ui.PostMessageW(buttons[choice], 0x00F5, 0, 0)
                time.sleep(.5)
        time.sleep(.2)
    assert process.poll() is not None, 'Official wizard timed out'
    assert process.returncode == 0 and installed and finished, (process.returncode, installed, finished)
    record['executableSha256'] = hashlib.sha256((directory / 'opencpn.exe').read_bytes()).hexdigest()
    assert record['executableSha256'] == '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
    record['status'] = 'passed'
except Exception as error:
    record['status'] = 'failed'
    record['error'] = repr(error)
    for handle, pid, title in ui.windows(process.pid):
        capture(handle, title, [ui.control_text(h) for h, _ in ui.children(handle)], 'Failure evidence; no further action')
    raise
finally:
    if process.poll() is None:
        subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'], capture_output=True)
        process.wait(timeout=15)
    output.write_text(json.dumps(record, indent=2), encoding='utf-8')
    print(json.dumps(record, indent=2))

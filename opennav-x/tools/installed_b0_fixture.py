"""Disposable genuine B0 Setup/Maintain observations; never product authority.

Reuse the installer smoke's actual Setup arguments, relocated Maintain identity,
owned wizard controls and original-handle exit observations. No legacy engine,
private elevated product action, UAC policy changes or automatic failure cleanup.
"""
import ctypes
from collections import Counter
import json
import os
from pathlib import Path
import subprocess
import sys
import time


LOCAL_APP_DATA = 'f1b32785-6fba-4fcf-9d55-7b8e7f157091'
PROGRAMS = 'a77f5d77-2e2b-44c3-a6a2-aba601054a51'
PROGRAM_FILES_X86 = '7c5a40ef-a0fb-4bfc-874a-c0f2e0b9fa8e'


def require(condition):
    if not condition:
        raise ValueError('Genuine B0 Setup/Maintain observation refused; preserve original state, processes and evidence.')


def customer_launch_evidence(value):
    # Project only native facts with fixed bounds. Never serialize an exception,
    # its cause, caller additions, paths or environment into failure evidence.
    require(type(value) is dict)
    result = {name: value.get(name) for name in
              ('pid', 'threadId', 'created', 'resumeAttempted', 'resumeResult', 'state')}
    for name in ('pid', 'threadId'):
        require(type(result[name]) is int and 0 <= result[name] <= 0xffffffff)
    require(result['created'] is None or
            (type(result['created']) is int and 0 < result['created'] <= 0xffffffffffffffff))
    require(type(result['resumeAttempted']) is bool)
    require(result['resumeResult'] is None or
            (type(result['resumeResult']) is int and 0 <= result['resumeResult'] <= 0xffffffff))
    require(type(result['state']) is str and result['state'] in
            ('not-created', 'creation-effect-uncertain', 'original-child-suspended',
             'resume-effect-uncertain', 'original-child-resumed'))
    return result


def write_customer_launch_refusal(evidence_directory, error):
    facts = customer_launch_evidence(getattr(error, 'launchEvidence', None))
    winerror = getattr(error, 'winerror', None)
    require(winerror is None or (type(winerror) is int and 0 <= winerror <= 0xffffffff))
    raw = (json.dumps({'schema': 1, 'state': 'customer-launch-refused',
                       'launchEvidence': facts, 'winerror': winerror}, sort_keys=True) + '\n').encode('ascii')
    require(len(raw) <= 1024)
    # The enclosing fixture already requires a fresh evidence directory. A
    # collision remains untouched; diagnostics cannot authorize a launch retry.
    with (evidence_directory / 'customer-launch-failure.json').open('xb') as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())


def setup_command(setup, stock, report):
    # NSIS GetOptions requires /NAME="value with spaces", rather than quoting
    # the whole /NAME=value argument. This matches smoke-installer-windows.setup.
    require(all('"' not in str(value) and '\x00' not in str(value) for value in (setup, stock, report)))
    return (subprocess.list2cmdline([str(setup)]) + ' /S /ACTION=Install /OPENCPN="' + str(stock) +
            '" /REPORT="' + str(report) + '"')


def setup_once(command, report, strict, popen=subprocess.Popen):
    require(not report.exists() and not report.is_symlink())
    original = popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # Popen.wait retains the original CreateProcess handle. Timeout never kills
    # a possibly effective installer or retries/cleans its partial namespace.
    require(original.wait(timeout=600) == 0)
    require(report.is_file() and not report.is_symlink() and 0 < report.stat().st_size <= 1 << 20)
    result = strict(report.read_bytes().decode('utf-8-sig').encode('utf-8'))
    require(isinstance(result, dict) and result.get('status') == 'passed' and result.get('action') == 'Install')
    return original.pid


class InstalledB0Fixture:
    def __init__(self, observer, prepared, phase):
        self.n, self.p, self.phase = observer, prepared, phase
        self.root = observer.folder(LOCAL_APP_DATA) / 'OpenNavXAlpha1'
        self.program = observer.folder(PROGRAM_FILES_X86)
        self.menu = observer.folder(PROGRAMS) / 'SKAGER'
        self.machine_recovery = observer.folder(phase.PROGRAM_DATA) / 'SKAGER' / 'ClientRegistrationRequests'
        self.startup_recovery = observer.folder(LOCAL_APP_DATA) / 'SKAGER-Broker-Startup-Recovery'
        self.stock = Path(prepared['request']['stock']['path'])
        require(not self.root.exists() and not self.root.is_symlink() and self.registration() is None)
        self.stock_before = self.inventory(self.stock.parent)

    def inventory(self, root):
        self.phase.updater._plain(root)
        require(root.is_dir())
        files, total = {}, 0
        for path in sorted(root.rglob('*')):
            self.phase.updater._plain(path)
            if path.is_dir():
                continue
            before = self.phase.stamp(path)
            total += before[2]
            require(path.is_file() and before[5] == 1 and len(files) < 20000 and total <= 4 << 30)
            files[path.relative_to(root).as_posix()] = self.phase.updater._hash(path)
            require(self.phase.stamp(path) == before)
        return files

    def registration(self):
        import winreg
        try:
            key = winreg.OpenKey(winreg.HKEY_CURRENT_USER,
                r'Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenNavXAlpha1', 0,
                winreg.KEY_READ | winreg.KEY_WOW64_32KEY)
        except FileNotFoundError:
            return None
        with key:
            count, values, _ = winreg.QueryInfoKey(key)
            require(count == 0 and values == 8)
            result = {}
            for index in range(values):
                name, value, kind = winreg.EnumValue(key, index)
                require(kind == winreg.REG_SZ and name not in result)
                result[name] = value
            return result

    def install_setup(self, evidence):
        self.evidence = evidence
        report = evidence / 'genuine-b0-setup.json'
        self.n.pin_image(self.p['setup'], self.p['setupSha256'])
        pid = setup_once(setup_command(self.p['setup'], self.stock, report), report, self.phase.strict)
        self.state_raw = (self.root / 'state.json').read_bytes()
        self.state = self.phase.strict(self.state_raw)
        require(self.state['owner'] == 'OpenNavX.Alpha1.SideBySide.1' and self.state['schema'] == 1 and
                self.state['previous'] == '' and self.state['shortcutModes'] == ['xnav', 'legacy', 'safe'])
        self.phase.pin(self.state['current'], 32)
        self.generation = self.root / 'generations' / self.state['current']
        self.phase.updater._plain(self.generation)
        ownership = self.phase.strict((self.generation / 'ownership.json').read_bytes())
        require(ownership['version'] == self.p['b0']['version'] and ownership['commit'] == self.p['b0']['commit'] and
                ownership['packageSha256'] == self.p['b0']['packageSha256'])
        # Current mutable manifests are comparison-only. Independently approved
        # original inventory authenticates the installed payload observations.
        for name, digest in self.p['b0']['inventory'].items():
            if name.startswith(('app/', 'docs/')) and name != 'app/OPENNAV_PORTABLE_PREVIEW':
                self.phase.approved_file({'path': str(self.generation / name), 'sha256': digest}, 1 << 30)
        self.maintain = self.generation / 'Maintain.exe'
        self.maintain_sha = self.phase.updater._hash(self.maintain)
        self.maintain_stamp = self.phase.stamp(self.maintain)
        self.payload_sha = self.phase.updater._hash(self.generation / 'maintenance' / 'payload.zip')
        registration = self.registration()
        require(registration is not None and registration['DisplayVersion'] == self.p['b0']['version'] and
                registration['InstallLocation'] == str(self.root) and registration['OpenNavOwner'] == self.state['owner'] and
                registration['UninstallString'] == '"' + str(self.maintain) + '" /ACTION=Uninstall')
        require(self.inventory(self.stock.parent) == self.stock_before)
        return {'state': 'genuine-default-setup-installed', 'setupSha256': self.p['setupSha256'],
                'generation': self.state['current'], 'originalProcessId': pid, 'shortcuts': ['xnav', 'legacy', 'safe']}

    def prepare_preservation(self):
        candidates = [name for name in sorted(self.p['b0']['inventory'])
                      if name.startswith('docs/') and (self.generation / name).stat().st_size <= 1 << 20]
        require(candidates)
        modified = self.generation / candidates[0]
        require(self.phase.updater._hash(modified) == self.p['b0']['inventory'][candidates[0]])
        with modified.open('ab') as stream:
            stream.write(b'\nSCRUM-27 disposable modified original: preserve these bytes.\n')
            stream.flush()
            os.fsync(stream.fileno())
        custom = self.generation / 'app' / 'scrum27-custom-preserve.txt'
        recovery = self.root / 'recovery' / 'scrum27-recovery-preserve.txt'
        for path in (custom, recovery):
            with path.open('xb') as stream:
                stream.write(b'SCRUM-27 disposable custom recovery: preserve exact bytes.\n')
                stream.flush()
                os.fsync(stream.fileno())
        self.recovery_before = self.inventory(self.root / 'recovery')
        self.logs_before = self.inventory(self.root / 'logs') if (self.root / 'logs').exists() else {}
        return {str(path.relative_to(self.root)): self.phase.updater._hash(path) for path in (modified, custom, recovery)}

    def service_process(self):
        w, ui = self.n.w, self.n.ui
        adv = ctypes.WinDLL('advapi32', use_last_error=True)
        scm = ui.declare(adv, 'OpenSCManagerW', w.HANDLE, w.LPCWSTR, w.LPCWSTR, w.DWORD)(None, None, 1)
        require(scm)
        service = None
        handle = None
        close = ui.declare(adv, 'CloseServiceHandle', w.BOOL, w.HANDLE)
        try:
            service = ui.declare(adv, 'OpenServiceW', w.HANDLE, w.HANDLE, w.LPCWSTR, w.DWORD)(scm, 'SkagerUpdateBroker', 4)
            require(service)
            class Status(ctypes.Structure):
                _fields_ = [(name, w.DWORD) for name in ('kind', 'state', 'controls', 'win32Exit', 'specificExit',
                                                       'checkpoint', 'waitHint', 'pid', 'flags')]
            query = ui.declare(adv, 'QueryServiceStatusEx', w.BOOL, w.HANDLE, ctypes.c_int,
                               ctypes.c_void_p, w.DWORD, ctypes.POINTER(w.DWORD))
            status, used = Status(), w.DWORD()
            require(query(service, 0, ctypes.byref(status), ctypes.sizeof(status), ctypes.byref(used)) and
                    status.kind == 0x10 and status.state == 4 and status.controls == 5 and status.pid > 0 and
                    status.win32Exit == 0 and status.specificExit == 0)
            pid = status.pid
            handle = ui.monitor_process(pid)
            self.process_identity(handle, self.program / 'SKAGER' / 'UpdateBroker' / 'skager-update-broker.exe',
                                  self.p['brokerSha256'])
            require(query(service, 0, ctypes.byref(status), ctypes.sizeof(status), ctypes.byref(used)) and
                    status.state == 4 and status.pid == pid)
            result, handle = handle, None
            return result
        finally:
            failed = False
            if handle is not None:
                failed = not ui.CloseHandle(handle) or failed
            if service is not None:
                failed = not close(service) or failed
            failed = not close(scm) or failed
            require(not failed)

    def process_identity(self, handle, path, digest):
        w, ui = self.n.w, self.n.ui
        query = ui.declare(ui.kernel, 'QueryFullProcessImageNameW', w.BOOL, w.HANDLE, w.DWORD,
                           w.LPWSTR, ctypes.POINTER(w.DWORD))
        size, image = w.DWORD(32768), ctypes.create_unicode_buffer(32768)
        require(query(handle, 0, image, ctypes.byref(size)))
        actual = Path(image.value)
        require((path is None or actual.samefile(path)) and self.phase.updater._hash(actual) == digest)
        times = [w.FILETIME() for _ in range(4)]
        require(ui.declare(ui.kernel, 'GetProcessTimes', w.BOOL, w.HANDLE, *([ctypes.POINTER(w.FILETIME)] * 4))(
            handle, *(ctypes.byref(value) for value in times)))
        require(ui.WaitForSingleObject(handle, 0) == 258)
        return (times[0].dwHighDateTime << 32) | times[0].dwLowDateTime

    def process_parent(self, pid):
        w, ui = self.n.w, self.n.ui
        class Entry(ctypes.Structure):
            _fields_ = [('size', w.DWORD), ('usage', w.DWORD), ('pid', w.DWORD), ('heap', ctypes.c_size_t),
                        ('module', w.DWORD), ('threads', w.DWORD), ('parent', w.DWORD),
                        ('priority', w.LONG), ('flags', w.DWORD), ('name', w.WCHAR * 260)]
        snapshot = ui.declare(ui.kernel, 'CreateToolhelp32Snapshot', w.HANDLE, w.DWORD, w.DWORD)(2, 0)
        require(snapshot not in (None, ctypes.c_void_p(-1).value))
        try:
            entry = Entry()
            entry.size = ctypes.sizeof(entry)
            require(ui.declare(ui.kernel, 'Process32FirstW', w.BOOL, w.HANDLE, ctypes.POINTER(Entry))(snapshot, ctypes.byref(entry)))
            while True:
                if entry.pid == pid:
                    return entry.parent
                if not ui.declare(ui.kernel, 'Process32NextW', w.BOOL, w.HANDLE, ctypes.POINTER(Entry))(snapshot, ctypes.byref(entry)):
                    require(ctypes.get_last_error() == 18)
                    raise ValueError('Original relocated Maintain process absent.')
        finally:
            require(ui.CloseHandle(snapshot))

    def press(self, frame, pid, handle, identifier):
        ui = self.n.ui
        require((frame, pid, 'SKAGER Maintenance') in ui.windows(pid) and ui.WaitForSingleObject(handle, 0) == 258)
        button = ui.declare(ui.user, 'GetDlgItem', self.n.w.HWND, self.n.w.HWND, ctypes.c_int)(frame, identifier)
        require(button and ui.IsWindowEnabled(button) and ui.IsWindowEnabled(frame))
        ui.SetForegroundWindow(frame)
        rect = self.n.w.RECT()
        require(ui.GetWindowRect(button, ctypes.byref(rect)))
        point = self.n.w.POINT((rect.left + rect.right) // 2, (rect.top + rect.bottom) // 2)
        require(ui.WindowFromPoint(point) == button and ui.SetCursorPos(point.x, point.y))
        ui.MouseEvent(2, 0, 0, 0, 0)
        time.sleep(.05)
        ui.MouseEvent(4, 0, 0, 0, 0)

    def customer_uninstall(self):
        from installed_customer_launch import CustomerLaunchRefused, inspect_customer_capability, open_same_account_launch
        require(self.phase.stamp(self.maintain) == self.maintain_stamp and
                self.phase.updater._hash(self.maintain) == self.maintain_sha)
        self.machine_recovery_before = self.inventory(self.machine_recovery)
        require(self.machine_recovery_before.get(self.n.sid + '.baseline.zip') == self.p['b0']['recovery']['sha256'])
        require(all(self.n.sid + suffix in self.machine_recovery_before
                    for suffix in ('.json', '.maintenance.once', '.maintenance.json')))
        self.startup_recovery_before = Counter(self.inventory(self.startup_recovery).values())
        # Capability is diagnostic evidence, never authority or consent. Retain
        # even a blocked route after genuine Setup/enrollment without changing
        # host policy, manufacturing a token or invoking a private product action.
        capability = inspect_customer_capability()
        with (self.evidence / 'customer-capability.json').open('x', encoding='utf-8') as stream:
            json.dump(capability, stream, sort_keys=True, indent=2)
            stream.write('\n')
        ui = self.n.ui
        require(not any(title == 'SKAGER Maintenance' for _, _, title in ui.windows()))
        broker, child, wrapper = self.service_process(), None, None
        try:
            try:
                wrapper = open_same_account_launch([str(self.maintain), '/ACTION=Uninstall'], self.n.sid)
            except CustomerLaunchRefused as error:
                try:
                    write_customer_launch_refusal(self.evidence, error)
                except Exception:
                    # Preserve the original native refusal even if diagnostics
                    # are malformed, unavailable or already present.
                    pass
                raise
            require(wrapper.sid == self.n.sid and not wrapper.elevated)
            launch = customer_launch_evidence(wrapper.launch_evidence)
            require(launch['state'] == 'original-child-resumed' and launch['pid'] == wrapper.pid and
                    launch['created'] == wrapper.created and launch['threadId'] > 0 and
                    launch['resumeAttempted'] and launch['resumeResult'] == 1)
            wrapper.check()
            frame, pid = ui.wait_window('SKAGER Maintenance', timeout=45)
            require([(h, process) for h, process, title in ui.windows() if title == 'SKAGER Maintenance'] == [(frame, pid)])
            require(pid != wrapper.pid and self.process_parent(pid) == wrapper.pid)
            child = ui.monitor_process(pid)
            require(self.process_identity(child, None, self.maintain_sha) >= wrapper.created)
            controls = ui.children(frame)
            require(sum(label == 'Maintain SKAGER' for _, label in controls) == 1)
            combos = []
            for control, _ in controls:
                kind = ctypes.create_unicode_buffer(128)
                ui.GetClassNameW(control, kind, len(kind))
                if kind.value == 'ComboBox':
                    combos.append(control)
            require(len(combos) == 1 and ui.control_text(combos[0]) == 'Uninstall' and ui.IsWindowEnabled(combos[0]))
            wrapper.check()
            self.press(frame, pid, child, 1)
            deadline = time.monotonic() + 180
            while time.monotonic() < deadline:
                wrapper.check()
                require(ui.WaitForSingleObject(child, 0) == 258)
                labels = [label for _, label in ui.children(frame)]
                require((frame, pid, 'SKAGER Maintenance') in ui.windows(pid))
                require(not any('Maintenance failed.' in label for dialog, _, _ in ui.windows(pid)
                                for _, label in ui.children(dialog)))
                button = ui.declare(ui.user, 'GetDlgItem', self.n.w.HWND, self.n.w.HWND, ctypes.c_int)(frame, 1)
                if button and ui.control_text(button).replace('&', '') == 'Finish' and ui.IsWindowEnabled(button):
                    require(any('SKAGER maintenance complete' in label for label in labels))
                    break
                time.sleep(.1)
            else:
                raise ValueError('Original Maintain did not reach completion; leave processes and recovery untouched.')
            wrapper.check()
            self.press(frame, pid, child, 1)
            require(self.wait_exit(child, 30000) == 0 and wrapper.wait(timeout=30) == 0)
            require(self.wait_exit(broker, 30000) == 0)
            return {'state': 'original-relocated-maintain-exited-cleanly', 'originalWrapperPid': wrapper.pid,
                    'originalWizardPid': pid, 'launchEvidence': launch, 'capability': wrapper.capability,
                    'consentPromptAcceptance': False}
        finally:
            pending = sys.exc_info()[0] is not None
            failed = False
            for handle in (child, broker):
                if handle is not None:
                    try:
                        failed = not ui.CloseHandle(handle) or failed
                    except Exception:
                        failed = True
            if wrapper is not None:
                try:
                    wrapper.close()
                except Exception:
                    failed = True
            if not pending:
                require(not failed)

    def wait_exit(self, handle, timeout):
        require(self.n.ui.WaitForSingleObject(handle, timeout) == 0)
        code = self.n.w.DWORD()
        require(self.n.ui.GetExitCodeProcess(handle, ctypes.byref(code)))
        return code.value

    def service_absent(self):
        w, ui = self.n.w, self.n.ui
        adv = ctypes.WinDLL('advapi32', use_last_error=True)
        scm = ui.declare(adv, 'OpenSCManagerW', w.HANDLE, w.LPCWSTR, w.LPCWSTR, w.DWORD)(None, None, 1)
        require(scm)
        close = ui.declare(adv, 'CloseServiceHandle', w.BOOL, w.HANDLE)
        try:
            service = ui.declare(adv, 'OpenServiceW', w.HANDLE, w.HANDLE, w.LPCWSTR, w.DWORD)(scm, 'SkagerUpdateBroker', 4)
            if service:
                require(close(service))
                return False
            error = ctypes.get_last_error()
            require(error in (1060, 1072))
            return error == 1060
        finally:
            require(close(scm))

    def verify_uninstall(self, preserved):
        deadline = time.monotonic() + 30
        while not self.service_absent() and time.monotonic() < deadline:
            time.sleep(.1)
        require(self.service_absent() and self.registration() is None)
        require(not (self.root / 'state.json').exists() and not (self.root / 'state.json').is_symlink())
        retired = self.root / 'retired-state.json'
        require(retired.read_bytes() == self.state_raw)
        require(not (self.generation / 'app' / 'opencpn.exe').exists())
        require(self.phase.updater._hash(self.maintain) == self.maintain_sha and
                self.phase.updater._hash(self.generation / 'maintenance' / 'payload.zip') == self.payload_sha)
        for name, digest in preserved.items():
            require(self.phase.updater._hash(self.root / name) == digest)
        require(self.inventory(self.root / 'recovery') == self.recovery_before and
                self.inventory(self.stock.parent) == self.stock_before)
        require(self.inventory(self.machine_recovery) == self.machine_recovery_before)
        startup_after = Counter(self.inventory(self.startup_recovery).values())
        require(all(startup_after[digest] >= count for digest, count in self.startup_recovery_before.items()))
        if self.logs_before:
            logs_after = self.inventory(self.root / 'logs')
            require(all(logs_after.get(name) == digest for name, digest in self.logs_before.items()))
        for name in ('Skager.lnk', 'Skager Legacy.lnk', 'Skager Safe Mode.lnk', 'OpenCPN Legacy.lnk', 'Maintain Skager.lnk'):
            require(not (self.menu / name).exists() and not (self.menu / name).is_symlink())
        return {'state': 'service-and-registration-absent-original-state-retired',
                'stock': 'preserved', 'baseline': 'preserved', 'recovery': 'preserved',
                'modifiedAndCustomFiles': len(preserved), 'nativeProductAcceptance': False}

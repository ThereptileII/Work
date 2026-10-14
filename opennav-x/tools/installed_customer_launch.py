"""Disposable installed-fixture normal-token launch; never approves UAC.

The caller authenticates original Maintain bytes and owns relocated UI/process
observations and uninstall postconditions. This helper accepts only the current
account's fixed integration Maintain command. It does not alter policy, enable
privileges, manufacture tokens, use credentials or invoke a private elevated
product action. A timeout never kills or retries a possibly effective operation.

Hosted images currently configure silent admin elevation; live observations
below take precedence over this image default:
https://github.com/actions/runner-images/blob/main/images/windows/scripts/build/Configure-BaseImage.ps1
"""
import ctypes as C
from ctypes import wintypes as W
import json
import math
import ntpath
import re
import subprocess
import sys
import time
import uuid


class CustomerLaunchRefused(RuntimeError):
    pass


def _require(value, message):
    if not value:
        raise CustomerLaunchRefused(message)


def _command(command, local):
    _require(isinstance(command, (list, tuple)) and len(command) == 2 and command[1] == '/ACTION=Uninstall',
             'Only the actual installed Maintain Uninstall command is allowed')
    image = str(command[0])
    _require(len(image) < 960 and not any(c in image for c in '\r\n\x00"') and
             ntpath.isabs(image) and ntpath.normpath(image) == image and image[1:3] == ':\\',
             'Canonical actual installed Maintain path required')
    relative = ntpath.relpath(image, local).split('\\')
    _require(len(relative) == 4 and relative[0].casefold() == 'opennavxalpha1' and
             relative[1] == 'generations' and re.fullmatch('[a-f0-9]{32}', relative[2]) and relative[3] == 'Maintain.exe',
             'Maintain must belong to this actual current account integration')
    return [image, '/ACTION=Uninstall']


def _normal(current, linked):
    return (isinstance(current, dict) and isinstance(linked, dict) and
            current.get('elevated') is True and current.get('elevationType') == 2 and
            current.get('tokenType') == 1 and current.get('session', 0) > 0 and
            isinstance(current.get('sid'), str) and re.fullmatch(r'S-1-5-21-(?:[1-9][0-9]*-){3}[1-9][0-9]*', current['sid']) is not None and
            linked.get('sid') == current.get('sid') and linked.get('session') == current.get('session') and
            linked.get('elevated') is False and linked.get('elevationType') == 3 and
            linked.get('tokenType') == 1 and linked.get('integrity') == 'S-1-16-8192')


def _capability(observed):
    current, linked = observed.get('current'), observed.get('linked')
    desktop, policy = observed.get('desktop', {}), observed.get('policy', {})
    normal = _normal(current, linked)
    interactive = desktop.get('station') == 'WinSta0' and desktop.get('visible') is True and desktop.get('input') == 'Default' and desktop.get('thread') == 'Default'
    consent = (policy.get('enableLUA') == 1 and policy.get('consentPromptBehaviorAdmin') in (2, 5) and
               policy.get('promptOnSecureDesktop') == 1)
    available = normal and interactive
    reason = ('normal-linked-token-unavailable' if not normal else
              'interactive-default-desktop-unavailable' if not interactive else
              'manual-secure-desktop-consent-required' if consent else 'actual-consent-prompt-not-available')
    return dict(schema=1, state='available' if available else 'blocked',
                route=('same-account-normal-token-manual-consent' if consent else 'same-account-normal-token-silent-elevation')
                if available else 'interactive-disposable-split-token-windows-required',
                normalTokenAvailable=normal, consentPromptAvailable=consent,
                consentPromptAcceptance=False, nativeCustomerAcceptance=False, reason=reason, observations=observed)


def inspect_customer_capability(*, _native_factory=None):
    """Read-only live observations. Availability is never customer acceptance."""
    if _native_factory is None and sys.platform != 'win32':
        return dict(schema=1, state='blocked', route='native-windows-required', normalTokenAvailable=False,
                    consentPromptAvailable=False, consentPromptAcceptance=False,
                    nativeCustomerAcceptance=False, reason='native-windows-unavailable', observations={})
    native = None
    try:
        native = (_native_factory or _Native)()
        result = _capability(native.observed)
        native.close()
        return result
    except (OSError, CustomerLaunchRefused) as error:
        if native is not None:
            try:
                native.close()
            except (OSError, CustomerLaunchRefused):
                pass  # Already a refused observation; never report availability.
        return dict(schema=1, state='blocked', route='interactive-disposable-split-token-windows-required',
                    normalTokenAvailable=False, consentPromptAvailable=False, consentPromptAcceptance=False,
                    nativeCustomerAcceptance=False, reason='native-capability-observation-refused',
                    observations={'nativeError': getattr(error, 'winerror', None)})


class NativeCustomerProcess:
    """Own the returned original process handle; no reopen-by-PID, kill or retry."""
    def __init__(self, native, child, command, capability):
        self.native, self.child, self.command, self.capability = native, child, command, capability
        self.pid, self.created = child['pid'], child['created']
        self.sid, self.session, self.elevated = child['token']['sid'], child['token']['session'], child['token']['elevated']
        self.closed, self.close_error = False, None

    def check(self):
        _require(not self.closed, 'Original customer process owner is closed')
        self.native.check(self.child)

    def poll(self):
        self.check()
        return self.native.poll(self.child)

    def wait(self, timeout=150):
        _require(isinstance(timeout, (int, float)) and math.isfinite(timeout) and 0 <= timeout <= 180,
                 'Bounded original customer process wait required')
        deadline = time.monotonic() + timeout
        while True:
            result = self.poll()
            if result is not None:
                return result
            if time.monotonic() >= deadline:
                raise subprocess.TimeoutExpired(self.command, timeout)
            self.native.wait_tick(self.child)

    def close(self):
        if not self.closed:
            self.closed = True
            try:
                self.native.close()
            except Exception as error:
                self.close_error = error
        if self.close_error is not None:
            raise self.close_error


def open_same_account_launch(command, expected_sid, *, _native_factory=None):
    """Launch only Maintain using the ORIGINAL actual same-account linked token.

    expected_sid can only veto an account mismatch. It cannot choose a token or
    user. Callers must close the returned owner even if later UI checks refuse.
    No UAC consent is approved, synthesized or claimed by this operation.
    """
    _require(_native_factory is not None or sys.platform == 'win32', 'Native Windows customer launch required')
    native = (_native_factory or _Native)()
    try:
        capability = _capability(native.observed)
        _require(capability['state'] == 'available', capability['reason'])
        _require(isinstance(expected_sid, str) and expected_sid == native.observed['current']['sid'],
                 'Approved client comparison differs from actual current account')
        fixed = _command(command, native.local)
        child = native.create(fixed)
        owner = NativeCustomerProcess(native, child, fixed, capability)
        owner.check()
        return owner
    except BaseException:
        native.close()
        raise


class _Native:
    def __init__(self):
        self.owned, self.closed, self.close_error = [], False, None
        self.kernel = C.WinDLL('kernel32.dll', use_last_error=True, winmode=0x800)
        self.adv = C.WinDLL('advapi32.dll', use_last_error=True, winmode=0x800)
        self.user = C.WinDLL('user32.dll', use_last_error=True, winmode=0x800)
        self.shell = C.WinDLL('shell32.dll', use_last_error=True, winmode=0x800)
        self.ole = C.WinDLL('ole32.dll', use_last_error=True, winmode=0x800)
        self.close_handle = self._fn(self.kernel, 'CloseHandle', W.BOOL, W.HANDLE)
        try:
            thread = W.HANDLE()
            call = self._fn(self.adv, 'OpenThreadToken', W.BOOL, W.HANDLE, W.DWORD, W.BOOL, C.POINTER(W.HANDLE))
            if call(self._fn(self.kernel, 'GetCurrentThread', W.HANDLE)(), 8, True, C.byref(thread)):
                self.owned.append((thread, self.close_handle))
                raise CustomerLaunchRefused('Impersonated fixture execution refused')
            _require(C.get_last_error() == 1008, 'Actual non-impersonated fixture token required')
            self.current = self._process_token(self._fn(self.kernel, 'GetCurrentProcess', W.HANDLE)())
            current = self._token(self.current)
            linked = W.HANDLE()
            try:
                self._info(self.current, 19, linked)
            except OSError as error:
                self.linked, self.linked_error = None, error.winerror
            else:
                _require(linked.value, 'Original linked token handle required')
                self.owned.append((linked, self.close_handle))
                self.linked, self.linked_error = linked, None
            self.observed = dict(current=current, linked=self._token(self.linked) if self.linked else None,
                                 linkedError=self.linked_error, desktop=self._desktop(), policy=self._policy())
            self.local = self._folder('f1b32785-6fba-4fcf-9d55-7b8e7f157091')
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _fn(dll, name, result, *args):
        value = getattr(dll, name)
        value.restype, value.argtypes = result, args
        return value

    @staticmethod
    def _ok(value):
        if not value:
            raise C.WinError(C.get_last_error())
        return value

    def _info(self, token, kind, target):
        size = W.DWORD()
        call = self._fn(self.adv, 'GetTokenInformation', W.BOOL, W.HANDLE, C.c_int, C.c_void_p, W.DWORD, C.POINTER(W.DWORD))
        self._ok(call(token, kind, C.byref(target), C.sizeof(target), C.byref(size)))
        _require(size.value == C.sizeof(target), 'Exact native token information required')
        return target

    def _buffer(self, token, kind):
        size = W.DWORD()
        call = self._fn(self.adv, 'GetTokenInformation', W.BOOL, W.HANDLE, C.c_int, C.c_void_p, W.DWORD, C.POINTER(W.DWORD))
        _require(not call(token, kind, None, 0, C.byref(size)) and C.get_last_error() == 122 and 0 < size.value <= 65536,
                 'Bounded native token observation required')
        buffer = C.create_string_buffer(size.value)
        self._ok(call(token, kind, buffer, len(buffer), C.byref(size)))
        return buffer

    def _sid(self, pointer):
        text = W.LPWSTR()
        self._ok(self._fn(self.adv, 'ConvertSidToStringSidW', W.BOOL, C.c_void_p, C.POINTER(W.LPWSTR))(pointer, C.byref(text)))
        try:
            return text.value
        finally:
            _require(not self._fn(self.kernel, 'LocalFree', W.HANDLE, C.c_void_p)(C.cast(text, C.c_void_p)), 'Native SID buffer close failed')

    def _token(self, token):
        user, integrity = self._buffer(token, 1), self._buffer(token, 25)
        sid = self._sid(C.cast(user, C.POINTER(C.c_void_p))[0])
        level = self._sid(C.cast(integrity, C.POINTER(C.c_void_p))[0])
        values = {key: self._info(token, kind, W.DWORD()).value for key, kind in
                  (('session', 12), ('tokenType', 8), ('elevationType', 18), ('elevated', 20))}
        values.update(sid=sid, integrity=level)
        values['elevated'] = bool(values['elevated'])
        return values

    def _process_token(self, process):
        token = W.HANDLE()
        self._ok(self._fn(self.adv, 'OpenProcessToken', W.BOOL, W.HANDLE, W.DWORD, C.POINTER(W.HANDLE))(process, 8, C.byref(token)))
        self.owned.append((token, self.close_handle))
        return token

    def _object_name(self, handle):
        size = W.DWORD()
        raw = C.create_unicode_buffer(256)
        self._ok(self._fn(self.user, 'GetUserObjectInformationW', W.BOOL, W.HANDLE, C.c_int, C.c_void_p, W.DWORD, C.POINTER(W.DWORD))(
            handle, 2, raw, C.sizeof(raw), C.byref(size)))
        return raw.value

    def _desktop(self):
        class Flags(C.Structure):
            _fields_ = [('inherit', W.BOOL), ('reserved', W.BOOL), ('flags', W.DWORD)]
        station = self._fn(self.user, 'GetProcessWindowStation', W.HANDLE)()
        flags, size = Flags(), W.DWORD()
        self._ok(self._fn(self.user, 'GetUserObjectInformationW', W.BOOL, W.HANDLE, C.c_int, C.c_void_p, W.DWORD, C.POINTER(W.DWORD))(
            station, 1, C.byref(flags), C.sizeof(flags), C.byref(size)))
        desktop = self._fn(self.user, 'OpenInputDesktop', W.HANDLE, W.DWORD, W.BOOL, W.DWORD)(0, False, 1)
        if not desktop:
            error = C.get_last_error()
            return dict(station=self._object_name(station), visible=bool(flags.flags & 1), input=None,
                        thread=None, nativeError=error)
        self.owned.append((desktop, self._fn(self.user, 'CloseDesktop', W.BOOL, W.HANDLE)))
        thread_id = self._fn(self.kernel, 'GetCurrentThreadId', W.DWORD)()
        thread_desktop = self._fn(self.user, 'GetThreadDesktop', W.HANDLE, W.DWORD)(thread_id)
        self._ok(thread_desktop)  # Borrowed native desktop handle: never close it.
        return dict(station=self._object_name(station), visible=bool(flags.flags & 1),
                    input=self._object_name(desktop), thread=self._object_name(thread_desktop))

    @staticmethod
    def _policy():
        import winreg
        result = {}
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r'SOFTWARE\Microsoft\Windows\CurrentVersion\Policies\System',
                            0, winreg.KEY_READ | winreg.KEY_WOW64_64KEY) as key:
            for public, native in (('enableLUA', 'EnableLUA'), ('consentPromptBehaviorAdmin', 'ConsentPromptBehaviorAdmin'),
                                   ('promptOnSecureDesktop', 'PromptOnSecureDesktop')):
                try:
                    value, kind = winreg.QueryValueEx(key, native)
                    result[public] = value if kind == winreg.REG_DWORD else None
                except FileNotFoundError:
                    result[public] = None
        return result

    def _folder(self, identifier):
        guid = (C.c_ubyte * 16).from_buffer_copy(uuid.UUID(identifier).bytes_le)
        raw = C.c_void_p()
        status = self._fn(self.shell, 'SHGetKnownFolderPath', C.c_long, C.c_void_p, W.DWORD, W.HANDLE, C.POINTER(C.c_void_p))(
            C.byref(guid), 0, None, C.byref(raw))
        _require(status == 0 and raw.value, 'Actual current LocalAppData required')
        try:
            return C.wstring_at(raw.value)
        finally:
            self._fn(self.ole, 'CoTaskMemFree', None, C.c_void_p)(raw)

    def _created(self, process):
        times = [W.FILETIME() for _ in range(4)]
        self._ok(self._fn(self.kernel, 'GetProcessTimes', W.BOOL, W.HANDLE, *([C.POINTER(W.FILETIME)] * 4))(
            process, *(C.byref(value) for value in times)))
        value = (times[0].dwHighDateTime << 32) | times[0].dwLowDateTime
        _require(value > 0, 'Original customer process creation required')
        return value

    def _image(self, process):
        raw, size = C.create_unicode_buffer(32768), W.DWORD(32768)
        self._ok(self._fn(self.kernel, 'QueryFullProcessImageNameW', W.BOOL, W.HANDLE, W.DWORD, W.LPWSTR, C.POINTER(W.DWORD))(
            process, 0, raw, C.byref(size)))
        _require(0 < size.value < len(raw), 'Bounded original customer process image required')
        return raw.value

    def create(self, command):
        _require(_normal(self._token(self.current), self._token(self.linked)), 'Original linked normal token changed')
        class Startup(C.Structure):
            _fields_ = [('cb', W.DWORD), ('reserved', W.LPWSTR), ('desktop', W.LPWSTR), ('title', W.LPWSTR),
                        ('x', W.DWORD), ('y', W.DWORD), ('width', W.DWORD), ('height', W.DWORD),
                        ('charsX', W.DWORD), ('charsY', W.DWORD), ('fill', W.DWORD), ('flags', W.DWORD),
                        ('show', W.WORD), ('reservedBytes', W.WORD), ('reservedPointer', C.c_void_p),
                        ('stdin', W.HANDLE), ('stdout', W.HANDLE), ('stderr', W.HANDLE)]
        class Process(C.Structure):
            _fields_ = [('process', W.HANDLE), ('thread', W.HANDLE), ('pid', W.DWORD), ('tid', W.DWORD)]
        startup, process = Startup(), Process()
        startup.cb = C.sizeof(startup)
        line = C.create_unicode_buffer(subprocess.list2cmdline(command))
        # Pass the ORIGINAL linked primary token. No duplication, credentials,
        # privilege adjustment, policy write or elevation verb is used here.
        call = self._fn(self.adv, 'CreateProcessWithTokenW', W.BOOL, W.HANDLE, W.DWORD, W.LPCWSTR, W.LPWSTR,
                        W.DWORD, C.c_void_p, W.LPCWSTR, C.POINTER(Startup), C.POINTER(Process))
        ok = call(self.linked, 0, command[0], line, 0, None, ntpath.dirname(command[0]), C.byref(startup), C.byref(process))
        for handle in (process.process, process.thread):
            if handle:
                self.owned.append((handle, self.close_handle))
        self._ok(ok)
        _require(process.process and process.thread and process.pid > 0, 'Original returned customer process handles required')
        thread_pid = self._fn(self.kernel, 'GetProcessIdOfThread', W.DWORD, W.HANDLE)(process.thread)
        _require(thread_pid == process.pid, 'Original returned primary thread differs')
        token = self._process_token(process.process)
        result = dict(handle=process.process, pid=process.pid, created=self._created(process.process),
                      token=self._token(token), tokenHandle=token, image=command[0])
        self.check(result)
        return result

    def check(self, child):
        _require(not self.closed, 'Original native launch owners closed')
        _require(self._token(self.current) == self.observed['current'] and self._token(self.linked) == self.observed['linked'],
                 'Original current/linked token observation changed')
        actual = self._token(child['tokenHandle'])
        _require(actual == child['token'] and _normal(self.observed['current'], actual), 'Customer process is not the actual same-account normal token')
        pid = self._fn(self.kernel, 'GetProcessId', W.DWORD, W.HANDLE)(child['handle'])
        _require(pid == child['pid'] and self._created(child['handle']) == child['created'], 'Original returned process identity changed')
        if self._state(child) == 258:
            _require(ntpath.normcase(self._image(child['handle'])) == ntpath.normcase(child['image']), 'Original customer image differs')

    def _state(self, child):
        value = self._fn(self.kernel, 'WaitForSingleObject', W.DWORD, W.HANDLE, W.DWORD)(child['handle'], 0)
        _require(value in (0, 258), 'Original returned process wait refused')
        return value

    def poll(self, child):
        if self._state(child) == 258:
            return None
        result = W.DWORD()
        self._ok(self._fn(self.kernel, 'GetExitCodeProcess', W.BOOL, W.HANDLE, C.POINTER(W.DWORD))(child['handle'], C.byref(result)))
        return result.value

    def wait_tick(self, child):
        value = self._fn(self.kernel, 'WaitForSingleObject', W.DWORD, W.HANDLE, W.DWORD)(child['handle'], 25)
        _require(value in (0, 258), 'Original customer process wait refused')

    def close(self):
        if self.closed:
            if self.close_error is not None:
                raise self.close_error
            return
        self.closed = True
        failures = []
        for handle, close in reversed(self.owned):
            if not close(handle):
                failures.append(C.get_last_error())
        if failures:
            self.close_error = CustomerLaunchRefused('Original native owner close failed: ' + ','.join(map(str, failures)))
            raise self.close_error


if __name__ == '__main__':
    _require(len(sys.argv) == 1, 'Capability inspection accepts no identity or launch arguments')
    print(json.dumps(inspect_customer_capability(), sort_keys=True))

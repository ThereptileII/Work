#!/usr/bin/env python3
"""One real account on an exact disposable GitHub-hosted job; no product action.

No caller command, password arguments/files/environment, privilege enablement,
restricted tokens, policy writes or desktop ACL changes. The original OS LIMITED
PRIMARY logon token launches only the original installed_customer_launch.py
fixed literal profile-file capability worker (the noargs CLI is unchanged). The linked FULL token is observed, never converted or
used by this entry. Original token statistics survive refusal. Limited-child
desktop observations do not enable the elevated-parent customer launch API.
Bounded flushed progress and the final report use one child-created fixed file
in the actual-token profile. No stdout/stderr handles, environment paths or
handle inheritance are assumed. Original verified exit remains necessary.
Explicit WinSta0\\Default avoids CreateProcessWithTokenW's implicit inherited
window-station/desktop permission additions. Failures preserve uncertain live
process/account/profile state for disposable-job destruction rather than kill.

Native contracts:
https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-logonuserw
https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-createprocesswithtokenw
https://learn.microsoft.com/en-us/windows/win32/api/handleapi/nf-handleapi-duplicatehandle
https://learn.microsoft.com/en-us/windows/win32/api/handleapi/nf-handleapi-compareobjecthandles
https://learn.microsoft.com/en-us/windows/win32/api/securitybaseapi/nf-securitybaseapi-duplicatetokenex
https://learn.microsoft.com/en-us/windows/win32/api/userenv/nf-userenv-deleteprofilew
https://learn.microsoft.com/en-us/windows/win32/api/lmaccess/nf-lmaccess-netuseradd
"""
import ctypes as C
from ctypes import wintypes as W
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import subprocess
import sys
import time

import installed_customer_launch as customer


class Refused(Exception):
    def __init__(self, reason, winerror=None):
        self.reason, self.winerror = reason, winerror


def require(value, reason):
    if not value:
        raise Refused(reason)


def number(value):
    return value if type(value) is int and 0 <= value <= 0xffffffff else None


def token_fact(value):
    require(type(value) is dict and type(value.get('sid')) is str and
            re.fullmatch(r'S-1-5-21-(?:[1-9][0-9]*-){3}[1-9][0-9]*', value['sid']), 'token-contract-refused')
    require(number(value.get('session')) is not None and type(value.get('tokenType')) is int and
            value['tokenType'] in (1, 2) and type(value.get('elevationType')) is int and
            value['elevationType'] in (1, 2, 3) and type(value.get('elevated')) is bool and
            value.get('integrity') in ('S-1-16-4096', 'S-1-16-8192', 'S-1-16-12288', 'S-1-16-16384'),
            'token-contract-refused')
    return {key: value[key] for key in ('sid', 'session', 'tokenType', 'elevationType', 'elevated', 'integrity')}


def original_full(first, linked, expected_sid):
    first, linked = token_fact(first), token_fact(linked)
    require(first['sid'] == linked['sid'] == expected_sid and first['session'] == linked['session'] and
            first['tokenType'] == 1, 'original-split-token-identity-refused')
    full, limited = (first, linked) if first['elevationType'] == 2 else (linked, first)
    require(full['elevated'] and full['elevationType'] == 2 and full['integrity'] == 'S-1-16-12288' and
            not limited['elevated'] and limited['elevationType'] == 3 and limited['integrity'] == 'S-1-16-8192' and
            limited['tokenType'] == 1,
            'original-split-token-unavailable')
    return 0 if full == first else 1


def capability_progress(output, diagnostics):
    """Sanitized diagnostic projection only; never return captured bytes."""
    require(len(output) <= 16384 and len(diagnostics) <= 1024, 'capability-progress-bound-refused')
    observed, valid = [], True
    lines = diagnostics.split(b'\n')
    for line in lines[:-1]:
        line = line.removesuffix(b'\r')  # Windows text-stream newline only.
        index = len(observed)
        if (valid and index < len(customer.CAPABILITY_PROGRESS_MARKERS) and
                line == (customer.CAPABILITY_PROGRESS_PREFIX + customer.CAPABILITY_PROGRESS_MARKERS[index]).encode('ascii')):
            observed.append(customer.CAPABILITY_PROGRESS_MARKERS[index])
        else:
            valid = False
    try:
        final_document = type(json.loads(output.decode('utf-8'))) is dict
    except (ValueError, UnicodeError, RecursionError):
        final_document = False
    return dict(markers=observed, lastMarker=observed[-1] if observed else None,
                markerProtocolValid=valid, markerSequenceComplete=valid and not lines[-1] and
                len(observed) == len(customer.CAPABILITY_PROGRESS_MARKERS),
                stdoutBytes=len(output), stderrBytes=len(diagnostics), finalDocumentPresent=final_document)



def file_capability_parts(raw):
    require(len(raw) <= customer.PROFILE_CAPABILITY_BOUND, 'capability-file-bound-refused')
    parts = raw.split(b'\n', 3)
    if len(parts) < 4:
        return b'', raw
    return parts[3], b'\n'.join(parts[:3]) + b'\n'


def file_capability_progress(raw):
    output, markers = file_capability_parts(raw)
    result = capability_progress(output, markers)
    result['reportBytes'] = result.pop('stdoutBytes')
    result['markerBytes'] = result.pop('stderrBytes')
    return dict(result, channel='actual-token-profile-file', fileBytes=len(raw))

def project_capability(raw, expected_sid):
    require(len(raw) <= 16384, 'capability-output-bound-refused')
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'capability-duplicate-field-refused')
            result[key] = value
        return result
    value = json.loads(raw.decode('utf-8'), object_pairs_hook=pairs)
    require(type(value) is dict and value.get('schema') == 1 and
            value.get('nativeCustomerAcceptance') is False and value.get('consentPromptAcceptance') is False,
            'capability-contract-refused')
    observed = value.get('observations')
    require(type(observed) is dict, 'capability-contract-refused')
    if value.get('reason') == 'native-capability-observation-refused':
        require(value.get('state') == 'blocked' and number(observed.get('nativeError')) == observed.get('nativeError'),
                'capability-contract-refused')
        return dict(schema=1, state='blocked', reason='native-capability-observation-refused',
                    nativeCustomerAcceptance=False, consentPromptAcceptance=False,
                    observations={'nativeError': number(observed.get('nativeError'))})
    current = token_fact(observed.get('current'))
    require(current['sid'] == expected_sid, 'child-capability-account-mismatch')
    linked = token_fact(observed['linked']) if observed.get('linked') is not None else None
    desktop, policy = observed.get('desktop'), observed.get('policy')
    require(type(desktop) is dict and type(policy) is dict and type(desktop.get('visible')) is bool,
            'capability-contract-refused')
    clean = dict(current=current, linked=linked, linkedError=number(observed.get('linkedError')),
        desktop=dict(station='WinSta0' if desktop.get('station') == 'WinSta0' else 'other',
                     visible=desktop['visible'], input='Default' if desktop.get('input') == 'Default' else None,
                     thread='Default' if desktop.get('thread') == 'Default' else None,
                     nativeError=number(desktop.get('nativeError'))),
        policy={key: number(policy.get(key)) for key in
                ('enableLUA', 'consentPromptBehaviorAdmin', 'promptOnSecureDesktop')})
    result = customer._capability(clean)
    require(all(value.get(key) == result[key] for key in ('state', 'route', 'reason', 'normalTokenAvailable',
                                                         'consentPromptAvailable')), 'capability-contract-refused')
    return result


def project_limited_capability(raw, expected_child):
    """Observe the LIMITED child; never grant the elevated-parent launch API."""
    result = project_capability(raw, expected_child['sid'])
    observed = result['observations']
    if result['reason'] == 'native-capability-observation-refused':
        return dict(result, route='limited-primary-read-only-child', limitedPrimaryChildObserved=False,
                    elevatedParentLaunchAvailable=False, sameAccountElevationTested=False)
    current = observed['current']
    require(current == expected_child and current['tokenType'] == 1 and
            current['elevationType'] == 3 and current['elevated'] is False and
            current['integrity'] == 'S-1-16-8192' and current['session'] > 0,
            'original-limited-child-observation-mismatch')
    desktop = observed['desktop']
    interactive = (desktop['station'] == 'WinSta0' and desktop['visible'] is True and
                   desktop['input'] == 'Default' and desktop['thread'] == 'Default' and
                   desktop['nativeError'] is None)
    # The existing noargs helper correctly refuses its elevated-parent route
    # from a limited process. This separate projection reports only what the
    # original limited child actually observed, not usable FULL-token custody.
    return dict(schema=1, state='available' if interactive else 'blocked',
                route='limited-primary-read-only-child',
                reason='limited-primary-interactive-desktop-observed' if interactive else
                       'limited-primary-interactive-desktop-unavailable',
                limitedPrimaryChildObserved=True, interactiveDesktopObserved=interactive,
                elevatedParentLaunchAvailable=False, sameAccountElevationTested=False,
                nativeCustomerAcceptance=False, consentPromptAcceptance=False, observations=observed)


def hosted_context():
    require(sys.platform == 'win32' and len(sys.argv) == 1 and
            os.environ.get('GITHUB_ACTIONS') == 'true' and
            os.environ.get('GITHUB_REPOSITORY') == 'ThereptileII/Work' and
            os.environ.get('GITHUB_EVENT_NAME') == 'workflow_dispatch' and
            os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted' and os.environ.get('RUNNER_OS') == 'Windows',
            'exact-disposable-host-required')
    root = Path(os.environ['GITHUB_WORKSPACE']).resolve()
    require(Path(__file__).resolve() == root / 'opennav-x/tools/probe-disposable-windows-account.py',
            'fixed-source-location-required')
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True, timeout=10).strip()
    require(re.fullmatch('[a-f0-9]{40}', commit) and commit == os.environ.get('GITHUB_SHA'), 'exact-checkout-required')
    helper = root / 'opennav-x/tools/installed_customer_launch.py'
    require(not helper.is_symlink() and helper.is_file(), 'fixed-helper-required')
    return root, helper, dict(checkoutCommit=commit, helperSHA256=hashlib.sha256(helper.read_bytes()).hexdigest())


class AccountProbe:
    def __init__(self, helper):
        self.native = customer._Native()
        self.password = None
        try:
            require(Path(helper).resolve() == Path(__file__).resolve().with_name('installed_customer_launch.py'),
                    'fixed-noargs-helper-required')
            self.helper = helper
            self.fn, self.ok = self.native._fn, self.native._ok
            self.net = C.WinDLL('netapi32.dll', use_last_error=True, winmode=0x800)
            self.env = C.WinDLL('userenv.dll', use_last_error=True, winmode=0x800)
            self.diagnostic = None
            self.name = 'skgprobe' + secrets.token_hex(6)
            self.sid, self.account_created = None, False
            self.launch_effect, self.process, self.thread = 'not-attempted', None, None
            self.exit_observed, self.profile_attempted = False, False
            self.password = C.create_unicode_buffer(49)
            # Fill the mutable native buffer directly; no joined plaintext password.
            alphabet = 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#%+-_'
            for index in range(48):
                self.password[index] = 'aA1!'[index] if index < 4 else secrets.choice(alphabet)
            self.facts = {}
            self.owned = []
        except BaseException:
            if self.password is not None:
                C.memset(self.password, 0, C.sizeof(self.password))
            try:
                self.native.close()
            except BaseException:
                pass
            raise

    def status(self, code, reason):
        if code:
            raise Refused(reason, number(code))

    def own(self, handle):
        require(handle, 'original-native-handle-missing')
        self.owned.append(handle)
        return handle

    def token_statistics(self, token):
        # Explicit Win32 widths also let portable checks verify the SDK layout.
        class Luid(C.Structure):
            _fields_ = [('low', C.c_uint32), ('high', C.c_int32)]
        class Statistics(C.Structure):
            _fields_ = [('tokenId', Luid), ('authenticationId', Luid), ('expiration', C.c_int64),
                ('tokenType', C.c_int32), ('impersonationLevel', C.c_int32),
                ('dynamicCharged', C.c_uint32), ('dynamicAvailable', C.c_uint32),
                ('groupCount', C.c_uint32), ('privilegeCount', C.c_uint32), ('modifiedId', Luid)]
        value = self.native._info(token, 10, Statistics())  # TokenStatistics
        require(value.tokenType in (1, 2) and value.impersonationLevel in (0, 1, 2, 3),
                'original-full-token-statistics-refused')
        return dict(authenticationId=(value.authenticationId.high & 0xffffffff) << 32 |
                    value.authenticationId.low, tokenType=value.tokenType,
                    impersonationLevel=value.impersonationLevel)

    def full_primary(self, original, expected):
        before = token_fact(self.native._token(original))
        require(before == expected and before['sid'] == self.sid and before['elevationType'] == 2 and
                before['elevated'] and before['integrity'] == 'S-1-16-12288', 'original-full-token-changed')
        statistics = self.token_statistics(original)
        # Keep discriminating native facts even when DuplicateTokenEx refuses.
        # A linked FULL may be identification-only; that is not launch authority.
        self.facts['originalFullStatistics'] = statistics
        require(statistics['tokenType'] == before['tokenType'], 'original-full-token-statistics-refused')
        if before['tokenType'] == 1:
            return original
        require(statistics['impersonationLevel'] >= 2, 'original-full-token-not-launchable')
        # Native 38531935235 proved the OS-linked FULL can be impersonation.
        # Convert that exact retained FULL only; never create FULL from LIMITED.
        primary = W.HANDLE()
        try:
            result = self.fn(self.native.adv, 'DuplicateTokenEx', W.BOOL, W.HANDLE, W.DWORD,
                C.c_void_p, C.c_int, C.c_int, C.POINTER(W.HANDLE))(
                original, 0x8 | 0x2 | 0x1, None, statistics['impersonationLevel'], 1, C.byref(primary))
            error = C.get_last_error()
        finally:
            if primary.value:
                self.own(primary)
        if not result:
            raise Refused('original-full-primary-conversion-refused', number(error))
        require(primary.value and primary.value != original.value, 'original-full-primary-handle-refused')
        converted = token_fact(self.native._token(primary))
        converted_statistics = self.token_statistics(primary)
        require(converted == dict(before, tokenType=1) and converted_statistics['tokenType'] == 1 and
                converted_statistics['authenticationId'] == statistics['authenticationId'] and
                token_fact(self.native._token(original)) == before and
                self.token_statistics(original) == statistics, 'original-full-primary-context-changed')
        self.facts['fullTokenConversion'] = dict(state='original-full-impersonation-converted-to-primary',
            originalAuthenticationId=statistics['authenticationId'],
            primaryAuthenticationId=converted_statistics['authenticationId'], primaryToken=converted)
        return primary

    def account_sid(self):
        class User23(C.Structure):
            _fields_ = [('name', W.LPWSTR), ('fullName', W.LPWSTR), ('comment', W.LPWSTR),
                        ('flags', W.DWORD), ('sid', C.c_void_p)]
        pointer = C.c_void_p()
        self.status(self.fn(self.net, 'NetUserGetInfo', W.DWORD, W.LPCWSTR, W.LPCWSTR, W.DWORD,
                            C.POINTER(C.c_void_p))(None, self.name, 23, C.byref(pointer)),
                    'fresh-account-sid-read-refused')
        try:
            require(pointer.value, 'fresh-account-sid-read-refused')
            account = C.cast(pointer, C.POINTER(User23)).contents
            require(account.name == self.name and account.sid and
                    self.fn(self.native.adv, 'IsValidSid', W.BOOL, C.c_void_p)(account.sid),
                    'fresh-account-sid-read-refused')
            length = self.fn(self.native.adv, 'GetLengthSid', W.DWORD, C.c_void_p)(account.sid)
            require(0 < length <= 256, 'fresh-account-sid-read-refused')
            raw = C.create_string_buffer(length)
            C.memmove(raw, account.sid, length)
            return self.native._sid(raw), raw
        finally:
            self.status(self.fn(self.net, 'NetApiBufferFree', W.DWORD, C.c_void_p)(pointer),
                        'account-native-buffer-close-refused')

    def profile(self):
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                r'SOFTWARE\Microsoft\Windows NT\CurrentVersion\ProfileList', 0,
                winreg.KEY_READ | winreg.KEY_WOW64_64KEY) as parent:
            try:
                key = winreg.OpenKey(parent, self.sid, 0, winreg.KEY_READ | winreg.KEY_WOW64_64KEY)
            except FileNotFoundError:
                return None
            with key:
                path, kind = winreg.QueryValueEx(key, 'ProfileImagePath')
                require(kind in (winreg.REG_SZ, winreg.REG_EXPAND_SZ) and type(path) is str,
                        'owned-profile-registration-refused')
                return Path(os.path.expandvars(path))

    def verify_profile(self):
        path = self.profile()
        if path is None:
            return None
        count = W.DWORD(32768)
        raw = C.create_unicode_buffer(count.value)
        self.ok(self.fn(self.env, 'GetProfilesDirectoryW', W.BOOL, W.LPWSTR, C.POINTER(W.DWORD))(raw, C.byref(count)))
        expected = Path(raw.value) / self.name
        require(path == expected and path.is_dir() and not path.is_symlink() and not path.is_junction(),
                'owned-profile-path-refused')
        return path

    def prepare(self):
        class User(C.Structure):
            _fields_ = [('name', W.LPWSTR), ('password', W.LPWSTR), ('age', W.DWORD), ('privilege', W.DWORD),
                        ('home', W.LPWSTR), ('comment', W.LPWSTR), ('flags', W.DWORD), ('script', W.LPWSTR)]
        user = User(self.name, C.cast(self.password, W.LPWSTR), 0, 1, None, None, 0x1 | 0x200, None)
        parameter = W.DWORD()
        self.status(self.fn(self.net, 'NetUserAdd', W.DWORD, W.LPCWSTR, W.DWORD, C.c_void_p,
                            C.POINTER(W.DWORD))(None, 1, C.byref(user), C.byref(parameter)), 'fresh-account-create-refused')
        self.account_created = True
        self.sid, sid_raw = self.account_sid()
        current = self.native.observed['current']
        require(self.sid.rsplit('-', 1)[0] == current['sid'].rsplit('-', 1)[0] and self.sid != current['sid'] and
                not self.sid.endswith('-500') and self.profile() is None, 'fresh-local-account-identity-refused')
        # Resolve BUILTIN Administrators by its fixed SID, never a guessed locale.
        admin_sid = C.c_void_p()
        self.ok(self.fn(self.native.adv, 'ConvertStringSidToSidW', W.BOOL, W.LPCWSTR, C.POINTER(C.c_void_p))(
            'S-1-5-32-544', C.byref(admin_sid)))
        try:
            name, domain = C.create_unicode_buffer(256), C.create_unicode_buffer(256)
            name_size, domain_size, use = W.DWORD(256), W.DWORD(256), W.DWORD()
            self.ok(self.fn(self.native.adv, 'LookupAccountSidW', W.BOOL, W.LPCWSTR, C.c_void_p, W.LPWSTR,
                    C.POINTER(W.DWORD), W.LPWSTR, C.POINTER(W.DWORD), C.POINTER(W.DWORD))(
                    None, admin_sid, name, C.byref(name_size), domain, C.byref(domain_size), C.byref(use)))
            require(use.value == 4, 'builtin-administrators-group-refused')
        finally:
            require(not self.fn(self.native.kernel, 'LocalFree', W.HANDLE, C.c_void_p)(admin_sid),
                    'native-sid-buffer-close-refused')
        member = C.c_void_p(C.addressof(sid_raw))
        self.status(self.fn(self.net, 'NetLocalGroupAddMembers', W.DWORD, W.LPCWSTR, W.LPCWSTR, W.DWORD,
                            C.c_void_p, W.DWORD)(None, name.value, 0, C.byref(member), 1), 'fresh-admin-membership-refused')
        token = W.HANDLE()
        self.ok(self.fn(self.native.adv, 'LogonUserW', W.BOOL, W.LPCWSTR, W.LPCWSTR, W.LPCWSTR,
                       W.DWORD, W.DWORD, C.POINTER(W.HANDLE))(
                       self.name, '.', self.password, 2, 0, C.byref(token)))
        self.own(token)
        C.memset(self.password, 0, C.sizeof(self.password))
        first = token_fact(self.native._token(token))
        self.facts['originalLogonToken'] = first
        linked = W.HANDLE()
        self.native._info(token, 19, linked)
        self.own(linked)
        partner = token_fact(self.native._token(linked))
        self.facts['originalLinkedToken'] = partner
        self.select_limited(token, linked, first, partner)

    def select_limited(self, token, linked, first, partner):
        selected = original_full(first, partner, self.sid)
        full = (token, linked)[selected]
        self.facts['originalFullStatistics'] = self.token_statistics(full)
        self.limited = (token, linked)[1 - selected]
        self.limited_fact = (first, partner)[1 - selected]
        self.limited_statistics = self.token_statistics(self.limited)
        self.facts['originalLimitedStatistics'] = self.limited_statistics
        require(self.limited_statistics['tokenType'] == 1, 'original-limited-primary-required')

    def check_limited(self):
        require(token_fact(self.native._token(self.limited)) == self.limited_fact and
                self.limited_fact['sid'] == self.sid and self.limited_fact['tokenType'] == 1 and
                self.limited_fact['elevationType'] == 3 and not self.limited_fact['elevated'] and
                self.limited_fact['integrity'] == 'S-1-16-8192' and
                self.token_statistics(self.limited) == self.limited_statistics,
                'original-limited-primary-changed')

    def observe(self):
        self.check_limited()
        class Startup(C.Structure):
            _fields_ = [('cb', W.DWORD), ('reserved', W.LPWSTR), ('desktop', W.LPWSTR), ('title', W.LPWSTR),
                ('x', W.DWORD), ('y', W.DWORD), ('width', W.DWORD), ('height', W.DWORD),
                ('charsX', W.DWORD), ('charsY', W.DWORD), ('fill', W.DWORD), ('flags', W.DWORD),
                ('show', W.WORD), ('reservedBytes', W.WORD), ('reservedPointer', C.c_void_p),
                ('stdin', W.HANDLE), ('stdout', W.HANDLE), ('stderr', W.HANDLE)]
        class Process(C.Structure):
            _fields_ = [('process', W.HANDLE), ('thread', W.HANDLE), ('pid', W.DWORD), ('tid', W.DWORD)]
        startup, process = Startup(), Process()
        startup.cb, startup.desktop = C.sizeof(startup), 'WinSta0\\Default'
        # No STARTF_USESTDHANDLES or inherited NUL/pipes. Do not infer destination
        # from lpEnvironment=NULL: only the original token chooses its profile.
        command = [sys.executable, '-I', '-B', str(self.helper), customer.PROFILE_CAPABILITY_MODE]
        line = C.create_unicode_buffer(subprocess.list2cmdline(command))
        require(len(line) < 1024, 'fixed-helper-command-bound-refused')
        self.check_limited()
        self.profile_attempted, self.launch_effect = True, 'uncertain'
        try:
            result = self.fn(self.native.adv, 'CreateProcessWithTokenW', W.BOOL, W.HANDLE, W.DWORD, W.LPCWSTR,
                W.LPWSTR, W.DWORD, C.c_void_p, W.LPCWSTR, C.POINTER(Startup), C.POINTER(Process))(
                self.limited, 1, sys.executable, line, 4, None, str(self.helper.parent), C.byref(startup), C.byref(process))
            error = C.get_last_error()
            self.launch_effect = 'created' if result else 'refused-no-child'
        finally:
            if process.process:
                self.process = self.own(process.process)
            if process.thread:
                self.thread = self.own(process.thread)
        if not result:
            raise Refused('original-limited-token-fixed-probe-launch-refused', number(error))
        require(self.process and self.thread and process.pid > 0 and process.tid > 0,
                'original-probe-child-handles-refused')
        child_token = self.native._process_token(self.process)
        child = token_fact(self.native._token(child_token))
        child_statistics = self.token_statistics(child_token)
        self.facts['originalChild'] = dict(pid=process.pid, threadId=process.tid,
            created=self.native._created(self.process), token=child, statistics=child_statistics, resumeAttempted=False)
        def check_created_child():
            self.check_limited()
            require(child['sid'] == self.sid and not child['elevated'] and child['elevationType'] == 3 and
                    child['tokenType'] == 1 and child['integrity'] == 'S-1-16-8192' and
                    child_statistics['authenticationId'] == self.limited_statistics['authenticationId'] and
                    child['session'] == self.native.observed['current']['session'] and
                    token_fact(self.native._token(child_token)) == child and
                    self.token_statistics(child_token) == child_statistics and
                    self.native._created(self.process) == self.facts['originalChild']['created'] and
                    self.fn(self.native.kernel, 'GetProcessId', W.DWORD, W.HANDLE)(self.process) == process.pid and
                    self.fn(self.native.kernel, 'GetProcessIdOfThread', W.DWORD, W.HANDLE)(self.thread) == process.pid and
                    self.fn(self.native.kernel, 'GetThreadId', W.DWORD, W.HANDLE)(self.thread) == process.tid and
                    os.path.normcase(self.native._image(self.process)) == os.path.normcase(sys.executable),
                    'original-probe-child-identity-refused')
        check_created_child()
        profile = self.verify_profile()
        require(profile is not None, 'original-profile-load-unobserved')
        self.diagnostic = customer._ProfileCapabilityFile(self.limited, expected_profile=profile, expected_sid=self.sid)
        check_created_child()
        self.facts['originalChild']['resumeAttempted'] = True
        previous = self.fn(self.native.kernel, 'ResumeThread', W.DWORD, W.HANDLE)(self.thread)
        self.facts['originalChild']['resumeResult'] = previous
        require(previous == 1, 'original-probe-resume-effect-uncertain')
        raw, deadline = b'', time.monotonic() + 60
        self.facts['capabilityProgress'] = file_capability_progress(raw)
        try:
            while True:
                raw = self.diagnostic.read()
                state = self.fn(self.native.kernel, 'WaitForSingleObject', W.DWORD, W.HANDLE, W.DWORD)(self.process, 25)
                require(state in (0, 258), 'original-probe-wait-refused')
                if state == 0:
                    self.exit_observed = True
                    # Final file size/prefix must be stable on the retained
                    # original file, after proven original-process exit only.
                    raw = self.diagnostic.read(final=True)
                    code = W.DWORD()
                    self.ok(self.fn(self.native.kernel, 'GetExitCodeProcess', W.BOOL, W.HANDLE, C.POINTER(W.DWORD))(
                        self.process, C.byref(code)))
                    self.facts['originalChild']['exitCode'] = code.value
                    require(code.value == 0, 'original-capability-child-exit-refused')
                    break
                require(time.monotonic() < deadline, 'original-capability-child-timeout')
        finally:
            self.facts['capabilityProgress'] = file_capability_progress(raw)
        require(self.facts['capabilityProgress']['markerSequenceComplete'], 'capability-progress-contract-refused')
        require(self.verify_profile() is not None, 'original-profile-load-unobserved')
        self.check_limited()
        return project_limited_capability(file_capability_parts(raw)[0], child)

    def cleanup(self):
        # No delete/kill if creation was interrupted or the original process may
        # still run. The entire job is disposable; never reinterpret PID reuse.
        safe = self.launch_effect in ('not-attempted', 'refused-no-child') or self.exit_observed
        closed = True
        if getattr(self, 'diagnostic', None) is not None:
            try:
                self.diagnostic.close()
            except BaseException:
                closed = False
        C.memset(self.password, 0, C.sizeof(self.password))
        try:
            self.native.close()
        except Exception:
            closed = False
        for handle in reversed(self.owned):
            if not self.native.close_handle(handle):
                closed = False
        self.owned = []
        if not safe or not closed:
            return dict(state='deferred-to-disposable-job', reason='original-process-or-handle-outcome-uncertain')
        if not self.account_created:
            return dict(state='nothing-created')
        try:
            require(self.sid is not None and self.account_sid()[0] == self.sid, 'cleanup-original-account-mismatch')
            profile = self.verify_profile()
            if profile is not None:
                require(self.profile_attempted, 'cleanup-unowned-profile-refused')
                self.ok(self.fn(self.env, 'DeleteProfileW', W.BOOL, W.LPCWSTR, W.LPCWSTR, W.LPCWSTR)(self.sid, None, None))
                require(self.profile() is None and not profile.exists(), 'cleanup-profile-removal-unproved')
            require(self.account_sid()[0] == self.sid, 'cleanup-original-account-mismatch')
            self.status(self.fn(self.net, 'NetUserDel', W.DWORD, W.LPCWSTR, W.LPCWSTR)(None, self.name),
                        'cleanup-original-account-delete-refused')
            pointer = C.c_void_p()
            status = self.fn(self.net, 'NetUserGetInfo', W.DWORD, W.LPCWSTR, W.LPCWSTR, W.DWORD,
                             C.POINTER(C.c_void_p))(None, self.name, 0, C.byref(pointer))
            if pointer.value:
                self.status(self.fn(self.net, 'NetApiBufferFree', W.DWORD, C.c_void_p)(pointer),
                            'account-native-buffer-close-refused')
            require(status == 2221, 'cleanup-account-absence-unproved')  # NERR_UserNotFound
            return dict(state='own-account-and-profile-removed')
        except Exception:
            return dict(state='deferred-to-disposable-job', reason='verified-own-account-profile-cleanup-refused')


def run_probe(helper, factory=AccountProbe):
    report = dict(schema=1, state='blocked', reason='probe-not-completed', nativeCustomerAcceptance=False,
                  consentPromptAcceptance=False, productInstallAttempted=False)
    owner = None
    try:
        owner = factory(helper)
        report['stage'] = 'fresh-account-interactive-logon'
        owner.prepare()
        report['stage'] = 'original-limited-primary-fixed-capability-child'
        capability = owner.observe()
        report.update(state=capability['state'], reason=capability['reason'], capability=capability)
    except Refused as error:
        report.update(state='blocked', reason=error.reason, nativeError=number(error.winerror))
    except (OSError, customer.CustomerLaunchRefused) as error:
        report.update(state='blocked', reason='native-account-probe-refused', nativeError=number(getattr(error, 'winerror', None)))
    except Exception:
        report.update(state='blocked', reason='structured-account-probe-refused')
    except (KeyboardInterrupt, SystemExit):
        report.update(state='blocked', reason='original-account-probe-interrupted')
    finally:
        if owner is not None:
            report['originalFacts'] = owner.facts
            try:
                report['cleanup'] = owner.cleanup()
            except BaseException:
                report['cleanup'] = dict(state='deferred-to-disposable-job', reason='native-cleanup-outcome-uncertain')
            if (report['state'] == 'available' and
                    report['cleanup']['state'] not in ('nothing-created', 'own-account-and-profile-removed')):
                report.update(state='blocked', reason='own-account-profile-cleanup-incomplete')
    return report


def main():
    require(len(sys.argv) == 1, 'no-caller-arguments-permitted')
    # No mutations, including account creation, before exact hosted/source gate.
    _, helper, source = hosted_context()
    directory = Path(os.environ['RUNNER_TEMP']) / 'disposable-account-probe'
    directory.mkdir()
    result = run_probe(helper)
    result['source'] = source
    result['run'] = {key: int(os.environ[value]) for key, value in
                     (('id', 'GITHUB_RUN_ID'), ('attempt', 'GITHUB_RUN_ATTEMPT'))
                     if re.fullmatch('[0-9]{1,20}', os.environ.get(value, ''))}
    with (directory / 'result.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, sort_keys=True, indent=2)
        stream.write('\n')
    print('Disposable limited-primary read-only desktop capability: ' + result['state'] +
          '; elevation and customer/UAC acceptance remain unqualified.')
    return 0 if result['state'] == 'available' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception:
        # Never print native traceback, credentials, paths or raw child output.
        print('Disposable account probe refused before structured completion.')
        raise SystemExit(1)

#!/usr/bin/env python3
"""Portable contracts for actual-token fixture launch; no Windows/API execution."""
import copy
import ctypes
import subprocess
import unittest
from unittest import mock

import installed_customer_launch as customer

SID = 'S-1-5-21-123-456-789-1001'
LOCAL = r'C:\Users\Fixture\AppData\Local'
COMMAND = [LOCAL + '\\OpenNavXAlpha1\\generations\\' + 'a' * 32 + r'\Maintain.exe', '/ACTION=Uninstall']
PROGRAM = r'C:\Program Files (x86)'
GENERATION = 'b' * 32
STARTUP = [PROGRAM + '\\SKAGER\\Applications\\' + SID + '\\generations\\' + GENERATION + r'\app\skager-start.exe',
           '--broker-start=' + GENERATION]


def observations(consent=0):
    current = dict(sid=SID, session=1, tokenType=1, integrity='S-1-16-12288', elevationType=2, elevated=True)
    linked = dict(current, integrity='S-1-16-8192', elevationType=3, elevated=False)
    return dict(current=current, linked=linked, policy=dict(enableLUA=1, consentPromptBehaviorAdmin=consent, promptOnSecureDesktop=1),
                desktop=dict(station='WinSta0', visible=True, input='Default', thread='Default'))


class FakeNative:
    def __init__(self, observed=None):
        self.observed = observed or observations()
        self.local, self.closed, self.creates, self.checks, self.close_calls = LOCAL, False, 0, 0, 0
        self.exit, self.create_error, self.check_error, self.close_error = 0, None, None, None

    def create(self, command):
        self.creates += 1
        if self.create_error:
            raise self.create_error
        return dict(pid=42, created=123450, token=copy.deepcopy(self.observed['linked']), image=command[0])

    def check(self, child):
        self.checks += 1
        if self.check_error:
            raise self.check_error
        if self.closed:
            raise AssertionError('closed native owner used')

    def poll(self, child):
        return self.exit

    def wait_tick(self, child):
        pass

    def close(self):
        self.close_calls += 1
        self.closed = True
        if self.close_error:
            raise self.close_error


class SequenceNative(customer._Native):
    """Exercise the real create/check/close sequence with inert native calls."""
    def __init__(self, fault=None, resume_result=1):
        self.observed, self.local = observations(), LOCAL
        self.current, self.linked = ctypes.c_void_p(16), ctypes.c_void_p(32)
        self.adv, self.kernel = object(), object()
        self.closed, self.close_error, self.resumes = False, None, 0
        self.events, self.closed_handles = [], []
        self.fault, self.resume_result = fault, resume_result
        self.close_handle = lambda handle: self.closed_handles.append(self.value(handle)) or True
        self.owned = [(self.current, self.close_handle), (self.linked, self.close_handle)]

    @staticmethod
    def value(handle):
        return handle.value if hasattr(handle, 'value') else handle

    @staticmethod
    def _ok(value):
        if not value:
            error = OSError('inert native refusal')
            error.winerror = 5
            raise error
        return value

    def _fn(self, dll, name, result, *types):
        def call(*args):
            self.events.append(name)
            if name == 'CreateProcessWithTokenW':
                assert args[0] is self.linked, 'Original linked token was substituted'
                assert args[4] == 0x4, 'NSIS wrapper must start suspended'
                if type(self) is SequenceNative:
                    assert args[-2]._obj.desktop is None, 'Maintain desktop behavior changed'
                process = args[-1]._obj
                process.process, process.thread = 0x111, 0x112
                process.pid, process.tid = 42, 43
                if self.fault == 'interrupt-after-create':
                    raise KeyboardInterrupt('inert interruption after original returned handles')
                return self.fault != 'create'
            if name == 'GetProcessIdOfThread':
                assert self.value(args[0]) == 0x112
                return 999 if self.fault == 'thread' else 42
            if name == 'GetThreadId':
                assert self.value(args[0]) == 0x112
                return 999 if self.fault == 'thread-id' else 43
            if name == 'GetProcessId':
                assert self.value(args[0]) == 0x111
                return 42
            if name == 'ResumeThread':
                assert self.value(args[0]) == 0x112
                assert 'capture-token' in self.events and 'verify-image' in self.events
                self.resumes += 1
                assert self.resumes == 1, 'Never resume twice'
                return self.resume_result
            if name == 'GetExitCodeProcess':
                args[1]._obj.value = 0
                return True
            raise AssertionError('Forbidden or unexpected native API: ' + name)
        return call

    def _created(self, handle):
        self.events.append('capture-creation')
        assert self.value(handle) == 0x111
        return 123450

    def _process_token(self, handle):
        self.events.append('capture-token')
        assert self.value(handle) == 0x111 and self.resumes == 0
        if self.fault == 'token':
            self._ok(False)
        token = ctypes.c_void_p(48)
        self.owned.append((token, self.close_handle))
        return token

    def _token(self, token):
        which = 'current' if self.value(token) == 16 else 'linked'
        return copy.deepcopy(self.observed[which])

    def _state(self, child):
        # Simulate the real NSIS bootstrap exiting immediately on its first
        # resume. Subsequent checks must use its already retained token handle.
        return 0 if self.resumes else 258

    def _image(self, handle):
        self.events.append('verify-image')
        return COMMAND[0] + ('.foreign' if self.fault == 'image' else '')


class SuspendedCustomerLaunchTests(unittest.TestCase):
    def test_capture_precedes_exact_single_resume_even_when_nsis_exits_immediately(self):
        native = SequenceNative()
        owner = customer.open_same_account_launch(COMMAND, SID, _native_factory=lambda: native)
        self.assertEqual(native.resumes, 1)
        self.assertLess(native.events.index('capture-creation'), native.events.index('ResumeThread'))
        self.assertLess(native.events.index('capture-token'), native.events.index('ResumeThread'))
        self.assertLess(native.events.index('verify-image'), native.events.index('ResumeThread'))
        self.assertEqual(native.events.count('capture-token'), 1)
        self.assertEqual(owner.wait(0), 0)
        self.assertEqual(owner.launch_evidence, dict(pid=42, threadId=43, created=123450,
            resumeAttempted=True, resumeResult=1, state='original-child-resumed'))
        owner.close()
        self.assertEqual(native.closed_handles, [48, 0x112, 0x111, 32, 16])

    def test_capture_refusal_never_resumes_or_loses_returned_handles(self):
        for fault in ('create', 'interrupt-after-create', 'thread', 'thread-id', 'token', 'image'):
            with self.subTest(fault=fault):
                native = SequenceNative(fault=fault)
                with self.assertRaises(customer.CustomerLaunchRefused) as caught:
                    customer.open_same_account_launch(COMMAND, SID, _native_factory=lambda: native)
                evidence = caught.exception.launchEvidence
                self.assertEqual((evidence['pid'], evidence['threadId']), (42, 43))
                self.assertFalse(evidence['resumeAttempted'])
                self.assertIsNone(evidence['resumeResult'])
                self.assertEqual(native.resumes, 0)
                self.assertTrue(native.closed)
                self.assertEqual(native.closed_handles.count(0x111), 1)
                self.assertEqual(native.closed_handles.count(0x112), 1)

    def test_resume_failure_or_changed_count_preserves_uncertainty_without_retry_or_termination(self):
        for previous in (0xFFFFFFFF, 0, 2):
            with self.subTest(previous=previous):
                native = SequenceNative(resume_result=previous)
                with self.assertRaises(customer.CustomerLaunchRefused) as caught:
                    customer.open_same_account_launch(COMMAND, SID, _native_factory=lambda: native)
                self.assertEqual(caught.exception.launchEvidence, dict(pid=42, threadId=43, created=123450,
                    resumeAttempted=True, resumeResult=previous, state='resume-effect-uncertain'))
                self.assertEqual(native.resumes, 1)
                self.assertTrue(native.closed)
                self.assertEqual(native.closed_handles, [48, 0x112, 0x111, 32, 16])
                if previous == 0xFFFFFFFF:
                    self.assertEqual(caught.exception.winerror, 5)


class ProtectedSequenceNative(SequenceNative, customer._ProtectedStartupNative):
    """Run the actual protected checks/create sequence without native execution."""
    def __init__(self, fault=None, resume_result=1):
        SequenceNative.__init__(self, fault, resume_result)
        self.program = PROGRAM
        self.authentication_id = 0x123400005678
        self.linked_logon_changed = False
        self.observed_token = None
        self.observed_authentication = self.authentication_id

    def _fn(self, dll, name, result, *types):
        call = super()._fn(dll, name, result, *types)
        if name != 'CreateProcessWithTokenW':
            return call
        def create(*args):
            assert args[2] == STARTUP[0]
            assert args[3].value == subprocess.list2cmdline(STARTUP)
            assert args[-2]._obj.desktop == 'WinSta0\\Default'
            return call(*args)
        return create

    def _image(self, handle):
        self.events.append('verify-image')
        return STARTUP[0] + ('.foreign' if self.fault == 'image' else '')

    def _authentication_id(self, token):
        value = self.value(token)
        self.events.append(('limited-authentication', value))
        if value == 16:
            return 0x999  # The FULL coordinator's logon is deliberately different.
        if value == 32 and self.linked_logon_changed:
            return self.authentication_id + 1
        if value == 48 and self.fault == 'launcher-logon':
            return self.authentication_id + 1
        return self.observed_authentication if value == 64 else self.authentication_id

    def _process_token(self, handle):
        if self.value(handle) == 0x221:
            self.events.append('capture-observed-token')
            token = ctypes.c_void_p(64)
            self.owned.append((token, self.close_handle))
            return token
        return super()._process_token(handle)

    def _token(self, token):
        if self.value(token) == 64 and self.observed_token is not None:
            return copy.deepcopy(self.observed_token)
        return super()._token(token)


class ProtectedStartupLaunchTests(unittest.TestCase):
    def launch(self, native):
        return customer.open_same_account_protected_startup(GENERATION, SID, _native_factory=lambda: native)

    def test_fixed_original_limited_launcher_corroborated_before_single_resume(self):
        native = ProtectedSequenceNative()
        owner = self.launch(native)
        self.assertEqual(owner.command, STARTUP)
        self.assertEqual((owner.sid, owner.session, owner.elevated), (SID, 1, False))
        self.assertLess(native.events.index(('limited-authentication', 48)), native.events.index('ResumeThread'))
        self.assertNotIn(('limited-authentication', 16), native.events)
        self.assertEqual(owner.poll(), 0)
        self.assertEqual(owner.wait(0), 0)
        self.assertFalse(owner.capability['nativeCustomerAcceptance'])
        self.assertFalse(owner.capability['consentPromptAcceptance'])
        owner.close()
        self.assertEqual(native.closed_handles, [48, 0x112, 0x111, 32, 16])

    def test_generation_cannot_supply_an_executable_or_another_argument(self):
        for generation in (None, b'b' * 32, '', 'B' * 32, 'b' * 31, 'b' * 33,
                           GENERATION + ' --maintenance=uninstall-current', '..\\' + GENERATION,
                           GENERATION + '\n', GENERATION + '\x00'):
            factory = mock.Mock()
            with self.subTest(generation=generation), self.assertRaises(customer.CustomerLaunchRefused):
                customer.open_same_account_protected_startup(generation, SID, _native_factory=factory)
            factory.assert_not_called()

    def test_actual_sid_is_only_a_veto_and_maintain_route_stays_exclusive(self):
        native = ProtectedSequenceNative()
        with self.assertRaises(customer.CustomerLaunchRefused):
            customer.open_same_account_protected_startup(GENERATION, SID + '1', _native_factory=lambda: native)
        self.assertNotIn('CreateProcessWithTokenW', native.events)
        self.assertTrue(native.closed)
        native = FakeNative()
        with self.assertRaises(customer.CustomerLaunchRefused):
            customer.open_same_account_launch(STARTUP, SID, _native_factory=lambda: native)
        self.assertEqual(native.creates, 0)

    def test_program_folder_cannot_redirect_the_fixed_route(self):
        for program in (None, '', r'\\server\share', r'\Program Files', r'C:/Program Files',
                        r'C:\Program Files\..\mutable', 'C:\\bad\nfolder', 'C:\\bad"folder'):
            native = ProtectedSequenceNative()
            native.program = program
            with self.subTest(program=program), self.assertRaises(customer.CustomerLaunchRefused):
                self.launch(native)
            self.assertNotIn('CreateProcessWithTokenW', native.events)
            self.assertTrue(native.closed)

    def test_missing_limited_token_or_interactive_desktop_never_launches(self):
        for fault in ('missing', 'elevated', 'session', 'primary', 'desktop', 'secure'):
            native = ProtectedSequenceNative()
            if fault == 'missing': native.observed['linked'] = None
            elif fault == 'elevated': native.observed['linked']['elevated'] = True
            elif fault == 'session': native.observed['linked']['session'] = 0
            elif fault == 'primary': native.observed['linked']['tokenType'] = 2
            elif fault == 'desktop': native.observed['desktop']['visible'] = False
            elif fault == 'secure': native.observed['desktop']['input'] = 'Winlogon'
            with self.subTest(fault=fault), self.assertRaises(customer.CustomerLaunchRefused):
                self.launch(native)
            self.assertNotIn('CreateProcessWithTokenW', native.events)
            self.assertTrue(native.closed)

    def test_changed_launcher_logon_refuses_before_resume_with_original_handles(self):
        native = ProtectedSequenceNative(fault='launcher-logon')
        with self.assertRaises(customer.CustomerLaunchRefused) as caught:
            self.launch(native)
        self.assertFalse(caught.exception.launchEvidence['resumeAttempted'])
        self.assertEqual(native.resumes, 0)
        self.assertEqual(native.closed_handles, [48, 0x112, 0x111, 32, 16])

    def test_capture_and_resume_uncertainty_have_no_fallback_retry_or_kill(self):
        for fault in ('create', 'interrupt-after-create', 'thread', 'thread-id', 'token', 'image'):
            native = ProtectedSequenceNative(fault=fault)
            with self.subTest(fault=fault), self.assertRaises(customer.CustomerLaunchRefused):
                self.launch(native)
            self.assertEqual(native.resumes, 0)
            self.assertEqual(native.events.count('CreateProcessWithTokenW'), 1)
            self.assertTrue(native.closed)
        for count in (0xFFFFFFFF, 0, 2):
            native = ProtectedSequenceNative(resume_result=count)
            with self.subTest(count=count), self.assertRaises(customer.CustomerLaunchRefused):
                self.launch(native)
            self.assertEqual(native.resumes, 1)
            self.assertTrue(native.closed)

    def test_original_observed_handle_token_bound_to_limited_logon(self):
        native = ProtectedSequenceNative()
        owner = self.launch(native)
        facts = owner.check_process_token(ctypes.c_void_p(0x221))
        self.assertEqual(facts, dict(native.observed['linked'], authenticationId=native.authentication_id))
        self.assertEqual(native.events.count('capture-observed-token'), 1)
        self.assertFalse(native.closed)
        owner.close()
        self.assertEqual(native.closed_handles, [64, 48, 0x112, 0x111, 32, 16])
        with self.assertRaises(customer.CustomerLaunchRefused):
            owner.check_process_token(ctypes.c_void_p(0x221))

    def test_foreign_or_elevated_observed_token_and_logon_refuse(self):
        for fault in ('sid', 'session', 'elevated', 'type', 'integrity', 'logon'):
            native = ProtectedSequenceNative()
            owner = self.launch(native)
            native.observed_token = copy.deepcopy(native.observed['linked'])
            if fault == 'sid': native.observed_token['sid'] = SID + '1'
            elif fault == 'session': native.observed_token['session'] = 2
            elif fault == 'elevated': native.observed_token['elevated'] = True
            elif fault == 'type': native.observed_token['tokenType'] = 2
            elif fault == 'integrity': native.observed_token['integrity'] = 'S-1-16-12288'
            elif fault == 'logon': native.observed_authentication += 1
            with self.subTest(fault=fault), self.assertRaises(customer.CustomerLaunchRefused):
                owner.check_process_token(ctypes.c_void_p(0x221))
            self.assertEqual(native.resumes, 1)
            owner.close()

    def test_linked_logon_change_refuses_original_owner_without_new_launch(self):
        native = ProtectedSequenceNative()
        owner = self.launch(native)
        native.linked_logon_changed = True
        with self.assertRaises(customer.CustomerLaunchRefused): owner.poll()
        self.assertEqual(native.events.count('CreateProcessWithTokenW'), 1)
        owner.close()

    def test_timeout_keeps_original_startup_owner_without_retry_or_termination(self):
        native = ProtectedSequenceNative()
        owner = self.launch(native)
        native._state = lambda child: 258
        with self.assertRaises(subprocess.TimeoutExpired): owner.wait(0)
        self.assertEqual(native.events.count('CreateProcessWithTokenW'), 1)
        self.assertEqual(native.resumes, 1)
        self.assertFalse(native.closed)
        owner.close()

    def test_primary_statistics_use_fixed_win32_layout_and_nonzero_original_logon(self):
        native = object.__new__(customer._ProtectedStartupNative)
        def info(token, kind, value):
            self.assertEqual(kind, 10)
            self.assertEqual(ctypes.sizeof(value), 56)
            self.assertEqual(type(value).authenticationId.offset, 8)
            self.assertEqual(type(value).modifiedId.offset, 48)
            value.tokenType = 1
            value.authenticationId.high, value.authenticationId.low = -1, 0x1234
            return value
        native._info = info
        self.assertEqual(native._authentication_id(32), 0xffffffff00001234)
        for fault in ('zero', 'impersonation'):
            def wrong(token, kind, value):
                value.tokenType = 2 if fault == 'impersonation' else 1
                value.authenticationId.low = 1 if fault == 'impersonation' else 0
                return value
            native._info = wrong
            with self.subTest(fault=fault), self.assertRaises(customer.CustomerLaunchRefused):
                native._authentication_id(32)


class CustomerLaunchTests(unittest.TestCase):
    def test_silent_hosted_policy_does_not_block_real_normal_token_or_claim_consent(self):
        native = FakeNative()
        capability = customer.inspect_customer_capability(_native_factory=lambda: native)
        self.assertEqual(capability['state'], 'available')
        self.assertTrue(capability['normalTokenAvailable'])
        self.assertFalse(capability['consentPromptAvailable'])
        self.assertFalse(capability['consentPromptAcceptance'])
        self.assertFalse(capability['nativeCustomerAcceptance'])
        self.assertEqual(capability['route'], 'same-account-normal-token-silent-elevation')
        self.assertTrue(native.closed)
        self.assertEqual(native.creates, 0)

    def test_real_consent_policy_is_only_available_not_accepted(self):
        native = FakeNative(observations(2))
        capability = customer.inspect_customer_capability(_native_factory=lambda: native)
        self.assertTrue(capability['consentPromptAvailable'])
        self.assertFalse(capability['consentPromptAcceptance'])
        self.assertEqual(capability['route'], 'same-account-normal-token-manual-consent')

    def test_missing_or_different_actual_normal_token_and_desktop_refuse_before_launch(self):
        for fault in ('missing linked', 'wrong SID', 'wrong session', 'still elevated', 'not limited', 'not medium', 'not primary', 'session zero', 'headless', 'secure desktop'):
            with self.subTest(fault=fault):
                observed = observations()
                if fault == 'missing linked': observed['linked'] = None
                elif fault == 'wrong SID': observed['linked']['sid'] = 'S-1-5-21-9-9-9-1001'
                elif fault == 'wrong session': observed['linked']['session'] = 2
                elif fault == 'still elevated': observed['linked']['elevated'] = True
                elif fault == 'not limited': observed['linked']['elevationType'] = 1
                elif fault == 'not medium': observed['linked']['integrity'] = 'S-1-16-12288'
                elif fault == 'not primary': observed['linked']['tokenType'] = 2
                elif fault == 'session zero': observed['current']['session'] = observed['linked']['session'] = 0
                elif fault == 'headless': observed['desktop']['visible'] = False
                elif fault == 'secure desktop': observed['desktop']['input'] = 'Winlogon'
                native = FakeNative(observed)
                with self.assertRaises(customer.CustomerLaunchRefused):
                    customer.open_same_account_launch(COMMAND, SID, _native_factory=lambda: native)
                self.assertEqual(native.creates, 0)
                self.assertTrue(native.closed)

    def test_approved_sid_is_only_comparison_veto(self):
        native = FakeNative()
        with self.assertRaises(customer.CustomerLaunchRefused):
            customer.open_same_account_launch(COMMAND, 'S-1-5-21-9-9-9-1001', _native_factory=lambda: native)
        self.assertEqual(native.creates, 0)
        self.assertTrue(native.closed)

    def test_only_fixed_current_account_maintain_uninstall_command_is_accepted(self):
        commands = [[], COMMAND + ['/S'], [COMMAND[0], '/ACTION=Repair'], [r'C:\mutable.exe', '/ACTION=Uninstall'],
                    [COMMAND[0].replace('Fixture', 'OtherUser'), COMMAND[1]],
                    [COMMAND[0].replace('a' * 32, 'A' * 32), COMMAND[1]],
                    [COMMAND[0].replace('Maintain.exe', 'skager-update-provisioner.exe'), COMMAND[1]],
                    [COMMAND[0].replace('generations\\', 'generations\\..\\'), COMMAND[1]]]
        for command in commands:
            native = FakeNative()
            with self.subTest(command=command), self.assertRaises(customer.CustomerLaunchRefused):
                customer.open_same_account_launch(command, SID, _native_factory=lambda: native)
            self.assertEqual(native.creates, 0)
            self.assertTrue(native.closed)

    def test_owner_waits_original_process_and_closes_once(self):
        native = FakeNative()
        owner = customer.open_same_account_launch(COMMAND, SID, _native_factory=lambda: native)
        self.assertEqual((owner.pid, owner.created, owner.sid, owner.session, owner.elevated), (42, 123450, SID, 1, False))
        self.assertEqual(owner.poll(), 0)
        self.assertEqual(owner.wait(1), 0)
        self.assertFalse(owner.capability['consentPromptAcceptance'])
        self.assertFalse(native.closed)
        owner.close(); owner.close()
        self.assertEqual(native.close_calls, 1)
        with self.assertRaises(customer.CustomerLaunchRefused): owner.poll()

    def test_timeout_preserves_process_no_kill_retry_or_automatic_close(self):
        native = FakeNative()
        native.exit = None
        owner = customer.open_same_account_launch(COMMAND, SID, _native_factory=lambda: native)
        with self.assertRaises(subprocess.TimeoutExpired): owner.wait(0)
        self.assertEqual(native.creates, 1)
        self.assertFalse(native.closed)
        owner.close()
        self.assertEqual(native.close_calls, 1)

    def test_launch_native_refusal_has_no_other_process_creation_fallback(self):
        native = FakeNative()
        native.create_error = OSError('actual CreateProcessWithToken refused')
        with self.assertRaises(OSError):
            customer.open_same_account_launch(COMMAND, SID, _native_factory=lambda: native)
        self.assertEqual(native.creates, 1)
        self.assertTrue(native.closed)

    def test_changed_original_process_refuses_and_owner_close_uncertainty_persists(self):
        native = FakeNative()
        owner = customer.open_same_account_launch(COMMAND, SID, _native_factory=lambda: native)
        native.check_error = customer.CustomerLaunchRefused('original creation/token/image changed')
        with self.assertRaises(customer.CustomerLaunchRefused): owner.poll()
        native.close_error = customer.CustomerLaunchRefused('native owner close uncertain')
        with self.assertRaises(customer.CustomerLaunchRefused): owner.close()
        with self.assertRaises(customer.CustomerLaunchRefused): owner.close()
        self.assertEqual(native.close_calls, 1)

    def test_unavailable_native_observation_and_cleanup_never_report_availability(self):
        with mock.patch.object(customer.sys, 'platform', 'linux'):
            result = customer.inspect_customer_capability()
        self.assertEqual(result['state'], 'blocked')
        self.assertFalse(result['nativeCustomerAcceptance'])
        native = FakeNative()
        native.close_error = customer.CustomerLaunchRefused('close failed')
        result = customer.inspect_customer_capability(_native_factory=lambda: native)
        self.assertEqual(result['state'], 'blocked')
        self.assertFalse(result['normalTokenAvailable'])


if __name__ == '__main__':
    unittest.main()

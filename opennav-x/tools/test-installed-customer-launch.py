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

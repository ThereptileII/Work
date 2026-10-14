#!/usr/bin/env python3
"""Inert account-probe contracts; never creates an account or calls Windows."""
import ctypes
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock

import installed_customer_launch as customer

spec = importlib.util.spec_from_file_location('account_probe', Path(__file__).with_name('probe-disposable-windows-account.py'))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)
SID = 'S-1-5-21-123-456-789-1001'
FULL = dict(sid=SID, session=0, tokenType=1, integrity='S-1-16-12288', elevationType=2, elevated=True)
LIMITED = dict(FULL, integrity='S-1-16-8192', elevationType=3, elevated=False)


def capability():
    return customer._capability(dict(current=dict(FULL, session=2), linked=dict(LIMITED, session=2),
        linkedError=None, policy=dict(enableLUA=1, consentPromptBehaviorAdmin=0, promptOnSecureDesktop=1),
        desktop=dict(station='WinSta0', visible=True, input='Default', thread='Default')))


class OriginalTokenSelection(unittest.TestCase):
    def test_os_split_token_order_is_observed_and_no_session_is_manufactured(self):
        self.assertEqual(probe.original_full(FULL, LIMITED, SID), 0)
        self.assertEqual(probe.original_full(LIMITED, FULL, SID), 1)
        self.assertEqual(FULL['session'], 0)

    def test_missing_unsplit_foreign_session_and_foreign_account_are_refused(self):
        for first, linked in [(FULL, None), (dict(FULL, elevationType=1), LIMITED),
                              (FULL, dict(LIMITED, session=2)), (FULL, dict(LIMITED, sid=SID[:-1]+'2')),
                              (dict(FULL, tokenType=2), LIMITED), (FULL, dict(LIMITED, elevated=True))]:
            with self.subTest(first=first['elevationType'], linked=linked is not None):
                with self.assertRaises(probe.Refused):
                    probe.original_full(first, linked, SID)


class SanitizedCapability(unittest.TestCase):
    def test_actual_silent_capability_is_never_customer_or_consent_acceptance(self):
        value = capability()
        value['privatePath'] = 'secret-path'
        value['observations']['environment'] = {'secret': 'secret-value'}
        result = probe.project_capability(json.dumps(value).encode(), SID)
        self.assertEqual(result['state'], 'available')
        self.assertFalse(result['consentPromptAcceptance'])
        self.assertFalse(result['nativeCustomerAcceptance'])
        self.assertNotIn('secret', json.dumps(result))

    def test_false_available_foreign_sid_and_duplicate_fields_refuse(self):
        wrong = capability()
        wrong['observations']['current']['sid'] = SID[:-1]+'2'
        blocked = capability()
        blocked['observations']['desktop']['visible'] = False
        for raw in [json.dumps(wrong).encode(), json.dumps(blocked).encode(),
                    b'{"schema":1,"schema":1}', b'x'*16385]:
            with self.assertRaises(probe.Refused):
                probe.project_capability(raw, SID)

    def test_native_refusal_keeps_only_numeric_error(self):
        value = dict(schema=1, state='blocked', reason='native-capability-observation-refused',
            nativeCustomerAcceptance=False, consentPromptAcceptance=False,
            observations=dict(nativeError=5, privatePath='secret'))
        result = probe.project_capability(json.dumps(value).encode(), SID)
        self.assertEqual(result['observations'], {'nativeError': 5})
        value['observations']['nativeError'] = 'private-error'
        with self.assertRaises(probe.Refused):
            probe.project_capability(json.dumps(value).encode(), SID)


class ProbeOutcomes(unittest.TestCase):
    def fake(self, prepare_error=None, observe_error=None, cleanup_state='own-account-and-profile-removed'):
        self.events = []
        def prepare():
            self.events.append('prepare')
            if prepare_error:
                raise prepare_error
        def observe():
            self.events.append('observe')
            if observe_error:
                raise observe_error
            return capability()
        def cleanup():
            self.events.append('cleanup')
            return dict(state=cleanup_state)
        return SimpleNamespace(prepare=prepare, observe=observe, cleanup=cleanup,
                               facts={'originalLogonToken': FULL})

    def test_success_requires_cleanup_and_only_fixed_helper_is_passed(self):
        owner = self.fake()
        factory = mock.Mock(return_value=owner)
        result = probe.run_probe(Path('fixed-helper'), factory)
        factory.assert_called_once_with(Path('fixed-helper'))
        self.assertEqual(self.events, ['prepare', 'observe', 'cleanup'])
        self.assertEqual(result['state'], 'available')
        self.assertFalse(result['productInstallAttempted'])
        self.assertFalse(result['nativeCustomerAcceptance'])
        self.assertFalse(result['consentPromptAcceptance'])

    def test_prepare_refusal_does_not_launch_and_always_cleans_original_owner(self):
        owner = self.fake(prepare_error=probe.Refused('original-split-token-unavailable', 1312))
        result = probe.run_probe(None, lambda helper: owner)
        self.assertEqual(self.events, ['prepare', 'cleanup'])
        self.assertEqual(result['reason'], 'original-split-token-unavailable')
        self.assertEqual(result['nativeError'], 1312)

    def test_uncertain_live_child_cannot_be_reclassified_as_available(self):
        owner = self.fake(cleanup_state='deferred-to-disposable-job')
        result = probe.run_probe(None, lambda helper: owner)
        self.assertEqual(result['state'], 'blocked')
        self.assertEqual(result['reason'], 'own-account-profile-cleanup-incomplete')

    def test_interruption_or_private_exception_never_leaks_or_skips_cleanup(self):
        for error in (KeyboardInterrupt(), RuntimeError('private password path and environment')):
            owner = self.fake(observe_error=error)
            result = probe.run_probe(None, lambda helper: owner)
            self.assertEqual(self.events, ['prepare', 'observe', 'cleanup'])
            self.assertEqual(result['state'], 'blocked')
            self.assertNotIn('private', json.dumps(result))

    def test_cleanup_exception_is_sanitized_and_blocks_success(self):
        owner = self.fake()
        owner.cleanup = mock.Mock(side_effect=OSError('private cleanup traceback'))
        result = probe.run_probe(None, lambda helper: owner)
        self.assertEqual(result['state'], 'blocked')
        self.assertEqual(result['cleanup']['reason'], 'native-cleanup-outcome-uncertain')
        self.assertNotIn('private', json.dumps(result))

    def test_cleanup_uncertainty_does_not_replace_first_native_failure(self):
        owner = self.fake(observe_error=probe.Refused('original-full-token-fixed-probe-launch-refused', 1314),
                          cleanup_state='deferred-to-disposable-job')
        result = probe.run_probe(None, lambda helper: owner)
        self.assertEqual(result['reason'], 'original-full-token-fixed-probe-launch-refused')
        self.assertEqual(result['nativeError'], 1314)
        self.assertEqual(result['cleanup']['state'], 'deferred-to-disposable-job')


class OriginalCleanup(unittest.TestCase):
    def owner(self, effect='created', exited=True, sid=SID):
        owner = probe.AccountProbe.__new__(probe.AccountProbe)
        owner.password = ctypes.create_unicode_buffer('private-password')
        owner.launch_effect, owner.exit_observed = effect, exited
        owner.account_created, owner.sid, owner.name = True, SID, 'fixed-own-generated-name'
        owner.owned = [101, 102]
        owner.native = SimpleNamespace(close=mock.Mock(), close_handle=mock.Mock(return_value=True))
        owner.account_sid = mock.Mock(return_value=(sid, None))
        owner.verify_profile = mock.Mock(return_value=None)
        owner.delete = mock.Mock(return_value=0)
        owner.readback = mock.Mock(return_value=2221)
        owner.fn = mock.Mock(side_effect=lambda dll, name, *args:
                             owner.delete if name == 'NetUserDel' else owner.readback)
        owner.net, owner.env = None, None
        owner.status = lambda code, reason: probe.require(code == 0, reason)
        return owner

    def test_live_or_uncertain_child_closes_handles_and_wipes_password_without_deletion(self):
        for effect, exited in [('created', False), ('uncertain', False)]:
            owner = self.owner(effect, exited)
            result = owner.cleanup()
            self.assertEqual(result['state'], 'deferred-to-disposable-job')
            self.assertEqual(owner.password.value, '')
            owner.native.close.assert_called_once()
            self.assertEqual(owner.native.close_handle.call_args_list, [mock.call(102), mock.call(101)])
            owner.account_sid.assert_not_called()
            owner.fn.assert_not_called()

    def test_same_name_with_changed_sid_is_never_deleted(self):
        owner = self.owner(sid=SID[:-1]+'2')
        self.assertEqual(owner.cleanup()['state'], 'deferred-to-disposable-job')
        owner.verify_profile.assert_not_called()
        owner.fn.assert_not_called()

    def test_known_exited_child_requires_original_account_identity_twice_before_delete(self):
        owner = self.owner()
        self.assertEqual(owner.cleanup()['state'], 'own-account-and-profile-removed')
        self.assertEqual(owner.account_sid.call_count, 2)
        owner.delete.assert_called_once_with(None, owner.name)
        self.assertEqual(owner.fn.call_args.args[1], 'NetUserGetInfo')

    def test_unproved_deletion_is_never_reported_as_clean(self):
        owner = self.owner()
        owner.readback.return_value = 5
        self.assertEqual(owner.cleanup()['state'], 'deferred-to-disposable-job')
        owner.delete.assert_called_once_with(None, owner.name)

    def test_close_uncertainty_prevents_account_or_profile_removal(self):
        owner = self.owner()
        owner.native.close_handle.side_effect = [False, True]
        self.assertEqual(owner.cleanup()['state'], 'deferred-to-disposable-job')
        self.assertEqual(owner.native.close_handle.call_count, 2)
        owner.account_sid.assert_not_called()

    def test_unknown_or_replaced_profile_is_never_deleted(self):
        owner = self.owner()
        owner.verify_profile.side_effect = probe.Refused('owned-profile-path-refused')
        self.assertEqual(owner.cleanup()['state'], 'deferred-to-disposable-job')
        owner.fn.assert_not_called()

    def test_profile_delete_uses_only_original_sid_after_original_handles_drain(self):
        owner = self.owner()
        owner.profile_attempted = True
        profile = SimpleNamespace(exists=lambda: False)
        owner.verify_profile.return_value = profile
        owner.profile = mock.Mock(return_value=None)
        owner.ok = lambda value: probe.require(value, 'native-failure')
        def delete_profile(sid, path, machine):
            owner.native.close.assert_called_once()
            self.assertEqual(owner.native.close_handle.call_count, 2)
            self.assertEqual((sid, path, machine), (SID, None, None))
            return True
        owner.fn.side_effect = lambda dll, name, *args: (delete_profile if name == 'DeleteProfileW' else
            owner.delete if name == 'NetUserDel' else owner.readback)
        self.assertEqual(owner.cleanup()['state'], 'own-account-and-profile-removed')


class FixedNativeCommand(unittest.TestCase):
    def test_existing_full_token_profile_explicit_desktop_and_noargs_helper_only(self):
        owner = probe.AccountProbe.__new__(probe.AccountProbe)
        owner.native = SimpleNamespace(kernel='kernel', adv='adv')
        owner.full, owner.helper, owner.owned = 987, Path('fixed-tools/installed_customer_launch.py'), []
        owner.ok = lambda value: probe.require(value, 'native-failure')
        calls = []
        def pipe(read, write, attributes, size):
            read._obj.value, write._obj.value = 101, 102
            return True
        def create(token, flags, executable, command, creation, environment, cwd, startup, process):
            calls.append('create')
            self.assertEqual((token, flags, creation, environment), (987, 1, 4, None))
            self.assertEqual(command.value, probe.subprocess.list2cmdline(
                [probe.sys.executable, '-I', '-B', str(owner.helper)]))
            self.assertEqual(executable, probe.sys.executable)
            self.assertEqual(startup._obj.desktop, 'WinSta0\\Default')
            self.assertEqual(startup._obj.flags, 0x100)
            self.assertEqual(startup._obj.stdout, 102)
            return False  # Existing privilege unavailable: never enable or retry.
        def fn(dll, name, *args):
            calls.append(name)
            return {'CreatePipe': pipe, 'SetHandleInformation': lambda *args: True,
                    'CreateFileW': lambda *args: 103, 'CreateProcessWithTokenW': create}[name]
        owner.fn = fn
        with mock.patch.object(probe.C, 'get_last_error', create=True, return_value=1314):
            with self.assertRaises(probe.Refused) as caught:
                owner.observe()
        self.assertEqual(caught.exception.reason, 'original-full-token-fixed-probe-launch-refused')
        self.assertEqual(caught.exception.winerror, 1314)
        self.assertEqual(owner.launch_effect, 'refused-no-child')
        self.assertEqual(owner.owned, [mock.ANY, mock.ANY, 103])
        self.assertEqual(calls.count('create'), 1)


class HostGate(unittest.TestCase):
    def test_non_windows_or_non_hosted_context_refuses_before_any_native_mutation(self):
        with mock.patch.object(probe.sys, 'platform', 'linux'):
            with mock.patch.object(probe.subprocess, 'check_output') as git:
                with self.assertRaises(probe.Refused):
                    probe.hosted_context()
                git.assert_not_called()
        with mock.patch.object(probe.sys, 'platform', 'win32'):
            with mock.patch.dict(probe.os.environ, {'GITHUB_ACTIONS': 'false'}, clear=True):
                with self.assertRaises(probe.Refused):
                    probe.hosted_context()


if __name__ == '__main__':
    unittest.main()

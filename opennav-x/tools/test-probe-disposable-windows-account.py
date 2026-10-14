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

    def test_actual_os_linked_full_impersonation_is_selected_for_conversion(self):
        self.assertEqual(probe.original_full(LIMITED, dict(FULL, tokenType=2), SID), 1)

    def test_missing_unsplit_foreign_session_and_foreign_account_are_refused(self):
        for first, linked in [(FULL, None), (dict(FULL, elevationType=1), LIMITED),
                              (FULL, dict(LIMITED, session=2)), (FULL, dict(LIMITED, sid=SID[:-1]+'2')),
                              (dict(FULL, tokenType=2), LIMITED), (FULL, dict(LIMITED, elevated=True))]:
            with self.subTest(first=first['elevationType'], linked=linked is not None):
                with self.assertRaises(probe.Refused):
                    probe.original_full(first, linked, SID)


class OriginalFullPrimaryConversion(unittest.TestCase):
    def owner(self, full_type=2, changed=None, authentication=0x123456789abcdef0):
        owner = probe.AccountProbe.__new__(probe.AccountProbe)
        owner.sid, owner.facts = SID, {}
        owner.original = probe.W.HANDLE(22)
        owner.owned = [probe.W.HANDLE(11), owner.original]
        owner.before = dict(FULL, tokenType=full_type)
        owner.after = dict(FULL, **(changed or {}))
        owner.native = SimpleNamespace(adv='adv', _token=lambda handle:
            owner.before.copy() if handle.value == 22 else owner.after.copy())
        owner.token_statistics = mock.Mock(side_effect=lambda handle:
            dict(authenticationId=authentication if handle.value == 22 else owner.primary_authentication,
                 tokenType=owner.before['tokenType'] if handle.value == 22 else owner.after['tokenType'],
                 impersonationLevel=2 if handle.value == 22 and full_type == 2 else 0))
        owner.primary_authentication = authentication
        def duplicate(original, access, attributes, level, kind, target):
            self.assertIs(original, owner.original)
            self.assertEqual((access, attributes, level, kind), (0xb, None, 2, 1))
            target._obj.value = 33
            return True
        owner.duplicate = mock.Mock(side_effect=duplicate)
        owner.fn = mock.Mock(return_value=owner.duplicate)
        return owner

    def test_only_original_full_is_converted_with_minimum_rights_and_original_handles_retained(self):
        owner = self.owner()
        with mock.patch.object(probe.C, 'get_last_error', create=True, return_value=0):
            primary = owner.full_primary(owner.original, owner.before.copy())
        self.assertEqual(primary.value, 33)
        self.assertEqual([handle.value for handle in owner.owned], [11, 22, 33])
        owner.duplicate.assert_called_once()
        self.assertEqual(owner.fn.call_args.args[1], 'DuplicateTokenEx')
        conversion = owner.facts['fullTokenConversion']
        self.assertEqual(conversion['originalAuthenticationId'], conversion['primaryAuthenticationId'])
        self.assertEqual(conversion['primaryToken'], FULL)

    def test_original_primary_is_reused_without_duplication(self):
        owner = self.owner(full_type=1)
        self.assertIs(owner.full_primary(owner.original, owner.before.copy()), owner.original)
        owner.fn.assert_not_called()
        self.assertEqual([handle.value for handle in owner.owned], [11, 22])

    def test_limited_or_changed_original_context_never_reaches_duplicate_api(self):
        for defect in [dict(LIMITED, tokenType=2), dict(FULL, tokenType=2, sid=SID[:-1]+'2')]:
            owner = self.owner()
            owner.before = defect
            with self.assertRaises(probe.Refused):
                owner.full_primary(owner.original, defect.copy())
            owner.fn.assert_not_called()

    def test_converted_security_context_or_authentication_mismatch_refuses(self):
        for change in [dict(sid=SID[:-1]+'2'), dict(session=2), dict(elevated=False),
                       dict(elevationType=3), dict(integrity='S-1-16-8192'), dict(tokenType=2), {}]:
            owner = self.owner(changed=change)
            if not change:
                owner.primary_authentication += 1
            with mock.patch.object(probe.C, 'get_last_error', create=True, return_value=0):
                with self.assertRaises(probe.Refused) as caught:
                    owner.full_primary(owner.original, owner.before.copy())
            self.assertEqual(caught.exception.reason, 'original-full-primary-context-changed')
            self.assertEqual([handle.value for handle in owner.owned], [11, 22, 33])

    def test_original_context_is_rechecked_after_conversion(self):
        owner = self.owner()
        original = owner.before.copy()
        owner.native._token = mock.Mock(side_effect=[original, FULL.copy(), dict(original, session=2)])
        with mock.patch.object(probe.C, 'get_last_error', create=True, return_value=0):
            with self.assertRaises(probe.Refused):
                owner.full_primary(owner.original, original)
        self.assertEqual([handle.value for handle in owner.owned], [11, 22, 33])

    def test_native_conversion_failure_does_not_enable_privileges_or_retry(self):
        owner = self.owner()
        owner.duplicate.side_effect = None
        owner.duplicate.return_value = False
        with mock.patch.object(probe.C, 'get_last_error', create=True, return_value=5):
            with self.assertRaises(probe.Refused) as caught:
                owner.full_primary(owner.original, owner.before.copy())
        self.assertEqual((caught.exception.reason, caught.exception.winerror),
                         ('original-full-primary-conversion-refused', 5))
        owner.duplicate.assert_called_once()
        self.assertEqual([handle.value for handle in owner.owned], [11, 22])
        self.assertEqual(owner.facts['originalFullStatistics']['impersonationLevel'], 2)

    def test_identification_full_statistics_survive_refusal_without_conversion(self):
        owner = self.owner()
        owner.token_statistics.return_value = None
        owner.token_statistics.side_effect = lambda handle: dict(authenticationId=17, tokenType=2,
                                                                impersonationLevel=1)
        with self.assertRaises(probe.Refused) as caught:
            owner.full_primary(owner.original, owner.before.copy())
        self.assertEqual(caught.exception.reason, 'original-full-token-not-launchable')
        self.assertEqual(owner.facts['originalFullStatistics'],
                         dict(authenticationId=17, tokenType=2, impersonationLevel=1))
        owner.fn.assert_not_called()

    def test_interrupted_return_still_retains_new_token_for_original_cleanup(self):
        owner = self.owner()
        def interrupted(*args):
            args[-1]._obj.value = 33
            raise KeyboardInterrupt()
        owner.duplicate.side_effect = interrupted
        with self.assertRaises(KeyboardInterrupt):
            owner.full_primary(owner.original, owner.before.copy())
        self.assertEqual([handle.value for handle in owner.owned], [11, 22, 33])

    def test_native_statistics_layout_and_full_authentication_luid_bits(self):
        owner = probe.AccountProbe.__new__(probe.AccountProbe)
        def read(token, kind, target):
            self.assertEqual(kind, 10)
            self.assertEqual(ctypes.sizeof(target), 56)
            self.assertEqual(type(target).authenticationId.offset, 8)
            self.assertEqual(type(target).tokenType.offset, 24)
            self.assertEqual(type(target).modifiedId.offset, 48)
            target.authenticationId.high, target.authenticationId.low = -1, 17
            target.tokenType, target.impersonationLevel = 2, 2
            return target
        owner.native = SimpleNamespace(_info=read)
        self.assertEqual(owner.token_statistics(None), dict(authenticationId=0xffffffff00000011,
                                                          tokenType=2, impersonationLevel=2))


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

    def test_progress_is_ordered_bounded_and_never_contains_raw_output(self):
        lines = [(customer.CAPABILITY_PROGRESS_PREFIX + marker + '\r\n').encode('ascii')
                 for marker in customer.CAPABILITY_PROGRESS_MARKERS]
        raw = b'{"privatePath":"secret-path","environment":"secret-value"}'
        result = probe.capability_progress(raw, b''.join(lines))
        self.assertTrue(result['markerSequenceComplete'])
        self.assertTrue(result['finalDocumentPresent'])
        self.assertEqual(result['lastMarker'], 'observation-end')
        self.assertEqual(result['stdoutBytes'], len(raw))
        self.assertNotIn('secret', json.dumps(result))
        for diagnostics in (lines[1], lines[0] + lines[0], lines[0] + b'private-secret\n',
                            b''.join(lines) + b'private-secret\n'):
            with self.subTest(diagnostics=len(diagnostics)):
                result = probe.capability_progress(raw, diagnostics)
                self.assertFalse(result['markerProtocolValid'])
                self.assertFalse(result['markerSequenceComplete'])
                self.assertNotIn('secret', json.dumps(result))
        partial = probe.capability_progress(b'{"schema":', lines[0] + lines[1][:-1])
        self.assertEqual(partial['markers'], ['helper-entry'])
        self.assertFalse(partial['markerSequenceComplete'])
        self.assertFalse(partial['finalDocumentPresent'])
        for output, diagnostics in ((b'x' * 16385, b''), (b'', b'x' * 1025)):
            with self.assertRaises(probe.Refused):
                probe.capability_progress(output, diagnostics)

    def test_limited_child_desktop_is_separate_from_elevated_parent_availability(self):
        expected = dict(LIMITED, session=2)
        value = customer._capability(dict(current=expected, linked=dict(FULL, session=2, tokenType=2),
            linkedError=None, policy=dict(enableLUA=1, consentPromptBehaviorAdmin=0, promptOnSecureDesktop=1),
            desktop=dict(station='WinSta0', visible=True, input='Default', thread='Default')))
        self.assertEqual(value['state'], 'blocked')  # Existing launch contract stays unchanged.
        value['privatePath'] = 'secret'
        result = probe.project_limited_capability(json.dumps(value).encode(), expected)
        self.assertEqual(result['state'], 'available')
        self.assertEqual(result['route'], 'limited-primary-read-only-child')
        for key in ('elevatedParentLaunchAvailable', 'sameAccountElevationTested',
                    'nativeCustomerAcceptance', 'consentPromptAcceptance'):
            self.assertFalse(result[key])
        self.assertNotIn('secret', json.dumps(result))
        self.assertFalse(customer._normal(expected, value['observations']['linked']))

    def test_limited_child_unreadable_desktop_is_blocked_even_with_actual_primary(self):
        expected = dict(LIMITED, session=2)
        observed = dict(current=expected, linked=None, linkedError=1312,
            policy=dict(enableLUA=1, consentPromptBehaviorAdmin=0, promptOnSecureDesktop=1),
            desktop=dict(station='WinSta0', visible=True, input=None, thread='Default', nativeError=5))
        result = probe.project_limited_capability(json.dumps(customer._capability(observed)).encode(), expected)
        self.assertEqual(result['state'], 'blocked')
        self.assertTrue(result['limitedPrimaryChildObserved'])
        self.assertFalse(result['interactiveDesktopObserved'])
        self.assertEqual(result['observations']['desktop']['nativeError'], 5)

    def test_limited_projection_rejects_elevated_foreign_session_or_changed_child(self):
        expected = dict(LIMITED, session=2)
        for change in ({'session': 3}, {'sid': SID[:-1]+'2'}, {'tokenType': 2},
                       {'elevated': True, 'elevationType': 2, 'integrity': 'S-1-16-12288'}):
            observed = dict(current=dict(expected, **change), linked=None, linkedError=1312,
                policy=dict(enableLUA=1, consentPromptBehaviorAdmin=0, promptOnSecureDesktop=1),
                desktop=dict(station='WinSta0', visible=True, input='Default', thread='Default'))
            with self.subTest(change=change), self.assertRaises(probe.Refused):
                probe.project_limited_capability(json.dumps(customer._capability(observed)).encode(), expected)

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

    def test_original_diagnostic_owner_drains_before_account_cleanup_and_failure_defers(self):
        owner = self.owner()
        owner.diagnostic = SimpleNamespace(close=mock.Mock())
        owner.native.close = mock.Mock(side_effect=lambda: owner.diagnostic.close.assert_called_once_with())
        self.assertEqual(owner.cleanup()['state'], 'own-account-and-profile-removed')
        owner = self.owner()
        owner.diagnostic = SimpleNamespace(close=mock.Mock(side_effect=customer.CustomerLaunchRefused('close refused')))
        self.assertEqual(owner.cleanup()['state'], 'deferred-to-disposable-job')
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
    def setUp(self):
        reset = mock.patch.object(probe.C, 'set_last_error', create=True)
        reset.start()
        self.addCleanup(reset.stop)
        channel = mock.patch.object(customer, '_ProfileCapabilityFile', side_effect=self.open_diagnostic)
        self.file_factory = channel.start()
        self.addCleanup(channel.stop)

    def open_diagnostic(self, token, **comparisons):
        self.assertEqual(token, 987)
        self.assertEqual(comparisons, dict(expected_profile=Path('original-profile'), expected_sid=SID))
        return self.current_diagnostic

    def child_owner(self, *, child_change=None, authentication=17, report_change=None,
                    raw_output=None, raw_diagnostics=None, wait_state=0, exit_status=0):
        owner = probe.AccountProbe.__new__(probe.AccountProbe)
        owner.sid, owner.limited, owner.owned, owner.facts = SID, 987, [], {}
        owner.limited_fact = dict(LIMITED, session=2)
        owner.limited_statistics = dict(authenticationId=17, tokenType=1, impersonationLevel=0)
        child = dict(owner.limited_fact, **(child_change or {}))
        reported = dict(owner.limited_fact, **(report_change or {}))
        value = customer._capability(dict(current=reported, linked=dict(FULL, session=2, tokenType=2),
            linkedError=None, policy=dict(enableLUA=1, consentPromptBehaviorAdmin=0, promptOnSecureDesktop=1),
            desktop=dict(station='WinSta0', visible=True, input='Default', thread='Default')))
        output = json.dumps(value).encode() if raw_output is None else raw_output
        diagnostics = (b''.join((customer.CAPABILITY_PROGRESS_PREFIX + marker + '\r\n').encode('ascii')
                       for marker in customer.CAPABILITY_PROGRESS_MARKERS)
                       if raw_diagnostics is None else raw_diagnostics)
        calls = []
        self.current_diagnostic = SimpleNamespace(close=mock.Mock())
        def diagnostic_read(**kwargs):
            calls.append('DiagnosticReadFinal' if kwargs.get('final') else 'DiagnosticRead')
            return diagnostics + output
        self.current_diagnostic.read = mock.Mock(side_effect=diagnostic_read)
        owner.exit_observed = False
        owner.kernelbase = 'kernelbase'
        owner.helper = Path('fixed-tools/installed_customer_launch.py')
        owner.native = SimpleNamespace(kernel='kernel', adv='adv', observed={'current': {'session': 2}},
            _token=lambda token: owner.limited_fact if token == 987 else child,
            _process_token=lambda handle: 77, _created=lambda handle: 123,
            _image=lambda handle: probe.sys.executable)
        owner.token_statistics = lambda token: (owner.limited_statistics if token == 987 else
            dict(authenticationId=authentication, tokenType=child['tokenType'], impersonationLevel=0))
        owner.ok = lambda value: probe.require(value, 'native-failure')
        owner.verify_profile = mock.Mock(return_value=Path('original-profile'))
        def create(token, flags, executable, command, creation, environment, cwd, startup, process):
            self.assertEqual((token, flags, creation, environment), (987, 1, 4, None))
            self.assertEqual(executable, probe.sys.executable)
            self.assertEqual(command.value, probe.subprocess.list2cmdline(
                [probe.sys.executable, '-I', '-B', str(owner.helper), customer.PROFILE_CAPABILITY_MODE]))
            self.assertEqual(startup._obj.desktop, 'WinSta0\\Default')
            self.assertEqual(startup._obj.flags, 0)
            self.assertEqual((startup._obj.stdin, startup._obj.stdout, startup._obj.stderr), (None, None, None))
            process._obj.process, process._obj.thread = 201, 202
            process._obj.pid, process._obj.tid = 301, 302
            return True
        def exit_code(handle, code):
            code._obj.value = exit_status
            return True
        apis = {'CreateProcessWithTokenW': create,
                'GetProcessId': lambda *args: 301, 'GetProcessIdOfThread': lambda *args: 301,
                'GetThreadId': lambda *args: 302, 'ResumeThread': lambda *args: 1,
                'WaitForSingleObject': lambda *args: wait_state, 'GetExitCodeProcess': exit_code}
        def fn(dll, name, *args):
            def call(*args):
                calls.append(name)
                return apis[name](*args)
            return call
        owner.fn, owner.apis = fn, apis
        return owner, calls

    def test_actual_child_handles_authentication_and_readonly_report_are_bound_through_exit(self):
        owner, calls = self.child_owner()
        with mock.patch.object(probe.C, 'get_last_error', create=True, return_value=0):
            result = owner.observe()
        self.assertEqual(result['state'], 'available')
        self.assertEqual(result['route'], 'limited-primary-read-only-child')
        self.assertEqual(calls.count('CreateProcessWithTokenW'), 1)
        self.assertEqual(calls.count('ResumeThread'), 1)
        self.assertTrue(owner.exit_observed)
        self.assertEqual(owner.facts['originalChild']['statistics']['authenticationId'], 17)
        self.assertFalse(result['sameAccountElevationTested'])
        self.assertTrue(owner.facts['capabilityProgress']['markerSequenceComplete'])
        self.assertTrue(owner.facts['capabilityProgress']['finalDocumentPresent'])
        self.assertNotIn('CreatePipe', calls)
        self.assertNotIn('DuplicateHandle', calls)
        self.assertIs(owner.diagnostic, self.current_diagnostic)
        self.assertLess(calls.index('DiagnosticReadFinal'), calls.index('GetExitCodeProcess'))

    def test_original_identity_is_rechecked_after_profile_reader_acquisition(self):
        for fault in ('creation', 'token', 'thread'):
            with self.subTest(fault=fault):
                owner, calls = self.child_owner()
                def acquire(*args, **kwargs):
                    if fault == 'creation':
                        owner.native._created = lambda handle: 124
                    elif fault == 'token':
                        token = owner.native._token
                        owner.native._token = lambda handle: dict(token(handle), session=3) if handle == 77 else token(handle)
                    else:
                        owner.apis['GetThreadId'] = lambda *args: 303
                    return self.current_diagnostic
                self.file_factory.side_effect = acquire
                with mock.patch.object(probe.C, 'get_last_error', create=True, return_value=0):
                    with self.assertRaises(probe.Refused) as caught:
                        owner.observe()
                self.assertEqual(caught.exception.reason, 'original-probe-child-identity-refused')
                self.assertNotIn('ResumeThread', calls)
                self.assertIs(owner.diagnostic, self.current_diagnostic)

    def test_foreign_or_preexisting_profile_file_refuses_before_resume(self):
        owner, calls = self.child_owner()
        self.file_factory.side_effect = customer.CustomerLaunchRefused('Preexisting capability file refused')
        with mock.patch.object(probe.C, 'get_last_error', create=True, return_value=0):
            with self.assertRaises(customer.CustomerLaunchRefused):
                owner.observe()
        self.assertNotIn('ResumeThread', calls)
        self.assertFalse(owner.exit_observed)

    def test_timeout_retains_progress_even_with_complete_document_but_never_accepts_it(self):
        for output, diagnostics, last, document in (
                (b'', b'', None, False),
                (b'', (customer.CAPABILITY_PROGRESS_PREFIX + 'helper-entry\n').encode(),
                 'helper-entry', False),
                (b'', b''.join((customer.CAPABILITY_PROGRESS_PREFIX + marker + '\n').encode()
                  for marker in customer.CAPABILITY_PROGRESS_MARKERS[:2]), 'observation-begin', False),
                (b'{"schema":', None, 'observation-end', False),
                (None, None, 'observation-end', True)):
            with self.subTest(last=last):
                owner, calls = self.child_owner(raw_output=output, raw_diagnostics=diagnostics, wait_state=258)
                owner.prepare = mock.Mock()
                owner.cleanup = mock.Mock(return_value=dict(state='deferred-to-disposable-job'))
                with mock.patch.object(probe.C, 'get_last_error', create=True, return_value=0), \
                        mock.patch.object(probe.time, 'monotonic', side_effect=[100, 161]):
                    result = probe.run_probe(owner.helper, lambda helper: owner)
                self.assertEqual(result['reason'], 'original-capability-child-timeout')
                self.assertEqual(result['state'], 'blocked')
                self.assertFalse(result['nativeCustomerAcceptance'])
                self.assertFalse(result['consentPromptAcceptance'])
                self.assertNotIn('capability', result)
                self.assertFalse(owner.exit_observed)
                progress = result['originalFacts']['capabilityProgress']
                self.assertEqual(progress['lastMarker'], last)
                self.assertEqual(progress['finalDocumentPresent'], document)
                self.assertNotIn('secret', json.dumps(result))
                self.assertEqual(calls.count('ResumeThread'), 1)
                self.assertEqual(calls.count('CreateProcessWithTokenW'), 1)
                self.assertNotIn('GetExitCodeProcess', calls)

    def test_failure_retains_safe_file_progress_and_verified_exit_is_still_required(self):
        for case in ({'exit_status': 1}, {'raw_diagnostics': b'private-secret\n'}):
            with self.subTest(case=list(case)):
                owner, calls = self.child_owner(**case)
                owner.prepare = mock.Mock()
                owner.cleanup = mock.Mock(return_value=dict(state='deferred-to-disposable-job'))
                with mock.patch.object(probe.C, 'get_last_error', create=True, return_value=0):
                    result = probe.run_probe(owner.helper, lambda helper: owner)
                self.assertEqual(result['state'], 'blocked')
                self.assertNotIn('capability', result)
                self.assertNotIn('secret', json.dumps(result))
                progress = result['originalFacts']['capabilityProgress']
                if case.get('exit_status') == 1:
                    self.assertEqual(result['reason'], 'original-capability-child-exit-refused')
                    self.assertTrue(progress['markerSequenceComplete'])
                    self.assertTrue(progress['finalDocumentPresent'])
                else:
                    self.assertEqual(result['reason'], 'capability-progress-contract-refused')
                    self.assertFalse(progress['markerProtocolValid'])

    def test_changed_created_child_never_resumes_or_retries(self):
        cases = [dict(child_change={'session': 3}), dict(authentication=18),
                 dict(child_change={'tokenType': 2}), dict(child_change=dict(FULL, session=2))]
        for case in cases:
            owner, calls = self.child_owner(**case)
            with self.subTest(case=case), mock.patch.object(probe.C, 'get_last_error', create=True, return_value=0):
                with self.assertRaises(probe.Refused) as caught:
                    owner.observe()
            self.assertEqual(caught.exception.reason, 'original-probe-child-identity-refused')
            self.assertEqual(calls.count('CreateProcessWithTokenW'), 1)
            self.assertNotIn('ResumeThread', calls)
            self.assertNotIn('DuplicateHandle', calls)
            self.assertEqual(owner.launch_effect, 'created')
            self.assertFalse(owner.facts['originalChild']['resumeAttempted'])

    def test_report_cannot_replace_the_observed_original_child_session(self):
        owner, calls = self.child_owner(report_change={'session': 3})
        with mock.patch.object(probe.C, 'get_last_error', create=True, return_value=0):
            with self.assertRaises(probe.Refused) as caught:
                owner.observe()
        self.assertEqual(caught.exception.reason, 'original-limited-child-observation-mismatch')
        self.assertEqual(calls.count('ResumeThread'), 1)
        self.assertTrue(owner.exit_observed)

    def test_original_limited_token_profile_explicit_desktop_and_noargs_helper_only(self):
        owner = probe.AccountProbe.__new__(probe.AccountProbe)
        owner.sid, owner.limited = SID, 987
        owner.limited_fact = dict(LIMITED, session=2)
        owner.limited_statistics = dict(authenticationId=17, tokenType=1, impersonationLevel=0)
        owner.native = SimpleNamespace(kernel='kernel', adv='adv', _token=lambda token: owner.limited_fact)
        owner.token_statistics = lambda token: owner.limited_statistics
        owner.full_primary = mock.Mock(side_effect=AssertionError('FULL conversion forbidden'))
        owner.helper, owner.owned = Path('fixed-tools/installed_customer_launch.py'), []
        owner.ok = lambda value: probe.require(value, 'native-failure')
        calls = []
        def create(token, flags, executable, command, creation, environment, cwd, startup, process):
            calls.append('create')
            self.assertEqual((token, flags, creation, environment), (987, 1, 4, None))
            self.assertEqual(command.value, probe.subprocess.list2cmdline(
                [probe.sys.executable, '-I', '-B', str(owner.helper), customer.PROFILE_CAPABILITY_MODE]))
            self.assertEqual(executable, probe.sys.executable)
            self.assertEqual(startup._obj.desktop, 'WinSta0\\Default')
            self.assertEqual(startup._obj.flags, 0)
            self.assertEqual((startup._obj.stdin, startup._obj.stdout, startup._obj.stderr), (None, None, None))
            return False  # Existing privilege unavailable: never enable or retry.
        def fn(dll, name, *args):
            calls.append(name)
            return {'CreateProcessWithTokenW': create}[name]
        owner.fn = fn
        with mock.patch.object(probe.C, 'get_last_error', create=True, return_value=1314):
            with self.assertRaises(probe.Refused) as caught:
                owner.observe()
        self.assertEqual(caught.exception.reason, 'original-limited-token-fixed-probe-launch-refused')
        self.assertEqual(caught.exception.winerror, 1314)
        self.assertEqual(owner.launch_effect, 'refused-no-child')
        self.assertEqual([handle.value if hasattr(handle, 'value') else handle for handle in owner.owned],
                         [])
        self.assertEqual(calls.count('create'), 1)
        owner.full_primary.assert_not_called()

    def test_selects_genuine_limited_primary_without_converting_identification_full(self):
        owner = probe.AccountProbe.__new__(probe.AccountProbe)
        owner.sid, owner.facts = SID, {}
        owner.native = SimpleNamespace(_token=lambda token: LIMITED)
        owner.token_statistics = lambda token: dict(authenticationId=17, tokenType=1 if token == 11 else 2,
                                                   impersonationLevel=0 if token == 11 else 1)
        owner.full_primary = mock.Mock(side_effect=AssertionError('FULL conversion forbidden'))
        owner.select_limited(11, 22, LIMITED, dict(FULL, tokenType=2))
        self.assertEqual(owner.limited, 11)
        self.assertEqual(owner.facts['originalFullStatistics']['impersonationLevel'], 1)
        self.assertEqual(owner.facts['originalLimitedStatistics']['tokenType'], 1)
        owner.check_limited()
        owner.full_primary.assert_not_called()

    def test_changed_original_limited_token_refuses_before_native_child_api(self):
        for change in ({'tokenType': 2}, {'sid': SID[:-1]+'2'}, {'session': 3}, {'elevated': True}):
            owner = probe.AccountProbe.__new__(probe.AccountProbe)
            owner.sid, owner.limited = SID, 11
            owner.limited_fact = dict(LIMITED, session=2)
            owner.limited_statistics = dict(authenticationId=17, tokenType=1, impersonationLevel=0)
            owner.native = SimpleNamespace(_token=lambda token: dict(owner.limited_fact, **change))
            owner.token_statistics = lambda token: owner.limited_statistics
            owner.fn = mock.Mock()
            with self.subTest(change=change), self.assertRaises(probe.Refused):
                owner.observe()
            owner.fn.assert_not_called()


class HostGate(unittest.TestCase):
    def test_caller_arguments_are_rejected_before_host_or_native_operations(self):
        for arg in ('--command=anything', '--product=Setup.exe', '--sid='+SID, 'anything'):
            with mock.patch.object(probe.sys, 'argv', ['probe.py', arg]):
                with mock.patch.object(probe, 'hosted_context') as host:
                    with self.assertRaises(probe.Refused):
                        probe.main()
                    host.assert_not_called()

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

#!/usr/bin/env python3
"""Joined lifecycle ordering/refusal tests; inert native/publication observations."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import installed_updater_journey as journey


PROFILE = dict(navigationStore='sqlite', navigationSha256='a' * 64,
               navigationDetailsSha256='b' * 64, navigationMigrationFixtureSha256='', settingsSha256=['c' * 64])


class State:
    def __init__(self, root):
        self.root, self.trace = root, []
        self.sid, self.profile_root = 'S-1-5-21-1-2-3-1001', root / 'profile'
        self.current, self.profile = False, copy.deepcopy(PROFILE)
        self.active, self.broker_live, self.broker_closed = set(), True, False
        self.fail_phase, self.fail_close = None, None
        self.fail_uninstall = False

    def history(self):
        return dict(state='selected-history', generation=('d' if self.current else 'a') * 32,
                    version='0.4.0-beta2.5' if self.current else '0.4.0-beta2.4',
                    commit=('e' if self.current else 'b') * 40,
                    completionSha256=('2' if self.current else '1') * 64,
                    selectedHeadSha256=('4' if self.current else '3') * 64,
                    qualificationSha256=('6' if self.current else '5') * 64,
                    qualificationClosedSha256=('8' if self.current else '7') * 64,
                    inventorySha256=('a' if self.current else '9') * 64,
                    ancestors=[dict(generation='a' * 32, completionSha256='1' * 64)] if self.current else [])


class Before:
    def __init__(self, state, prepared):
        self.state, self.sid, self.profile_root, self.closed = state, state.sid, state.profile_root, False

    def profile(self): return copy.deepcopy(self.state.profile)
    def pin_image(self, *args): pass
    def close(self):
        self.state.trace.append('before:close')
        self.closed = True


class Native:
    def __init__(self, state, prepared, provisioner, label):
        self.state, self.sid, self.profile_root = state, state.sid, state.profile_root
        self.prepared, self.label = prepared, label
        assert not state.active, 'Previous observer image owners were retained'
        state.active.add(label)
        state.trace.append(label + ':open')

    def profile(self): return copy.deepcopy(self.state.profile)
    def history(self): return self.state.history()

    def publication(self, name, timestamp):
        self.state.trace.append((name, 'observe-publication', timestamp))

    def start_and_close(self, name, before):
        self.state.trace.append((self.label, 'start', name))
        if name == self.state.fail_phase and self.label != 'initial':
            raise ValueError('inert original native phase refusal')
        if name == 'now': self.state.current = True

    def close(self):
        self.state.trace.append(self.label + ':close')
        if self.label == self.state.fail_close:
            raise ValueError('inert original owner close uncertain')
        self.state.active.remove(self.label)


class Fixture:
    def __init__(self, state, before, prepared, phase):
        self.state, self.n, self.prepared = state, before, prepared

    def install_setup(self, evidence):
        self.state.trace.append('setup')
        return dict(state='genuine-default-setup-installed')

    def prepare_preservation(self): return {'inert-custom-file': 'a' * 64}

    def customer_uninstall(self):
        assert not self.state.active and self.n.closed and self.state.broker_closed
        self.state.trace.append('maintain')
        if self.state.fail_uninstall:
            raise ValueError('inert actual Maintain refusal')
        return dict(state='original-relocated-maintain-exited-cleanly', consentPromptAcceptance=False)

    def verify_uninstall(self, preserved):
        assert preserved == {'inert-custom-file': 'a' * 64}
        self.state.trace.append('uninstalled')
        return dict(state='service-and-registration-absent-original-state-retired')


class Broker:
    def __init__(self, fixture):
        self.state = fixture.state
        assert not self.state.active, 'Initial image owners must drain first'
        self.state.trace.append('broker:retain')

    def check(self):
        if not self.state.broker_live: raise ValueError('inert original broker exited')

    def close(self):
        self.state.trace.append('broker:close')
        self.state.broker_closed = True


class JoinedTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.state = State(self.root)
        self.initial_record = self.original('b0-admission.json')
        self.phase_record = self.original('fixtures.json')
        shared = [self.original(name) for name in ('b0-recovery.zip', 'b0-source.zip', 'b0-policy.json', 'request.json', 'kit.zip')]
        b0 = dict(version='0.4.0-beta2.4', commit='b' * 40, trustSha256='c' * 64,
                  paths=dict(zip(('recovery', 'source', 'policy'), (item[0] for item in shared))))
        request = dict(clientSid=self.state.sid, installed=dict(generation='a' * 32,
                       version=b0['version'], commit=b0['commit']))
        self.initial = dict(b0=b0, request=request, kit=shared[-1][0], settings=[dict(section='Settings', key='inert')],
                            retained=[self.initial_record, *shared], directories=[], requestPath=shared[-2][0],
                            trustPath=self.root / 'trust.json', cache=self.root / 'cache', artifact=self.root / 'artifact',
                            brokerPolicy=dict(provisioner=dict(sha256='1' * 64), broker=dict(sha256='2' * 64)))
        self.initial['artifact'].mkdir()
        self.phases = copy.deepcopy({key: self.initial[key] for key in ('b0', 'request', 'kit', 'settings')})
        self.phases.update(retained=[self.phase_record, *shared],
                           b1=dict(version='0.4.0-beta2.5', commit='e' * 40, trustSha256='c' * 64))
        self.pins = dict(request='d' * 64, policy='e' * 64, provisioner='1' * 64, broker='2' * 64, timestamp='f' * 64)
        self.output = self.root / 'evidence' / 'result.json'

    def original(self, name):
        path = self.root / name
        path.write_bytes(('INERT original ' + name).encode())
        return path, journey.phase.stamp(path), journey.admission.updater._hash(path)

    def stage(self, prepared, destination):
        self.state.trace.append('stage-authenticated')
        prepared.update(provisionerSha256=self.pins['provisioner'], brokerSha256=self.pins['broker'])
        return destination / 'skager-update-provisioner.exe'

    def handoff(self, name, head):
        assert not self.state.active, 'Handoff occurred with image owners retained'
        self.state.trace.append(('handoff', name, head))
        return None if name == 'offline' else ('f' if name == 'no-update' else 'a') * 64

    def run_joined(self, handoff=None, phase_native_factory=None):
        return journey.run_joined(self.initial, self.phases, self.pins, self.output,
            handoff=handoff or self.handoff,
            initial_native_factory=lambda p, exe: Native(self.state, p, exe, 'initial'),
            phase_native_factory=phase_native_factory or (lambda p, exe: Native(self.state, p, exe, 'phase-' +
                [item[1] for item in self.state.trace if isinstance(item, tuple) and item[0] == 'handoff'][-1])),
            preflight_factory=lambda p: Before(self.state, p), stage=self.stage,
            publish=lambda *args: None, enroll=lambda command: self.state.trace.append('enroll'),
            fixture_factory=lambda before, p, phase: Fixture(self.state, before, p, phase), broker_factory=Broker)

    def test_one_setup_enrollment_and_original_profile_four_phases_then_maintain(self):
        original_phase_inputs = copy.deepcopy(self.phases)
        result = self.run_joined()
        trace = self.state.trace
        self.assertEqual(trace.count('setup'), 1)
        self.assertEqual(trace.count('enroll'), 1)
        self.assertEqual(trace.count('maintain'), 1)
        self.assertEqual([(item[1], item[2]) for item in trace if isinstance(item, tuple) and item[0] == 'handoff'],
                         [('no-update', '3' * 64), ('later', '3' * 64), ('now', '3' * 64), ('offline', '4' * 64)])
        self.assertLess(trace.index('initial:close'), trace.index('broker:retain'))
        self.assertLess(trace.index('phase-offline:close'), trace.index('broker:close'))
        self.assertLess(trace.index('broker:close'), trace.index('before:close'))
        self.assertLess(trace.index('before:close'), trace.index('maintain'))
        self.assertEqual(self.phases, original_phase_inputs)
        self.assertEqual(result['state'], 'genuine-setup-b0-four-phases-maintain-uninstalled')
        self.assertTrue(result['nativeInitialB0Acceptance'])
        self.assertTrue(result['nativeInstalledUninstallAcceptance'])
        self.assertFalse(result['nativeProductAcceptance'])
        self.assertFalse(result['customerUacAcceptance'])
        for name in journey.PHASES:
            saved = json.loads((self.output.parent / name / 'result.json').read_text())
            self.assertTrue(saved['nativeInstalledPhaseAcceptance'])
            self.assertEqual(saved['phase'], name)
            self.assertFalse(saved['nativeProductAcceptance'])
        self.assertEqual(json.loads(self.output.read_text()), result)

    def test_native_phase_refusal_retains_prior_and_partial_results_without_later_phase_or_uninstall(self):
        self.state.fail_phase = 'later'
        with self.assertRaises(ValueError): self.run_joined()
        self.assertNotIn('maintain', self.state.trace)
        self.assertFalse(self.state.active)
        saved = json.loads(self.output.read_text())
        self.assertEqual(saved['state'], 'refused')
        self.assertTrue(saved['phases'][0]['nativeInstalledPhaseAcceptance'])
        self.assertFalse(saved['phases'][1]['nativeInstalledPhaseAcceptance'])
        self.assertEqual(saved['phases'][1]['status'], 'refused')
        self.assertFalse((self.output.parent / 'now').exists())
        self.assertEqual(self.state.trace.count('enroll'), 1)

    def test_handoff_refusal_or_invalid_timestamp_never_starts_that_phase(self):
        for fault in ('raise', 'missing', 'offline-pin'):
            with self.subTest(fault=fault):
                self.output = self.root / ('evidence-' + fault) / 'result.json'
                self.state = State(self.root)
                def handoff(name, head):
                    answer = self.handoff(name, head)
                    if fault == 'raise' and name == 'later': raise RuntimeError('inert operator refusal')
                    if fault == 'missing' and name == 'later': return None
                    if fault == 'offline-pin' and name == 'offline': return 'a' * 64
                    return answer
                with self.assertRaises((ValueError, RuntimeError)): self.run_joined(handoff)
                phase = 'offline' if fault == 'offline-pin' else 'later'
                self.assertNotIn('phase-' + phase + ':open', self.state.trace)
                self.assertNotIn('maintain', self.state.trace)

    def test_changed_original_phase_input_during_handoff_refuses_before_native_creation(self):
        def handoff(name, head):
            answer = self.handoff(name, head)
            if name == 'later': self.phase_record[0].write_bytes(b'changed original declaration')
            return answer
        with self.assertRaises(ValueError): self.run_joined(handoff)
        self.assertNotIn('phase-later:open', self.state.trace)
        self.assertNotIn('maintain', self.state.trace)
        self.assertEqual(self.phase_record[0].read_bytes(), b'changed original declaration')

    def test_profile_change_between_phases_cannot_hide_behind_phase_local_preservation(self):
        def handoff(name, head):
            answer = self.handoff(name, head)
            if name == 'later': self.state.profile['navigationSha256'] = 'f' * 64
            return answer
        with self.assertRaises(ValueError): self.run_joined(handoff)
        self.assertNotIn(('phase-later', 'start', 'later'), self.state.trace)
        self.assertNotIn('maintain', self.state.trace)

    def test_original_broker_exit_during_handoff_stops_before_phase_and_uninstall(self):
        def handoff(name, head):
            answer = self.handoff(name, head)
            if name == 'now': self.state.broker_live = False
            return answer
        with self.assertRaises(ValueError): self.run_joined(handoff)
        self.assertNotIn('phase-now:open', self.state.trace)
        self.assertNotIn('maintain', self.state.trace)
        self.assertTrue(self.state.broker_closed)

    def test_uncertain_owner_drain_never_advances_or_retries_close(self):
        for label in ('initial', 'phase-later'):
            with self.subTest(label=label):
                self.output = self.root / ('evidence-' + label) / 'result.json'
                self.state = State(self.root)
                self.state.fail_close = label
                with self.assertRaises(ValueError): self.run_joined()
                self.assertEqual(self.state.trace.count(label + ':close'), 1)
                self.assertNotIn('maintain', self.state.trace)
                self.assertNotIn('phase-now:open', self.state.trace)

    def test_different_observed_account_or_profile_refuses_before_phase_launch(self):
        for mismatch in ('sid', 'profile'):
            with self.subTest(mismatch=mismatch):
                self.output = self.root / ('evidence-' + mismatch) / 'result.json'
                self.state = State(self.root)
                def factory(prepared, provisioner):
                    native = Native(self.state, prepared, provisioner, 'phase-no-update')
                    if mismatch == 'sid': native.sid += '1'
                    else: native.profile_root = self.root / 'different-profile'
                    return native
                with self.assertRaises(ValueError): self.run_joined(phase_native_factory=factory)
                self.assertNotIn(('phase-no-update', 'start', 'no-update'), self.state.trace)
                self.assertNotIn('maintain', self.state.trace)
                self.assertFalse(self.state.active)

    def test_maintain_failure_preserves_four_scoped_phase_results_without_uninstall_claim(self):
        self.state.fail_uninstall = True
        with self.assertRaises(ValueError): self.run_joined()
        result = json.loads(self.output.read_text())
        self.assertEqual(result['state'], 'refused')
        self.assertEqual(len(result['phases']), 4)
        self.assertTrue(all(item['nativeInstalledPhaseAcceptance'] for item in result['phases']))
        self.assertFalse(result['nativeInstalledUninstallAcceptance'])
        self.assertNotIn('uninstalled', self.state.trace)
        self.assertEqual(self.state.trace.count('maintain'), 1)

    def test_inequivalent_originals_or_nonfresh_evidence_refuse_before_setup(self):
        original = copy.deepcopy(self.phases)
        for key in ('b0', 'request', 'kit', 'settings', 'retained'):
            self.phases = copy.deepcopy(original)
            if key == 'b0': self.phases['b0']['commit'] = '0' * 40
            elif key == 'request': self.phases['request']['clientSid'] += '1'
            elif key == 'kit': self.phases['kit'] = self.root / 'different-kit.zip'
            elif key == 'settings': self.phases['settings'] = [dict(section='Settings', key='other')]
            elif key == 'retained': self.phases['retained'][-1] = (*self.phases['retained'][-1][:2], 'f' * 64)
            with self.subTest(key=key), self.assertRaises(ValueError): self.run_joined()
            self.assertNotIn('setup', self.state.trace)
        self.phases = original
        self.output.parent.mkdir()
        (self.output.parent / 'retained-refusal.txt').write_text('preserve')
        with self.assertRaises(ValueError): self.run_joined()
        self.assertNotIn('setup', self.state.trace)
        self.assertEqual((self.output.parent / 'retained-refusal.txt').read_text(), 'preserve')


if __name__ == '__main__':
    unittest.main()

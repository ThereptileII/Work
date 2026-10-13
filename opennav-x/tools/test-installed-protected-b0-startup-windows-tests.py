"""Inert admission tests; no Windows service, network, or application is run."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock


PATH = Path(__file__).with_name('test-installed-protected-b0-startup-windows.py')
spec = importlib.util.spec_from_file_location('initial_b0', PATH)
b0 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(b0)


class FakeBefore:
    def __init__(self, prepared):
        self.sid = prepared['request']['clientSid']

    def profile(self):
        return dict(navigationStore='sqlite', navigationSha256='a' * 64,
                    navigationDetailsSha256='a' * 64, navigationMigrationFixtureSha256='', settingsSha256=['b' * 64])

    def pin_image(self, path, digest):
        self.pinned = (path, digest)

    def close(self):
        pass


class FakeNative:
    instances = []

    def __init__(self, prepared, provisioner):
        self.prepared = prepared
        self.events = []
        self.history_value = {'state': 'selected-history', 'generation': 'a' * 32,
                              'version': '0.4.2-beta.1', 'commit': 'b' * 40,
                              'ancestors': [], **{field: 'c' * 64 for field in
                              ('completionSha256', 'selectedHeadSha256', 'qualificationSha256',
                               'qualificationClosedSha256', 'inventorySha256')}}
        self.instances.append(self)

    def profile(self):
        self.events.append('profile')
        return dict(navigationStore='sqlite', navigationSha256='a' * 64,
                    navigationDetailsSha256='a' * 64, navigationMigrationFixtureSha256='', settingsSha256=['b' * 64])

    def start_and_close(self, mode, before):
        self.events.append(('start', mode, before))

    def history(self):
        self.events.append('history')
        return self.history_value

    def close(self):
        self.events.append('close')


class InitialB0Tests(unittest.TestCase):
    def setUp(self):
        FakeNative.instances = []
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        (self.base / 'artifact').mkdir()
        self.prepared = {
            'retained': [], 'directories': [], 'requestPath': self.base / 'approved-request.json',
            'trustPath': self.base / 'trust.json', 'cache': self.base / 'cache',
            'artifact': self.base / 'artifact', 'kit': self.base / 'kit.zip', 'settings': [],
            'request': {'clientSid': 'S-1-5-21-1-2-3-4', 'installed': {'generation': 'a' * 32,
                        'version': '0.4.2-beta.1', 'commit': 'b' * 40}},
            'b0': {'trustSha256': 'd' * 64},
            'brokerPolicy': {'broker': {'sha256': '2' * 64}, 'provisioner': {'sha256': '1' * 64}},
        }
        self.pins = {'request': 'e' * 64, 'policy': 'f' * 64, 'provisioner': '1' * 64,
                     'broker': '2' * 64, 'timestamp': '3' * 64}

    def stage(self, prepared, destination):
        self.assertEqual(destination.parent, self.base / 'artifact')
        self.assertTrue(destination.name.startswith('b0-verified-operator-'))
        prepared['provisionerSha256'] = self.pins['provisioner']
        prepared['brokerSha256'] = self.pins['broker']
        return destination / 'skager-update-provisioner.exe'

    def test_exact_one_enrollment_then_original_initial_launch_and_readback(self):
        events = []
        result = b0.run(self.prepared, self.pins, self.base / 'result.json',
                        native_factory=FakeNative, preflight_factory=FakeBefore, stage=self.stage,
                        publish=lambda *args: events.append(('publication', args[1])),
                        enroll=lambda command: events.append(('enroll', command)))
        self.assertEqual([name for name, *_ in events], ['publication', 'enroll', 'publication'])
        command = events[1][1]
        self.assertEqual(command.count('--activate-startup=true'), 1)
        self.assertIn('--client-registration-request-sha256=' + self.pins['request'], command)
        self.assertIn('--policy-sha256=' + self.pins['policy'], command)
        self.assertEqual(FakeNative.instances[0].events,
                         ['profile', ('start', 'no-update', None), 'history', 'profile', 'close'])
        self.assertTrue(result['nativeInitialB0Acceptance'])
        self.assertFalse(result['nativeProductAcceptance'])
        self.assertEqual(result['selected']['generation'], 'a' * 32)

    def test_unapproved_staged_executable_refuses_before_enrollment(self):
        attempts = []
        def wrong(prepared, destination):
            self.stage(prepared, destination)
            prepared['provisionerSha256'] = '0' * 64
            return destination / 'skager-update-provisioner.exe'
        with self.assertRaises(ValueError):
            b0.run(self.prepared, self.pins, self.base / 'result.json',
                   native_factory=FakeNative, preflight_factory=FakeBefore, stage=wrong,
                   publish=lambda *args: None, enroll=lambda command: attempts.append(command))
        self.assertEqual(attempts, [])

    def test_signed_policy_broker_mismatch_refuses_before_enrollment(self):
        self.prepared['brokerPolicy']['broker']['sha256'] = '9' * 64
        attempts = []
        with self.assertRaises(ValueError):
            b0.run(self.prepared, self.pins, self.base / 'result.json',
                   native_factory=FakeNative, preflight_factory=FakeBefore, stage=self.stage,
                   publish=lambda *args: None, enroll=lambda command: attempts.append(command))
        self.assertEqual(attempts, [])

    def test_uncertain_enrollment_is_never_retried_or_followed_by_launch(self):
        attempts = []
        def uncertain(command):
            attempts.append(command)
            raise ValueError('uncertain native result')
        with self.assertRaises(ValueError):
            b0.run(self.prepared, self.pins, self.base / 'result.json',
                   native_factory=FakeNative, preflight_factory=FakeBefore, stage=self.stage,
                   publish=lambda *args: None, enroll=uncertain)
        self.assertEqual(len(attempts), 1)
        self.assertEqual(FakeNative.instances, [])

    def test_changed_initial_selection_refuses_and_closes_owner(self):
        class Wrong(FakeNative):
            def history(self):
                result = super().history()
                return dict(result, generation='0' * 32)
        with self.assertRaises(ValueError):
            b0.run(self.prepared, self.pins, self.base / 'result.json', native_factory=Wrong,
                   preflight_factory=FakeBefore, stage=self.stage, publish=lambda *args: None,
                   enroll=lambda command: None)
        self.assertEqual(Wrong.instances[0].events[-1], 'close')

    def test_profile_change_after_enrollment_refuses_before_initial_launch(self):
        class Changed(FakeNative):
            def profile(self):
                self.events.append('profile')
                return dict(navigationStore='sqlite', navigationSha256='c' * 64,
                            navigationDetailsSha256='a' * 64, navigationMigrationFixtureSha256='', settingsSha256=['b' * 64])
        with self.assertRaises(ValueError):
            b0.run(self.prepared, self.pins, self.base / 'result.json', native_factory=Changed,
                   preflight_factory=FakeBefore, stage=self.stage, publish=lambda *args: None,
                   enroll=lambda command: None)
        self.assertNotIn(('start', 'no-update', None), Changed.instances[0].events)
        self.assertEqual(Changed.instances[0].events[-1], 'close')

    def test_timeout_leaves_original_enrollment_process_alive(self):
        calls = []
        class Process:
            def wait(self, timeout):
                calls.append(('wait', timeout))
                raise subprocess.TimeoutExpired(['provisioner'], timeout)
        with self.assertRaises(ValueError):
            b0.enroll_once(['provisioner'], popen=lambda command, **kw: Process())
        self.assertEqual(calls, [('wait', 1800)])

    def test_request_hash_must_be_approved_separately(self):
        declaration = self.base / 'declaration.json'
        declaration.write_text(json.dumps({'schema': 1, 'b0': {}, 'operatorKit': {},
                                          'clientRequest': {'path': 'unused', 'sha256': '1' * 64},
                                          'brokerBootstrapPolicy': {}, 'approvedBaseline': {}, 'trustConfig': {},
                                          'bootstrapCache': 'unused', 'artifactDirectory': 'unused',
                                          'preserveSettings': []}))
        from hashlib import sha256
        digest = sha256(declaration.read_bytes()).hexdigest()
        with mock.patch.object(b0.phase, 'fixture', return_value={}):
            with self.assertRaises(ValueError):
                b0.prepare(declaration, digest, '2' * 64, '3' * 64)

    def test_pre_enrollment_observer_uses_actual_broker_programdata_profile(self):
        folders = []
        with mock.patch.object(b0.phase, 'module', return_value=None), \
                mock.patch.object(b0.phase.Native, 'folder', side_effect=lambda guid: folders.append(guid) or self.base), \
                mock.patch.object(b0.phase.Native, 'actual_sid', return_value=self.prepared['request']['clientSid']):
            before = b0.BeforeEnrollment(self.prepared)
        self.assertEqual(folders, [b0.phase.PROGRAM_DATA])
        self.assertEqual(before.profile_root, self.base / 'opencpn')

    def test_initial_b0_accepts_only_controlled_fixture_xml_to_database_migration(self):
        xml = dict(navigationStore='xml', navigationSha256=b0.phase.CONTROLLED_NAV_SHA256,
                   navigationDetailsSha256='c' * 64, navigationMigrationFixtureSha256=b0.phase.CONTROLLED_XML_SHA256,
                   settingsSha256=['b' * 64])
        database = dict(xml, navigationStore='sqlite', navigationDetailsSha256='d' * 64)
        class Before(FakeBefore):
            def profile(self):
                return xml
        class Migrated(FakeNative):
            def __init__(self, prepared, provisioner):
                super().__init__(prepared, provisioner)
                self.profiles = iter((xml, database))
            def profile(self):
                return next(self.profiles)
        result = b0.run(self.prepared, self.pins, self.base / 'migration-result.json',
                        native_factory=Migrated, preflight_factory=Before, stage=self.stage,
                        publish=lambda *args: None, enroll=lambda command: None)
        self.assertTrue(result['nativeInitialB0Acceptance'])
        self.assertFalse(result['nativeProductAcceptance'])


if __name__ == '__main__':
    unittest.main()

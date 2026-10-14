"""Inert admission tests; no Windows service, network, or application is run."""
import importlib.util
import json
import ctypes
from pathlib import Path
import subprocess
import installed_b0_fixture as fixture
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


PATH = Path(__file__).with_name('test-installed-protected-b0-startup-windows.py')
spec = importlib.util.spec_from_file_location('initial_b0', PATH)
b0 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(b0)


class FakeBefore:
    def __init__(self, prepared):
        self.sid = prepared['request']['clientSid']
        self.closed = False

    def profile(self):
        return dict(navigationStore='sqlite', navigationSha256='a' * 64,
                    navigationDetailsSha256='a' * 64, navigationMigrationFixtureSha256='', settingsSha256=['b' * 64])

    def pin_image(self, path, digest):
        self.pinned = (path, digest)

    def close(self):
        self.closed = True


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


class FakeFixture:
    instances = []

    def __init__(self, before, prepared, phase):
        self.before, self.prepared, self.events = before, prepared, []
        self.instances.append(self)

    def install_setup(self, evidence):
        self.events.append('setup')
        return {'state': 'genuine-default-setup-installed'}

    def prepare_preservation(self):
        self.events.append('preserve')
        return {'modified': 'a' * 64}

    def customer_uninstall(self):
        assert self.before.closed, 'Original source pins still open before Maintain'
        assert FakeNative.instances[-1].events[-1] == 'close', 'Original launcher pins still open before Maintain'
        self.events.append('maintain')
        return {'state': 'original-relocated-maintain-exited-cleanly', 'consentPromptAcceptance': False}

    def verify_uninstall(self, preserved):
        assert preserved == {'modified': 'a' * 64}
        self.events.append('postconditions')
        return {'state': 'service-and-registration-absent-original-state-retired'}


class InitialB0Tests(unittest.TestCase):
    def setUp(self):
        FakeNative.instances = []
        FakeFixture.instances = []
        self.run = lambda *args, **kwargs: b0.run(*args, fixture_factory=FakeFixture, **kwargs)
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
        result = self.run(self.prepared, self.pins, self.base / 'result.json',
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
        self.assertEqual(FakeFixture.instances[0].events, ['setup', 'preserve', 'maintain', 'postconditions'])
        self.assertTrue(result['nativeInstalledUninstallAcceptance'])
        self.assertFalse(result['customerUacAcceptance'])

    def test_real_setup_refusal_precedes_any_protected_enrollment(self):
        class FailedSetup(FakeFixture):
            def install_setup(self, evidence):
                raise ValueError('actual Setup failed; do not substitute extracted files')
        attempts = []
        with self.assertRaises(ValueError):
            b0.run(self.prepared, self.pins, self.base / 'result.json', fixture_factory=FailedSetup,
                   native_factory=FakeNative, preflight_factory=FakeBefore, stage=self.stage,
                   publish=lambda *args: None, enroll=lambda command: attempts.append(command))
        self.assertFalse(attempts)
        self.assertFalse(FakeNative.instances)

    def test_customer_failure_is_not_initial_b0_or_uninstall_success(self):
        class FailedMaintain(FakeFixture):
            def customer_uninstall(self):
                super().customer_uninstall()
                raise ValueError('original relocated wizard refused')
        attempts = []
        with self.assertRaises(ValueError):
            b0.run(self.prepared, self.pins, self.base / 'result.json', fixture_factory=FailedMaintain,
                   native_factory=FakeNative, preflight_factory=FakeBefore, stage=self.stage,
                   publish=lambda *args: None, enroll=lambda command: attempts.append(command))
        self.assertEqual(len(attempts), 1)
        self.assertEqual(FakeNative.instances[0].events[-1], 'close')
        self.assertNotIn('postconditions', FailedMaintain.instances[-1].events)

    def test_unapproved_staged_executable_refuses_before_enrollment(self):
        attempts = []
        def wrong(prepared, destination):
            self.stage(prepared, destination)
            prepared['provisionerSha256'] = '0' * 64
            return destination / 'skager-update-provisioner.exe'
        with self.assertRaises(ValueError):
            self.run(self.prepared, self.pins, self.base / 'result.json',
                   native_factory=FakeNative, preflight_factory=FakeBefore, stage=wrong,
                   publish=lambda *args: None, enroll=lambda command: attempts.append(command))
        self.assertEqual(attempts, [])

    def test_signed_policy_broker_mismatch_refuses_before_enrollment(self):
        self.prepared['brokerPolicy']['broker']['sha256'] = '9' * 64
        attempts = []
        with self.assertRaises(ValueError):
            self.run(self.prepared, self.pins, self.base / 'result.json',
                   native_factory=FakeNative, preflight_factory=FakeBefore, stage=self.stage,
                   publish=lambda *args: None, enroll=lambda command: attempts.append(command))
        self.assertEqual(attempts, [])

    def test_uncertain_enrollment_is_never_retried_or_followed_by_launch(self):
        attempts = []
        def uncertain(command):
            attempts.append(command)
            raise ValueError('uncertain native result')
        with self.assertRaises(ValueError):
            self.run(self.prepared, self.pins, self.base / 'result.json',
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
            self.run(self.prepared, self.pins, self.base / 'result.json', native_factory=Wrong,
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
            self.run(self.prepared, self.pins, self.base / 'result.json', native_factory=Changed,
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
        declaration.write_text(json.dumps({'schema': 1, 'b0': {}, 'setup': {}, 'operatorKit': {},
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
        result = self.run(self.prepared, self.pins, self.base / 'migration-result.json',
                        native_factory=Migrated, preflight_factory=Before, stage=self.stage,
                        publish=lambda *args: None, enroll=lambda command: None)
        self.assertTrue(result['nativeInitialB0Acceptance'])
        self.assertFalse(result['nativeProductAcceptance'])


class GenuineSetupCompletion(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.report = self.root / 'setup.json'

    def test_real_nsis_arguments_keep_default_shortcuts_and_exact_report(self):
        command = fixture.setup_command(Path('C:/approved input/Setup.exe'), Path('C:/stock OpenCPN/opencpn.exe'), Path('C:/evidence/setup.json'))
        self.assertIn('/S /ACTION=Install /OPENCPN="C:/stock OpenCPN/opencpn.exe"', command)
        self.assertIn('/REPORT="C:/evidence/setup.json"', command)
        self.assertNotIn('ShortcutModes', command)
        with self.assertRaises(ValueError):
            fixture.setup_command('bad"Setup.exe', 'stock', 'report')

    def test_genuine_setup_matches_original_approved_release_policy_not_a_sidecar(self):
        setup = self.root / 'Setup.exe'
        setup.write_bytes(b'inert original Setup')
        policy = self.root / 'beta.json'
        record = {'path': str(setup), 'sha256': b0.updater._hash(setup)}
        baseline = {'commit': 'a' * 40, 'paths': {'policy': policy}}
        artifact = {'path': 'artifacts/' + baseline['commit'] + '/SKAGER-Beta2-Setup.exe',
                    'url': 'https://approved.invalid/Setup.exe', 'sha256': record['sha256'], 'bytes': setup.stat().st_size}
        for defect in ('valid', 'hash', 'length', 'commit', 'unknown'):
            changed = dict(artifact)
            if defect == 'hash': changed['sha256'] = 'f' * 64
            if defect == 'length': changed['bytes'] += 1
            if defect == 'commit': changed['path'] = 'artifacts/' + 'b' * 40 + '/SKAGER-Beta2-Setup.exe'
            if defect == 'unknown': changed['extra'] = True
            policy.write_text(json.dumps({'installer': changed}))
            retained = []
            if defect == 'valid':
                self.assertEqual(b0.approved_setup(record, baseline, retained), setup)
                self.assertEqual(len(retained), 1)
            else:
                with self.assertRaises(ValueError):
                    b0.approved_setup(record, baseline, retained)
                self.assertFalse(retained)

    def test_original_zero_exit_requires_actual_successful_install_report(self):
        calls = []
        class Process:
            pid = 17
            def wait(inner, timeout):
                calls.append(timeout)
                self.report.write_text('{"status":"passed","action":"Install"}')
                return 0
        self.assertEqual(fixture.setup_once('original Setup', self.report, b0.phase.strict,
                                          popen=lambda *args, **kwargs: Process()), 17)
        self.assertEqual(calls, [600])
        for result in ({'status': 'failed', 'action': 'Install'}, {'status': 'passed', 'action': 'Update'}, None):
            self.report.unlink()
            class Incomplete:
                pid = 18
                def wait(inner, timeout):
                    if result is not None:
                        self.report.write_text(json.dumps(result))
                    return 0
            with self.assertRaises(ValueError):
                fixture.setup_once('original Setup', self.report, b0.phase.strict,
                                   popen=lambda *args, **kwargs: Incomplete())
            if not self.report.exists():
                self.report.write_text('absent original report')

    def test_timeout_never_kills_retries_or_removes_original_state(self):
        calls = []
        class Process:
            pid = 17
            def wait(inner, timeout):
                calls.append(timeout)
                raise subprocess.TimeoutExpired('original Setup', timeout)
        with self.assertRaises(subprocess.TimeoutExpired):
            fixture.setup_once('original Setup', self.report, b0.phase.strict,
                               popen=lambda *args, **kwargs: Process())
        self.assertEqual(calls, [600])
        self.assertFalse(self.report.exists())


class GenuineMaintainCompletion(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / 'Maintain.exe'
        self.path.write_bytes(b'original inert Setup-created Maintain bytes')
        self.events, self.finish, self.exit_code, self.close_failure = [], False, 0, False
        self.owner = SimpleNamespace(pid=20, created=100, sid='actual-current-user', elevated=False,
            capability={'normalTokenAvailable': True, 'consentPromptAvailable': False}, check=lambda: None,
            wait=lambda timeout: self.events.append(('wrapper-exit', timeout)) or 0,
            close=lambda: self.events.append('wrapper-close'))
        self.observer = SimpleNamespace()
        self.observer.ui = SimpleNamespace(
            windows=self.windows, wait_window=lambda *args, **kwargs: (10, 21),
            monitor_process=lambda pid: 101, WaitForSingleObject=lambda *args: 258,
            children=lambda frame: [(30, 'SKAGER maintenance complete' if self.finish else 'Maintain SKAGER'), (31, 'Uninstall')],
            GetClassNameW=lambda control, buffer, length: setattr(buffer, 'value', 'ComboBox' if control == 31 else 'Static'),
            control_text=lambda control: 'Uninstall' if control == 31 else ('Finish' if self.finish else 'Continue'),
            IsWindowEnabled=lambda control: True, declare=lambda *args: lambda *args: 32,
            CloseHandle=lambda handle: self.events.append(('close', handle)) or not self.close_failure,
            user=None)
        self.observer.w = b0.wintypes
        self.observer.sid = self.owner.sid
        self.f = fixture.InstalledB0Fixture.__new__(fixture.InstalledB0Fixture)
        self.f.n = self.observer
        self.f.phase = b0.phase
        self.f.maintain = self.path
        self.f.maintain_stamp = b0.phase.stamp(self.path)
        self.f.maintain_sha = b0.updater._hash(self.path)
        self.f.evidence = self.root
        self.f.machine_recovery = self.root / 'machine-recovery'
        self.f.startup_recovery = self.root / 'startup-recovery'
        self.f.p = {'b0': {'recovery': {'sha256': 'a' * 64}}}
        self.f.inventory = lambda path: ({self.owner.sid + suffix: 'a' * 64 for suffix in
            ('.json', '.baseline.zip', '.maintenance.once', '.maintenance.json')} if path == self.f.machine_recovery else {'original': 'b' * 64})
        self.f.service_process = lambda: 102
        self.f.process_identity = lambda *args: 101
        self.f.process_parent = lambda pid: self.owner.pid
        self.f.press = self.press
        self.f.wait_exit = lambda handle, timeout: self.events.append(('original-exit', handle, timeout)) or self.exit_code
        def launch(command, sid):
            self.assertEqual(command, [str(self.path), '/ACTION=Uninstall'])
            self.assertEqual(sid, self.owner.sid)
            self.events.append('launch-normal')
            return self.owner
        self.patch = mock.patch.dict(sys.modules, {'installed_customer_launch': SimpleNamespace(
            open_same_account_launch=launch, inspect_customer_capability=lambda: self.owner.capability)})
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def windows(self, pid=None):
        return [] if not self.events else [(10, 21, 'SKAGER Maintenance')]

    def press(self, frame, pid, handle, identifier):
        self.events.append(('press', identifier))
        self.finish = True

    def test_actual_normal_wrapper_and_relocated_child_exit_precede_success(self):
        result = self.f.customer_uninstall()
        self.assertEqual(result['state'], 'original-relocated-maintain-exited-cleanly')
        self.assertFalse(result['consentPromptAcceptance'])
        self.assertEqual([event for event in self.events if isinstance(event, tuple) and event[0] == 'original-exit'],
                         [('original-exit', 101, 30000), ('original-exit', 102, 30000)])
        self.assertEqual(self.events[-3:], [('close', 101), ('close', 102), 'wrapper-close'])

    def test_wrapper_zero_cannot_replace_actual_relocated_child_exit(self):
        self.exit_code = 1
        with self.assertRaises(ValueError):
            self.f.customer_uninstall()
        self.assertNotIn(('wrapper-exit', 30), self.events)
        self.assertEqual(self.events[-3:], [('close', 101), ('close', 102), 'wrapper-close'])

    def test_foreign_parent_or_original_image_refuses_before_click(self):
        for defect in ('parent', 'image'):
            self.events.clear()
            if (self.root / 'customer-capability.json').exists():
                (self.root / 'customer-capability.json').unlink()
            if defect == 'parent':
                self.f.process_parent = lambda pid: 999
            else:
                self.f.process_parent = lambda pid: self.owner.pid
                self.f.process_identity = lambda *args: 99
            with self.assertRaises(ValueError):
                self.f.customer_uninstall()
            self.assertFalse(any(isinstance(event, tuple) and event[0] == 'press' for event in self.events))
            self.assertEqual(self.events[-1], 'wrapper-close')

    def test_close_uncertainty_drains_all_original_owners_and_refuses_success(self):
        self.close_failure = True
        with self.assertRaises(ValueError):
            self.f.customer_uninstall()
        self.assertEqual(self.events[-3:], [('close', 101), ('close', 102), 'wrapper-close'])


class NativeUninstallPostconditions(unittest.TestCase):
    def make(self, directory):
        f = fixture.InstalledB0Fixture.__new__(fixture.InstalledB0Fixture)
        f.phase = b0.phase
        f.root = directory / 'integration'
        f.generation = f.root / 'generations' / ('a' * 32)
        f.menu = directory / 'menu'
        f.stock = directory / 'stock' / 'opencpn.exe'
        f.machine_recovery = directory / 'machine-recovery'
        f.startup_recovery = directory / 'startup-recovery'
        f.state_raw = b'original Setup state'
        bodies = {
            f.root / 'retired-state.json': f.state_raw,
            f.generation / 'Maintain.exe': b'preserved original Maintain',
            f.generation / 'maintenance' / 'payload.zip': b'preserved original payload',
            f.generation / 'docs' / 'modified.txt': b'original with customer modifications',
            f.generation / 'app' / 'custom.txt': b'custom addition',
            f.root / 'recovery' / 'before.json': b'original Setup recovery',
            f.root / 'logs' / 'setup.log': b'original Setup diagnostics',
            f.stock: b'untouched working stock',
            f.machine_recovery / 'original.baseline.zip': b'original baseline',
            f.startup_recovery / 'original.lnk.bytes': b'original shortcut recovery',
        }
        for path, body in bodies.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(body)
        f.menu.mkdir()
        f.maintain = f.generation / 'Maintain.exe'
        f.maintain_sha = b0.updater._hash(f.maintain)
        f.payload_sha = b0.updater._hash(f.generation / 'maintenance' / 'payload.zip')
        f.recovery_before = f.inventory(f.root / 'recovery')
        f.logs_before = f.inventory(f.root / 'logs')
        f.stock_before = f.inventory(f.stock.parent)
        f.machine_recovery_before = f.inventory(f.machine_recovery)
        f.startup_recovery_before = fixture.Counter(f.inventory(f.startup_recovery).values())
        f.service_absent = lambda: True
        f.registration = lambda: None
        preserved = {str(path.relative_to(f.root)): b0.updater._hash(path) for path in
                     (f.generation / 'docs' / 'modified.txt', f.generation / 'app' / 'custom.txt')}
        return f, preserved

    def test_only_actual_complete_removal_with_preserved_originals_passes(self):
        with tempfile.TemporaryDirectory() as directory:
            f, preserved = self.make(Path(directory))
            result = f.verify_uninstall(preserved)
            self.assertEqual(result['modifiedAndCustomFiles'], 2)
            self.assertEqual(result['stock'], 'preserved')
            self.assertFalse(result['nativeProductAcceptance'])

    def test_missing_preservation_or_incomplete_cleanup_is_never_success(self):
        for defect in ('stock', 'modified', 'custom', 'baseline', 'startup-recovery', 'setup-recovery',
                       'diagnostics', 'owned-app-remains', 'active-state-remains', 'shortcut-remains', 'registration-remains'):
            with self.subTest(defect=defect), tempfile.TemporaryDirectory() as directory:
                f, preserved = self.make(Path(directory))
                if defect == 'stock': f.stock.write_bytes(b'changed stock')
                if defect == 'modified': (f.generation / 'docs' / 'modified.txt').unlink()
                if defect == 'custom': (f.generation / 'app' / 'custom.txt').unlink()
                if defect == 'baseline': (f.machine_recovery / 'original.baseline.zip').write_bytes(b'changed original baseline')
                if defect == 'startup-recovery': (f.startup_recovery / 'original.lnk.bytes').unlink()
                if defect == 'setup-recovery': (f.root / 'recovery' / 'before.json').write_bytes(b'changed recovery')
                if defect == 'diagnostics': (f.root / 'logs' / 'setup.log').unlink()
                if defect == 'owned-app-remains': (f.generation / 'app' / 'opencpn.exe').write_bytes(b'owned app was not removed')
                if defect == 'active-state-remains': (f.root / 'state.json').write_bytes(f.state_raw)
                if defect == 'shortcut-remains': (f.menu / 'Skager.lnk').write_bytes(b'active integration link')
                if defect == 'registration-remains': f.registration = lambda: {'OpenNavOwner': 'still installed'}
                with self.assertRaises((ValueError, OSError)):
                    f.verify_uninstall(preserved)


if __name__ == '__main__':
    unittest.main()

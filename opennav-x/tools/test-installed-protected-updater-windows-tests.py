#!/usr/bin/env python3
"""Inert portable driver checks; no native process, network or installed state."""
import copy
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import ssl
import sqlite3
import tempfile
import unittest
import urllib.error
import zipfile
from contextlib import closing, redirect_stderr
from types import SimpleNamespace
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('installed_driver', Path(__file__).with_name('test-installed-protected-updater-windows.py'))
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)


def raw(value):
    return json.dumps(value, separators=(',', ':')).encode()


def digest(value):
    return hashlib.sha256(value).hexdigest()


def profile_observation(navigation='a'):
    return dict(navigationStore='sqlite', navigationSha256=navigation * 64,
                navigationDetailsSha256='a' * 64, navigationMigrationFixtureSha256='', settingsSha256=['b' * 64])


class Tests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.trust = raw({'metadataUrl': 'https://selected.example.test', 'channel': 'beta'})
        self.b0 = self.fixture('b0', '0.4.0-beta2.4', 'a' * 40)
        self.b1 = self.fixture('b1', '0.4.0-beta2.5', 'b' * 40)
        b0 = driver.fixture(self.b0, [])
        installed = dict(version=self.b0['version'], commit=self.b0['commit'], generation='c' * 32,
                         packageSha256=b0['packageSha256'], productBuildSha256=b0['productSha256'],
                         executableSha256=b0['product']['executable_sha256'], inventorySha256=b0['inventorySha256'])
        self.request = dict(schema=1, purpose='client-registration', clientSid='S-1-5-21-1-2-3-500', installed=installed,
                            stock=dict(path='C:\\inert-stock\\opencpn.exe', version='5.12.4', arch='x86', executableSha256='a' * 64, upstreamCommit='b' * 40),
                            trustConfiguration=json.loads(self.trust), trustConfigSha256=digest(self.trust))
        self.value = dict(schema=1, b0=self.b0, b1=self.b1, operatorKit=self.file('kit.zip', b'INERT KIT; NOT EXECUTED'),
                          clientRequest=self.file('request.json', raw(self.request)), trustSha256=digest(self.trust),
                          preserveSettings=[dict(section='Settings', key='ShowStatusBar')])
        self.manifest = self.root / 'fixtures.json'
        self.save()

    def save(self):
        self.manifest.write_bytes(raw(self.value))
        return digest(self.manifest.read_bytes())

    def file(self, name, data):
        path = self.root / name
        path.write_bytes(data)
        return dict(path=str(path), sha256=digest(data))

    def fixture(self, name, version, commit):
        app = b'INERT ' + commit.encode()
        product = dict(version=version, commit=commit, test_fixtures=False, build_purpose='INSTALLED PRODUCT',
                       update_startup_health=1, commissioning_restart_protocol=1,
                       xnav_hardware_output_policy='status-only', xnav_manual_control_contract=0,
                       executable_sha256=digest(app), startup_launcher_sha256=digest(b'INERT LAUNCHER'),
                       restart_helper_sha256=digest(b'INERT RESTART'),
                       update_trust=dict(configSha256=digest(self.trust), sourceSha256=digest(self.trust), productCommit=commit))
        capability = dict(schema=1, purpose='skager-qualified-update-v1', commit=commit, executableSha256=digest(app),
                          configurationProtocol=1, coreOutputSuppression=1, pluginPolicy='deferred-until-ordinary-start')
        members = {'app/opencpn.exe': app, 'app/skager-start.exe': b'INERT LAUNCHER',
                   'app/opennav-restart.exe': b'INERT RESTART', 'app/skager-update-prompt.exe': b'INERT PROMPT',
                   'app/update-qualification.json': raw(capability), 'app/update-trust.json': self.trust,
                   'docs/PRODUCT_BUILD.json': raw(product), 'installer/package.json': raw(dict(version=version, commit=commit))}
        members['FILE_SHA256.json'] = raw({name: digest(data) for name, data in members.items()})
        path = self.root / (name + '-recovery.zip')
        with zipfile.ZipFile(path, 'w') as archive:
            for key, data in members.items():
                archive.writestr(driver.PREFIX + key, data)
        recovery = dict(path=str(path), sha256=digest(path.read_bytes()))
        source_files = {'opennav-x/src/application/Version.h': ('inline constexpr char Version[] = "' + version + '";').encode(),
                        'opennav-x/installer/windows/staging-update-trust.json': self.trust}
        reference = dict(productCommit=commit, files={name: {'sha256': digest(data)} for name, data in source_files.items()})
        path = self.root / (name + '-source.zip')
        with zipfile.ZipFile(path, 'w') as archive:
            for key, data in source_files.items():
                archive.writestr(key, data)
            archive.writestr('SOURCE_REFERENCE.json', raw(reference))
        source = dict(path=str(path), sha256=digest(path.read_bytes()))
        policy = dict(schema=1, product='skager', channel='beta', version=version, commit=commit)
        for key, record, filename in (('recovery', recovery, 'SKAGER-Beta2-Portable-Recovery.zip'),
                                       ('correspondingSource', source, 'SKAGER-Beta2-source.zip')):
            policy[key] = dict(path='artifacts/' + commit + '/' + filename, sha256=record['sha256'], bytes=Path(record['path']).stat().st_size)
        return dict(version=version, commit=commit, recovery=recovery, source=source,
                    policy=self.file(name + '-policy.json', raw(policy)))

    def prepared(self):
        return driver.prepare(self.manifest, self.save())

    def history(self, successor=False):
        return dict(schema=1, requestId='f' * 32, state='selected-history', generation=('d' if successor else 'c') * 32,
                    completionSha256=('2' if successor else '1') * 64, selectedHeadSha256=('4' if successor else '3') * 64,
                    version=self.b1['version'] if successor else self.b0['version'],
                    commit=self.b1['commit'] if successor else self.b0['commit'],
                    qualificationSha256=('6' if successor else '5') * 64,
                    qualificationClosedSha256=('8' if successor else '7') * 64,
                    inventorySha256=('a' if successor else '9') * 64, page=0,
                    ancestors=[dict(generation='c' * 32, completionSha256='1' * 64)] if successor else [])

    def test_exact_genuine_fixture_contract_and_original_identity_retained(self):
        prepared = self.prepared()
        self.assertEqual(prepared['request'], self.request)
        driver.unchanged(prepared)
        path = Path(self.b0['policy']['path'])
        content = path.read_bytes()
        path.unlink()
        path.write_bytes(content)
        with self.assertRaises(ValueError):
            driver.unchanged(prepared)

    def test_missing_capability_changed_commit_trust_and_source_refuse(self):
        for part in ('app/update-qualification.json', 'docs/PRODUCT_BUILD.json',
                     'app/update-trust.json', 'app/opencpn.exe'):
            with self.subTest(part=part):
                original = Path(self.b0['recovery']['path']).read_bytes()
                with zipfile.ZipFile(io.BytesIO(original)) as source:
                    members = {item.filename: source.read(item) for item in source.infolist()}
                del members[driver.PREFIX + part]
                path = self.root / 'broken.zip'
                with zipfile.ZipFile(path, 'w') as archive:
                    for name, data in members.items():
                        archive.writestr(name, data)
                bad = copy.deepcopy(self.b0)
                bad['recovery'] = dict(path=str(path), sha256=digest(path.read_bytes()))
                with self.assertRaises((ValueError, KeyError)):
                    driver.fixture(bad, [])
        self.value['b1']['version'] = self.b0['version']
        with self.assertRaises(ValueError):
            self.prepared()

    def test_link_malformed_duplicate_unknown_and_wrong_approval_refuse(self):
        with self.assertRaises(ValueError):
            driver.prepare(self.manifest, '0' * 64)
        original = self.manifest.read_bytes()
        self.manifest.write_bytes(b'{"schema":1,' + original[1:])
        with self.assertRaises(ValueError):
            driver.prepare(self.manifest, digest(self.manifest.read_bytes()))
        self.value['unknown'] = 'not accepted'
        with self.assertRaises(ValueError):
            self.prepared()
        del self.value['unknown']
        path = Path(self.value['operatorKit']['path'])
        path.unlink()
        path.symlink_to(self.manifest)
        with self.assertRaises(ValueError):
            self.prepared()

    def test_phase_order_and_same_head_vs_now_original_health_and_close(self):
        prepared = self.prepared()
        for phase in driver.PHASES:
            events = []
            before = self.history(phase == 'offline')
            after = self.history(phase in ('now', 'offline'))
            observations = iter([before, after])
            native = SimpleNamespace(history=lambda: events.append('history') or next(observations),
                publication=lambda *args: events.append('publication'),
                profile=lambda: events.append('profile') or profile_observation(),
                start_and_close=lambda *args: events.append('actual-start-ui-close'))
            report = {}
            driver.run_phase(prepared, phase, before['selectedHeadSha256'], None if phase == 'offline' else 'f' * 64, native, report)
            self.assertEqual(report['status'], 'passed')
            self.assertEqual(events, ['history', 'publication', 'profile', 'actual-start-ui-close', 'history', 'profile', 'publication'])

    def test_unknown_head_pending_diagnostic_ui_failure_profile_mutation_never_pass(self):
        prepared = self.prepared()
        for fault in ('head', 'diagnostic', 'ui', 'profile', 'now-health', 'now-parent'):
            events, report = [], {'status': 'refused'}
            before, after = self.history(), self.history(True)
            if fault == 'now-health':
                after['qualificationSha256'] = before['qualificationSha256']
            if fault == 'now-parent':
                after['ancestors'] = []
            observations = iter([before, after])
            profiles = iter([profile_observation(), profile_observation('b' if fault == 'profile' else 'a')])
            def history():
                events.append('history')
                if fault == 'diagnostic':
                    raise ValueError('pending/refused native history')
                return next(observations)
            def launch(*unused):
                events.append('start')
                if fault == 'ui':
                    raise ValueError('actual process/window uncertainty')
            native = SimpleNamespace(history=history, publication=lambda *args: None, profile=lambda: next(profiles), start_and_close=launch)
            with self.subTest(fault=fault), self.assertRaises(ValueError):
                driver.run_phase(prepared, 'now', ('0' if fault == 'head' else '3') * 64, 'f' * 64, native, report)
            self.assertEqual(report['status'], 'refused')
            if fault in ('head', 'diagnostic'):
                self.assertNotIn('start', events)
            self.assertLessEqual(events.count('start'), 1)

    def test_diagnostic_pages_bind_request_original_inventory_and_head(self):
        calls = []
        def run(command, **unused):
            calls.append(command)
            result = self.history()
            result['requestId'] = next(a.split('=', 1)[1] for a in command if a.startswith('--request-id='))
            if len(calls) == 1:
                result['nextPage'] = 1
            else:
                result['page'] = 1
                result['ancestors'] = [dict(generation='e' * 32, completionSha256='f' * 64)]
            return SimpleNamespace(returncode=0, stdout=raw(result), stderr=b'')
        result = driver.diagnostic(Path('/inert/provisioner.exe'), run=run)
        self.assertEqual(len(calls), 2)
        self.assertEqual(len(result['ancestors']), 1)
        self.assertIn('--expected-selected-head-sha256=' + '3' * 64, calls[1])
        self.assertIn('--inventory-sha256=' + '9' * 64, calls[1])
        def mismatch(command, **unused):
            result = self.history()
            return SimpleNamespace(returncode=0, stdout=raw(result), stderr=b'')
        with self.assertRaises(ValueError):
            driver.diagnostic(Path('/inert/provisioner.exe'), run=mismatch)

    def test_manual_rollback_actual_command_and_strict_one_shot_output(self):
        before, target = self.history(True), self.history()
        calls = []
        def popen(command, **streams):
            calls.append(command)
            self.assertEqual(streams['stdin'], driver.subprocess.DEVNULL)
            self.assertEqual(command[:2], ['/inert/provisioner.exe', '--maintenance=manual-rollback'])
            self.assertEqual(command[3:], ['--expected-selected-head-sha256=' + '4' * 64,
                '--target-generation=' + 'c' * 32, '--target-completion-sha256=' + '1' * 64])
            request = command[2].removeprefix('--request-id=')
            driver.pin(request, 32)
            streams['stdout'].write(raw(dict(schema=1, requestId=request, state='manual-selected', generation='c' * 32,
                completionSha256='1' * 64, selectedHeadSha256='b' * 64)))
            return SimpleNamespace(wait=lambda timeout: self.assertEqual(timeout, 1800) or 0)
        result = driver.manual_rollback(Path('/inert/provisioner.exe'), before, target, popen=popen)
        self.assertEqual(result['selectedHeadSha256'], 'b' * 64)
        self.assertEqual(len(calls), 1)

    def test_manual_rollback_wrong_target_output_and_timeout_never_retry(self):
        before, target = self.history(True), self.history()
        faults = ('schema', 'requestId', 'state', 'generation', 'completionSha256', 'same-head', 'bad-head',
                  'extra', 'duplicate', 'oversized', 'stderr', 'exit', 'timeout')
        for fault in faults:
            calls = []
            def popen(command, **streams):
                calls.append(command)
                value = dict(schema=1, requestId=command[2].split('=', 1)[1], state='manual-selected',
                             generation='c' * 32, completionSha256='1' * 64, selectedHeadSha256='b' * 64)
                if fault == 'schema': value['schema'] = True
                elif fault in ('requestId', 'state', 'generation', 'completionSha256'): value[fault] = 'wrong'
                elif fault == 'same-head': value['selectedHeadSha256'] = before['selectedHeadSha256']
                elif fault == 'bad-head': value['selectedHeadSha256'] = 'not-a-head'
                elif fault == 'extra': value['untrusted'] = True
                data = raw(value)
                if fault == 'duplicate': data = data[:-1] + b',"state":"manual-selected"}'
                if fault == 'oversized': data += b' ' * 4097
                streams['stdout'].write(data)
                if fault == 'stderr': streams['stderr'].write(b'uncertain')
                def wait(timeout):
                    if fault == 'timeout': raise driver.subprocess.TimeoutExpired(command, timeout)
                    return 1 if fault == 'exit' else 0
                return SimpleNamespace(wait=wait, kill=lambda: self.fail('Never kill an uncertain operator'),
                                       terminate=lambda: self.fail('Never terminate an uncertain operator'))
            with self.subTest(fault=fault), self.assertRaises((ValueError, driver.subprocess.TimeoutExpired)):
                driver.manual_rollback(Path('/inert/provisioner.exe'), before, target, popen=popen)
            self.assertEqual(len(calls), 1)
        for mutation in ('generation', 'completion'):
            wrong = copy.deepcopy(target)
            wrong['generation' if mutation == 'generation' else 'completionSha256'] = 'f' * (32 if mutation == 'generation' else 64)
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                driver.manual_rollback(Path('/inert/provisioner.exe'), before, wrong,
                    popen=lambda *args, **kwargs: self.fail('Wrong ancestor must refuse before request'))

    def test_manual_rollback_observes_exact_original_b0_then_explicit_restored_startup(self):
        target, before = self.history(), self.history(True)
        selected = dict(target, selectedHeadSha256='b' * 64)
        events, observations = [], iter((before, selected, selected))
        native = SimpleNamespace(history=lambda: next(observations),
            publication=lambda *args: events.append(('publication', *args)), profile=profile_observation,
            manual_rollback=lambda *args: events.append('request') or {'selectedHeadSha256': 'b' * 64},
            start_restored_b0_and_close=lambda value: events.append(('restored-b0', value)))
        report = {}
        driver.run_manual_rollback(self.prepared(), before['selectedHeadSha256'], target, native, report)
        self.assertEqual(report['status'], 'passed')
        self.assertEqual(events, [('publication', 'offline', None), 'request', ('restored-b0', selected),
                                  ('publication', 'offline', None)])
        self.assertEqual(report['after']['completionSha256'], target['completionSha256'])

    def test_manual_rollback_wrong_head_history_or_profile_cannot_launch_or_pass(self):
        for fault in ('target', 'head', 'ancestor', 'completion', 'qualification', 'result-head', 'old-head',
                      'profile-before-start', 'profile-after-start', 'after-start-head'):
            target, before = self.history(), self.history(True)
            selected = dict(target, selectedHeadSha256='b' * 64)
            after = dict(selected)
            if fault == 'target': target['generation'] = 'f' * 32
            if fault == 'head': before['selectedHeadSha256'] = 'f' * 64
            if fault == 'ancestor': before['ancestors'] = []
            if fault == 'completion': selected['completionSha256'] = 'f' * 64
            if fault == 'qualification': selected['qualificationClosedSha256'] = 'f' * 64
            if fault == 'result-head': selected['selectedHeadSha256'] = 'f' * 64
            if fault == 'old-head': selected['selectedHeadSha256'] = target['selectedHeadSha256']
            if fault == 'after-start-head': after['selectedHeadSha256'] = 'f' * 64
            histories, events = iter((before, selected, after)), []
            profiles = iter((profile_observation(),
                profile_observation('f' if fault == 'profile-before-start' else 'a'),
                profile_observation('f' if fault == 'profile-after-start' else 'a')))
            native = SimpleNamespace(history=lambda: next(histories), publication=lambda *args: None,
                profile=lambda: next(profiles), manual_rollback=lambda *args: events.append('request') or
                    {'selectedHeadSha256': target['selectedHeadSha256'] if fault == 'old-head' else 'b' * 64},
                start_restored_b0_and_close=lambda *args: events.append('restored-startup'))
            report = {'status': 'refused'}
            with self.subTest(fault=fault), self.assertRaises(ValueError):
                driver.run_manual_rollback(self.prepared(), '4' * 64, target, native, report)
            self.assertEqual(report['status'], 'refused')
            if fault not in ('profile-after-start', 'after-start-head'):
                self.assertNotIn('restored-startup', events)
            self.assertLessEqual(events.count('request'), 1)

    def test_offline_requires_connection_failure_not_http_tls_or_malformed_success(self):
        prepared = self.prepared()
        for error, accepted in ((ConnectionRefusedError(), True), (urllib.error.URLError(TimeoutError()), True),
                (urllib.error.URLError(ssl.SSLError()), False),
                (urllib.error.HTTPError('https://selected.example.test', 503, 'unavailable', None, None), False)):
            def fail(url):
                raise error
            if accepted:
                driver.publication(prepared, 'offline', None, get=fail)
            else:
                with self.assertRaises(ValueError):
                    driver.publication(prepared, 'offline', None, get=fail)
        with self.assertRaises(ValueError):
            driver.publication(prepared, 'offline', None, get=lambda _: b'invalid')

    def test_online_publication_exact_timestamp_graph_and_phase_policy(self):
        prepared = self.prepared()
        for phase, selected in (('no-update', 'b0'), ('later', 'b1'), ('now', 'b1')):
            policy = prepared[selected]['policy']
            targets = raw({'signed': {'targets': {'releases/beta.json': {
                'length': prepared[selected]['paths']['policy'].stat().st_size,
                'hashes': {'sha256': policy['sha256']}}}}})
            snapshot = raw({'signed': {'meta': {'targets.json': {'version': 2,
                'length': len(targets), 'hashes': {'sha256': digest(targets)}}}}})
            timestamp = raw({'signed': {'meta': {'snapshot.json': {'version': 3,
                'length': len(snapshot), 'hashes': {'sha256': digest(snapshot)}}}}})
            records = {'timestamp.json': timestamp, '3.snapshot.json': snapshot, '2.targets.json': targets}
            get = lambda url: records[url.rsplit('/', 1)[1]]
            driver.publication(prepared, phase, digest(timestamp), get=get)
            with self.assertRaises(ValueError):
                driver.publication(prepared, phase, '0' * 64, get=get)
            with self.assertRaises(ValueError):
                driver.publication(prepared, 'now' if phase == 'no-update' else 'no-update', digest(timestamp), get=get)
            records['2.targets.json'] += b' '
            with self.assertRaises(ValueError):
                driver.publication(prepared, phase, digest(timestamp), get=get)

    def test_product_output_contract_and_unknown_capability(self):
        driver.require_product_output_policy({'xnav_hardware_output_policy': 'manual-commissioning',
                                              'xnav_manual_control_contract': 1})
        with self.assertRaises(ValueError):
            driver.require_product_output_policy({'xnav_hardware_output_policy': 'manual-commissioning',
                                                  'xnav_manual_control_contract': 0})

    def test_main_result_close_uncertainty_and_collision_are_refusals(self):
        argv = ['--fixtures', str(self.manifest), '--fixtures-sha256', self.save(),
                '--disposable-opt-in', driver.OPT_IN, '--phase', 'no-update',
                '--expected-selected-head-sha256', '3' * 64, '--timestamp-sha256', 'f' * 64]
        env = dict(GITHUB_ACTIONS='true', RUNNER_ENVIRONMENT='github-hosted', GITHUB_REPOSITORY='ThereptileII/Work')
        for fault in ('none', 'close', 'phase', 'collision'):
            evidence = self.root / fault
            if fault == 'collision':
                evidence.mkdir()
            calls = []
            def close():
                calls.append('close')
                if fault == 'close':
                    raise ValueError('native cleanup uncertainty')
            def phase(*args):
                calls.append('phase')
                if fault == 'phase':
                    raise ValueError('native uncertain original process')
                args[-1]['status'] = 'passed'
            native = SimpleNamespace(close=close)
            with patch.object(driver.sys, 'platform', 'win32'), patch.dict(os.environ, env, clear=True), \
                    patch.object(driver, 'stage_operator', return_value=self.root / 'inert.exe') as stage, \
                    patch.object(driver, 'Native', return_value=native) as constructor, \
                    patch.object(driver, 'run_phase', side_effect=phase), redirect_stderr(io.StringIO()):
                self.assertEqual(driver.main(argv + ['--evidence', str(evidence)]), 0 if fault == 'none' else 1)
                if fault == 'collision':
                    stage.assert_not_called()
                    constructor.assert_not_called()
                    self.assertEqual(calls, [])
                else:
                    report = json.loads((evidence / 'result.json').read_bytes())
                    self.assertEqual(report['nativeInstalledPhaseAcceptance'], fault == 'none')
                    self.assertFalse(report['nativeProductAcceptance'])
                    self.assertEqual(report['status'], 'passed' if fault == 'none' else 'refused')
                    self.assertEqual(calls.count('phase'), 1)

    def test_cli_refuses_non_disposable_duplicate_unknown_and_missing_pins_before_side_effects(self):
        argv = ['--fixtures', str(self.manifest), '--fixtures-sha256', self.save(), '--evidence', str(self.root / 'evidence'),
                '--disposable-opt-in', driver.OPT_IN, '--phase', 'no-update', '--expected-selected-head-sha256', '3' * 64,
                '--timestamp-sha256', 'f' * 64]
        for args, platform, env in ((argv, 'linux', {}), (argv, 'win32', {}),
                (argv + ['--phase', 'now'], 'win32', {}),
                (argv + ['--unknown', 'PRIVATE-MUST-NOT-ECHO'], 'win32', {})):
            with patch.object(driver.sys, 'platform', platform), patch.dict(os.environ, env, clear=True), \
                    patch.object(driver, 'prepare') as prepare, patch.object(driver, 'Native') as native, \
                    redirect_stderr(io.StringIO()) as err:
                self.assertEqual(driver.main(args), 1)
                prepare.assert_not_called()
                native.assert_not_called()
                self.assertEqual(err.getvalue(), driver.FAILURE + '\n')
                self.assertFalse((self.root / 'evidence').exists())


class NavigationTests(unittest.TestCase):
    XML = b'''<gpx xmlns="http://www.topografix.com/GPX/1/1" xmlns:opencpn="http://www.opencpn.org" version="1.1">
      <wpt lat="50" lon="10"><name>Waypoint</name><desc>Keep &amp; preserve</desc><extensions><opencpn:guid>w</opencpn:guid></extensions></wpt>
      <rte><name>Route</name><desc>Original route</desc><extensions><opencpn:guid>r</opencpn:guid></extensions>
        <rtept lat="51" lon="11"><name>P1</name><time>2026-10-13T12:00:00Z</time><extensions><opencpn:guid>p1</opencpn:guid></extensions></rtept>
        <rtept lat="52" lon="12"><name>P2</name><extensions><opencpn:guid>p2</opencpn:guid></extensions></rtept>
      </rte>
      <trk><name>Track</name><extensions><opencpn:guid>t</opencpn:guid></extensions><trkseg>
        <trkpt lat="53" lon="13"><time>2026-10-13T12:02:00Z</time></trkpt>
        <trkpt lat="54" lon="14"><time>2026-10-13T12:03:00Z</time></trkpt>
      </trkseg></trk>
    </gpx>'''

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.xml = self.root / 'navobj.xml'
        self.xml.write_bytes(self.XML)
        self.db = self.root / 'navobj.db'

    def make_database(self, controlled=False):
        # Inert schema/rows use the supported upstream tables, GUID links and
        # point_order conventions; no application or converter is run here.
        with closing(sqlite3.connect(self.db)) as db, db:
            # Verbatim CreateTables SQL from pinned navobj_db.cpp:67-174.
            db.executescript('''
        CREATE TABLE IF NOT EXISTS tracks (
            guid TEXT PRIMARY KEY NOT NULL,
            name TEXT,
            description TEXT,
            visibility INTEGER,
            start_string TEXT,
            end_string TEXT,
            width INTEGER,
            style INTEGER,
            color TEXT,
            created_at DATETIME DEFAULT CURRENT_TIMESTAMP
        );

        CREATE TABLE IF NOT EXISTS trk_points (
            track_guid TEXT NOT NULL,
            latitude REAL NOT NULL,
            longitude REAL NOT NULL,
            timestamp TEXT NOT NULL,
            point_order INTEGER,
            FOREIGN KEY (track_guid) REFERENCES tracks(guid) ON DELETE CASCADE
        );


        CREATE TABLE IF NOT EXISTS track_html_links (
            guid TEXT PRIMARY KEY,
            track_guid TEXT NOT NULL,
            html_link TEXT,
            html_description TEXT,
            html_type TEXT,
            FOREIGN KEY (track_guid) REFERENCES tracks(guid) ON DELETE CASCADE
        );


        CREATE TABLE IF NOT EXISTS routes (
            guid TEXT PRIMARY KEY NOT NULL,
            name TEXT,
            start_string TEXT,
            end_string TEXT,
            description TEXT,
            planned_departure TEXT,
            plan_speed REAL,
            time_format TEXT,
            style INTEGER,
            width INTEGER,
            color TEXT,
            visibility INTEGER,
            shared_wp_viz INTEGER,
            created_at DATETIME DEFAULT CURRENT_TIMESTAMP
        );


        CREATE TABLE IF NOT EXISTS routepoints (
            guid TEXT PRIMARY KEY NOT NULL,
            lat REAL,
            lon REAL,
            Symbol TEXT,
            Name TEXT,
            description TEXT,
            TideStation TEXT,
            plan_speed REAL,
            etd INTEGER,
            Type TEXT,
            Time TEXT,
            ArrivalRadius REAL,
            RangeRingsNumber INTEGER,
            RangeRingsStep REAL,
            RangeRingsStepUnits INTEGER,
            RangeRingsVisible INTEGER,
            RangeRingsColour TEXT,
            ScaleMin INTEGER,
            ScaleMax INTEGER,
            UseScale INTEGER,
            visibility INTEGER,
            viz_name INTEGER,
            shared INTEGER,
            isolated INTEGER,
            created_at DATETIME DEFAULT CURRENT_TIMESTAMP
        );

        CREATE TABLE IF NOT EXISTS routepoints_link (
            route_guid TEXT,
            point_guid TEXT,
            point_order INTEGER,
            PRIMARY KEY (route_guid, point_order),
            FOREIGN KEY (route_guid) REFERENCES routes(guid) ON DELETE CASCADE
        );

        CREATE TABLE IF NOT EXISTS route_html_links (
            guid TEXT PRIMARY KEY,
            route_guid TEXT NOT NULL,
            html_link TEXT,
            html_description TEXT,
            html_type TEXT,
            FOREIGN KEY (route_guid) REFERENCES routes(guid) ON DELETE CASCADE
        );

        CREATE TABLE IF NOT EXISTS routepoint_html_links (
            guid TEXT PRIMARY KEY,
            routepoint_guid TEXT NOT NULL,
            html_link TEXT,
            html_description TEXT,
            html_type TEXT,
            FOREIGN KEY (routepoint_guid) REFERENCES routepoints(guid) ON DELETE CASCADE
        );

                    ''')
            if controlled:
                guids = ['ae43f488-caa5-445b-88e3-0112875a347' + str(n) for n in range(1, 6)]
                points = [(guids[0], 56.7, 12.6, 'SIMULATED persistence mark', '', '2026-01-01T11:00:00Z', 'diamond', 1, 1),
                          (guids[2], 56.71, 12.61, 'SIM start', '', '2026-01-01T11:01:00Z', 'diamond', 1, 0),
                          (guids[3], 56.72, 12.63, 'SIM finish', '', '2026-01-01T11:02:00Z', 'diamond', 1, 0)]
                route = (guids[1], 'SIMULATED persistence route', '', 1, 5)
                links = [(guids[1], guids[2], 1), (guids[1], guids[3], 2)]
                track = (guids[4], 'SIMULATED persistence track', '', 1)
                trackpoints = [(guids[4], 56.68, 12.6, '2026-01-01T12:00:00Z', 0),
                               (guids[4], 56.69, 12.61, '2026-01-01T12:01:00Z', 1)]
            else:
                points = [('w', 50, 10, 'Waypoint', 'Keep & preserve', '', 'circle', 1, 0),
                          ('p1', 51, 11, 'P1', '', '2026-10-13T13:00:00+01:00', 'circle', 1, 0),
                          ('p2', 52, 12, 'P2', '', '', 'circle', 1, 0)]
                route = ('r', 'Route', 'Original route', 1, 5)
                links = [('r', 'p1', 1), ('r', 'p2', 2)]
                track = ('t', 'Track', '', 1)
                trackpoints = [('t', 53, 13, '2026-10-13T12:02:00Z', 0), ('t', 54, 14, '2026-10-13T12:03:00Z', 1)]
            db.executemany('INSERT INTO routepoints(guid,lat,lon,Name,description,Time,Symbol,visibility,shared) VALUES(?,?,?,?,?,?,?,?,?)', points)
            db.execute('INSERT INTO routes(guid,name,description,visibility,plan_speed) VALUES(?,?,?,?,?)', route)
            db.executemany('INSERT INTO routepoints_link VALUES(?,?,?)', links)
            db.execute('INSERT INTO tracks(guid,name,description,visibility) VALUES(?,?,?,?)', track)
            db.executemany('INSERT INTO trk_points VALUES(?,?,?,?,?)', trackpoints)
            # Native LoadAllRoutes/LoadAllTracks do not tolerate NULL TEXT
            # payloads even in columns whose SQL declaration permits NULL.
            for table, columns in (('routepoints', ('TideStation', 'RangeRingsColour')),
                                   ('routes', ('start_string', 'end_string', 'time_format', 'color')),
                                   ('tracks', ('start_string', 'end_string', 'color'))):
                for column in columns:
                    db.execute('UPDATE ' + table + ' SET ' + column + "='' WHERE " + column + ' IS NULL')
            db.execute("UPDATE routepoints SET Type='type'")
            db.execute("UPDATE routes SET planned_departure='-1'")

    def observe(self):
        return dict(driver.navigation_observation(self.root), settingsSha256=['a' * 64])

    def mutate(self, sql):
        with closing(sqlite3.connect(self.db)) as db, db:
            db.execute(sql)

    def test_arbitrary_xml_migration_refuses_even_when_projected_navigation_matches(self):
        before = self.observe()
        expected = {'points': {'w': ['Waypoint', 'Keep & preserve', 50.0, 10.0, ''],
                               'p1': ['P1', '', 51.0, 11.0, '2026-10-13T12:00:00+00:00'],
                               'p2': ['P2', '', 52.0, 12.0, '']},
                    'routes': {'r': ['Route', 'Original route', ['p1', 'p2']]},
                    'tracks': {'t': ['Track', '', [[53.0, 13.0, '2026-10-13T12:02:00+00:00'],
                                                 [54.0, 14.0, '2026-10-13T12:03:00+00:00']]]}}
        self.assertEqual(before['navigationSha256'], driver._nav_digest(expected))
        self.make_database()
        self.xml.unlink()  # Actual migration removes the source XML.
        files = {p.name: p.read_bytes() for p in self.root.iterdir()}
        after = self.observe()
        self.assertFalse(driver.profile_preserved(before, after))
        self.assertEqual(after['navigationStore'], 'sqlite')
        self.assertEqual(files, {p.name: p.read_bytes() for p in self.root.iterdir()})
        self.assertFalse(driver.profile_preserved(after, before))

    def test_exact_controlled_fixture_migration_preserves_represented_properties_read_only(self):
        fixture = Path(__file__).resolve().parents[1] / 'tests/fixtures/mode-persistence.gpx'
        self.xml.write_bytes(fixture.read_bytes().replace(b'\r\n', b'\n'))
        self.assertEqual(digest(self.xml.read_bytes()), driver.CONTROLLED_XML_SHA256)
        before = self.observe()
        self.make_database(controlled=True)
        self.xml.unlink()
        files = {p.name: p.read_bytes() for p in self.root.iterdir()}
        after = self.observe()
        self.assertTrue(driver.profile_preserved(before, after))
        self.assertEqual(before['navigationSha256'], driver.CONTROLLED_NAV_SHA256)
        self.assertEqual(before['navigationMigrationFixtureSha256'], after['navigationMigrationFixtureSha256'])
        self.assertEqual(files, {p.name: p.read_bytes() for p in self.root.iterdir()})

    def test_controlled_windows_crlf_form_is_pinned_without_allowing_other_xml_changes(self):
        fixture = Path(__file__).resolve().parents[1] / 'tests/fixtures/mode-persistence.gpx'
        crlf = fixture.read_bytes().replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
        self.assertEqual(digest(crlf), driver.CONTROLLED_XML_CRLF_SHA256)
        self.xml.write_bytes(crlf)
        before = self.observe()
        self.make_database(controlled=True)
        self.assertTrue(driver.profile_preserved(before, self.observe()))
        self.db.unlink()
        self.xml.write_bytes(crlf + b' ')
        self.assertEqual(self.observe()['navigationMigrationFixtureSha256'], '')

    def test_controlled_migration_refuses_lost_properties_links_or_changed_xml(self):
        fixture = Path(__file__).resolve().parents[1] / 'tests/fixtures/mode-persistence.gpx'
        self.xml.write_bytes(fixture.read_bytes())
        before = self.observe()
        for sql in ("UPDATE routepoints SET Symbol='circle'",
                    "UPDATE routepoints SET shared=0 WHERE guid LIKE '%3471'",
                    "UPDATE routepoints SET visibility=0 WHERE guid LIKE '%3471'",
                    "UPDATE routes SET visibility=0", "UPDATE routes SET plan_speed=6",
                    "UPDATE tracks SET visibility=0",
                    "INSERT INTO route_html_links VALUES('link','ae43f488-caa5-445b-88e3-0112875a3472','https://inert.invalid','new','text')"):
            with self.subTest(sql=sql):
                self.make_database(controlled=True)
                self.mutate(sql)
                self.assertFalse(driver.profile_preserved(before, self.observe()))
                self.db.unlink()
        for xml in (fixture.read_bytes().replace(b'diamond', b'anchor', 1),
                    fixture.read_bytes().replace(b'opencpn:', b'o:').replace(b'xmlns:opencpn=', b'xmlns:o=')):
            self.xml.write_bytes(xml)
            original = self.observe()
            self.assertEqual(original['navigationMigrationFixtureSha256'], '')
            self.make_database(controlled=True)
            self.assertFalse(driver.profile_preserved(original, self.observe()))
            self.db.unlink()

    def test_current_database_wins_over_stale_xml_and_detects_route_point_track_changes(self):
        for sql in ("UPDATE routes SET name='changed'", "UPDATE routepoints SET lat=lat+1 WHERE guid='p1'",
                    "UPDATE routepoints SET description='lost details' WHERE guid='w'",
                    "UPDATE tracks SET name='changed'", "UPDATE trk_points SET longitude=longitude+1 WHERE point_order=1",
                    "DELETE FROM trk_points WHERE point_order=1", "DELETE FROM routepoints_link WHERE point_order=2"):
            with self.subTest(sql=sql):
                self.make_database()
                before = self.observe()
                self.mutate(sql)
                self.assertFalse(driver.profile_preserved(before, self.observe()))
                self.db.unlink()

    def test_same_sqlite_semantics_ignore_file_layout_but_preserve_other_navigation_fields(self):
        self.make_database()
        before = self.observe()
        self.mutate('VACUUM')
        self.assertTrue(driver.profile_preserved(before, self.observe()))
        self.mutate("UPDATE routepoints SET Symbol='changed' WHERE guid='w'")
        self.assertFalse(driver.profile_preserved(before, self.observe()))

    def test_same_database_preserves_properties_and_html_link_rows(self):
        for sql in ("UPDATE tracks SET color='red'", "UPDATE routes SET plan_speed=7",
                    "UPDATE routepoints SET RangeRingsNumber=3",
                    "INSERT INTO routepoint_html_links VALUES('link','w','https://inert.invalid','original link','text')"):
            with self.subTest(sql=sql):
                self.make_database()
                before = self.observe()
                self.mutate(sql)
                after = self.observe()
                self.assertEqual(before['navigationSha256'], after['navigationSha256'])
                self.assertFalse(driver.profile_preserved(before, after))
                self.db.unlink()

    def test_corrupt_unknown_or_orphaned_database_never_falls_back_to_xml(self):
        for sql in ("CREATE VIEW unreviewed AS SELECT * FROM routepoints",
                    "ALTER TABLE routepoints DROP COLUMN TideStation",
                    "CREATE INDEX unreviewed ON routes(name)",
                    "UPDATE routes SET start_string=NULL",
                    "UPDATE routepoints_link SET point_guid='missing' WHERE point_order=1",
                    "UPDATE routepoints_link SET point_order=3 WHERE point_order=2",
                    "UPDATE trk_points SET point_order=0 WHERE point_order=1",
                    "UPDATE trk_points SET track_guid='missing' WHERE point_order=0"):
            with self.subTest(sql=sql):
                self.make_database()
                self.mutate(sql)
                with self.assertRaises((ValueError, sqlite3.DatabaseError)):
                    self.observe()
                self.db.unlink()
        self.db.write_bytes(b'not a sqlite database')
        with self.assertRaises(sqlite3.DatabaseError):
            self.observe()

    def test_inflight_sidecars_symlinks_and_hardlinks_refuse_without_cleanup(self):
        self.make_database()
        for suffix in ('-wal', '-shm', '-journal'):
            sidecar = self.db.with_name(self.db.name + suffix)
            sidecar.write_bytes(b'uncertain transaction')
            with self.subTest(suffix=suffix), self.assertRaises(ValueError):
                self.observe()
            self.assertEqual(sidecar.read_bytes(), b'uncertain transaction')
            sidecar.unlink()
        link = self.root / 'alias.db'
        link.hardlink_to(self.db)
        with self.assertRaises(ValueError):
            self.observe()
        link.unlink()
        self.db.rename(link)
        self.db.symlink_to(link)
        with self.assertRaises(ValueError):
            self.observe()

    def test_xml_entities_missing_guids_nonfinite_coordinates_and_segments_refuse(self):
        for raw_xml in (self.XML.replace(b'<gpx ', b'<!DOCTYPE gpx [<!ENTITY x "bad">]><gpx ', 1),
                        self.XML.replace(b'<opencpn:guid>w</opencpn:guid>', b''),
                        self.XML.replace(b'lat="50"', b'lat="nan"'),
                        self.XML.replace(b'<opencpn:guid>p2</opencpn:guid>', b'<opencpn:guid>p1</opencpn:guid>'),
                        self.XML.replace(b'</trkseg>', b'</trkseg><trkseg/>')):
            self.xml.write_bytes(raw_xml)
            with self.subTest(raw=raw_xml[:40]), self.assertRaises(ValueError):
                self.observe()

    def test_settings_and_unknown_observations_cannot_be_hidden_by_navigation_migration(self):
        before = self.observe()
        after = dict(before, settingsSha256=['b' * 64])
        self.assertFalse(driver.profile_preserved(before, after))
        for field in before:
            bad = dict(before)
            del bad[field]
            with self.subTest(field=field), self.assertRaises(ValueError):
                driver.profile_preserved(before, bad)

    def test_native_observer_uses_the_same_programdata_knownfolder_as_broker(self):
        folders = []
        ui = SimpleNamespace(kernel=object(), declare=lambda *args: None)
        prepared = dict(request={'clientSid': 'S-1-5-21-1-2-3-500'}, provisionerSha256='a' * 64, brokerSha256='b' * 64)
        with patch.object(driver, 'module', return_value=ui), \
                patch.object(driver.Native, 'folder', side_effect=lambda guid: folders.append(guid) or self.root), \
                patch.object(driver.Native, 'actual_sid', return_value=prepared['request']['clientSid']), \
                patch.object(driver.Native, 'pin_image'):
            native = driver.Native(prepared, self.root / 'not-executed.exe')
        self.assertEqual(native.profile_root, self.root / 'opencpn')
        self.assertIn(driver.PROGRAM_DATA, folders)
        self.assertNotIn('3eb685db-65f9-4cf6-a03a-e3ef65729f3d', folders)


class CustomerPhaseLaunchTests(unittest.TestCase):
    """Exercise the actual phase method; replace only native OS observations."""

    def fixture(self, phase, *, refuse_token=None, refuse_close=False):
        import ctypes.wintypes as w
        events = []
        native = object.__new__(driver.Native)
        native.sid, native.profile_root = 'S-1-5-21-1-2-3-1001', Path('/inert-profile')
        generation = 'c' * 32
        product = dict(startup_launcher_sha256='a' * 64, executable_sha256='b' * 64)
        native.p = dict(request={'installed': {'generation': generation}},
                        b0=dict(version='0.4.0-beta2.4', commit='a' * 40, product=product,
                                inventory={'app/skager-update-prompt.exe': 'd' * 64}),
                        b1=dict(version='0.4.0-beta2.5', commit='b' * 40, product=product))
        target = native.p['b1' if phase in ('now', 'offline') else 'b0']
        native.history = lambda: dict(generation=generation, version=target['version'], commit=target['commit'],
            completionSha256='1' * 64, selectedHeadSha256='2' * 64, qualificationSha256='3' * 64,
            qualificationClosedSha256='4' * 64, inventorySha256='5' * 64)
        native.w = w
        native.no_existing_clients = lambda: None
        native.image = lambda generation, name: Path('/inert') / generation / name
        native.pin_image = lambda *args: events.append('pin-image')
        native.process = lambda pid, *args: pid
        native.wait_exit = lambda handle, timeout: 10 if handle == 11 and phase == 'now' else 0
        native.runtime = SimpleNamespace(startup_log=lambda *args: 'before',
                                         wait_startup_ready=lambda *args: events.append('startup-ready'))
        screens = [[]]
        if phase in ('later', 'now'):
            screens.append([(101, 11, 'SKAGER software update')])
        screens.append([(102, 12, 'SKAGER / OpenCPN')])

        def window_owner(window, pointer):
            pointer._obj.value = 11 if window == 101 else 12

        def check_token(handle):
            events.append(('token', handle))
            if handle == refuse_token:
                raise ValueError('inert wrong child token')

        def close():
            events.append('close-owner')
            if refuse_close:
                raise RuntimeError('inert retained owner close failure')

        native.ui = SimpleNamespace(kernel=object(), declare=lambda *args: lambda *args: None,
                                    windows=lambda: screens.pop(0),
                                    children=lambda window: [(201, 'LATER' if phase == 'later' else 'UPDATE NOW')],
                                    IsWindowEnabled=lambda *args: True,
                                    GetWindowThreadProcessId=window_owner,
                                    WaitForSingleObject=lambda *args: 258,
                                    PostMessageW=lambda *args: events.append(('message', args[0], args[1])) or True,
                                    CloseHandle=lambda handle: events.append(('close-handle', handle)) or True)
        owner = SimpleNamespace(poll=lambda: 0, check_process_token=check_token, close=close)
        return native, owner, events, generation

    def test_every_real_phase_uses_fixed_limited_owner_and_checks_children_before_input(self):
        for phase in driver.PHASES:
            with self.subTest(phase=phase):
                native, owner, events, generation = self.fixture(phase)
                with patch.object(driver, 'open_same_account_protected_startup', return_value=owner) as launch, \
                        patch.object(driver.subprocess, 'Popen', side_effect=AssertionError('elevated fallback forbidden')):
                    native.start_and_close(phase, {'generation': generation})
                launch.assert_called_once_with(generation, native.sid)
                self.assertEqual(events[-1], 'close-owner')
                self.assertLess(events.index(('token', 12)), events.index('startup-ready'))
                self.assertLess(events.index(('token', 12)), events.index(('message', 102, 0x0010)))
                if phase in ('later', 'now'):
                    self.assertLess(events.index(('token', 11)), events.index(('message', 201, 0x0201)))

    def test_wrong_prompt_token_prevents_button_input_and_closes_original_owner(self):
        native, owner, events, generation = self.fixture('now', refuse_token=11)
        with patch.object(driver, 'open_same_account_protected_startup', return_value=owner), self.assertRaises(ValueError):
            native.start_and_close('now', {'generation': generation})
        self.assertFalse(any(isinstance(item, tuple) and item[0] == 'message' for item in events))
        self.assertIn(('close-handle', 11), events)
        self.assertEqual(events[-1], 'close-owner')

    def test_restored_b0_is_explicit_real_limited_startup_not_an_offline_b1_phase(self):
        native, owner, events, generation = self.fixture('manual-rollback')
        with patch.object(driver, 'open_same_account_protected_startup', return_value=owner) as launch:
            native.start_restored_b0_and_close(native.history())
        launch.assert_called_once_with(generation, native.sid)
        self.assertLess(events.index(('token', 12)), events.index('startup-ready'))
        self.assertLess(events.index('startup-ready'), events.index(('message', 102, 0x0010)))
        self.assertEqual(events[-1], 'close-owner')

    def test_restored_b0_wrong_target_or_changed_head_refuses_acceptance(self):
        for fault in ('generation', 'version', 'head'):
            native, owner, events, generation = self.fixture('manual-rollback')
            selected = native.history()
            if fault == 'head': selected['selectedHeadSha256'] = 'f' * 64
            else: selected[fault] = 'wrong'
            with patch.object(driver, 'open_same_account_protected_startup', return_value=owner) as launch, \
                    self.subTest(fault=fault), self.assertRaises(ValueError):
                native.start_restored_b0_and_close(selected)
            self.assertNotIn('startup-ready', events)
            if fault == 'head': self.assertEqual(events[-1], 'close-owner')
            else: launch.assert_not_called()

    def test_wrong_application_token_prevents_close_input_and_acceptance(self):
        native, owner, events, generation = self.fixture('no-update', refuse_token=12)
        with patch.object(driver, 'open_same_account_protected_startup', return_value=owner), self.assertRaises(ValueError):
            native.start_and_close('no-update', None)
        self.assertNotIn('startup-ready', events)
        self.assertIn(('close-handle', 12), events)
        self.assertEqual(events[-1], 'close-owner')

    def test_owner_cleanup_failure_is_not_a_passing_phase(self):
        native, owner, events, generation = self.fixture('offline', refuse_close=True)
        with patch.object(driver, 'open_same_account_protected_startup', return_value=owner), self.assertRaises(RuntimeError):
            native.start_and_close('offline', None)
        self.assertEqual(events[-1], 'close-owner')


if __name__ == '__main__':
    unittest.main()

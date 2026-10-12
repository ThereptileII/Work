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
import tempfile
import unittest
import urllib.error
import zipfile
from contextlib import redirect_stderr
from types import SimpleNamespace
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('installed_driver', Path(__file__).with_name('test-installed-protected-updater-windows.py'))
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)


def raw(value):
    return json.dumps(value, separators=(',', ':')).encode()


def digest(value):
    return hashlib.sha256(value).hexdigest()


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
                profile=lambda: events.append('profile') or {'navigationSha256': 'a' * 64},
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
            profiles = iter([{'navigationSha256': 'a' * 64}, {'navigationSha256': ('b' if fault == 'profile' else 'a') * 64}])
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


if __name__ == '__main__':
    unittest.main()

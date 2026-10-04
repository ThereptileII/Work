#!/usr/bin/env python3
"""Inert package/path adapter checks. Never launches an application."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

spec = importlib.util.spec_from_file_location('probe', Path(__file__).with_name('probe-peer-candidate.py'))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)
COMMIT = 'a' * 40


class PreparationTests(unittest.TestCase):
    def fixture(self, root, marker=False, config=None, different_runtime=False,
                recovery_prefix='SKAGER-Beta2-Portable-Recovery/'):
        files = {'app/opencpn.exe': b'inert executable fixture', 'app/plugins/dashboard_pi.dll': b'inert plugin fixture'}
        product = {'commit': COMMIT, 'test_fixtures': False, 'build_purpose': 'INSTALLED PRODUCT',
                   'xnav_hardware_output_policy': 'status-only',
                   'executable_sha256': hashlib.sha256(files['app/opencpn.exe']).hexdigest()}
        files['docs/PRODUCT_BUILD.json'] = json.dumps(product).encode()
        if marker:
            files['app/OPENNAV_PORTABLE_PREVIEW'] = b'marker must never be removed'
        with zipfile.ZipFile(root / 'payload.zip', 'w') as z:
            for name, data in files.items():
                z.writestr(name, data)
        package = {'schema': 1, 'commit': COMMIT, 'payloadSha256': probe.sha(root / 'payload.zip'),
                   'files': [{'path': n, 'sha256': hashlib.sha256(d).hexdigest()} for n, d in files.items()]}
        (root / 'package.json').write_text(json.dumps(package))
        with zipfile.ZipFile(root / 'recovery.zip', 'w') as z:
            for name, data in files.items():
                z.writestr(recovery_prefix + name, b'different' if different_runtime and name == 'app/opencpn.exe' else data)
            if not marker:
                z.writestr(recovery_prefix + 'app/OPENNAV_PORTABLE_PREVIEW', b'original marker')
            z.writestr(recovery_prefix + 'profile/opencpn.conf', config or '[Settings]\r\nConfigVersionString=Version 5.12.4+37fd0cd Build 2026-10-02\r\n')

    def test_preserves_complete_runtime_and_derives_header(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.fixture(root)
            original = probe.sha(root / 'recovery.zip')
            result = probe.prepare(root, root / 'recovery.zip', root / 'out', COMMIT)
            self.assertEqual(result['verified_runtime_files'], 3)
            self.assertEqual((root / 'out/profile-adapter/include/config.h').read_text(),
                             '#define VERSION_FULL "5.12.4+37fd0cd"\n#define VERSION_DATE "2026-10-02"\n')
            self.assertEqual(original, probe.sha(root / 'recovery.zip'))
            self.assertEqual((root / 'out/app/opencpn.exe').read_bytes(), b'inert executable fixture')

    def test_mismatches_and_marker_fail_closed(self):
        cases = [{'marker': True}, {'different_runtime': True}, {'config': '[Settings]\n'},
                 {'config': 'ConfigVersionString=Version 5.12.4 Build 2026-10-02\n' * 2}]
        for options in cases:
            with self.subTest(options=options), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self.fixture(root, **options)
                with self.assertRaises(ValueError):
                    probe.prepare(root, root / 'recovery.zip', root / 'out', COMMIT)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.fixture(root)
            with self.assertRaisesRegex(ValueError, 'candidate mismatch'):
                probe.prepare(root, root / 'recovery.zip', root / 'out', 'b' * 40)
            with (root / 'payload.zip').open('ab') as stream:
                stream.write(b'tampered')
            with self.assertRaisesRegex(ValueError, 'digest mismatch'):
                probe.prepare(root, root / 'recovery.zip', root / 'out', COMMIT)

    def test_previous_product_recovery_identity_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.fixture(root, recovery_prefix='OpenNavX-Beta2-Portable-Recovery/')
            with self.assertRaisesRegex(ValueError, 'recovery marker missing'):
                probe.prepare(root, root / 'recovery.zip', root / 'out', COMMIT)

    def test_unsafe_names(self):
        for name in ('../escape', '/absolute', 'app\\escape', 'app/C:stream', 'app/NUL.dll',
                     'app/trailing.', 'app//double', 'app/./dot'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                probe.safe_name(name)

    def eligibility_fixture(self):
        run = {'head_sha': COMMIT, 'path': '.github/workflows/opennav-baseline.yml',
               'repository': {'full_name': probe.REPO}, 'status': 'in_progress', 'conclusion': None,
               'run_started_at': '2026-10-02T00:00:00Z', 'run_attempt': 1}
        artifact = {'workflow_run': {'id': 123, 'head_sha': COMMIT}, 'expired': False,
                    'created_at': '2026-10-02T01:00:00Z',
                    'name': 'beta2-boat-review-pending-endurance-' + COMMIT}
        jobs = [{'name': 'Native MSVC XNav / Legacy / Safe slice', 'conclusion': None,
                 'steps': [{'name': name, 'status': 'completed', 'conclusion': 'success'}
                           for name in probe.PREREQUISITES]}]
        receipt = {'schema': 1, 'owner': 'OpenNavX.CI.RestartGates.1', 'commit': COMMIT,
                   'runId': '123', 'runAttempt': '1', 'actualBoat': False,
                   **{key: 'success' for key in ('transport', 'maintenance', 'broker', 'prepareArm')}}
        return run, artifact, jobs, receipt

    def test_pending_and_completed_eligibility(self):
        run, artifact, jobs, receipt = self.eligibility_fixture()
        self.assertEqual(probe.eligibility(run, artifact, jobs, receipt, COMMIT, '123'),
                         'pending-endurance-development-only')
        artifact['name'] = 'beta-candidate-' + COMMIT
        with self.assertRaises(ValueError):
            probe.eligibility(run, artifact, jobs, receipt, COMMIT, '123')
        run.update(status='completed', conclusion='success')
        self.assertEqual(probe.eligibility(run, artifact, jobs, receipt, COMMIT, '123'),
                         'candidate-complete-ci')

    def test_eligibility_rejects_failed_missing_stale_or_wrong_identity(self):
        changes = (
            lambda r, a, j, q: r.update(conclusion='failure'),
            lambda r, a, j, q: r.update(conclusion='cancelled'),
            lambda r, a, j, q: r.update(head_sha='b' * 40),
            lambda r, a, j, q: j[0].update(conclusion='failure'),
            lambda r, a, j, q: j[0]['steps'].pop(),
            lambda r, a, j, q: j[0]['steps'][0].update(conclusion='skipped'),
            lambda r, a, j, q: q.update(runAttempt='2'),
            lambda r, a, j, q: q.update(runId='124'),
            lambda r, a, j, q: q.update(prepareArm='failure'),
            lambda r, a, j, q: a['workflow_run'].update(id=124),
            lambda r, a, j, q: a.update(created_at='2026-10-01T00:00:00Z'),
            lambda r, a, j, q: a.update(expired=True),
        )
        for index, change in enumerate(changes):
            with self.subTest(case=index):
                data = copy.deepcopy(self.eligibility_fixture())
                change(*data)
                with self.assertRaises(ValueError):
                    probe.eligibility(*data, COMMIT, '123')

    def test_early_cli_receipt_verification_step_remains_mandatory(self):
        name = 'Installed native peer CLI refuses key changes on the disposable runner'
        for conclusion in ('missing', 'skipped', 'failure'):
            with self.subTest(conclusion=conclusion):
                run, artifact, jobs, receipt = self.eligibility_fixture()
                if conclusion == 'missing':
                    jobs[0]['steps'] = [step for step in jobs[0]['steps'] if step['name'] != name]
                else:
                    next(step for step in jobs[0]['steps'] if step['name'] == name)['conclusion'] = conclusion
                with self.assertRaisesRegex(ValueError, 'required native prerequisite not successful'):
                    probe.eligibility(run, artifact, jobs, receipt, COMMIT, '123')

    def test_case_alias_zip(self):
        with tempfile.TemporaryDirectory() as tmp:
            archive = Path(tmp) / 'case.zip'
            with zipfile.ZipFile(archive, 'w') as z:
                z.writestr('app/file', b'a')
                z.writestr('app/FILE', b'b')
            with zipfile.ZipFile(archive) as z, self.assertRaisesRegex(ValueError, 'case-aliased'):
                probe.members(z)


if __name__ == '__main__':
    unittest.main()

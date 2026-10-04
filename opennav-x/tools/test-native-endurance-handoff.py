#!/usr/bin/env python3
"""Inert transfer/identity tests; no application, network, build or real profile."""
import copy
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest import mock

spec = importlib.util.spec_from_file_location('handoff', Path(__file__).with_name('native-endurance-handoff.py'))
handoff = importlib.util.module_from_spec(spec)
spec.loader.exec_module(handoff)
IDENTITY = handoff.identity_values('ThereptileII/Work', 'a' * 40, '12345', '2')


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        base = Path(self.temp.name)
        self.producer, self.consumer, self.stage = base / 'producer', base / 'consumer', base / 'stage'
        for root in (self.producer, self.consumer):
            for name in handoff.SOURCES:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(('inert source ' + name + '\r\n').encode())
            (root / 'release/qualification.json').write_text(json.dumps({'enduranceSeconds': 10800}))
        exe = self.producer / handoff.EXE
        exe.parent.mkdir(parents=True)
        # Minimal inert PE identity bytes, never executed.
        data = bytearray(128)
        data[:2] = b'MZ'; struct.pack_into('<I', data, 60, 64)
        data[64:70] = b'PE\0\0\x4c\x01'
        exe.write_bytes(data)
        (exe.parent / 'runtime.dll').write_bytes(b'complete runtime\0\r\n')
        (exe.parent / 'plugins/empty').mkdir(parents=True)
        (exe.parent / 'plugins/fixture.dll').write_bytes(b'not executable')
        config = self.producer / handoff.CONFIG
        config.parent.mkdir(parents=True)
        config.write_bytes(b'#define VERSION_FULL "5.12.4"\r\n#define VERSION_DATE "2026-10-04"\r\n')

    def prepared(self):
        return handoff.prepare(self.producer, self.stage, IDENTITY)

    def report(self, manifest):
        return {'status': 'passed', 'authority': 'native Windows', 'release_duration': True,
                'requested_seconds': 10800, 'elapsed_seconds': 10800.125,
                'harness_commit': IDENTITY['commit'], 'build_commit': IDENTITY['commit'],
                'binary_matches_harness_commit': True,
                'harness_sha256': manifest['sources']['tools/soak-runtime.py']['sha256'],
                'executable': {'path': str(self.consumer / handoff.EXE),
                               **manifest['files'][handoff.EXE]}}

    def test_round_trip_preserves_all_bytes_empty_directories_and_result(self):
        manifest = self.prepared()
        handoff.verify(self.consumer, self.stage, IDENTITY)
        for name in manifest['files']:
            self.assertEqual((self.producer / name).read_bytes(), (self.consumer / name).read_bytes())
        self.assertTrue((self.consumer / handoff.RUNTIME / 'plugins/empty').is_dir())
        evidence = self.consumer / 'evidence/local/soak'; evidence.mkdir(parents=True)
        report = evidence / 'results.json'; report.write_text(json.dumps(self.report(manifest)))
        output = evidence / 'handoff-qualified.json'
        receipt = handoff.result(self.consumer, self.stage, IDENTITY, report, output)
        self.assertEqual(receipt['report'], handoff.record(report))
        self.assertEqual(receipt['stage_manifest'], handoff.record(self.stage / 'manifest.json'))
        self.assertTrue(receipt['runtime_unchanged'])
        self.assertEqual(receipt['elapsed_seconds'], 10800.125)
        self.assertEqual(handoff.read_json(output), receipt)

    def test_old_attempt_wrong_commit_source_or_duration_rejected_before_install(self):
        self.prepared()
        for key, value in [('run_attempt', '1'), ('run_id', '999'), ('commit', 'b' * 40),
                           ('repository', 'other/repo'), ('producer_job', 'other'), ('architecture', 'x64')]:
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'identity'):
                handoff.verify(self.consumer, self.stage, {**IDENTITY, key: value})
        source = self.consumer / 'tools/chart-render-check.py'; original = source.read_bytes()
        source.write_bytes(b'changed tool')
        with self.assertRaisesRegex(ValueError, 'source mismatch'):
            handoff.verify(self.consumer, self.stage, IDENTITY)
        source.write_bytes(original)
        (self.consumer / 'release/qualification.json').write_text('{"enduranceSeconds":12000}')
        with self.assertRaises(ValueError):
            handoff.verify(self.consumer, self.stage, IDENTITY)
        self.assertFalse((self.consumer / handoff.RUNTIME).exists())

    def test_artifact_empty_directory_loss_is_restored_without_synthesizing_files(self):
        manifest = self.prepared()
        empty = self.stage / 'payload' / handoff.RUNTIME / 'plugins/empty'
        empty.rmdir()  # upload-artifact stores files, not empty directory entries.
        handoff.verify(self.consumer, self.stage, IDENTITY)
        self.assertTrue(empty.is_dir())
        self.assertTrue((self.consumer / handoff.RUNTIME / 'plugins/empty').is_dir())
        self.assertEqual(handoff.runtime_inventory(self.consumer),
                         (manifest['files'], manifest['directories']))

    def test_manifest_directories_cannot_redirect_or_alias_reconstruction(self):
        manifest = self.prepared()
        path = self.stage / 'manifest.json'
        for name in ('../escape', 'build/elsewhere', 'build/xnav-install/../escape',
                     'build/xnav-install/RUNTIME.DLL', 'build/xnav-install/profiles'):
            bad = copy.deepcopy(manifest)
            bad['directories'] = sorted([*bad['directories'], name])
            path.write_text(json.dumps(bad))
            with self.subTest(name=name), self.assertRaises(ValueError):
                handoff.verify(self.consumer, self.stage, IDENTITY)
        self.assertFalse((self.consumer / 'build').exists())

    def test_manifest_file_case_alias_is_rejected_before_install(self):
        manifest = self.prepared()
        name = handoff.RUNTIME + '/runtime.dll'
        manifest['files'][handoff.RUNTIME + '/RUNTIME.DLL'] = manifest['files'][name].copy()
        (self.stage / 'manifest.json').write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, 'Case alias'):
            handoff.verify(self.consumer, self.stage, IDENTITY)
        self.assertFalse((self.consumer / 'build').exists())

    def test_changed_missing_and_unlisted_payload_fail_closed(self):
        self.prepared()
        dll = self.stage / 'payload' / handoff.RUNTIME / 'runtime.dll'
        original = dll.read_bytes()
        dll.write_bytes(b'x' * len(original))
        with self.assertRaisesRegex(ValueError, 'payload changed'):
            handoff.verify(self.consumer, self.stage, IDENTITY)
        dll.unlink()
        with self.assertRaisesRegex(ValueError, 'missing handoff entry'):
            handoff.verify(self.consumer, self.stage, IDENTITY)
        dll.write_bytes(original)
        extra = self.stage / 'payload/build/xnav-windows/unlisted.txt'; extra.write_bytes(b'extra')
        with self.assertRaisesRegex(ValueError, 'Unlisted'):
            handoff.verify(self.consumer, self.stage, IDENTITY)
        extra.unlink()
        (self.stage / 'extra-empty-directory').mkdir()
        with self.assertRaisesRegex(ValueError, 'Unlisted'):
            handoff.verify(self.consumer, self.stage, IDENTITY)
        self.assertFalse((self.consumer / handoff.RUNTIME).exists())

    def test_existing_stage_or_destination_never_overwritten(self):
        self.prepared()
        with self.assertRaisesRegex(ValueError, 'Stage must be new'):
            self.prepared()
        destination = self.consumer / handoff.RUNTIME
        destination.mkdir(parents=True); sentinel = destination / 'user-owned'; sentinel.write_bytes(b'keep')
        with self.assertRaisesRegex(ValueError, 'must be absent'):
            handoff.verify(self.consumer, self.stage, IDENTITY)
        self.assertEqual(sentinel.read_bytes(), b'keep')

    def test_profiles_unsafe_paths_and_nonwin32_inputs_rejected(self):
        runtime = self.producer / handoff.RUNTIME
        forbidden = runtime / 'opencpn.ini'; forbidden.write_bytes(b'profile')
        with self.assertRaisesRegex(ValueError, 'profile entry'):
            self.prepared()
        forbidden.unlink()
        exe = self.producer / handoff.EXE
        data = bytearray(exe.read_bytes()); data[68:70] = b'\x64\x86'; exe.write_bytes(data)
        with self.assertRaisesRegex(ValueError, 'Win32'):
            self.prepared()
        for name in ('../file', '/file', 'build/./file', 'app/NUL.dll', 'app/file:stream',
                     'app/back\\slash', 'app/trailing.', 'app//empty'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                handoff.safe_name(name)

    def test_symlink_input_and_consumer_parent_rejected(self):
        outside = Path(self.temp.name) / 'outside'; outside.mkdir()
        link = self.producer / handoff.RUNTIME / 'redirect'
        try:
            link.symlink_to(outside, target_is_directory=True)
        except OSError as error:
            self.skipTest(f'Symlink rejection coverage NOT RUN on {handoff.sys.platform}: {error}')
        with self.assertRaisesRegex(ValueError, 'Symlink/reparse'):
            self.prepared()
        link.unlink(); self.prepared()
        (self.consumer / 'build').symlink_to(outside, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'Symlink/reparse'):
            handoff.verify(self.consumer, self.stage, IDENTITY)
        self.assertEqual(list(outside.iterdir()), [])

    def test_report_refuses_short_fake_or_mismatched_endurance(self):
        manifest = self.prepared()
        bad_values = {'status': ['running', 'failed', 'observed'],
                      'authority': ['Linux development'], 'release_duration': [False, 1],
                      'requested_seconds': [120, True, 10800.0],
                      'elapsed_seconds': [10799.9, float('nan'), float('inf'), True],
                      'harness_commit': ['b' * 40], 'build_commit': ['b' * 40],
                      'binary_matches_harness_commit': [False, 1], 'harness_sha256': ['0' * 64]}
        for key, values in bad_values.items():
            for value in values:
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    handoff.validate_report({**self.report(manifest), key: value}, manifest, IDENTITY)
        bad = self.report(manifest); bad['requested_seconds'] = 12000
        with self.assertRaisesRegex(ValueError, 'duration'):
            handoff.validate_report(bad, manifest, IDENTITY)
        for key, value in [('bytes', 1), ('sha256', '0' * 64)]:
            bad = self.report(manifest); bad['executable'][key] = value
            with self.assertRaisesRegex(ValueError, 'executable'):
                handoff.validate_report(bad, manifest, IDENTITY)

    def test_post_soak_rehash_refuses_mutation_even_with_passed_report(self):
        manifest = self.prepared(); handoff.verify(self.consumer, self.stage, IDENTITY)
        report = self.consumer / 'report.json'; report.write_text(json.dumps(self.report(manifest)))
        output = self.consumer / 'qualified.json'
        dll = self.consumer / handoff.RUNTIME / 'runtime.dll'; dll.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'runtime changed'):
            handoff.result(self.consumer, self.stage, IDENTITY, report, output)
        self.assertFalse(output.exists())

    def test_duplicate_manifest_keys_and_boolean_sizes_rejected(self):
        self.prepared()
        path = self.stage / 'manifest.json'; original = path.read_text()
        path.write_text(original.rstrip()[:-1] + ',"schema":1}')
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            handoff.verify(self.consumer, self.stage, IDENTITY)
        path.write_text(original); value = json.loads(original)
        value['files'][handoff.EXE]['bytes'] = True; path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, 'size/hash'):
            handoff.verify(self.consumer, self.stage, IDENTITY)

    def test_cli_identity_requires_hosted_windows_exact_job_and_checkout(self):
        env = {'GITHUB_ACTIONS': 'true', 'RUNNER_OS': 'Windows', 'RUNNER_ENVIRONMENT': 'github-hosted',
               'GITHUB_JOB': 'windows-endurance', 'GITHUB_REPOSITORY': handoff.REPOSITORY,
               'GITHUB_SHA': IDENTITY['commit'], 'GITHUB_RUN_ID': '12345', 'GITHUB_RUN_ATTEMPT': '2'}
        with mock.patch.dict(handoff.os.environ, env, clear=True), mock.patch.object(handoff.sys, 'platform', 'win32'), \
                mock.patch.object(handoff.subprocess, 'check_output', return_value=IDENTITY['commit'] + '\n'), \
                mock.patch.object(handoff.subprocess, 'run'):
            self.assertEqual(handoff.ci_identity(self.consumer, 'verify'), IDENTITY)
            with self.assertRaisesRegex(ValueError, 'Wrong native CI job'):
                handoff.ci_identity(self.consumer, 'prepare')
            with mock.patch.object(handoff.subprocess, 'check_output', return_value='b' * 40):
                with self.assertRaisesRegex(ValueError, 'Checkout commit'):
                    handoff.ci_identity(self.consumer, 'result')


if __name__ == '__main__':
    unittest.main()

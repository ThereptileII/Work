#!/usr/bin/env python3
"""Deterministic reconciliation fixtures; Windows-only smoke is explicitly skipped elsewhere."""
from contextlib import contextmanager, redirect_stdout
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import win32_module_observation as api


class ProcessFixture:
    source = 'Deterministic test fixture; not a native observation'

    def __init__(self, exe, dll):
        self.before = {'pid': 123, 'executablePath': str(exe), 'creationTime100ns': 123456}
        self.after = dict(self.before)
        self.rows = [{'path': str(exe), 'baseAddress': 0x10000, 'imageBytes': 4096},
                     {'path': str(dll), 'baseAddress': 0x20000, 'imageBytes': 8192}]
        self.later_rows = [dict(row) for row in self.rows]
        self.identities = 0
        self.snapshots = 0
        self.on_second_snapshot = lambda: None

    def identity(self):
        self.identities += 1
        return dict(self.before if self.identities == 1 else self.after)

    def modules(self, limit):
        self.snapshots += 1
        if self.snapshots == 2:
            self.on_second_snapshot()
            return self.later_rows
        return self.rows


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='module-observation-fixture-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.exe = self.root / 'SKAGER App.exe'
        self.dll = self.root / 'å plugin_pi.dll'
        self.exe.write_bytes(b'fixture executable, never run')
        self.dll.write_bytes(b'fixture module disk bytes, never loaded')
        self.digest = hashlib.sha256(self.exe.read_bytes()).hexdigest()
        self.process = ProcessFixture(self.exe, self.dll)

    def observe(self, **kwargs):
        return api.observe(self.process, self.exe, self.digest, [self.dll], **kwargs)

    def incomplete(self, report, contains):
        self.assertEqual(report['status'], 'observation-incomplete')
        self.assertTrue(any(contains in error for error in report['errors']), report['errors'])
        self.assertFalse(report['loaderClosureAccepted'])
        self.assertFalse(report['mappedImageIdentityEstablished'])

    def test_stable_observation_retains_separate_os_and_disk_evidence(self):
        report = self.observe()
        self.assertEqual(report['status'], 'module-paths-and-disk-files-observed')
        plugin = next(row for row in report['modules'] if row['path'] == str(self.dll))
        self.assertEqual(plugin['baseAddress'], 0x20000)
        self.assertEqual(plugin['imageBytes'], 8192)
        self.assertEqual(plugin['disk']['sha256'], hashlib.sha256(self.dll.read_bytes()).hexdigest())
        self.assertNotEqual(plugin['disk']['bytes'], plugin['imageBytes'])
        self.assertFalse(report['runtimeEnforcement'])
        self.assertFalse(report['loaderClosureAccepted'])
        self.assertFalse(report['mappedImageIdentityEstablished'])
        self.assertEqual(report['observedDiskBytes'], self.exe.stat().st_size + self.dll.stat().st_size)
        self.assertEqual(self.process.identities, 2)
        self.assertEqual(self.process.snapshots, 2)
        self.assertEqual(json.loads(json.dumps(report)), report)

    def test_wrong_executable_stops_before_module_observation(self):
        self.process.before['executablePath'] = str(self.dll)
        self.incomplete(self.observe(), 'executable path differs')
        self.assertEqual(self.process.snapshots, 0)

    def test_wrong_executable_disk_hash_stops_before_module_observation(self):
        self.digest = '0' * 64
        self.incomplete(self.observe(), 'SHA-256 differs')
        self.assertEqual(self.process.snapshots, 0)

    def test_pid_reuse_same_executable_is_incomplete(self):
        self.process.after['creationTime100ns'] += 1
        self.incomplete(self.observe(), 'Process identity changed')

    def test_process_exit_is_incomplete(self):
        original = self.process.identity
        def identity():
            if self.process.identities:
                raise ValueError('Selected process exited')
            return original()
        self.process.identity = identity
        self.incomplete(self.observe(), 'process exited')

    def test_module_path_base_or_size_change_is_incomplete(self):
        for field, value in (('path', str(self.root / 'substitute.dll')), ('baseAddress', 12), ('imageBytes', 22)):
            with self.subTest(field=field):
                self.process = ProcessFixture(self.exe, self.dll)
                self.process.later_rows[1][field] = value
                self.incomplete(self.observe(), 'changed between snapshots')

    def test_missing_required_plugin_and_missing_executable(self):
        self.process.rows = self.process.rows[:1]
        self.process.later_rows = self.process.rows
        self.incomplete(self.observe(), 'Required module absent')
        self.process.rows = self.process.later_rows = [{'path': str(self.dll), 'baseAddress': 123, 'imageBytes': 12}]
        self.incomplete(self.observe(), 'Executable missing')

    def test_access_failure_never_becomes_an_empty_success(self):
        self.process.modules = lambda limit: (_ for _ in ()).throw(OSError('snapshot access denied'))
        self.incomplete(self.observe(), 'access denied')

    def test_missing_module_disk_file_retains_os_path_without_fabricated_hash(self):
        self.dll.unlink()
        report = self.observe()
        self.incomplete(report, 'disk identity unavailable')
        plugin = next(row for row in report['modules'] if row['path'] == str(self.dll))
        self.assertIn('diskError', plugin)
        self.assertNotIn('disk', plugin)

    def test_module_disk_mutation_after_read_is_incomplete(self):
        self.process.on_second_snapshot = lambda: self.dll.write_bytes(b'changed after hashing')
        self.incomplete(self.observe(), 'File changed after disk observation')

    def test_symlink_is_not_followed(self):
        link = self.root / 'alias.dll'
        try:
            link.symlink_to(self.dll)
        except OSError:
            self.skipTest('Symlink creation unavailable')
        self.process.rows[1]['path'] = str(link)
        self.process.later_rows = self.process.rows
        report = self.observe()
        self.incomplete(report, 'disk identity unavailable')
        self.assertTrue(any('Symlink/reparse' in row.get('diskError', '') for row in report['modules']))

    def test_empty_duplicate_and_budget_exhaustion(self):
        self.incomplete(self.observe(max_modules=1), 'budget exceeded')
        self.process = ProcessFixture(self.exe, self.dll)
        self.incomplete(self.observe(max_bytes=self.exe.stat().st_size), 'disk identity unavailable')
        self.process = ProcessFixture(self.exe, self.dll)
        self.process.rows.append(self.process.rows[0])
        self.incomplete(self.observe(), 'Duplicate')
        self.process.rows = []
        self.incomplete(self.observe(), 'Empty module snapshot')

    def test_invalid_expectation_and_limits_rejected(self):
        with self.assertRaisesRegex(ValueError, 'SHA-256'):
            api.observe(self.process, self.exe, 'bad')
        with self.assertRaisesRegex(ValueError, 'Positive'):
            self.observe(max_bytes=-1)

    def test_observer_source_drift_cannot_emit_success(self):
        @contextmanager
        def fixture_process(pid):
            yield self.process
        real_read = api.read_file
        source_reads = {}
        def drifting_read(path, *args):
            result = real_read(path, *args)
            if Path(path).name == 'win32_module_observation.py':
                source_reads[str(path)] = source_reads.get(str(path), 0) + 1
                if source_reads[str(path)] == 2:
                    return result[0], result[1] + b'# changed source', result[2]
            return result
        output = io.StringIO()
        argv = ['observer', '--pid', '123', '--expected-exe', str(self.exe),
                '--expected-sha256', self.digest]
        with patch.object(sys, 'argv', argv), patch.object(api, 'windows_process', fixture_process), \
                patch.object(api, 'read_file', drifting_read), redirect_stdout(output):
            self.assertEqual(api.main(), 1)
        report = json.loads(output.getvalue())
        self.incomplete(report, 'Observer source changed')
        self.assertIn('win32_module_observation.py', report['observer']['sourceSha256'])

    @unittest.skipIf(sys.platform == 'win32', 'Non-Windows refusal only')
    def test_nonwindows_cli_cannot_emit_native_success(self):
        done = subprocess.run([sys.executable, str(Path(api.__file__)), '--pid', '123',
                               '--expected-exe', str(self.exe), '--expected-sha256', self.digest],
                              capture_output=True, text=True, timeout=10)
        self.assertEqual(done.returncode, 1, done.stderr)
        self.incomplete(json.loads(done.stdout), 'Native Windows required')

    @unittest.skipUnless(sys.platform == 'win32', 'Native Windows module API gate remains pending')
    def test_native_observer_process_smoke(self):
        # Only inspect this already-running interpreter; no inspected executable or DLL is launched.
        exe = Path(sys.executable).absolute()
        digest = hashlib.sha256(exe.read_bytes()).hexdigest()
        with api.windows_process(os.getpid()) as process:
            report = api.observe(process, exe, digest)
        self.assertEqual(report['status'], 'module-paths-and-disk-files-observed', report['errors'])
        self.assertEqual(report['processBefore']['pid'], os.getpid())
        self.assertTrue(report['modules'])


if __name__ == '__main__':
    unittest.main()

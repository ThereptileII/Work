#!/usr/bin/env python3
"""Portable timing/fixture policy tests; no NSIS or Windows acceptance claim."""
import ast
from contextlib import contextmanager
import json
import hashlib
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

SOURCE = Path(__file__).with_name('smoke-installer-windows.py')
TREE = ast.parse(SOURCE.read_text())
FUNCTIONS = ast.Module(body=[node for node in TREE.body
                            if isinstance(node, ast.FunctionDef)
                            and node.name in ('engine', 'maintenance', 'installer_fixture')],
                       type_ignores=[])


class Completion(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.result = self.root / 'result.json'
        self.clock = 0.0
        self.publish_at = None
        self.status = 'passed'
        self.report = {'operations': []}
        def sleep(seconds):
            self.clock += seconds
            if self.publish_at is not None and self.clock >= self.publish_at:
                self.result.write_text(json.dumps({'status': self.status}))
        def operation_report(action):
            self.report['operations'].append({'action': action})
            return self.result
        self.environment = dict(
            contextmanager=contextmanager, tempfile=tempfile, shutil=shutil,
            report=self.report, json=json, generation=lambda: self.root,
            operation_report=operation_report, PS=self.root / 'powershell.exe',
            subprocess=SimpleNamespace(list2cmdline=subprocess.list2cmdline,
                                       run=lambda *a, **kw: SimpleNamespace(returncode=0)),
            time=SimpleNamespace(monotonic=lambda: self.clock, sleep=sleep))
        exec(compile(FUNCTIONS, str(SOURCE), 'exec'), self.environment)

    def tearDown(self):
        self.temp.cleanup()

    def test_relocated_cleanup_after_two_minutes_must_finish(self):
        self.publish_at = 153
        result = self.environment['maintenance']('Uninstall')
        self.assertEqual(result['status'], 'passed')
        self.assertGreaterEqual(self.report['operations'][-1]['completion_seconds'], 153)

    def test_bootstrap_exit_without_report_is_not_success(self):
        with self.assertRaisesRegex(AssertionError, '600 seconds'):
            self.environment['maintenance']('Uninstall')
        self.assertLess(self.clock, 601)
        self.assertNotIn('completion_seconds', self.report['operations'][-1])

    def test_failed_durable_result_remains_failure(self):
        self.publish_at, self.status = 153, 'failed'
        with self.assertRaises(AssertionError):
            self.environment['maintenance']('Uninstall')
        self.assertNotIn('completion_seconds', self.report['operations'][-1])

    def test_diagnostics_keeps_its_existing_deadline(self):
        with self.assertRaisesRegex(AssertionError, '120 seconds'):
            self.environment['maintenance']('Diagnostics')
        self.assertLess(self.clock, 121)

    def test_direct_cleanup_after_two_minutes_uses_the_same_bound(self):
        calls=[]
        def run(command, *, timeout, capture_output):
            calls.append(timeout)
            self.assertTrue(capture_output)
            self.assertEqual(command[-4:-2], ['-Action', 'Uninstall'])
            self.assertGreater(timeout, 153)
            self.clock=153
            self.result.write_text('{"status":"passed"}')
            return SimpleNamespace(returncode=0)
        self.environment['subprocess'].run=run
        self.assertEqual(self.environment['engine']('Uninstall')['status'], 'passed')
        self.assertEqual(calls, [600])
        self.assertEqual(self.report['operations'][-1]['completion_seconds'], 153)

    def test_direct_cleanup_timeout_stays_failure_without_retry(self):
        calls=[]
        def run(command, *, timeout, capture_output):
            calls.append(timeout)
            raise subprocess.TimeoutExpired(command, timeout)
        self.environment['subprocess'].run=run
        with self.assertRaises(subprocess.TimeoutExpired):
            self.environment['engine']('Uninstall')
        self.assertEqual(calls, [600])
        self.assertNotIn('completion_seconds', self.report['operations'][-1])

    def test_direct_successful_exit_with_failed_report_is_refused(self):
        self.result.write_text('{"status":"failed"}')
        with self.assertRaises(AssertionError):
            self.environment['engine']('Uninstall')
        self.assertNotIn('completion_seconds', self.report['operations'][-1])

    def test_direct_missing_report_is_not_success(self):
        with self.assertRaises(FileNotFoundError):
            self.environment['engine']('Uninstall')
        self.assertNotIn('completion_seconds', self.report['operations'][-1])

    def test_direct_nonzero_exit_is_refused(self):
        self.result.write_text('{"status":"passed"}')
        self.environment['subprocess'].run=lambda *a, **kw: SimpleNamespace(returncode=1,stdout=b'',stderr=b'failed')
        with self.assertRaises(AssertionError):
            self.environment['engine']('Uninstall')
        self.assertNotIn('completion_seconds', self.report['operations'][-1])

    def test_other_direct_actions_keep_their_existing_deadline(self):
        calls=[]
        def run(command, *, timeout, capture_output):
            calls.append(timeout)
            self.result.write_text('{"status":"passed"}')
            return SimpleNamespace(returncode=0)
        self.environment['subprocess'].run=run
        self.environment['engine']('Diagnostics')
        self.assertEqual(calls, [120])

    def test_failure_preserves_fixture_for_relocated_child(self):
        directory = None
        try:
            with self.assertRaisesRegex(RuntimeError, 'incomplete'):
                with self.environment['installer_fixture']() as directory:
                    Path(directory, 'stock.exe').write_bytes(b'fixture')
                    raise RuntimeError('incomplete')
            self.assertEqual(self.report['retained_failed_fixture'], directory)
            self.assertEqual(Path(directory, 'stock.exe').read_bytes(), b'fixture')
        finally:
            if directory:
                shutil.rmtree(directory)

    def test_success_removes_disposable_fixture(self):
        with self.environment['installer_fixture']() as directory:
            Path(directory, 'stock.exe').write_bytes(b'fixture')
        self.assertFalse(Path(directory).exists())
        self.assertNotIn('retained_failed_fixture', self.report)


class PackagedStartupCompletion(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.profile = self.root / 'profile'; self.profile.mkdir()
        (self.root / 'app').mkdir()
        self.executable = self.root / 'app/opencpn.exe'
        self.executable.write_bytes(b'inert file; never execute')
        self.log = self.profile / 'opencpn.log'
        self.ready = b'------- OpenCPN version fixture\nOnInitTimer...Finalize Canvases\n'
        self.log.write_bytes(self.ready)  # The old completed run is not readiness.
        self.clock = 0.0; self.publish_at = None; self.rotation = False
        self.closed = []; self.exit_waits = []; self.close_failure = False
        self.report = {'operations': []}; self.owned = set()
        def sleep(seconds):
            self.clock += seconds
            if self.publish_at is not None and self.clock >= self.publish_at:
                self.log.write_bytes((b'rotated\n' if self.rotation else self.ready) + self.ready)
        source = SOURCE.with_name('test-packaged-updater-windows.py')
        spec = importlib.util.spec_from_file_location('packaged_completion_fixture', source)
        self.helper = importlib.util.module_from_spec(spec); spec.loader.exec_module(self.helper)
        self.time = SimpleNamespace(monotonic=lambda: self.clock, sleep=sleep)
        self.clock_patch = mock.patch.object(self.helper, 'time', self.time); self.clock_patch.start()
        nested = next(node for node in ast.walk(ast.parse(source.read_text()))
                      if isinstance(node, ast.FunctionDef) and node.name == 'launch_and_close')
        def wait_exit(handle, timeout_ms):
            self.exit_waits.append(timeout_ms)
            if self.close_failure:
                raise RuntimeError('original close failure')
        parent = SimpleNamespace(pid=1, wait=lambda **kw: 0, poll=lambda: 0)
        environment = dict(
            generation=lambda identifier: self.root, profile=self.profile,
            evidence=self.root, report=self.report, owned=self.owned,
            stock=self.root / 'stock/opencpn.exe', stock_before={},
            inventory=lambda path: {}, navigation_matches=lambda: True,
            startup_log=self.helper.startup_log, wait_startup_ready=self.helper.wait_startup_ready,
            _startup=self.helper._startup, hashlib=hashlib, time=self.time,
            process_image=lambda pid: self.executable,
            subprocess=SimpleNamespace(Popen=lambda *args, **kw: parent, STDOUT=subprocess.STDOUT),
            ui=SimpleNamespace(windows=lambda: [], wait_window=lambda *args, **kw: (10, 2),
                               IsWindowEnabled=lambda handle: True, monitor_process=lambda pid: 20,
                               close=lambda handle: self.closed.append(self.clock), wait_clean_exit=wait_exit))
        exec(compile(ast.Module(body=[nested], type_ignores=[]), str(source), 'exec'), environment)
        self.launch = environment['launch_and_close']

    def tearDown(self):
        self.clock_patch.stop(); self.temp.cleanup()

    def test_visible_window_and_exited_launcher_wait_for_fresh_deferred_init(self):
        self.publish_at = 6
        self.launch(['inert'], 'fixture', 'restored-startup', False)
        self.assertEqual(len(self.closed), 1)
        self.assertGreaterEqual(self.closed[0], 6.6)
        self.assertEqual(self.exit_waits, [30000])
        self.assertEqual(self.report['operations'][0]['status'], 'passed')
        self.assertFalse(self.owned)

    def test_rotated_fresh_log_uses_existing_readiness_policy(self):
        self.publish_at = 6; self.rotation = True
        self.launch(['inert'], 'fixture', 'rotated-startup', False)
        self.assertGreaterEqual(self.closed[0], 6.6)
        self.assertEqual(self.exit_waits, [30000])

    def test_stale_ready_log_never_closes_or_claims_startup(self):
        with self.assertRaisesRegex(RuntimeError, 'fresh startup'):
            self.launch(['inert'], 'fixture', 'incomplete-startup', False)
        self.assertFalse(self.closed); self.assertFalse(self.exit_waits)
        self.assertLess(self.clock, 46)
        record = self.report['operations'][0]
        self.assertEqual(record['lastStage'], 'fresh-startup-readiness')
        self.assertFalse(record['startupLog']['freshInitialization'])
        self.assertIn(2, self.owned)  # Existing outer failure cleanup owns child.

    def test_close_failure_stays_failure_with_bounded_actual_log(self):
        self.publish_at = 6; self.close_failure = True
        with self.assertRaisesRegex(RuntimeError, 'original close failure'):
            self.launch(['inert'], 'fixture', 'failed-close', False)
        self.assertEqual(self.exit_waits, [30000]); self.assertEqual(len(self.closed), 1)
        record = self.report['operations'][0]
        self.assertEqual(record['lastStage'], 'normal-close')
        self.assertEqual(record['status'], 'failed')
        self.assertTrue(record['startupLog']['freshInitialization'])
        self.assertEqual((self.root / record['startupLog']['path']).read_bytes(), self.log.read_bytes())
        self.assertEqual(record['startupLog']['sha256'], hashlib.sha256(self.log.read_bytes()).hexdigest())


if __name__ == '__main__':
    unittest.main()

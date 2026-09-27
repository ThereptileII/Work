#!/usr/bin/env python3
"""Portable timing/fixture policy tests; no NSIS or Windows acceptance claim."""
import ast
from contextlib import contextmanager
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest

SOURCE = Path(__file__).with_name('smoke-installer-windows.py')
TREE = ast.parse(SOURCE.read_text())
FUNCTIONS = ast.Module(body=[node for node in TREE.body
                            if isinstance(node, ast.FunctionDef)
                            and node.name in ('maintenance', 'installer_fixture')],
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
            operation_report=operation_report,
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


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3
"""Portable mocked harness checks; these do not run or qualify native Windows."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

SPEC = importlib.util.spec_from_file_location(
    'startup_update_harness', Path(__file__).with_name('test-startup-update-windows.py'))
HARNESS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HARNESS)


def record(path):
    raw = Path(path).read_bytes()
    return {'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest()}


def native_success():
    observation = ' '.join('a' * size for size in (16, 32, 16, 16, 32, 16, 64))
    return '\n'.join(('PASS original CONFIG identity ' + observation,
                      *HARNESS.QUALIFICATION_SUCCESS_LINES)) + '\n'


class QualificationHarnessTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'source'
        self.root.mkdir()
        self.evidence = Path(self.temp.name) / 'evidence'
        for path in HARNESS.fixture_source_paths(self.root):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(path.relative_to(self.root).as_posix() + '\n')
        for name in ('integration/UpdateQualification.h', 'integration/UpdateStartupReceipt.h',
                     'platform/windows/UpdateQualificationNative.h'):
            path = self.root / 'src' / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(name + '\n')
        (self.root / 'tools/windows-wx.lock.json').write_text('{"archives":[]}\n')
        self.commands = []
        self.output = native_success()
        self.native_failure = False
        self.mutate_executable = False
        self.mutate_source = False

    def fake_run(self, command, log, **kwargs):
        command = [str(value) for value in command]
        self.commands.append(command)
        log = Path(log)
        log.parent.mkdir(parents=True, exist_ok=True)
        log.write_text('mocked harness operation; not native evidence\n')
        if command[:2] == ['cmake', '--build']:
            build = Path(command[2]) / 'Release'
            build.mkdir(parents=True, exist_ok=True)
            targets = command[command.index('--target') + 1:command.index('--parallel')]
            for target in targets:
                (build / (target + '.exe')).write_bytes(target.encode())
        elif Path(command[0]).name == HARNESS.QUALIFICATION_TARGET + '.exe':
            log.write_text(self.output)
            if self.mutate_executable:
                Path(command[0]).write_bytes(b'changed executable')
            if self.mutate_source:
                (self.root / 'tests/update_qualification_native_test.cpp').write_bytes(b'drift')
            if self.native_failure:
                raise subprocess.CalledProcessError(1, command)

    def main(self, platform='win32', environment=None):
        api = SimpleNamespace(record=record, run=self.fake_run,
                              fetch=mock.Mock(), stage_native_runtime=lambda *args: {})
        spec = SimpleNamespace(loader=SimpleNamespace(exec_module=lambda module: None))
        env = {'GITHUB_ACTIONS': 'true', 'RUNNER_ENVIRONMENT': 'github-hosted'}
        if environment is not None:
            env = environment
        with (mock.patch.object(HARNESS, 'ROOT', self.root),
              mock.patch.object(HARNESS.sys, 'platform', platform),
              mock.patch.object(HARNESS.sys, 'argv', ['harness', '--evidence', str(self.evidence)]),
              mock.patch.dict(os.environ, env, clear=True),
              mock.patch.object(HARNESS.importlib.util, 'spec_from_file_location', return_value=spec),
              mock.patch.object(HARNESS.importlib.util, 'module_from_spec', return_value=api),
              mock.patch.object(HARNESS.subprocess, 'check_output', return_value='a' * 40 + '\n'),
              mock.patch.object(HARNESS, 'test_prompt') as prompt):
            HARNESS.main()
            return prompt

    def summary(self):
        return json.loads((self.evidence / 'summary.json').read_text())

    def test_builds_and_runs_fixture_once_with_exact_source_closure(self):
        prompt = self.main()
        build = next(command for command in self.commands if command[:2] == ['cmake', '--build'])
        self.assertIn(HARNESS.QUALIFICATION_TARGET, build)
        native = [command for command in self.commands
                  if Path(command[0]).name == HARNESS.QUALIFICATION_TARGET + '.exe']
        self.assertEqual(len(native), 1)
        prompt.assert_called_once()
        summary = self.summary()
        self.assertEqual(summary['status'], 'passed')
        result = summary['qualifiedProfileFixture']
        self.assertEqual(result['status'], 'passed')
        self.assertFalse(result['nativeProductAcceptance'])
        self.assertFalse(summary['nativeProductAcceptance'])
        self.assertEqual([len(value) for value in result['originalConfigObservation']],
                         [16, 32, 16, 16, 32, 16, 64])
        self.assertEqual((self.evidence / result['log']).read_text(), self.output)
        for name in ('tests/startup_update/CMakeLists.txt',
                     'tests/update_qualification_native_test.cpp',
                     'src/application/StartupUpdate.cpp', 'src/integration/UpdateQualification.cpp',
                     'src/integration/UpdateQualification.h',
                     'src/platform/windows/UpdateQualificationNative.cpp',
                     'src/platform/windows/UpdateQualificationNative.h', 'cmake/StartupUpdate.cmake'):
            self.assertEqual(summary['sources'][str(Path(name))], record(self.root / name))

    def test_refuses_non_disposable_host_before_build_or_evidence(self):
        for platform, env in (
            ('linux', {'GITHUB_ACTIONS': 'true', 'RUNNER_ENVIRONMENT': 'github-hosted'}),
            ('win32', {}),
            ('win32', {'GITHUB_ACTIONS': 'true', 'RUNNER_ENVIRONMENT': 'self-hosted'}),
        ):
            with self.subTest(platform=platform, env=env), self.assertRaises(SystemExit):
                self.main(platform, env)
            self.assertFalse(self.evidence.exists())
            self.assertEqual(self.commands, [])

    def test_native_refusal_retains_failure_log_and_failed_summary(self):
        self.output = 'FAIL Refusing existing fixed profile; no existing bytes were touched\n'
        self.native_failure = True
        with self.assertRaises(subprocess.CalledProcessError):
            self.main()
        self.assertEqual(self.summary()['status'], 'failed')
        self.assertEqual(self.summary()['qualifiedProfileFixture']['status'], 'failed')
        self.assertEqual((self.evidence / (HARNESS.QUALIFICATION_TARGET + '.log')).read_text(),
                         self.output)
        self.assertFalse(any(command[0] == 'powershell.exe' for command in self.commands))
        self.assertFalse(any(Path(command[0]).name in ('cmd.exe', 'powershell.exe')
                             for command in self.commands))

    def test_zero_exit_without_complete_actual_result_refuses(self):
        cases = {
            'missing-completion': native_success().replace(HARNESS.QUALIFICATION_SUCCESS_LINES[1], ''),
            'missing-custody': native_success().replace(HARNESS.QUALIFICATION_SUCCESS_LINES[0], ''),
            'missing-config': '\n'.join(HARNESS.QUALIFICATION_SUCCESS_LINES) + '\n',
            'malformed-config': native_success().replace('a' * 64, 'A' * 64),
            'duplicate-config': native_success() + native_success().splitlines()[0] + '\n',
            'reported-failure': native_success() + 'FAIL partial fixture\n',
        }
        for name, output in cases.items():
            with self.subTest(case=name):
                evidence = Path(self.temp.name) / name
                with mock.patch.object(self, 'evidence', evidence):
                    self.output = output
                    with self.assertRaisesRegex(ValueError, 'did not complete'):
                        self.main()
                    self.assertEqual(self.summary()['qualifiedProfileFixture']['status'], 'failed')

    def test_changed_native_executable_refuses(self):
        self.mutate_executable = True
        with self.assertRaisesRegex(ValueError, 'changed during replay'):
            self.main()
        self.assertEqual(self.summary()['status'], 'failed')
        self.assertEqual(self.summary()['qualifiedProfileFixture']['status'], 'failed')

    def test_native_test_source_drift_refuses_final_success(self):
        self.mutate_source = True
        with self.assertRaisesRegex(RuntimeError, 'did not pass'):
            self.main()
        self.assertTrue(self.summary()['sourceDrift'])
        self.assertEqual(self.summary()['status'], 'failed')


if __name__ == '__main__':
    unittest.main()

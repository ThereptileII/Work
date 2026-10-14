"""Portable rejection checks for the native-only inert adapter fixture."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
spec = importlib.util.spec_from_file_location('native_fixture', Path(__file__).with_name('test-installed-updater-coordinator-native-windows.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


class NativeFixtureContractTests(unittest.TestCase):
    def setUp(self):
        self.environment = dict(GITHUB_ACTIONS='true', GITHUB_REPOSITORY='ThereptileII/Work',
                               GITHUB_SHA='a' * 40, GITHUB_RUN_ID='123', GITHUB_RUN_ATTEMPT='2')

    def test_fixture_binds_actual_runner_context_and_explicit_inert_hashes(self):
        record = fixture.fixture_record(self.environment, 'b' * 64)
        self.assertEqual(record['headSha'], self.environment['GITHUB_SHA'])
        self.assertEqual(record['runId'], '123')
        self.assertEqual(record['runAttempt'], '2')
        self.assertEqual(record['requestSha256'], '1' * 64)
        self.assertEqual(record['approvalSha256'], '2' * 64)
        self.assertIn('fixture', record['jobName'])

    def test_missing_foreign_or_noncanonical_context_refuses(self):
        for key, value in (('GITHUB_ACTIONS', 'false'), ('GITHUB_REPOSITORY', 'other/repo'),
                           ('GITHUB_SHA', 'bad'), ('GITHUB_RUN_ID', '01'), ('GITHUB_RUN_ATTEMPT', '0')):
            with self.subTest(key=key):
                with self.assertRaises(ValueError):
                    fixture.fixture_record(dict(self.environment, **{key: value}), 'b' * 64)
        for key in self.environment:
            with self.subTest(missing=key):
                changed = dict(self.environment)
                del changed[key]
                with self.assertRaises((KeyError, ValueError)):
                    fixture.fixture_record(changed, 'b' * 64)

    def test_non_windows_or_wrong_arguments_never_start_native_process(self):
        with patch.object(fixture.os, 'name', 'posix'), \
             patch.object(fixture.coordinator.subprocess, 'Popen') as process:
            self.assertEqual(fixture.main(['helper', 'a' * 64, 'parent', 'evidence', 'sid']), 1)
            self.assertEqual(fixture.main([]), 1)
            process.assert_not_called()


if __name__ == '__main__':
    unittest.main()

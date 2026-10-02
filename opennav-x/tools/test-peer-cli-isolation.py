#!/usr/bin/env python3
"""Focused filesystem checks for the peer CLI test; no product is executed."""
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('peer_cli', Path(__file__).with_name('test-peer-cli.py'))
CLI = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CLI)


class IsolationTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='peer-cli-isolation-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.common, self.runner, self.outside = (self.root / name for name in ('common', 'runner', 'outside'))
        for directory in (self.common, self.runner, self.outside):
            directory.mkdir()
        self.directory = self.common / 'opencpn'
        self.addCleanup(patch.stopall)
        patch.object(CLI, 'common_app_data_dir', return_value=self.common).start()
        patch.dict(os.environ, {'ProgramData': str(self.common), 'RUNNER_TEMP': str(self.runner)}).start()

    def test_absent_directory_seeds_exact_config(self):
        directory, config = CLI.windows_config_target(self.runner)
        self.assertEqual(directory, self.directory)
        self.assertEqual(config.read_bytes(), CLI.initial_config())
        before = CLI.snapshot_config([config])
        CLI.assert_config_unchanged(before, [config])

    def test_existing_profile_is_refused_without_changes(self):
        directory, config = CLI.windows_config_target(self.runner)
        before = config.read_bytes()
        with self.assertRaisesRegex(RuntimeError, 'already exists'):
            CLI.windows_config_target(self.runner)
        self.assertEqual(config.read_bytes(), before)

    def test_changed_config_is_detected(self):
        directory, config = CLI.windows_config_target(self.runner)
        before = CLI.snapshot_config([config])
        config.write_bytes(b'tampered')
        with self.assertRaisesRegex(AssertionError, 'changed'):
            CLI.assert_config_unchanged(before, [config])

    def test_empty_gui_home_directory_is_still_refused(self):
        self.directory.mkdir()
        with self.assertRaisesRegex(RuntimeError, 'already exists'):
            CLI.windows_config_target(self.runner)
        self.assertTrue(self.directory.is_dir())
        self.assertEqual(list(self.directory.iterdir()), [])

    def test_temp_outside_runner_is_refused(self):
        with self.assertRaisesRegex(RuntimeError, 'outside RUNNER_TEMP'):
            CLI.windows_config_target(self.outside)
        self.assertFalse(self.directory.exists())

    def test_mismatched_program_data_is_refused(self):
        with patch.dict(os.environ, {'ProgramData': str(self.outside)}):
            with self.assertRaisesRegex(RuntimeError, 'does not match'):
                CLI.windows_config_target(self.runner)
        self.assertFalse(self.directory.exists())

    def test_symlink_profile_is_refused_without_changes(self):
        try:
            self.directory.symlink_to(self.outside, target_is_directory=True)
        except OSError as error:
            if os.name == 'nt' and getattr(error, 'winerror', None) == 1314:
                self.skipTest('Windows account lacks symlink privilege')
            raise
        with self.assertRaisesRegex(RuntimeError, 'already exists'):
            CLI.windows_config_target(self.runner)
        self.assertTrue(self.directory.is_symlink())
        self.assertEqual(list(self.outside.iterdir()), [])


if __name__ == '__main__':
    unittest.main()

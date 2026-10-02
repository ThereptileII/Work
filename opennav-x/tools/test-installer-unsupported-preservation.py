#!/usr/bin/env python3
"""Filesystem cleanup contracts only; these tests never execute NSIS or OpenCPN."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest import mock

spec = importlib.util.spec_from_file_location('unsupported', Path(__file__).with_name('test-installer-unsupported-candidate.py'))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class PreservationTests(unittest.TestCase):
    def fixture(self, profile):
        profile.mkdir()
        (profile / 'OPENNAV_TEST_PROFILE').write_bytes(probe.MARKER)
        (profile / 'opencpn.ini').write_bytes(b'isolated test settings')

    def test_restores_original_after_success_and_exception(self):
        for throws in (False, True):
            with self.subTest(throws=throws), tempfile.TemporaryDirectory() as raw:
                profile = Path(raw) / 'OpenCPN'; profile.mkdir()
                (profile / 'original.ini').write_bytes(b'original bytes')
                (profile / 'empty-directory').mkdir()
                before = probe.snapshot(profile); report = {}
                try:
                    with probe.preserve_profile(profile, report):
                        self.assertFalse(profile.exists())
                        self.fixture(profile)
                        if throws:
                            raise RuntimeError('injected operation failure')
                except RuntimeError:
                    self.assertTrue(throws)
                self.assertEqual(probe.snapshot(profile), before)
                self.assertEqual(report['profileRestoration'], 'verified')
                self.assertEqual(list(profile.parent.glob('*.scrum22-backup-*')), [])

    def test_original_absence_restored(self):
        with tempfile.TemporaryDirectory() as raw:
            profile = Path(raw) / 'OpenCPN'; report = {}
            with probe.preserve_profile(profile, report):
                self.fixture(profile)
            self.assertFalse(profile.exists())
            self.assertFalse(report['originalProfilePresent'])
            self.assertEqual(report['profileRestoration'], 'verified')

    def test_unowned_replacement_is_quarantined_and_original_restored(self):
        with tempfile.TemporaryDirectory() as raw:
            profile = Path(raw) / 'OpenCPN'; profile.mkdir()
            (profile / 'original.ini').write_bytes(b'original')
            before = probe.snapshot(profile); report = {}
            with probe.preserve_profile(profile, report):
                profile.mkdir(); (profile / 'unexpected.ini').write_bytes(b'do not delete')
            self.assertEqual(probe.snapshot(profile), before)
            self.assertEqual(list(profile.parent.glob('*.scrum22-backup-*')), [])
            self.assertEqual((Path(report['quarantinedProfile']) / 'unexpected.ini').read_bytes(), b'do not delete')
            self.assertEqual(report['profileRestoration'], 'verified')

    def test_failed_preparation_before_marker_still_restores_original(self):
        with tempfile.TemporaryDirectory() as raw:
            profile = Path(raw) / 'opencpn'; profile.mkdir()
            (profile / 'original.ini').write_bytes(b'original')
            before = probe.snapshot(profile); report = {}
            with self.assertRaisesRegex(RuntimeError, 'preparation failed'):
                with probe.preserve_profile(profile, report):
                    profile.mkdir()
                    raise RuntimeError('preparation failed before marker')
            self.assertEqual(probe.snapshot(profile), before)
            self.assertTrue(Path(report['quarantinedProfile']).is_dir())

    def test_malformed_and_ambiguous_reports_cannot_bypass_restoration(self):
        validator = probe.module('test-plugin-download-guard-windows')
        for content in ('{', '{"status":"passed","status":"failed"}'):
            with self.subTest(content=content), tempfile.TemporaryDirectory() as raw:
                profile = Path(raw) / 'opencpn'; profile.mkdir()
                (profile / 'original.ini').write_bytes(b'original')
                before = probe.snapshot(profile); report = {}
                result = Path(raw) / 'result.json'; result.write_text(content)
                with self.assertRaises(ValueError):
                    with probe.preserve_profile(profile, report):
                        self.fixture(profile)
                        validator.read_json(result)
                self.assertEqual(probe.snapshot(profile), before)
                self.assertEqual(report['profileRestoration'], 'verified')

    def test_redirected_fixture_is_quarantined_without_following_or_deleting_it(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw); profile = root / 'opencpn'; profile.mkdir()
            (profile / 'original.ini').write_bytes(b'original')
            target = root / 'untouched'; target.write_bytes(b'outside')
            before = probe.snapshot(profile); report = {}
            with probe.preserve_profile(profile, report):
                self.fixture(profile)
                try:
                    (profile / 'redirect').symlink_to(target)
                except OSError as error:
                    self.skipTest(str(error))
            self.assertEqual(probe.snapshot(profile), before)
            self.assertTrue((Path(report['quarantinedProfile']) / 'redirect').is_symlink())
            self.assertEqual(target.read_bytes(), b'outside')

    def test_unverified_process_termination_retains_original_isolated(self):
        with tempfile.TemporaryDirectory() as raw:
            profile = Path(raw) / 'opencpn'; profile.mkdir()
            (profile / 'original.ini').write_bytes(b'original')
            before = probe.snapshot(profile); report = {}
            with self.assertRaises(probe.ProcessTreeTerminationError):
                with probe.preserve_profile(profile, report):
                    self.fixture(profile)
                    raise probe.ProcessTreeTerminationError('still running')
            self.assertEqual(probe.snapshot(Path(report['originalProfileBackup'])), before)
            self.assertFalse((profile / 'original.ini').exists())
            self.assertIn('unverified', report['profileRestoration'])

    def test_profile_path_refuses_traversal_wrong_root_and_redirect(self):
        with tempfile.TemporaryDirectory() as raw:
            parent = Path(raw) / 'ProgramData'; parent.mkdir()
            self.assertEqual(probe.normal_profile_path(str(parent / 'opencpn'), parent), parent / 'opencpn')
            for value in (str(parent / '..' / 'outside'), str(parent / 'other'),
                          str(parent) + '/./opencpn', str(parent), 'opencpn'):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    probe.normal_profile_path(value, parent)
            target = Path(raw) / 'outside'; target.mkdir()
            try:
                (parent / 'opencpn').symlink_to(target, target_is_directory=True)
            except OSError as error:
                self.skipTest(str(error))
            with self.assertRaisesRegex(ValueError, 'Reparse/symlink'):
                probe.normal_profile_path(str(parent / 'opencpn'), parent)

    def test_timeout_restoration_requires_successful_tree_kill_and_reap(self):
        for kill_result, reap_timeout in ((0, False), (1, False), (0, True)):
            with self.subTest(kill_result=kill_result, reap_timeout=reap_timeout), tempfile.TemporaryDirectory() as raw:
                child = mock.Mock(pid=123)
                child.wait.side_effect = [probe.subprocess.TimeoutExpired('setup', 1),
                    probe.subprocess.TimeoutExpired('setup', 30) if reap_timeout else 1]
                expected = (probe.subprocess.TimeoutExpired if kill_result == 0 and not reap_timeout
                            else probe.ProcessTreeTerminationError)
                report = {}
                with mock.patch.object(probe.subprocess, 'Popen', return_value=child), \
                     mock.patch.object(probe.subprocess, 'run', return_value=mock.Mock(returncode=kill_result)), \
                     self.assertRaises(expected):
                    probe.run(['setup'], Path(raw) / 'setup.log', timeout=1, report=report)
                self.assertEqual(report['processTimeouts'][0]['processTreeTermination'],
                                 'verified' if expected is probe.subprocess.TimeoutExpired else 'unverified')

    def test_snapshots_detect_same_length_tamper_and_empty_directory(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw); data = root / 'opencpn.exe'; data.write_bytes(b'original')
            before = probe.snapshot(root)
            data.write_bytes(b'tampered')
            self.assertNotEqual(probe.snapshot(root), before)
            data.write_bytes(b'original'); (root / 'unexpected-empty').mkdir()
            self.assertNotEqual(probe.snapshot(root), before)

    def test_snapshot_refuses_redirected_files(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw); target = root / 'target'; target.write_bytes(b'outside')
            directory = root / 'profile'; directory.mkdir()
            try:
                (directory / 'redirect').symlink_to(target)
            except OSError as error:
                self.skipTest(str(error))
            with self.assertRaisesRegex(ValueError, 'Reparse/symlink'):
                probe.snapshot(directory)


if __name__ == '__main__':
    unittest.main()

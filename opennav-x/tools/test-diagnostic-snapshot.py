#!/usr/bin/env python3
"""Fresh publication boundaries for restarted chart/workspace checks."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from diagnostic_snapshot import read_json_snapshot

spec = importlib.util.spec_from_file_location(
    'workspace_fixtures', Path(__file__).with_name('workspace-fixtures.py'))
workspace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(workspace)


def publication(epoch, **values):
    return dict(publication_clock='live monotonic clock',
                publication_monotonic_ms=epoch, **values)


class DiagnosticSnapshot(unittest.TestCase):
    def test_ordinary_reads_still_accept_non_publication_json(self):
        path = Mock()
        path.read_text.return_value = '{"ordinary": true}'
        self.assertEqual(read_json_snapshot(path), {'ordinary': True})
        path.read_text.assert_called_once_with(encoding='utf-8')

    def test_waits_for_new_publication_after_missing_or_older_file(self):
        before = publication('100')
        fresh = publication('101', result='new')
        path = Mock()
        path.read_text.side_effect = [FileNotFoundError(), PermissionError(),
                                     json.dumps(publication('99')),
                                     json.dumps(before), json.dumps(fresh)]
        with patch('diagnostic_snapshot.time.sleep') as sleep:
            self.assertEqual(read_json_snapshot(path, after=before), fresh)
        self.assertEqual(path.read_text.call_count, 5)
        self.assertEqual(sleep.call_count, 4)

    def test_first_new_publication_does_not_hide_workspace_failure(self):
        # Real failure's state and nondefault geometry are supplied by the
        # maintained fixture: 2098125 is visible; XNav must suppress it to 2098127.
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory)
            (profile / 'OPENNAV_TEST_PROFILE').touch()
            workspace.seed(profile)
            perspective = (profile / 'opencpn.conf').read_text().split(
                'AUIPerspective=', 1)[1].strip()
        stale = publication('100', perspective=perspective)
        failed = publication('101', perspective=perspective)
        later = publication('102', perspective=perspective.replace(
            'state=2098125;', 'state=2098127;'))
        path = Mock()
        path.read_text.side_effect = [json.dumps(stale), json.dumps(failed),
                                     json.dumps(later)]
        with patch('diagnostic_snapshot.time.sleep'):
            found = read_json_snapshot(path, after=stale)
        with self.assertRaisesRegex(AssertionError, 'Expected temporary XNav hiding'):
            workspace.assert_restored(found['perspective'], suppressed=True)
        self.assertEqual(path.read_text.call_count, 2)
        workspace.assert_restored(later['perspective'], suppressed=True)

    def test_unchanged_publication_times_out(self):
        before = publication('100')
        path = Mock()
        path.read_text.return_value = json.dumps(before)
        with patch('diagnostic_snapshot.time.monotonic', side_effect=[0, 2]), \
                self.assertRaisesRegex(AssertionError, 'did not arrive'):
            read_json_snapshot(path, after=before)
        path.read_text.assert_called_once()

    def test_access_failure_still_raises_at_deadline(self):
        for error in (FileNotFoundError, PermissionError):
            path = Mock()
            path.read_text.side_effect = error()
            with self.subTest(error=error), \
                    patch('diagnostic_snapshot.time.monotonic', side_effect=[0, 2]), \
                    self.assertRaises(error):
                read_json_snapshot(path, after=publication('100'))

    def test_corrupt_json_is_not_retried(self):
        path = Mock()
        path.read_text.side_effect = ['{', json.dumps(publication('101'))]
        with self.assertRaises(json.JSONDecodeError):
            read_json_snapshot(path, after=publication('100'))
        path.read_text.assert_called_once()

    def test_invalid_baseline_and_new_publication_fail_closed(self):
        invalid = [publication(value) for value in
                   (None, 100, True, '', '-1', '1.0', 'NaN', '１', '9' * 20,
                    '9223372036854775808')]
        invalid += [[], {}, dict(publication('101'), publication_clock='recorded session clock')]
        for value in invalid:
            original = copy.deepcopy(value)
            path = Mock()
            with self.subTest(value=value), self.assertRaisesRegex(AssertionError, 'epoch'):
                read_json_snapshot(path, after=value)
            path.read_text.assert_not_called()
            path.read_text.side_effect = [json.dumps(value), json.dumps(publication('102'))]
            with self.subTest(value=value), self.assertRaisesRegex(AssertionError, 'epoch'):
                read_json_snapshot(path, after=publication('100'))
            path.read_text.assert_called_once()
            self.assertEqual(value, original)


if __name__ == '__main__':
    unittest.main()

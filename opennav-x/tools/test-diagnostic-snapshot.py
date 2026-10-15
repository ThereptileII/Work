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


def workspace_publication(epoch, pid, ready, **values):
    return publication(epoch, runtime={
        'test_workspace_readiness': {'process_id': pid, 'dashboard_ready': ready},
        **values})


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


class WorkspaceReadiness(unittest.TestCase):
    def test_waits_for_returned_process_and_completed_phase(self):
        previous = workspace_publication('100', 71, True)
        fresh = workspace_publication('104', 72, True)
        path = Mock()
        path.read_text.side_effect = [json.dumps(value) for value in (
            previous, workspace_publication('102', 71, True),
            workspace_publication('103', 72, False), fresh)]
        with patch('diagnostic_snapshot.time.sleep'):
            self.assertEqual(read_json_snapshot(path, after=previous,
                                               workspace_pid=72), fresh)
        self.assertEqual(path.read_text.call_count, 4)

    def test_reused_pid_still_requires_publication_after_pre_restart_baseline(self):
        previous = workspace_publication('100', 71, True)
        fresh = workspace_publication('101', 71, True)
        path = Mock()
        path.read_text.side_effect = [json.dumps(previous), json.dumps(fresh)]
        with patch('diagnostic_snapshot.time.sleep'):
            self.assertEqual(read_json_snapshot(path, after=previous,
                                               workspace_pid=71), fresh)
        self.assertEqual(path.read_text.call_count, 2)

    def test_first_ready_publication_preserves_workspace_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory)
            (profile / 'OPENNAV_TEST_PROFILE').touch()
            workspace.seed(profile)
            perspective = (profile / 'opencpn.conf').read_text().split(
                'AUIPerspective=', 1)[1].strip()
        pending = workspace_publication('100', 72, False)
        failed = workspace_publication('101', 72, True,
                                       test_workspace_perspective=perspective)
        later = copy.deepcopy(failed)
        later['publication_monotonic_ms'] = '102'
        later['runtime']['test_workspace_perspective'] = perspective.replace(
            'state=2098125;', 'state=2098127;')
        path = Mock()
        path.read_text.side_effect = [json.dumps(pending), json.dumps(failed),
                                     json.dumps(later)]
        with patch('diagnostic_snapshot.time.sleep'):
            found = read_json_snapshot(path, workspace_pid=72)
        with self.assertRaisesRegex(AssertionError, 'Expected temporary XNav hiding'):
            workspace.assert_restored(found['runtime']['test_workspace_perspective'],
                                      suppressed=True)
        self.assertEqual(path.read_text.call_count, 2)

    def test_readiness_does_not_poll_enc_or_rail_contents(self):
        # Content assertions belong to the caller and must fail this exact frame.
        failed = workspace_publication('101', 72, True,
                                       chart={'quilt_members': []},
                                       display={'rail_regions': [{'visible': False}]})
        path = Mock()
        path.read_text.return_value = json.dumps(failed)
        self.assertEqual(read_json_snapshot(path, workspace_pid=72), failed)
        path.read_text.assert_called_once()

    def test_stale_identity_phase_or_epoch_times_out(self):
        before = workspace_publication('100', 72, True)
        for value in (workspace_publication('101', 71, True),
                      workspace_publication('101', 72, False), before):
            path = Mock()
            path.read_text.return_value = json.dumps(value)
            with self.subTest(value=value), \
                    patch('diagnostic_snapshot.time.monotonic', side_effect=[0, 2]), \
                    self.assertRaisesRegex(AssertionError, 'did not arrive for process 72'):
                read_json_snapshot(path, after=before, workspace_pid=72)
            path.read_text.assert_called_once()

    def test_missing_or_malformed_readiness_fails_without_retry(self):
        invalid = [publication('101'), publication('101', runtime={}),
                   publication('101', runtime={'test_workspace_readiness': None})]
        invalid += [workspace_publication('101', pid, True)
                    for pid in (None, True, '72', 0, -1, 4294967296)]
        invalid += [workspace_publication('101', 72, ready)
                    for ready in (None, 0, 1, 'true')]
        for value in invalid:
            path = Mock()
            path.read_text.side_effect = [json.dumps(value),
                json.dumps(workspace_publication('102', 72, True))]
            with self.subTest(value=value), self.assertRaisesRegex(AssertionError, 'readiness'):
                read_json_snapshot(path, workspace_pid=72)
            path.read_text.assert_called_once()

    def test_invalid_requested_process_fails_before_reading(self):
        for pid in (True, '72', 0, -1, 4294967296):
            path = Mock()
            with self.subTest(pid=pid), self.assertRaisesRegex(AssertionError, 'process identity'):
                read_json_snapshot(path, workspace_pid=pid)
            path.read_text.assert_not_called()


if __name__ == '__main__':
    unittest.main()

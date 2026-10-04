#!/usr/bin/env python3
"""Offline provenance/refusal checks for SCRUM-287; never contact live CI."""
from copy import deepcopy
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('readiness', Path(__file__).with_name('wait-native-endurance.py'))
GATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GATE)

ENV = {'GITHUB_REPOSITORY': 'example/product', 'GITHUB_RUN_ID': '123',
       'GITHUB_RUN_ATTEMPT': '2', 'GITHUB_SHA': 'a' * 40, 'GITHUB_EVENT_NAME': 'push'}
NOW = datetime(2026, 10, 4, 3, tzinfo=timezone.utc)


class FakeClock:
    def __init__(self):
        self.value, self.sleeps = 0, []

    def __call__(self):
        return self.value

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.value += seconds


class Metadata:
    def __init__(self):
        self.calls = []
        self.run = {'id': 123, 'run_attempt': 2, 'head_sha': 'a' * 40,
                    'path': GATE.WORKFLOW, 'event': 'push', 'status': 'in_progress',
                    'conclusion': None, 'run_started_at': '2026-10-04T01:00:00Z',
                    'repository': {'id': 45, 'full_name': 'example/product'},
                    'head_repository': {'id': 45, 'full_name': 'example/product'}}
        self.job = {'id': 456, 'run_id': 123, 'run_attempt': 2, 'head_sha': 'a' * 40,
                    'name': GATE.PRODUCER, 'status': 'in_progress', 'conclusion': None,
                    'steps': [{'name': name, 'number': i + 2, 'status': 'completed',
                               'conclusion': 'success'} for i, name in enumerate(GATE.REQUIRED_STEPS)]}
        self.artifact = {'id': 789, 'name': 'native-endurance-runtime-' + 'a' * 40 + '-2',
                         'size_in_bytes': 2048, 'digest': 'sha256:' + 'b' * 64,
                         'expired': False, 'created_at': '2026-10-04T02:00:00Z',
                         'expires_at': '2026-10-11T02:00:00Z',
                         'workflow_run': {'id': 123, 'head_sha': 'a' * 40,
                                          'repository_id': 45, 'head_repository_id': 45}}
        self.jobs, self.artifacts = [self.job], [self.artifact]

    def __call__(self, endpoint, timeout):
        self.calls.append((endpoint, timeout))
        if endpoint.endswith('/runs/123'):
            return deepcopy(self.run)
        if endpoint.endswith('/artifacts/789'):
            return deepcopy(self.artifact)
        for ending, key, values in (('/attempts/2/jobs', 'jobs', self.jobs),
                                    ('/artifacts', 'artifacts', self.artifacts)):
            if ending + '?' in endpoint:
                page = int(endpoint.rsplit('page=', 1)[1])
                return {'total_count': len(values), key: deepcopy(values[(page-1)*100:page*100])}
        raise AssertionError('Unexpected read-only endpoint: ' + endpoint)


class ReadinessTests(unittest.TestCase):
    def wait(self, metadata, **kwargs):
        clock = kwargs.pop('clock', FakeClock())
        return GATE.wait_ready(GATE.context(ENV), api=metadata, timeout=kwargs.pop('timeout', 61),
            clock=clock, sleep=clock.sleep, utcnow=lambda: NOW, **kwargs)

    def test_exact_handoff_and_safe_receipt(self):
        metadata = Metadata()
        metadata.artifact['archive_download_url'] = 'https://secret.invalid/token'
        result = self.wait(metadata)
        self.assertEqual(result['artifact']['id'], 789)
        self.assertEqual(result['artifact']['digest'], 'sha256:' + 'b' * 64)
        self.assertEqual(result['runAttempt'], 2)
        self.assertEqual(len(result['requiredSteps']), len(GATE.REQUIRED_STEPS))
        self.assertNotIn('secret.invalid', json.dumps(result))
        self.assertTrue(any('/attempts/2/jobs?' in p for p, _ in metadata.calls))
        self.assertTrue(all(0 < budget <= 30 for _, budget in metadata.calls))

    def test_full_pagination_finds_producer_and_artifact_on_second_pages(self):
        metadata = Metadata()
        metadata.jobs = [{'id': i + 1000, 'name': 'other'} for i in range(100)] + [metadata.job]
        metadata.artifacts = [{'id': i + 2000, 'name': 'other'} for i in range(100)] + [metadata.artifact]
        self.assertEqual(self.wait(metadata)['artifact']['id'], 789)
        self.assertEqual(sum('page=2' in p for p, _ in metadata.calls), 2)

    def test_wrong_run_attempt_source_workflow_event_or_repository_refuses(self):
        for key, bad in [('id', 124), ('run_attempt', 1), ('head_sha', 'c' * 40),
                         ('path', '.github/workflows/other.yml'), ('event', 'pull_request'),
                         ('repository', {'id': 46, 'full_name': 'other/product'}),
                         ('head_repository', {'id': 99, 'full_name': 'fork/product'})]:
            with self.subTest(field=key):
                metadata = Metadata(); metadata.run[key] = bad
                with self.assertRaises(GATE.Refusal):
                    self.wait(metadata)

    def test_artifact_wrong_owner_digest_expiry_or_creation_refuses(self):
        for key, bad in [('workflow_run', {'id': 124}), ('digest', None),
                         ('digest', 'sha256:not-a-hash'), ('expired', True),
                         ('size_in_bytes', 0), ('created_at', '2026-10-04T00:59:59Z'),
                         ('created_at', '2026-10-05T00:00:00Z'),
                         ('expires_at', '2026-10-04T02:59:59Z')]:
            with self.subTest(field=key, value=bad):
                metadata = Metadata(); metadata.artifact[key] = bad
                with self.assertRaises(GATE.Refusal):
                    self.wait(metadata)

    def test_old_attempt_name_is_never_selected(self):
        metadata = Metadata(); metadata.artifact['name'] = metadata.artifact['name'][:-1] + '1'
        metadata.job['status'] = 'completed'; metadata.job['conclusion'] = 'success'
        with self.assertRaisesRegex(GATE.Refusal, 'without eligible'):
            self.wait(metadata)

    def test_duplicate_name_on_later_page_refuses(self):
        metadata = Metadata()
        duplicate = dict(metadata.artifact, id=999)
        metadata.artifacts = [metadata.artifact] + [{'id': i + 2000, 'name': 'other'} for i in range(99)] + [duplicate]
        with self.assertRaisesRegex(GATE.Refusal, 'Duplicate runtime artifact'):
            self.wait(metadata)

    def test_duplicate_producer_and_duplicate_required_step_refuse(self):
        metadata = Metadata(); metadata.jobs.append(dict(metadata.job, id=999))
        with self.assertRaisesRegex(GATE.Refusal, 'Duplicate producer'):
            self.wait(metadata)
        metadata = Metadata(); metadata.job['steps'].append(dict(metadata.job['steps'][0], number=99))
        with self.assertRaisesRegex(GATE.Refusal, 'Duplicate required'):
            self.wait(metadata)

    def test_wrong_job_attempt_and_unexpected_conclusion_refuse(self):
        for key, bad in [('run_id', 124), ('run_attempt', 1), ('head_sha', 'c'*40),
                         ('conclusion', 'https://signed.invalid/secret')]:
            with self.subTest(field=key):
                metadata = Metadata(); metadata.job[key] = bad
                with self.assertRaises(GATE.Refusal):
                    self.wait(metadata)

    def test_incomplete_pagination_and_duplicate_ids_refuse(self):
        metadata = Metadata()
        def incomplete(endpoint, timeout):
            value = metadata(endpoint, timeout)
            if '/artifacts?' in endpoint:
                value['total_count'] = 2
            return value
        with self.assertRaisesRegex(GATE.Refusal, 'Incomplete metadata'):
            self.wait(incomplete)
        metadata.artifacts.append(deepcopy(metadata.artifact))
        with self.assertRaisesRegex(GATE.Refusal, 'Duplicate metadata'):
            self.wait(metadata)

    def test_visible_artifact_waits_for_upload_success(self):
        metadata = Metadata(); clock = FakeClock()
        metadata.job['steps'][-1].update(status='in_progress', conclusion=None)
        def read(endpoint, timeout):
            if clock.value >= 30:
                metadata.job['steps'][-1].update(status='completed', conclusion='success')
            return metadata(endpoint, timeout)
        result = self.wait(read, clock=clock)
        self.assertEqual(result['polls'], 2)
        self.assertEqual(clock.sleeps, [30])

    def test_failed_or_skipped_prerequisite_and_wrong_order_refuse(self):
        for conclusion in ('failure', 'skipped', 'cancelled'):
            metadata = Metadata(); metadata.job['steps'][1]['conclusion'] = conclusion
            with self.subTest(conclusion=conclusion), self.assertRaisesRegex(GATE.Refusal, 'did not succeed'):
                self.wait(metadata)
        metadata = Metadata(); metadata.job['steps'][1]['number'] = 1
        with self.assertRaisesRegex(GATE.Refusal, 'order differs'):
            self.wait(metadata)

    def test_terminal_producer_without_artifact_refuses_immediately(self):
        for conclusion in ('failure', 'success', 'cancelled'):
            metadata = Metadata(); metadata.artifacts = []
            metadata.job.update(status='completed', conclusion=conclusion)
            clock = FakeClock()
            with self.subTest(conclusion=conclusion), self.assertRaisesRegex(GATE.Refusal, 'without eligible'):
                self.wait(metadata, clock=clock)
            self.assertEqual(clock.sleeps, [])

    def test_later_producer_failure_preserves_eligible_diagnostic_handoff(self):
        metadata = Metadata()
        metadata.job.update(status='completed', conclusion='failure')
        metadata.run.update(status='completed', conclusion='failure')
        result = self.wait(metadata)
        self.assertEqual(result['producer']['conclusion'], 'failure')
        self.assertIn('gates remain required', result['qualification'])

    def test_timeout_is_bounded_and_outputs_no_ready_result(self):
        metadata = Metadata(); metadata.artifacts = []; clock = FakeClock()
        with self.assertRaisesRegex(GATE.Refusal, 'timed out'):
            self.wait(metadata, clock=clock, timeout=61)
        self.assertEqual(clock.value, 61)
        self.assertEqual(clock.sleeps, [30, 30, 1])

    def test_transient_error_recovers_but_repeated_errors_refuse(self):
        metadata = Metadata(); attempts = []
        def once(endpoint, timeout):
            attempts.append(endpoint)
            if len(attempts) == 1:
                raise GATE.TransientError('not logged')
            return metadata(endpoint, timeout)
        self.assertEqual(self.wait(once)['polls'], 2)
        def always(endpoint, timeout):
            raise GATE.TransientError('not logged')
        with self.assertRaisesRegex(GATE.Refusal, 'Repeated metadata'):
            self.wait(always, timeout=121)

    def test_rerun_between_listing_and_output_refuses(self):
        metadata = Metadata(); reads = []
        def read(endpoint, timeout):
            if endpoint.endswith('/runs/123'):
                reads.append(endpoint)
                if len(reads) > 1:
                    metadata.run['run_attempt'] = 3
            return metadata(endpoint, timeout)
        with self.assertRaisesRegex(GATE.Refusal, 'attempt differs'):
            self.wait(read)

    def test_detail_digest_change_refuses(self):
        metadata = Metadata()
        def read(endpoint, timeout):
            value = metadata(endpoint, timeout)
            if endpoint.endswith('/artifacts/789'):
                value['digest'] = 'sha256:' + 'c' * 64
            return value
        with self.assertRaisesRegex(GATE.Refusal, 'metadata changed'):
            self.wait(read)

    def test_gh_is_read_only_and_does_not_echo_secret_error(self):
        reply = subprocess.CompletedProcess([], 1, '', 'token=secret https://signed.invalid (HTTP 403)')
        with patch.object(GATE.subprocess, 'run', return_value=reply) as call:
            with self.assertRaisesRegex(GATE.Refusal, '^Metadata access refused$'):
                GATE.gh_api('repos/example/product/actions/runs/123', 10)
        self.assertIn('GET', call.call_args.args[0])
        self.assertTrue(call.call_args.kwargs['capture_output'])

    def test_main_emits_outputs_only_after_success(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw); output = root/'outputs'
            result = self.wait(Metadata())
            with patch.dict(os.environ, {**ENV, 'GH_TOKEN': 'never-print-me', 'GITHUB_OUTPUT': str(output)}, clear=True), \
                    patch('sys.argv', ['wait', '--evidence', str(root/'ok')]), \
                    patch.object(GATE, 'wait_ready', return_value=result), patch('builtins.print'):
                self.assertEqual(GATE.main(), 0)
            self.assertEqual(output.read_text(), 'artifact-id=789\nartifact-digest=sha256:' + 'b'*64 + '\n')
            with patch.dict(os.environ, {**ENV, 'GH_TOKEN': 'never-print-me', 'GITHUB_OUTPUT': str(root/'failed-output')}, clear=True), \
                    patch('sys.argv', ['wait', '--evidence', str(root/'failed')]), \
                    patch.object(GATE, 'wait_ready', side_effect=GATE.Refusal('Artifact is stale or expired')), patch('builtins.print'):
                self.assertEqual(GATE.main(), 1)
            self.assertFalse((root/'failed-output').exists())
            self.assertNotIn('never-print-me', (root/'failed/readiness.json').read_text())


if __name__ == '__main__':
    unittest.main()

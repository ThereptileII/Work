"""Inert CLI/parser/handoff checks; no Windows, API writes or product effects."""
from copy import deepcopy
from datetime import datetime, timedelta, timezone
import importlib.util
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
spec = importlib.util.spec_from_file_location('journey_cli', Path(__file__).with_name('run-installed-updater-journey-windows.py'))
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)


class HostAndParserTests(unittest.TestCase):
    def setUp(self):
        self.environment = dict(GITHUB_ACTIONS='true', RUNNER_OS='Windows', RUNNER_ENVIRONMENT='github-hosted',
            GITHUB_REPOSITORY='ThereptileII/Work', GITHUB_EVENT_NAME='workflow_dispatch')
        self.args = ['--config=/approved/config.json', '--config-sha256=' + 'a' * 64,
                     '--disposable-opt-in=' + cli.OPT_IN]

    def test_hosted_opt_in_calls_existing_composition_once(self):
        with patch.dict(cli.os.environ, self.environment, clear=True), patch.object(cli.sys, 'platform', 'win32'), \
             patch.object(cli, 'original', return_value=b'{}') as original, \
             patch.object(cli, 'execute') as execute, patch('sys.stdout', new=io.StringIO()):
            self.assertEqual(cli.main(self.args), 0)
            original.assert_called_once()
            execute.assert_called_once()

    def test_wrong_platform_boat_non_disposable_and_unapproved_repo_refuse_before_reads(self):
        changes = [('platform', 'linux'), ('GITHUB_ACTIONS', 'false'), ('RUNNER_ENVIRONMENT', 'self-hosted'),
                   ('RUNNER_OS', 'Linux'), ('GITHUB_REPOSITORY', 'other/repo'), ('GITHUB_EVENT_NAME', 'push')]
        for name, value in changes:
            with self.subTest(name=name):
                env = dict(self.environment)
                if name != 'platform': env[name] = value
                with patch.dict(cli.os.environ, env, clear=True), \
                     patch.object(cli.sys, 'platform', value if name == 'platform' else 'win32'), \
                     patch.object(cli, 'original') as original, patch.object(cli, 'execute') as execute, \
                     patch('sys.stderr', new=io.StringIO()):
                    self.assertEqual(cli.main(self.args), 1)
                    original.assert_not_called()
                    execute.assert_not_called()

    def test_wrong_optin_duplicate_abbreviated_unknown_inputs_are_generic_without_reads(self):
        for args in (self.args[:-1] + ['--disposable-opt-in=no'], self.args + [self.args[0]],
                     self.args + ['--token=DO-NOT-LOG'], [a.replace('--config=', '--conf=') for a in self.args]):
            with self.subTest(args=args):
                error = io.StringIO()
                with patch.dict(cli.os.environ, self.environment, clear=True), patch.object(cli.sys, 'platform', 'win32'), \
                     patch.object(cli, 'original') as original, patch.object(cli, 'execute') as execute, \
                     patch('sys.stderr', new=error):
                    self.assertEqual(cli.main(args), 1)
                    self.assertEqual(error.getvalue(), cli.FAILURE + '\n')
                    original.assert_not_called()
                    execute.assert_not_called()

    def test_read_or_composition_failure_never_retries_or_logs_exception_token(self):
        with patch.dict(cli.os.environ, self.environment, clear=True), patch.object(cli.sys, 'platform', 'win32'), \
             patch.object(cli, 'original', return_value=b'{}'), \
             patch.object(cli, 'execute', side_effect=ValueError('SECRET-TOKEN')) as execute, \
             patch('sys.stderr', new=io.StringIO()) as error:
            self.assertEqual(cli.main(self.args), 1)
            self.assertNotIn('SECRET-TOKEN', error.getvalue())
            execute.assert_called_once()


class PhaseReadinessTests(unittest.TestCase):
    def setUp(self):
        self.runner = dict(repository='ThereptileII/Work', headSha='a' * 40, runId='123', runAttempt='1', jobId='456', jobName='Pending journey')
        self.challenge, self.head, self.plan_sha = 'b' * 64, 'c' * 64, 'd' * 64
        self.plan = dict(zip(cli.journey.STEPS, ['e' * 64] * 3 + [None, None]))
        self.now = datetime(2026, 10, 14, 12, 0, tzinfo=timezone.utc)
        self.tick, self.calls, self.emitted = 0, [], []
        self.context, self.url = cli.phase_locator(self.runner, self.challenge, 'now', self.head, self.plan_sha)
        self.status = dict(id=123, state='success', context=self.context, target_url=self.url, creator=dict(id=88),
                           created_at='2026-10-14T12:00:01Z', updated_at='2026-10-14T12:00:01Z')
        self.rows = []
        self.reads = Mock(read_api=self.read)

    def read(self, endpoint, **kwargs):
        self.calls.append(endpoint)
        self.assertIn('/commits/' + self.runner['headSha'] + '/statuses?', endpoint)
        self.assertGreater(kwargs['timeout_seconds'], 0)
        self.assertLessEqual(kwargs['timeout_seconds'], 30)
        return deepcopy(self.rows)

    def sleep(self, value):
        self.tick += value
        self.rows = [self.status]

    def handoff(self, **changes):
        options = dict(utcnow=lambda: self.now + timedelta(seconds=self.tick), monotonic=lambda: self.tick,
                       sleep=self.sleep, emit=lambda *a, **k: self.emitted.append(a[0]))
        options.update(changes)
        return cli.phase_handoff(self.plan, self.plan_sha, self.runner, self.challenge, '88', self.reads, **options)

    def test_exact_post_wait_operator_signal_returns_only_preapproved_pin(self):
        self.assertEqual(self.handoff()('now', self.head), 'e' * 64)
        self.assertEqual(len(self.calls), 2)
        self.assertEqual(json.loads(self.emitted[0])['target_url'], self.url)
        self.assertTrue(all('/statuses?' in endpoint for endpoint in self.calls))

    def test_phase_head_challenge_job_and_plan_are_all_bound(self):
        original = self.context
        for phase, head, challenge, plan, runner in [('later', self.head, self.challenge, self.plan_sha, self.runner),
            ('now', '0' * 64, self.challenge, self.plan_sha, self.runner),
            ('now', self.head, '0' * 64, self.plan_sha, self.runner),
            ('now', self.head, self.challenge, '0' * 64, self.runner),
            ('now', self.head, self.challenge, self.plan_sha, dict(self.runner, jobId='999'))]:
            self.assertNotEqual(cli.phase_locator(runner, challenge, phase, head, plan)[0], original)

    def test_foreign_creator_old_signal_wrong_payload_duplicate_or_changed_time_refuse(self):
        for kind in ('creator', 'old', 'url', 'duplicate', 'edited'):
            with self.subTest(kind=kind):
                self.tick, self.rows = 0, []
                status = deepcopy(self.status)
                if kind == 'creator': status['creator']['id'] = 99
                if kind == 'old': status.update(created_at='2026-10-14T12:00:00Z', updated_at='2026-10-14T12:00:00Z')
                if kind == 'url': status['target_url'] += 'other'
                if kind == 'edited': status['updated_at'] = '2026-10-14T12:00:02Z'
                def sleep(value):
                    self.tick += value
                    self.rows = [status, status] if kind == 'duplicate' else [status]
                with self.assertRaises(ValueError): self.handoff(sleep=sleep)('now', self.head)

    def test_partial_pagination_and_timeout_refuse_without_phase_retry(self):
        self.rows = [dict(context='unrelated')] * 100
        with self.assertRaises(ValueError): self.handoff()('now', self.head)
        self.assertEqual(len(self.calls), 5)
        self.rows = []
        def sleep(value): self.tick += value
        with self.assertRaises(ValueError): self.handoff(sleep=sleep)('now', self.head)
        self.assertEqual(self.tick, 900)

    def test_offline_plan_never_accepts_timestamp_or_implies_connectivity_mutation(self):
        self.plan['offline'] = 'a' * 64
        with self.assertRaises(ValueError): self.handoff()


class ProducerTests(unittest.TestCase):
    def setUp(self):
        self.artifact = dict(repository='ThereptileII/Work', headSha='b' * 40, runId='120', runAttempt='1', jobId='450',
            artifactId='800', artifactName='original-candidate', artifactDigest='sha256:' + 'c' * 64, artifactBytes=12345)
        self.record = dict(artifact=self.artifact, workflow='.github/workflows/candidate.yml', jobName='Candidate')
        self.run = dict(id=120, run_attempt=1, head_sha='b' * 40, path=self.record['workflow'],
            head_repository=dict(full_name='ThereptileII/Work'), event='workflow_dispatch', status='completed', conclusion='success')
        self.job = dict(id=450, name='Candidate', run_id=120, run_attempt=1, head_sha='b' * 40, status='completed', conclusion='success')
        self.metadata = dict(id=800, name=self.artifact['artifactName'], digest=self.artifact['artifactDigest'], size_in_bytes=12345,
            expired=False, workflow_run=dict(id=120, head_sha='b' * 40))
        self.calls = []
        self.reads = Mock(read_api=self.read)

    def read(self, endpoint, **kwargs):
        self.calls.append(endpoint)
        if endpoint.endswith('/jobs?per_page=100&page=1'): return dict(total_count=1, jobs=[self.job])
        if '/artifacts/' in endpoint: return self.metadata
        return self.run

    def test_original_exact_producer_job_and_artifact_are_reused(self):
        self.assertEqual(cli.producer(self.record, self.reads), self.artifact)
        self.assertEqual(len(self.calls), 3)
        self.assertTrue(all('/actions/' in endpoint for endpoint in self.calls))

    def test_wrong_workflow_job_digest_size_or_partial_jobs_refuses(self):
        for name in ('workflow', 'job', 'digest', 'size', 'partial'):
            with self.subTest(name=name):
                run, job, metadata = deepcopy(self.run), deepcopy(self.job), deepcopy(self.metadata)
                if name == 'workflow': self.run['path'] = '.github/workflows/foreign.yml'
                if name == 'job': self.job['id'] = 999
                if name == 'digest': self.metadata['digest'] = 'sha256:' + '0' * 64
                if name == 'size': self.metadata['size_in_bytes'] = 1
                reads = self.reads
                if name == 'partial':
                    reads = Mock(read_api=lambda endpoint, **kwargs: dict(total_count=2, jobs=[self.job]) if '/jobs?' in endpoint else self.read(endpoint, **kwargs))
                with self.assertRaises(ValueError): cli.producer(self.record, reads)
                self.run, self.job, self.metadata = run, job, metadata


class ExecuteTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('approval_fixture', Path(__file__).with_name('test-installed-updater-approval-transport.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.f = module.TransportTests()
        self.f.setUp()
        f = self.f.f
        self.environment = dict(GITHUB_REPOSITORY=f.runner['repository'], GITHUB_SHA=f.runner['headSha'],
            GITHUB_RUN_ID=f.runner['runId'], GITHUB_RUN_ATTEMPT=f.runner['runAttempt'], SKAGER_APPROVAL_READ_TOKEN='DO-NOT-LOG')
        record = lambda name: dict(path='/approved/' + name, sha256='a' * 64)
        self.config = dict(schema=1, purpose='installed-updater-journey-cli', request=record('request'),
            runner=f.runner, challenge='f' * 64, pendingWorkflow='.github/workflows/pending.yml',
            admission=record('admission'), phases=record('phases'),
            artifacts=dict(candidate=dict(artifact=f.artifact), operatorKit=dict(artifact=f.kit), successor=dict(artifact=dict(f.artifact, headSha='9' * 40))),
            source=dict(status_creator_id='88', producer_commit='8' * 40, producer_actor_id='10', producer_triggering_actor_id='10'),
            consumer=record('consumer'), qualificationParent='/approved/qualification', consumptionReceipt='/evidence/receipt',
            output='/evidence/journey/result.json', phasePlan=record('plan'))
        self.initial = dict(requestPath=Path('/approved/client'), b0=dict(policy=record('policy')), setup=Mock(), setupSha256='0' * 64)
        self.initial['setup'].__str__ = Mock(return_value='/approved/setup')
        self.initial['setup'].stat.return_value.st_size = 1234
        self.phases = dict(b1=dict(commit='9' * 40))
        self.plan = dict(zip(cli.journey.STEPS, [f.pins['timestampSha256']] * 3 + [None, None]))
        def original(record, limit):
            name = Path(record['path']).name
            return {'request': f.request_bytes, 'client': f.client_bytes, 'admission': f.declaration_bytes,
                    'policy': f.policy_bytes, 'plan': json.dumps(self.plan).encode()}[name]
        def api(endpoint, **kwargs):
            if '/jobs?' in endpoint:
                return dict(total_count=1, jobs=[dict(id=int(f.runner['jobId']), name=f.runner['jobName'], status='in_progress',
                    run_id=int(f.runner['runId']), run_attempt=int(f.runner['runAttempt']), head_sha=f.runner['headSha'])])
            return dict(id=int(f.runner['runId']), run_attempt=int(f.runner['runAttempt']), head_sha=f.runner['headSha'],
                path=self.config['pendingWorkflow'], event='workflow_dispatch', status='in_progress',
                head_repository=dict(full_name='ThereptileII/Work'))
        self.patches = [patch.object(cli, 'original', side_effect=original),
            patch.object(cli.coordinator.http, 'GitHubReads', return_value=Mock(read_api=api)),
            patch.object(cli, 'producer', side_effect=lambda record, reads: record['artifact']),
            patch.object(cli.journey.admission, 'prepare', return_value=self.initial),
            patch.object(cli.journey.phase, 'prepare', return_value=self.phases),
            patch.object(cli.journey, '_same_inputs'), patch.object(cli.coordinator, 'run_live', return_value={'state': 'observed'})]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)

    def test_entrypoint_forwards_originals_explicit_pins_and_existing_live_mechanism_once(self):
        self.assertEqual(cli.execute(self.config, self.environment), {'state': 'observed'})
        args = cli.coordinator.run_live.call_args.kwargs
        self.assertEqual(args['bind_arguments']['request_bytes'], self.f.f.request_bytes)
        self.assertEqual(args['bind_arguments']['expected_candidate_artifact'], self.f.f.artifact)
        self.assertEqual(args['bind_arguments']['expected_operator_kit_artifact'], self.f.f.kit)
        self.assertEqual(args['source'].status_creator_id, '88')
        self.assertEqual(args['consumer_sha256'], 'a' * 64)
        self.assertTrue(callable(args['handoff']))
        cli.coordinator.run_live.assert_called_once()

    def test_wrong_pending_context_artifact_source_or_successor_never_calls_live(self):
        for kind in ('context', 'artifact', 'source', 'successor', 'plan'):
            with self.subTest(kind=kind):
                config = deepcopy(self.config)
                if kind == 'context': config['runner'] = dict(config['runner'], runId='999')
                if kind == 'artifact': config['artifacts']['candidate']['artifact'] = dict(self.f.f.artifact, artifactDigest='sha256:' + '0' * 64)
                if kind == 'source': del config['source']['status_creator_id']
                if kind == 'successor': config['artifacts']['successor']['artifact'] = dict(self.f.f.artifact, headSha='0' * 40)
                if kind == 'plan': self.plan['no-update'] = '0' * 64
                with self.assertRaises(ValueError): cli.execute(config, self.environment)
                cli.coordinator.run_live.assert_not_called()


if __name__ == '__main__':
    unittest.main()

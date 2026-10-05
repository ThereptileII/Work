#!/usr/bin/env python3
"""Offline authorization, immutable retry and provenance tests; no network/app runs."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import github_release_delivery as delivery

spec = importlib.util.spec_from_file_location('manifest_fixture', Path(__file__).with_name('test-release-manifest.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
COMMIT = fixture.COMMIT
HARNESS = '2' * 40


class FakeGitHub:
    def __init__(self):
        self.base = 'repos/owner/repo'
        self.releases = {}
        self.blobs = {}
        self.calls = []
        self.next_id = 1
        self.stage_run = dict(id=42, run_attempt=1, head_sha=COMMIT, path=delivery.STAGING_WORKFLOW,
                              event='push', status='completed', conclusion='success')
        self.production_run = dict(id=99, run_attempt=2, head_sha=HARNESS,
                                   path=delivery.PRODUCTION_WORKFLOW, event='workflow_dispatch',
                                   status='in_progress', conclusion=None)
        self.stage_attempts = {}
        self.stage_jobs = [dict(name=delivery.STAGING_QUALIFICATION_JOB, status='completed', conclusion='success')]
        self.jobs = [dict(name='Qualify retained Windows package', status='completed', conclusion='success')]

    def api(self, endpoint):
        self.calls.append(('api', endpoint))
        if '/runs/42/' in endpoint:
            attempt = endpoint.split('/attempts/')[1].split('/')[0]
            return self.stage_attempts.get(attempt, self.stage_run)
        if '/runs/99/' in endpoint:
            return self.production_run
        raise AssertionError(endpoint)

    def pages(self, endpoint):
        self.calls.append(('pages', endpoint))
        return self.stage_jobs if '/runs/42/' in endpoint else self.jobs

    def release(self, tag):
        self.calls.append(('release', tag))
        return self.releases.get(tag)

    def assets(self, release):
        self.calls.append(('assets', release['tag_name']))
        return list(release['assets'])

    def create(self, tag, commit, name, prerelease):
        self.calls.append(('create', tag))
        release = dict(id=self.next_id, tag_name=tag, target_commitish=commit,
                       draft=True, prerelease=prerelease, name=name, assets=[])
        self.next_id += 1
        self.releases[tag] = release
        return release

    def upload(self, release, path):
        self.calls.append(('upload', path.name))
        ident = self.next_id
        self.next_id += 1
        data = path.read_bytes()
        self.blobs[ident] = data
        release['assets'].append(dict(id=ident, name=path.name, size=len(data), state='uploaded'))

    def download(self, asset, path):
        self.calls.append(('download', asset['name']))
        with path.open('xb') as stream:
            stream.write(self.blobs[asset['id']])


class DeliveryTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixture.ManifestTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.root = self.fixture.root
        self.qualification = dict(schemaVersion=1, channel='staging', commit=COMMIT, runId='42', runAttempt='1',
                                  gates=dict(linux='passed', windows='passed', installer='passed', package='passed', restart='passed'),
                                  designReview='not-requested', endurance='skipped', publicAccess=False)
        self.support = dict(artifactName='staging-retest-' + COMMIT + '-attempt1', runId='42',
                            runAttempt='1', commit=COMMIT,
                            archiveName='SKAGER-Beta2-Retest-Support.zip', sha256='a' * 64, size=123)
        for name, value in [('QUALIFICATION.json', self.qualification), ('RETEST_SUPPORT.json', self.support)]:
            (self.root / name).write_text(json.dumps(value))
        self.record = self.fixture.create()
        self.tag = delivery.staging_tag(self.record)
        self.gh = FakeGitHub()
        self.env = patch.dict(os.environ, dict(GITHUB_SHA=COMMIT, GITHUB_RUN_ID='42', GITHUB_RUN_ATTEMPT='1',
                                               GITHUB_EVENT_NAME='push'), clear=True)
        self.env.start()
        self.addCleanup(self.env.stop)
        self.extra = tempfile.TemporaryDirectory()
        self.addCleanup(self.extra.cleanup)
        self.work = Path(self.extra.name)

    def publish(self):
        delivery.publish_staging(self.gh, self.root)
        self.gh.calls.clear()

    def report(self):
        os.environ.update(GITHUB_SHA=HARNESS, GITHUB_RUN_ID='99', GITHUB_RUN_ATTEMPT='2',
                          GITHUB_EVENT_NAME='workflow_dispatch')
        result = dict(schemaVersion=1, status='passed', commit=COMMIT,
                      manifestSha256=delivery.file_hash(self.root / 'RELEASE.json'),
                      setupSha256=delivery.file_hash(self.root / 'SKAGER-Beta2-Setup.exe'),
                      qualificationRunId='99', harnessCommit=HARNESS, runAttempt='2',
                      gates=dict(installer='passed', functional='passed', package='passed', linux='passed'))
        path = self.work / 'report.json'
        path.write_text(json.dumps(result))
        return path, result

    def promote(self, report, **kwargs):
        values = dict(gh=self.gh, directory=self.root, tag=self.tag, confirmation='PROMOTE ' + self.tag,
                      instruction='User instruction recorded in SCRUM-290', report_path=report, qualify_run_id='99')
        delivery.publish_production(**dict(values, **kwargs))

    def test_publish_retry_compares_all_bytes_and_adds_only_missing(self):
        self.publish()
        release = self.gh.releases[self.tag]
        original = copy.deepcopy(self.gh.blobs)
        delivery.publish_staging(self.gh, self.root)
        self.assertFalse(any(call[0] in {'upload', 'create'} for call in self.gh.calls))
        self.assertEqual(self.gh.blobs, original)
        removed = release['assets'].pop()
        self.gh.calls.clear()
        delivery.publish_staging(self.gh, self.root)
        self.assertEqual([call for call in self.gh.calls if call[0] == 'upload'], [('upload', removed['name'])])
        self.assertTrue(release['draft'])
        self.assertTrue(release['prerelease'])

    def test_existing_asset_same_size_tamper_refuses_before_upload(self):
        self.publish()
        release = self.gh.releases[self.tag]
        asset = release['assets'][0]
        original = self.gh.blobs[asset['id']]
        self.gh.blobs[asset['id']] = bytes([original[0] ^ 1]) + original[1:]
        release['assets'].pop()
        with self.assertRaises(ValueError):
            delivery.publish_staging(self.gh, self.root)
        self.assertFalse(any(call[0] in {'upload', 'create'} for call in self.gh.calls))

    def test_existing_public_wrong_commit_or_extra_asset_refused(self):
        for field, value in [('draft', False), ('prerelease', False), ('target_commitish', HARNESS)]:
            with self.subTest(field=field):
                self.publish()
                release = self.gh.releases[self.tag]
                old = release[field]
                release[field] = value
                with self.assertRaises(ValueError):
                    delivery.publish_staging(self.gh, self.root)
                self.assertFalse(any(call[0] == 'upload' for call in self.gh.calls))
                release[field] = old
        release['assets'].append(dict(name='unexpected.txt', state='uploaded', id=999, size=1))
        with self.assertRaises(ValueError):
            delivery.publish_staging(self.gh, self.root)

    def test_local_tamper_or_failed_qualification_no_api(self):
        data = (self.root / 'SKAGER-Beta2-Setup.exe').read_bytes()
        (self.root / 'SKAGER-Beta2-Setup.exe').write_bytes(data + b'tamper')
        with self.assertRaises(ValueError):
            delivery.publish_staging(self.gh, self.root)
        self.assertEqual(self.gh.calls, [])
        (self.root / 'SKAGER-Beta2-Setup.exe').write_bytes(data)
        (self.root / 'RELEASE.json').unlink()
        self.qualification['gates']['installer'] = 'failed'
        (self.root / 'QUALIFICATION.json').write_text(json.dumps(self.qualification))
        self.fixture.create()
        with self.assertRaises(ValueError):
            delivery.publish_staging(self.gh, self.root)
        self.assertEqual(self.gh.calls, [])

    def test_retest_support_cannot_claim_another_attempt(self):
        for changes in ({'runAttempt': '2'}, {'runAttempt': 1},
                        {'artifactName': 'staging-retest-' + COMMIT},
                        {'artifactName': 'staging-retest-' + COMMIT + '-attempt2'}):
            with self.subTest(changes=changes):
                (self.root / 'RELEASE.json').unlink()
                (self.root / 'RETEST_SUPPORT.json').write_text(json.dumps(dict(self.support, **changes)))
                self.fixture.create()
                with self.assertRaises(ValueError):
                    delivery.publish_staging(self.gh, self.root)
                self.assertEqual(self.gh.calls, [])

    def retry_fixture(self):
        (self.root / 'RELEASE.json').unlink()
        self.qualification['runAttempt'] = '3'
        self.support.update(runAttempt='2', artifactName='staging-retest-' + COMMIT + '-attempt2')
        (self.root / 'QUALIFICATION.json').write_text(json.dumps(self.qualification))
        (self.root / 'RETEST_SUPPORT.json').write_text(json.dumps(self.support))
        self.record = self.fixture.create(run_attempt='3')
        self.tag = delivery.staging_tag(self.record)
        self.gh.stage_run['run_attempt'] = 3
        self.gh.stage_attempts['2'] = dict(self.gh.stage_run, run_attempt=2, conclusion='failure')
        os.environ['GITHUB_RUN_ATTEMPT'] = '3'

    def test_publish_only_retry_authenticates_prior_qualification_without_relabeling(self):
        self.retry_fixture()
        support_bytes = (self.root / 'RETEST_SUPPORT.json').read_bytes()
        setup_bytes = (self.root / 'SKAGER-Beta2-Setup.exe').read_bytes()
        delivery.publish_staging(self.gh, self.root)
        self.assertIn(('pages', self.gh.base + '/actions/runs/42/attempts/2/jobs'), self.gh.calls)
        self.assertIn('attempt3', self.tag)
        self.assertEqual((self.root / 'RETEST_SUPPORT.json').read_bytes(), support_bytes)
        self.assertEqual((self.root / 'SKAGER-Beta2-Setup.exe').read_bytes(), setup_bytes)
        self.assertEqual(delivery.local_record(self.root)[1]['runAttempt'], '2')
        self.gh.calls.clear()
        output = self.work / 'outputs'
        os.environ['GITHUB_OUTPUT'] = str(output)
        delivery.fetch_staging(self.gh, self.tag, self.work / 'retry-fetch')
        self.assertIn(('pages', self.gh.base + '/actions/runs/42/attempts/2/jobs'), self.gh.calls)
        self.assertIn('support_artifact=' + self.support['artifactName'] + '\n', output.read_text())
        self.assertEqual((self.work / 'retry-fetch/RETEST_SUPPORT.json').read_bytes(), support_bytes)

    def test_prior_support_rejects_future_noncanonical_and_different_run_or_commit(self):
        self.retry_fixture()
        for changes in ({'runAttempt': '4', 'artifactName': 'staging-retest-' + COMMIT + '-attempt4'},
                        {'runAttempt': '02'}, {'runAttempt': '0'}, {'runAttempt': 2},
                        {'runId': '41'}, {'commit': HARNESS},
                        {'artifactName': 'staging-retest-' + COMMIT + '-attempt3'}):
            with self.subTest(changes=changes):
                (self.root / 'RELEASE.json').unlink()
                (self.root / 'RETEST_SUPPORT.json').write_text(json.dumps(dict(self.support, **changes)))
                self.fixture.create(run_attempt='3')
                with self.assertRaises(ValueError):
                    delivery.publish_staging(self.gh, self.root)
                self.assertEqual(self.gh.calls, [])

    def test_prior_support_hash_tamper_is_rejected_before_network(self):
        self.retry_fixture()
        (self.root / 'RETEST_SUPPORT.json').write_text(json.dumps(dict(self.support, sha256='f' * 64)))
        with self.assertRaises(ValueError):
            delivery.publish_staging(self.gh, self.root)
        self.assertEqual(self.gh.calls, [])

    def test_prior_qualification_failure_wrong_identity_or_ambiguous_job_blocks_publish_and_fetch(self):
        self.retry_fixture()
        self.publish()
        original_job = copy.deepcopy(self.gh.stage_jobs)
        original_run = copy.deepcopy(self.gh.stage_attempts['2'])
        variants = [('job-failed', 'job', {'conclusion': 'failure'}),
                    ('job-running', 'job', {'status': 'in_progress'}),
                    ('job-wrong', 'job', {'name': 'windows-integration'}),
                    ('wrong-commit', 'run', {'head_sha': HARNESS}),
                    ('wrong-attempt', 'run', {'run_attempt': 1}),
                    ('wrong-run', 'run', {'id': 41}),
                    ('wrong-workflow', 'run', {'path': delivery.PRODUCTION_WORKFLOW}),
                    ('unfinished', 'run', {'status': 'in_progress'}),
                    ('duplicate', 'duplicate', {})]
        for label, kind, changes in variants:
            with self.subTest(label=label):
                self.gh.stage_jobs = copy.deepcopy(original_job)
                self.gh.stage_attempts['2'] = copy.deepcopy(original_run)
                if kind == 'job': self.gh.stage_jobs[0].update(changes)
                elif kind == 'run': self.gh.stage_attempts['2'].update(changes)
                else: self.gh.stage_jobs += copy.deepcopy(original_job)
                self.gh.calls.clear()
                with self.assertRaises(ValueError):
                    delivery.publish_staging(self.gh, self.root)
                self.assertFalse(any(call[0] in {'create', 'upload'} for call in self.gh.calls))
                target = self.work / label
                with self.assertRaises(ValueError):
                    delivery.fetch_staging(self.gh, self.tag, target)
                self.assertFalse(target.exists())

    def test_fetch_retains_original_assets_and_safe_outputs(self):
        self.publish()
        target = self.work / 'download'
        os.environ['GITHUB_OUTPUT'] = str(self.work / 'outputs')
        delivery.fetch_staging(self.gh, self.tag, target)
        self.assertEqual({p.name: p.read_bytes() for p in target.iterdir()},
                         {p.name: p.read_bytes() for p in self.root.iterdir()})
        output = (self.work / 'outputs').read_text()
        self.assertIn('commit=' + COMMIT + '\n', output)
        self.assertIn('support_artifact=staging-retest-' + COMMIT + '-attempt1\n', output)

    def test_fetch_failed_wrong_attempt_or_workflow_does_not_deliver(self):
        self.publish()
        for field, value in [('conclusion', 'failure'), ('status', 'in_progress'), ('run_attempt', 2),
                             ('head_sha', HARNESS), ('path', '.github/workflows/other.yml')]:
            with self.subTest(field=field):
                old = self.gh.stage_run[field]
                self.gh.stage_run[field] = value
                with self.assertRaises(ValueError):
                    delivery.fetch_staging(self.gh, self.tag, self.work / field)
                self.assertFalse((self.work / field).exists())
                self.gh.stage_run[field] = old

    def test_fetch_rejects_latest_and_traversal_without_api(self):
        for tag in ['latest', '../skager-staging-x', self.tag + '\ninjected=1']:
            with self.assertRaises(ValueError):
                delivery.fetch_staging(self.gh, tag, self.work / 'download')
        self.assertEqual(self.gh.calls, [])

    def test_production_missing_approval_or_tampered_report_no_api(self):
        self.publish()
        path, report = self.report()
        for overrides in [{'confirmation': ''}, {'instruction': ''}, {'qualify_run_id': '98'}]:
            with self.assertRaises(ValueError):
                self.promote(path, **overrides)
            self.assertEqual(self.gh.calls, [])
        for field, value in [('status', 'failed'), ('manifestSha256', 'f' * 64),
                             ('setupSha256', 'f' * 64), ('commit', HARNESS), ('runAttempt', '1')]:
            changed = dict(report, **{field: value})
            path.write_text(json.dumps(changed))
            with self.assertRaises(ValueError):
                self.promote(path)
            self.assertEqual(self.gh.calls, [])
        path.write_text(json.dumps(report))
        os.environ['GITHUB_EVENT_NAME'] = 'push'
        with self.assertRaises(ValueError):
            self.promote(path)
        self.assertEqual(self.gh.calls, [])

    def test_production_refuses_locally_rehashed_substitution_before_production_api(self):
        self.publish()
        (self.root / 'SKAGER-Beta2-Setup.exe').write_bytes(b'A different inert setup')
        self.fixture.sums()
        (self.root / 'RELEASE.json').unlink()
        self.fixture.create()
        path, _ = self.report()
        with self.assertRaises(ValueError):
            self.promote(path)
        self.assertFalse(any(call[0] == 'create' or
                             (call[0] == 'api' and '/runs/99/' in call[1]) for call in self.gh.calls))

    def test_production_checks_native_job_and_current_harness(self):
        self.publish()
        path, _ = self.report()
        self.gh.jobs[0]['conclusion'] = 'failure'
        with self.assertRaises(ValueError):
            self.promote(path)
        self.assertFalse(any(call[0] == 'create' for call in self.gh.calls))
        self.gh.jobs[0]['conclusion'] = 'success'
        self.gh.production_run['head_sha'] = COMMIT
        with self.assertRaises(ValueError):
            self.promote(path)
        self.assertFalse(any(call[0] == 'create' for call in self.gh.calls))

    def test_production_exact_payload_extra_receipt_and_idempotent_retry(self):
        self.publish()
        path, _ = self.report()
        before = {p.name: p.read_bytes() for p in self.root.iterdir()}
        self.promote(path)
        release = self.gh.releases['skager-production-' + self.record['candidateId']]
        self.assertTrue(release['draft'])
        self.assertFalse(release['prerelease'])
        copied = {a['name']: self.gh.blobs[a['id']] for a in release['assets']}
        self.assertEqual({name: copied[name] for name in before}, before)
        self.assertEqual(set(copied) - set(before), {'PRODUCTION.json'})
        self.gh.calls.clear()
        self.promote(path)
        self.assertFalse(any(call[0] in {'upload', 'create'} for call in self.gh.calls))


class BoundaryTests(unittest.TestCase):
    def test_retained_support_records_producing_attempt(self):
        spec = importlib.util.spec_from_file_location('retain_inputs',
                Path(__file__).with_name('retain-release-inputs.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name in ('build/beta-installer/package.json', 'build/beta-installer/payload.zip',
                         'build/production-windows/include/config.h',
                         'build/xnav-windows/include/config.h', 'build/xnav-install/opencpn.exe'):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b'Inert qualification fixture')
            (root / 'build/beta-artifacts').mkdir()
            receipt = module.retain(root, COMMIT, '42', '2')
            self.assertEqual(receipt['runAttempt'], '2')
            self.assertEqual(receipt['artifactName'], 'staging-retest-' + COMMIT + '-attempt2')
            self.assertEqual(json.loads((root / 'build/beta-artifacts/RETEST_SUPPORT.json').read_text()), receipt)
            self.assertEqual(delivery.file_hash(root / 'build/release-retest' / receipt['archiveName']),
                             receipt['sha256'])

    def test_api_failure_does_not_echo_token_and_uses_no_shell(self):
        gh = delivery.GitHub('owner/repo')
        failed = subprocess.CompletedProcess([], 1, b'secret-token', b'secret-token')
        with patch.object(delivery.subprocess, 'run', return_value=failed) as run:
            with self.assertRaises(ValueError) as error:
                gh.api('repos/owner/repo/releases')
        self.assertNotIn('secret-token', str(error.exception))
        self.assertNotIn('shell', run.call_args.kwargs)
        self.assertEqual(run.call_args.args[0][0], 'gh')

    def test_create_is_always_draft_and_never_latest(self):
        gh = delivery.GitHub('owner/repo')
        with patch.object(gh, 'api', return_value={}) as api:
            gh.create('safe-tag', COMMIT, 'Name', True)
        payload = api.call_args.args[1]
        self.assertIs(payload['draft'], True)
        self.assertEqual(payload['make_latest'], 'false')

    def test_release_lookup_paginates_including_drafts(self):
        gh = delivery.GitHub('owner/repo')
        first = [dict(tag_name='other-' + str(i)) for i in range(100)]
        target = dict(tag_name='skager-staging-test', draft=True)
        with patch.object(gh, 'api', side_effect=[first, [target]]) as api:
            self.assertEqual(gh.release(target['tag_name']), target)
        self.assertIn('page=2', api.call_args.args[0])


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3
"""Transport and ZIP refusal tests; never enroll a service or execute an image."""
import copy
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import retained_broker_enrollment as subject

SELECTION = dict(schema=1, repository=subject.REPO, runId='123', runAttempt='1',
                 headSha='a'*40, jobId='456', artifactId='789',
                 artifactName='update-verifier-windows-2022-386-'+'a'*40,
                 artifactDigest='sha256:'+'b'*64)


class GitHubFixture:
    repo = subject.REPO
    base = 'repos/' + repo

    def __init__(self):
        self.run = dict(id=123, run_attempt=1, head_sha='a'*40, path=subject.WORKFLOW,
                        head_repository={'full_name': self.repo}, event='workflow_dispatch',
                        status='completed', conclusion='failure')
        self.job = dict(id=456, run_id=123, run_attempt=1, head_sha='a'*40,
                        name=subject.JOB, status='completed', conclusion='failure',
                        steps=[dict(name=name, status='completed', conclusion='success')
                               for name in subject.PRODUCER_STEPS] +
                              [dict(name=subject.FAILED_STEP, status='completed', conclusion='failure')])
        self.artifact = dict(id=789, name=SELECTION['artifactName'], digest=SELECTION['artifactDigest'],
                             expired=False, size_in_bytes=1024, workflow_run={'id': 123, 'head_sha': 'a'*40})

    def api(self, endpoint):
        return copy.deepcopy({self.base+'/actions/runs/123/attempts/1': self.run,
                              self.base+'/actions/jobs/456': self.job,
                              self.base+'/actions/artifacts/789': self.artifact}[endpoint])


class RetainedProducerTests(unittest.TestCase):
    def test_file_cli_reauthenticates_exact_producer_without_inline_code(self):
        with tempfile.TemporaryDirectory() as directory:
            selection = Path(directory)/'selection.json'
            selection.write_text(json.dumps(SELECTION))
            args = ['retained_broker_enrollment.py', '--selection', str(selection),
                    '--receipt', str(Path(directory)/'receipt.json'), '--validate-only',
                    '--expected-producer-commit', SELECTION['headSha']]
            with patch.dict(os.environ, GITHUB_REPOSITORY=subject.REPO, GITHUB_SHA='c'*40), \
                 patch('sys.argv', args), patch.object(subject, 'GitHub'), \
                 patch.object(subject, 'validate_receipt', return_value={}) as validate, \
                 patch.object(subject, 'fetch') as fetch, patch('builtins.print'):
                subject.main()
                self.assertEqual(validate.call_args.args[1], SELECTION)
                self.assertEqual(validate.call_args.args[3], 'c'*40)
                fetch.assert_not_called()
            args[-1] = 'd'*40
            with patch.dict(os.environ, GITHUB_REPOSITORY=subject.REPO, GITHUB_SHA='c'*40), \
                 patch('sys.argv', args), patch.object(subject, 'validate_receipt') as validate:
                with self.assertRaises(ValueError):
                    subject.main()
                validate.assert_not_called()

    def test_original_failure_is_preserved(self):
        result = subject.authenticate(GitHubFixture(), SELECTION)
        self.assertEqual(result['originalConclusion'], 'failure')
        self.assertEqual(result['headSha'], 'a'*40)
        self.assertIn('SCM retest only', result['scope'])

    def test_wrong_original_identity_and_producer_steps_refuse(self):
        mutations = [
            lambda f: f.run.update(head_sha='c'*40),
            lambda f: f.run.update(path='.github/workflows/unknown.yml'),
            lambda f: f.run.update(run_attempt=2),
            lambda f: f.run.update(conclusion='success'),
            lambda f: f.job.update(run_id=124),
            lambda f: f.job.update(head_sha='c'*40),
            lambda f: f.job.update(name='unrelated'),
            lambda f: f.job['steps'][0].update(conclusion='failure'),
            lambda f: f.job['steps'].append(dict(f.job['steps'][0])),
            lambda f: f.job['steps'][-1].update(name='Compile native launcher with exact dependency sources'),
            lambda f: f.artifact.update(digest='sha256:'+'c'*64),
            lambda f: f.artifact.update(expired=True),
            lambda f: f.artifact['workflow_run'].update(head_sha='c'*40),
            lambda f: f.artifact.update(size_in_bytes=subject.MAX_BYTES+1),
        ]
        for index, mutate in enumerate(mutations):
            with self.subTest(index=index):
                fixture = GitHubFixture()
                mutate(fixture)
                with self.assertRaises(ValueError):
                    subject.authenticate(fixture, SELECTION)

    def test_unknown_selection_refuses(self):
        with self.assertRaises(ValueError):
            subject.authenticate(GitHubFixture(), dict(SELECTION, extra='ignored'))

    def test_safe_selective_extraction_and_no_existing_adoption(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root/'fixture.zip'
            with zipfile.ZipFile(archive, 'w') as stream:
                stream.writestr('broker-install/skager-update-broker.exe', b'inert')
                stream.writestr('broker-install/opennav/third-party/update-broker/build.json', b'{}')
                stream.writestr('unrelated/never-extracted.exe', b'inert')
            subject.unpack_broker(archive, root/'result')
            self.assertEqual((root/'result/skager-update-broker.exe').read_bytes(), b'inert')
            self.assertFalse((root/'result/unrelated').exists())
            with self.assertRaises(ValueError):
                subject.unpack_broker(archive, root/'result')

    def test_unsafe_entire_outer_inventory_refuses_before_writes(self):
        for name in ('../escape', 'broker-install/CON', 'unrelated/bad:stream',
                     'broker-install/../escape', 'broker-install/x\\y', '/absolute'):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                archive = root/'fixture.zip'
                with zipfile.ZipFile(archive, 'w') as stream:
                    stream.writestr('broker-install/skager-update-broker.exe', b'inert')
                    stream.writestr('broker-install/opennav/third-party/update-broker/build.json', b'{}')
                    stream.writestr(name, b'inert')
                with self.assertRaises(ValueError):
                    subject.unpack_broker(archive, root/'result')
                self.assertFalse((root/'result').exists())


if __name__ == '__main__':
    unittest.main()

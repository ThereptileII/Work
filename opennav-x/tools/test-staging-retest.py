#!/usr/bin/env python3
"""Exact-attempt artifact selection with GitHub transport mocked; no downloads."""
import importlib.util
import json
from pathlib import Path
import unittest
from github_release_delivery import GitHub

spec = importlib.util.spec_from_file_location('staging_retest', Path(__file__).with_name('retest-staging-windows.py'))
retest = importlib.util.module_from_spec(spec)
spec.loader.exec_module(retest)
COMMIT = 'a' * 40
NAME = 'staging-build-' + COMMIT + '-run123-attempt2'


class Transport(GitHub):
    def __init__(self, artifacts):
        super().__init__('ThereptileII/Work')
        self.artifacts = artifacts
        self.calls = []

    def _run(self, arguments, *, data=None, output=None):
        assert data is None and output is None and arguments[0] == 'api'
        endpoint = arguments[1]
        self.calls.append(endpoint)
        if endpoint == self.base + '/actions/runs/123/attempts/2':
            return json.dumps(dict(id=123, run_attempt=2, head_sha=COMMIT)).encode()
        prefix = self.base + '/actions/runs/123/artifacts?per_page=100&page='
        assert endpoint.startswith(prefix), endpoint
        page = int(endpoint[len(prefix):])
        return json.dumps(dict(artifacts=self.artifacts[(page-1)*100:page*100])).encode()


class Selection(unittest.TestCase):
    def test_exact_attempt_on_later_page_is_selected_without_latest_fallback(self):
        older = [dict(id=i+1, name=f'staging-build-{COMMIT}-run123-attempt1', digest='sha256:'+'0'*64)
                 for i in range(100)]
        match = dict(id=501, name=NAME, digest='sha256:'+'b'*64)
        gh = Transport(older + [match])
        selected = retest.selection_for_run(gh, '123', '2')
        self.assertEqual(selected, dict(repository=gh.repo, runId='123', runAttempt='2', headSha=COMMIT,
            artifactId='501', artifactName=NAME, artifactDigest='sha256:'+'b'*64))
        self.assertEqual(gh.calls[0], gh.base + '/actions/runs/123/attempts/2')
        self.assertTrue(gh.calls[-1].endswith('page=2'))

    def test_missing_requested_attempt_or_ambiguous_exact_artifact_is_refused(self):
        earlier = dict(id=1, name=f'staging-build-{COMMIT}-run123-attempt1', digest='sha256:'+'0'*64)
        exact = dict(id=2, name=NAME, digest='sha256:'+'b'*64)
        for artifacts in ([earlier], [exact, dict(exact, id=3)]):
            with self.subTest(artifacts=artifacts), self.assertRaisesRegex(ValueError, 'missing or ambiguous'):
                retest.selection_for_run(Transport(artifacts), '123', '2')


if __name__ == '__main__':
    unittest.main()

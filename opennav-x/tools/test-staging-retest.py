#!/usr/bin/env python3
"""Exact-attempt artifact selection with GitHub transport mocked; no downloads."""
import importlib.util
import json
from pathlib import Path
import unittest
import os
import subprocess
from unittest import mock
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


class PinnedRetest(unittest.TestCase):
    def setUp(self):
        self.path = retest.ROOT / 'tools/staging-installer-retest.json'
        self.request = json.loads(self.path.read_text())
        self.environment = dict(GITHUB_EVENT_NAME='push', GITHUB_REF='refs/heads/skager-staging-retest',
                                GITHUB_REPOSITORY='ThereptileII/Work', GITHUB_SHA='f'*40)

    def test_only_exact_trusted_push_and_installer_charts_selection(self):
        with mock.patch.dict(os.environ, self.environment, clear=True):
            self.assertEqual(retest.pinned_selection(self.path), self.request)
            for event, ref in [('pull_request','refs/heads/skager-staging-retest'),
                               ('push','refs/heads/staging'),('push','refs/tags/skager-staging-retest')]:
                with mock.patch.dict(os.environ, GITHUB_EVENT_NAME=event, GITHUB_REF=ref):
                    with self.assertRaisesRegex(ValueError,'trusted branch'):
                        retest.pinned_selection(self.path)
            for mutation in (dict(scope='all'),dict(schema=True),dict(archiveSha256='invalid')):
                with mock.patch.object(retest,'read_json',return_value=dict(self.request,**mutation)):
                    with self.assertRaises(ValueError):retest.pinned_selection(self.path)
            with self.assertRaisesRegex(ValueError,'committed installer'):
                retest.pinned_selection(self.path.with_name('another.json'))

    def test_push_cli_refuses_unpinned_prepare_and_broader_scope_before_effects(self):
        environment=dict(self.environment,GITHUB_ACTIONS='true',RUNNER_ENVIRONMENT='github-hosted')
        for arguments in (['prepare','--producer-run-id','37330218586','--producer-run-attempt','1'],
                          ['test','--scope','all'],['test','--scope','installer']):
            with mock.patch.dict(os.environ,environment,clear=True), \
                 mock.patch.object(retest.sys,'platform','win32'), \
                 mock.patch.object(retest.sys,'argv',['retest']+arguments), \
                 mock.patch.object(retest,'prepare') as prepare, \
                 mock.patch.object(retest,'run_checks') as checks:
                with self.assertRaisesRegex(ValueError,'only the pinned'):
                    retest.main()
                prepare.assert_not_called();checks.assert_not_called()

    def test_prepare_keeps_exact_artifact_and_product_harness_separate(self):
        request = self.request
        with mock.patch.dict(os.environ,self.environment,clear=True), \
             mock.patch.object(retest,'fetch',return_value={'archiveSha256':request['archiveSha256']}) as fetch, \
             mock.patch.object(retest.subprocess,'check_output',return_value='f'*40), \
             mock.patch.object(retest,'restore') as restore:
            self.assertEqual(retest.prepare(None,None,self.path),request['artifact']['headSha'])
            self.assertEqual(fetch.call_args.args[1],request['artifact'])
            self.assertEqual(fetch.call_args.kwargs,dict(kind='staging'))
            arguments=restore.call_args.args
            self.assertEqual(arguments[2],request['archiveSha256'])
            self.assertEqual(arguments[3]['commit'],request['artifact']['headSha'])
            self.assertEqual(arguments[3]['runId'],'37330218586')
            self.assertEqual(arguments[3]['runAttempt'],'1')
            self.assertEqual(arguments[4],'f'*40)
            fetch.return_value={'archiveSha256':'e'*64};restore.reset_mock()
            with self.assertRaisesRegex(ValueError,'inner retained archive'):
                retest.prepare(None,None,self.path)
            restore.assert_not_called()

    def test_installer_then_existing_chart_command_and_failure_stops(self):
        for failure in (None,0,1):
            report=dict(status='running',productCommit=COMMIT,releaseQualification=False)
            calls=[]
            def execute(command,**kwargs):
                calls.append(command)
                if len(calls)-1==failure:raise subprocess.CalledProcessError(1,command)
            with mock.patch.object(retest.subprocess,'run',side_effect=execute):
                if failure is None:retest.run_checks('installer-charts',report,lambda:None)
                else:
                    with self.assertRaises(subprocess.CalledProcessError):
                        retest.run_checks('installer-charts',report,lambda:None)
            self.assertEqual(calls[0],[retest.sys.executable,str(retest.ROOT/'tools/smoke-installer-windows.py'),
                '--mode','staging','--compiled-input-receipt',str(retest.ROOT/'evidence/local/staging-inputs.json')])
            if failure!=0:
                self.assertEqual(calls[1],[retest.sys.executable,str(retest.ROOT/'tools/smoke-charts.py')])
            self.assertEqual([c['status'] for c in report['checks']],
                             ['passed','passed'] if failure is None else ['failed','not-run'] if failure==0 else ['passed','failed'])
            self.assertEqual(report['status'],'passed' if failure is None else 'failed')
            self.assertFalse(report['releaseQualification'])

    def test_workflow_keeps_dispatch_and_exact_push_without_build_or_promotion(self):
        import yaml
        checkout=Path(subprocess.check_output(['git','-C',str(retest.ROOT),'rev-parse','--show-toplevel'],text=True).strip())
        # BaseLoader keeps GitHub's 'on' as a string instead of YAML1.1 bool.
        workflow=yaml.load((checkout/'.github/workflows/skager-staging-retest.yml').read_text(),Loader=yaml.BaseLoader)
        self.assertEqual(set(workflow['on']),{'push','workflow_dispatch'})
        self.assertEqual(workflow['on']['push']['branches'],['skager-staging-retest'])
        self.assertIn('installer-charts',workflow['on']['workflow_dispatch']['inputs']['scope']['options'])
        self.assertEqual(workflow['env']['SKAGER_DESIGN_VALIDATION'],'false')
        steps=workflow['jobs']['retest']['steps']
        test=next(x for x in steps if 'RETEST_SCOPE' in x.get('env',{}))
        self.assertEqual(test['env']['RETEST_SCOPE'],"${{ github.event_name == 'push' && 'installer-charts' || inputs.scope }}")
        self.assertNotIn('GH_TOKEN',test['env'])
        for step in steps:
            self.assertNotRegex(step.get('run',''),r'(?i)(build-pristine|cmake|msbuild|makensis|gh release|package-preview|package-alpha)')


if __name__ == '__main__':
    unittest.main()

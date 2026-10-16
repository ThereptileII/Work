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
                          ['test','--scope','all'],['test','--scope','installer'],
                          ['test','--scope','installer-lifecycle'],
                          ['test','--scope','installer-startup-diagnostic']):
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

    def test_lifecycle_runs_only_existing_installer_matrix_with_prepared_inputs(self):
        for failure in (None, subprocess.CalledProcessError(1, ['installer'])):
            report=dict(status='running',productCommit=COMMIT,releaseQualification=False)
            with mock.patch.object(retest.subprocess,'run',side_effect=failure) as run:
                if failure is None:
                    retest.run_checks('installer-lifecycle',report,lambda:None)
                else:
                    with self.assertRaises(subprocess.CalledProcessError):
                        retest.run_checks('installer-lifecycle',report,lambda:None)
            run.assert_called_once_with([retest.sys.executable,
                str(retest.ROOT/'tools/smoke-installer-windows.py'),'--mode','production',
                '--compiled-input-receipt',str(retest.ROOT/'evidence/local/staging-inputs.json'),
                '--prepared-prior-fixtures'],check=True)
            status='failed' if failure else 'passed'
            self.assertEqual(report['checks'],[dict(name='installer-lifecycle',status=status)])
            self.assertEqual(report['status'],status)
            self.assertFalse(report['releaseQualification'])

    def test_lifecycle_cli_refuses_each_download_credential_before_test_effects(self):
        for name in ('GH_TOKEN','GITHUB_TOKEN','OPENNAV_ARTIFACT_TOKEN'):
            environment=dict(GITHUB_ACTIONS='true',RUNNER_ENVIRONMENT='github-hosted',
                             GITHUB_EVENT_NAME='workflow_dispatch',**{name:'inert test sentinel'})
            with mock.patch.dict(os.environ,environment,clear=True), \
                 mock.patch.object(retest.sys,'platform','win32'), \
                 mock.patch.object(retest.sys,'argv',['retest','test','--scope','installer-lifecycle']), \
                 mock.patch.object(retest,'run_checks') as checks:
                with self.assertRaisesRegex(ValueError,'Remove Actions credentials'):
                    retest.main()
                checks.assert_not_called()

    def test_workflow_keeps_dispatch_and_exact_push_without_build_or_promotion(self):
        import yaml
        checkout=Path(subprocess.check_output(['git','-C',str(retest.ROOT),'rev-parse','--show-toplevel'],text=True).strip())
        # BaseLoader keeps GitHub's 'on' as a string instead of YAML1.1 bool.
        workflow=yaml.load((checkout/'.github/workflows/skager-staging-retest.yml').read_text(),Loader=yaml.BaseLoader)
        self.assertEqual(set(workflow['on']),{'push','workflow_dispatch'})
        self.assertEqual(workflow['on']['push']['branches'],['skager-staging-retest'])
        self.assertIn('installer-charts',workflow['on']['workflow_dispatch']['inputs']['scope']['options'])
        self.assertIn('installer-lifecycle',workflow['on']['workflow_dispatch']['inputs']['scope']['options'])
        self.assertIn('installer-startup-diagnostic',workflow['on']['workflow_dispatch']['inputs']['scope']['options'])
        self.assertEqual(workflow['env']['SKAGER_DESIGN_VALIDATION'],'false')
        steps=workflow['jobs']['retest']['steps']
        self.assertEqual(steps[0]['with']['persist-credentials'],'false')
        preflight=next(x for x in steps if 'host-preflight' in x.get('run',''))
        self.assertEqual(preflight['if'], "${{ github.event_name == 'workflow_dispatch' && inputs.scope == 'installer-startup-diagnostic' }}")
        self.assertLess(steps.index(preflight), next(i for i, x in enumerate(steps) if x.get('id') == 'prepare'))
        self.assertNotIn('GH_TOKEN', preflight.get('env', {}))
        self.assertIn('if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }', preflight['run'])
        test=next(x for x in steps if 'RETEST_SCOPE' in x.get('env',{}))
        self.assertEqual(test['env']['RETEST_SCOPE'],"${{ github.event_name == 'push' && 'installer-charts' || inputs.scope }}")
        self.assertNotIn('GH_TOKEN',test['env'])
        self.assertNotIn('GITHUB_TOKEN',test['env'])
        self.assertNotIn('OPENNAV_ARTIFACT_TOKEN',test['env'])
        prefetch=next(x for x in steps if 'OPENNAV_ARTIFACT_TOKEN' in x.get('env',{}))
        self.assertEqual(prefetch['if'],"${{ github.event_name == 'workflow_dispatch' && inputs.scope == 'installer-lifecycle' }}")
        self.assertEqual(prefetch['env'],dict(OPENNAV_ARTIFACT_TOKEN='${{ github.token }}'))
        self.assertEqual(prefetch['run'].splitlines(),[
            'python opennav-x/tools/build-installer-prior-fixture.py',
            'if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }',
            'python opennav-x/tools/build-installer-prior-fixture.py --early-beta2-layout',
            'if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }'])
        self.assertLess(next(i for i,x in enumerate(steps) if x.get('id')=='prepare'),steps.index(prefetch))
        self.assertLess(steps.index(prefetch),steps.index(test))
        cold=workflow['jobs']['installer-cold-hold']
        self.assertEqual(cold['if'],prefetch['if'])
        self.assertNotIn('needs',cold)
        self.assertEqual(cold['runs-on'],'windows-2022')
        self.assertEqual(cold['steps'][0]['with']['persist-credentials'],'false')
        native=next(step for step in cold['steps'] if 'run' in step)
        self.assertEqual(native['shell'],'powershell')
        self.assertNotIn('env',native)
        self.assertIn('powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass -File opennav-x/tools/boat/test-hold-incompatible-rtlsdr.ps1 -Evidence ',native['run'])
        self.assertNotIn('-PortableContracts',native['run'])
        self.assertIn('if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }',native['run'])
        self.assertIn('helperSha256 = (Get-FileHash -Algorithm SHA256',native['run'])
        self.assertIn('testSha256 = (Get-FileHash -Algorithm SHA256',native['run'])
        self.assertIn("foreach ($Name in @('hold-incompatible-rtlsdr.ps1', 'test-hold-incompatible-rtlsdr.ps1'))",native['run'])
        self.assertIn("Copy-Item -LiteralPath (Join-Path 'opennav-x/tools/boat' $Name) -Destination (Join-Path $EvidenceDirectory $Name) -ErrorAction Stop",native['run'])
        for field in ('preparationSha256','commonSha256','profileLockSha256'):
            self.assertIn(field+' = (Get-FileHash -Algorithm SHA256',native['run'])
        self.assertEqual(cold['steps'][-1]['if'],'always()')
        self.assertIn('run${{ github.run_id }}-attempt${{ github.run_attempt }}',cold['steps'][-1]['with']['name'])
        for step in steps:
            self.assertNotRegex(step.get('run',''),r'(?i)(build-pristine|cmake|msbuild|makensis|gh release|package-preview|package-alpha)')

    def test_diagnostic_never_accepts_or_continues_even_if_child_returns_zero(self):
        for failure in (None, subprocess.CalledProcessError(1, ['diagnostic'])):
            report=dict(status='running', productCommit=COMMIT)
            with mock.patch.object(retest.subprocess, 'run', side_effect=failure) as run:
                with self.assertRaises((ValueError, subprocess.CalledProcessError)):
                    retest.run_checks('installer-startup-diagnostic', report, lambda: None)
            self.assertEqual(run.call_count, 1)
            self.assertEqual(run.call_args.args[0][-1], '--startup-supervisor-diagnostic')
            self.assertEqual(report['status'], 'failed')
            self.assertEqual(report['checks'], [dict(name='installer-startup-diagnostic', status='failed')])
            self.assertTrue(report['diagnosticOnly']); self.assertFalse(report['releaseQualification'])


if __name__ == '__main__':
    unittest.main()

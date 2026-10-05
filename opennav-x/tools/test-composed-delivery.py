#!/usr/bin/env python3
"""Offline composed-delivery integration; inert bytes and fake GitHub only."""
import base64
import copy
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch

import github_release_delivery as delivery
import staging_composition as composition
import yaml


def load(name):
    spec = importlib.util.spec_from_file_location(name.replace('-', '_'), Path(__file__).with_name(name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


legacy = load('test-github-release-delivery')


class ComposedGitHub(legacy.FakeGitHub):
    def __init__(self, request, raw, execution):
        super().__init__()
        self.repo = composition.REPOSITORY;self.base = 'repos/' + self.repo
        self.request = request;self.raw = raw
        self.runs = {};self.run_jobs = {};self.artifacts = {}
        for identity, workflow, conclusion in ((request['producer'],composition.BASELINE,'failure'),
                (request['retest'],composition.RETEST,'success'),(execution,composition.WORKFLOW,None)):
            run = dict(id=int(identity['runId']),run_attempt=int(identity['runAttempt']),
                       head_sha=identity['commit'],path=workflow,event='push',
                       head_repository={'full_name':self.repo},conclusion=conclusion,
                       status='completed' if conclusion else 'in_progress')
            if workflow==composition.WORKFLOW:run['head_branch']='skager-staging-compose'
            self.runs[identity['runId']]=run
        def job(identity,name,conclusion,ident):
            return dict(id=ident,name=name,status='completed',conclusion=conclusion,
                        run_id=int(identity['runId']),run_attempt=int(identity['runAttempt']),head_sha=identity['commit'])
        producer=request['producer'];retest=request['retest']
        self.run_jobs[producer['runId']]=[job(producer,name,'success',1000+i)
                                           for i,name in enumerate(composition.REQUIRED_JOBS)]
        self.run_jobs[producer['runId']].append(job(producer,composition.RUNTIME_JOB,'failure',int(request['original']['jobId'])))
        self.run_jobs[retest['runId']]=[job(retest,'retest','success',int(retest['jobId']))]
        self.run_jobs[execution['runId']]=[job(execution,composition.ASSEMBLE_JOB,'success',999)]
        for pin,identity in ((request['build'],producer),(request['original'],producer),(retest,retest)):
            self.artifacts[pin['artifactId']]=dict(id=int(pin['artifactId']),name=pin['artifactName'],
                digest=pin['artifactDigest'],size_in_bytes=123,expired=False,
                workflow_run=dict(id=int(identity['runId']),head_sha=identity['commit']))

    def api(self,endpoint):
        if '/contents/' in endpoint:
            self.calls.append(('api',endpoint))
            return dict(type='file',encoding='base64',size=len(self.raw),content=base64.b64encode(self.raw).decode())
        if '/actions/artifacts/' in endpoint:
            self.calls.append(('api',endpoint));return self.artifacts[endpoint.rsplit('/',1)[1]]
        if '/actions/runs/' in endpoint:
            run=endpoint.split('/actions/runs/')[1].split('/')[0]
            if run in self.runs:
                self.calls.append(('api',endpoint));return self.runs[run]
        return super().api(endpoint)

    def pages(self,endpoint):
        run=endpoint.split('/actions/runs/')[1].split('/')[0]
        if run in self.run_jobs:
            self.calls.append(('pages',endpoint));return self.run_jobs[run]
        return super().pages(endpoint)


class DeliveryIntegration(unittest.TestCase):
    def setUp(self):
        self.base = legacy.DeliveryTests()
        self.base.setUp()
        self.addCleanup(self.base.doCleanups)

    def composed(self):
        base=self.base;product=legacy.COMMIT;retest='2'*40
        execution=dict(commit='3'*40,runId='44',runAttempt='1')
        def pin(ident,name):return dict(artifactId=str(ident),artifactName=name,artifactDigest='sha256:'+'a'*64)
        def reports(names):return [dict(path=name,sha256='b'*64) for name in sorted(names)]
        request=dict(schema=1,repository=composition.REPOSITORY,
            producer=dict(commit=product,runId='42',runAttempt='1'),
            build=dict(pin(11,f'staging-build-{product}-run42-attempt1'),archiveSha256='c'*64),
            original=dict(pin(12,f'windows-qualification-{product}-attempt1'),jobId='900',
                          prefixReports=reports(composition.PREFIX_REPORTS)),
            retest=dict(pin(13,f'staging-retest-evidence-42-attempt1-harness{retest}-run43-attempt1'),
                        commit=retest,runId='43',runAttempt='1',jobId='901',reports=reports(composition.RETEST_REPORTS)))
        raw=json.dumps(request,sort_keys=True).encode()
        self.q=composition.qualification(request,composition.digest(raw),execution)
        base.qualification=self.q;base.support.update(runId='44')
        self.reseal()
        base.gh=ComposedGitHub(request,raw,execution)
        os.environ.update(GITHUB_SHA=execution['commit'],GITHUB_RUN_ID='44',GITHUB_RUN_ATTEMPT='1',
                          GITHUB_REF='refs/heads/skager-staging-compose',GITHUB_REPOSITORY=composition.REPOSITORY)
        return base

    def reseal(self):
        base=self.base
        for name,value in [('QUALIFICATION.json',base.qualification),('RETEST_SUPPORT.json',base.support)]:
            (base.root/name).write_text(json.dumps(value))
        (base.root/'RELEASE.json').unlink();base.record=base.fixture.create()

    def test_v2_real_validation_and_provenance_precede_draft_publication(self):
        base=self.composed()
        delivery.publish_staging(base.gh,base.root)
        calls=base.gh.calls
        create=next(i for i,item in enumerate(calls) if item[0]=='create')
        self.assertLess(calls.index(('pages',base.gh.base+'/actions/runs/44/attempts/1/jobs')),create)
        self.assertLess(calls.index(('api',base.gh.base+'/actions/artifacts/13')),create)
        release=base.gh.releases[base.tag]
        self.assertTrue(release['draft']);self.assertTrue(release['prerelease'])
        self.assertEqual(release['target_commitish'],legacy.COMMIT)
        self.assertEqual(base.gh.runs['42']['conclusion'],'failure')

    def test_v2_rejects_current_identity_and_invalid_provenance_before_writes(self):
        base=self.composed()
        for key,value in [('GITHUB_SHA','f'*40),('GITHUB_RUN_ID','45'),('GITHUB_RUN_ATTEMPT','2')]:
            with self.subTest(key=key),patch.dict(os.environ,{key:value}):
                with self.assertRaisesRegex(ValueError,'Current composition identity'):
                    delivery.publish_staging(base.gh,base.root)
                self.assertEqual(base.gh.calls,[])
        original=copy.deepcopy(base.gh.run_jobs['44'])
        for state in ('failure','cancelled','skipped'):
            base.gh.run_jobs['44'][0]['conclusion']=state;base.gh.calls.clear()
            with self.assertRaises(ValueError):delivery.publish_staging(base.gh,base.root)
            self.assertFalse(any(call[0] in ('create','upload') for call in base.gh.calls))
        base.gh.run_jobs['44']=original
        base.gh.raw=b'{}';base.gh.calls.clear()
        with self.assertRaisesRegex(ValueError,'committed request'):
            delivery.publish_staging(base.gh,base.root)
        self.assertFalse(any(call[0] in ('create','upload') for call in base.gh.calls))

    def test_v2_invalid_local_schema_cannot_reach_api(self):
        base=self.composed();original=copy.deepcopy(base.qualification)
        for mutate in (lambda q:q.update(schemaVersion=2.0),lambda q:q.update(publicAccess=True),
                       lambda q:q['gates'].update(installer='failed'),
                       lambda q:q['composition']['producer'].update(commit='f'*40)):
            base.qualification=copy.deepcopy(original);mutate(base.qualification);self.reseal()
            with self.assertRaises(ValueError):delivery.publish_staging(base.gh,base.root)
            self.assertEqual(base.gh.calls,[])

    def test_v2_fetch_requires_completed_composition_and_separates_support_run(self):
        base=self.composed();base.publish()
        for status,conclusion in [('in_progress',None),('completed','failure'),('completed','cancelled')]:
            base.gh.runs['44'].update(status=status,conclusion=conclusion)
            target=base.work/('refused-'+str(conclusion))
            with self.assertRaises(ValueError):delivery.fetch_staging(base.gh,base.tag,target)
            self.assertFalse(target.exists())
        base.gh.runs['44'].update(status='completed',conclusion='success')
        output=base.work/'outputs';os.environ['GITHUB_OUTPUT']=str(output)
        target=base.work/'download';record=delivery.fetch_staging(base.gh,base.tag,target)
        self.assertEqual(record['commit'],legacy.COMMIT);self.assertEqual(record['runId'],'42')
        self.assertIn('run_id=42\n',output.read_text());self.assertIn('support_run_id=44\n',output.read_text())
        self.assertEqual({p.name:p.read_bytes() for p in target.iterdir()},
                         {p.name:p.read_bytes() for p in base.root.iterdir()})

    def test_v2_production_still_needs_approval_and_uses_composed_provenance(self):
        base=self.composed();base.publish()
        base.gh.runs['44'].update(status='completed',conclusion='success')
        report,_=base.report()
        for change in ({'instruction':''},{'confirmation':''}):
            with self.assertRaises(ValueError):base.promote(report,**change)
            self.assertEqual(base.gh.calls,[])
        with patch.dict(os.environ,GITHUB_EVENT_NAME='push'):
            with self.assertRaisesRegex(ValueError,'Manual production dispatch'):base.promote(report)
            self.assertEqual(base.gh.calls,[])
        base.promote(report)
        release=base.gh.releases['skager-production-'+base.record['candidateId']]
        self.assertTrue(release['draft']);self.assertFalse(release['prerelease'])
        self.assertEqual(release['target_commitish'],legacy.COMMIT)
        self.assertIn(('api',base.gh.base+'/actions/runs/44/attempts/1'),base.gh.calls)
        self.assertEqual(base.gh.runs['42']['conclusion'],'failure')

    def test_v1_still_requires_current_producer_and_same_run_support(self):
        base = self.base
        for field, value in [('GITHUB_SHA', 'e'*40), ('GITHUB_RUN_ID', '777'), ('GITHUB_RUN_ATTEMPT', '2')]:
            with self.subTest(field=field), patch.dict(os.environ, {field:value}):
                with self.assertRaisesRegex(ValueError, 'Current producer identity'):
                    delivery.publish_staging(base.gh, base.root)
                self.assertEqual(base.gh.calls, [])
        base.support['runId'] = '777'
        (base.root/'RETEST_SUPPORT.json').write_text(json.dumps(base.support))
        (base.root/'RELEASE.json').unlink();base.fixture.create()
        with self.assertRaisesRegex(ValueError, 'Retest support identity'):
            delivery.local_record(base.root)

    def test_v1_failed_producer_still_blocks_fetch(self):
        base = self.base;base.publish()
        base.gh.stage_run['conclusion'] = 'failure'
        target = base.work/'refused'
        with self.assertRaisesRegex(ValueError, 'not completed successfully'):
            delivery.fetch_staging(base.gh, base.tag, target)
        self.assertFalse(target.exists())

    def test_composition_workflow_remains_draft_only_and_has_no_build(self):
        checkout = Path(subprocess.check_output(['git','-C',str(Path(__file__).resolve().parents[1]),
                         'rev-parse','--show-toplevel'],text=True).strip())
        workflows = checkout/'.github/workflows'
        compose = yaml.load((workflows/'skager-staging-compose.yml').read_text(),Loader=yaml.BaseLoader)
        self.assertEqual(compose['permissions'],{'contents':'read','actions':'read'})
        self.assertEqual(compose['on']['push']['branches'],['skager-staging-compose'])
        self.assertEqual(set(compose['on']),{'push','workflow_dispatch'})
        self.assertEqual(compose['jobs']['publish']['needs'],'assemble')
        self.assertEqual(compose['jobs']['publish']['permissions'],{'contents':'write','actions':'read'})
        self.assertNotIn('permissions',compose['jobs']['assemble'])
        self.assertEqual(compose['env']['SKAGER_DESIGN_VALIDATION'],'false')
        for job in compose['jobs'].values():
            self.assertNotIn('continue-on-error',job)
            for step in job['steps']:
                self.assertNotIn('continue-on-error',step)
                self.assertNotRegex(step.get('run',''),r'(?i)(cmake|msbuild|makensis|go build|build-pristine|publish-production|gh release)')
        production = yaml.load((workflows/'skager-production.yml').read_text(),Loader=yaml.BaseLoader)
        self.assertEqual(set(production['on']),{'workflow_dispatch'})
        self.assertEqual(production['on']['workflow_dispatch']['inputs']['instruction']['required'],'true')
        self.assertEqual(production['on']['workflow_dispatch']['inputs']['confirmation']['required'],'true')
        support = next(step for step in production['jobs']['qualify']['steps']
                       if step.get('with',{}).get('path')=='retained-support')
        self.assertEqual(support['with']['run-id'],'${{ needs.inspect.outputs.support_run_id }}')


if __name__ == '__main__':
    unittest.main()

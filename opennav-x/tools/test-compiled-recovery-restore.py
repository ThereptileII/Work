#!/usr/bin/env python3
"""Inert exact recovery trust/source contracts. No network, compiler or app launch."""
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import compiled_recovery_restore as r


def write_zip(path, values):
    with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
        for name, data in values.items():
            info = r.sealed.archive_entry(name)
            # These adversarial fixtures must put the requested raw name on the
            # wire: ZipInfo's constructor otherwise sanitizes it on Windows.
            info.filename = info.orig_filename = name
            z.writestr(info, data)


def record(data): return dict(size=len(data), sha256=r.digest(data))


class FakeGitHub:
    repo = r.REPOSITORY
    base = 'repos/' + r.REPOSITORY
    def __init__(self):
        endpoint = f'{self.base}/actions/runs/{r.RUN}/attempts/{r.ATTEMPT}'
        self.data = {endpoint: dict(id=int(r.RUN), run_attempt=1, head_sha=r.COMMIT, path=r.WORKFLOW,
            head_repository=dict(full_name=r.REPOSITORY), status='completed', conclusion='failure', event='push', head_branch='staging')}
        steps = [dict(name=n, number=i+1, status='completed', conclusion='success') for i,n in enumerate(r.REQUIRED_STEPS)]
        steps += [dict(name=r.FAILURE_STEP, number=25, status='completed', conclusion='failure'),
                  dict(name='Seal compiled inputs before any desktop test can fail', number=26, status='completed', conclusion='skipped')]
        def job(name, result, number): return dict(id=number, name=name, run_id=int(r.RUN), run_attempt=1, head_sha=r.COMMIT, status='completed', conclusion=result)
        native = job('windows-integration', 'failure', 112143482519); native['steps'] = steps
        self.data[endpoint+'/jobs'] = [native, job('updater-contracts / verify (windows-2022, 386)', 'success', 123),
                                     job('Qualify retained native Staging inputs', 'skipped', 112158432202)]
        for pin in r.ARTIFACTS.values():
            self.data[f"{self.base}/actions/artifacts/{pin['artifactId']}"] = dict(id=int(pin['artifactId']), name=pin['artifactName'], digest=pin['artifactDigest'], expired=False,
                workflow_run=dict(id=int(pin.get('runId',r.RUN)), head_sha=pin.get('headSha',r.COMMIT)), size_in_bytes=100)
        pin=r.ARTIFACTS['sdk']; endpoint=f"{self.base}/actions/runs/{pin['runId']}/attempts/1"
        self.data[endpoint]=dict(id=int(pin['runId']), run_attempt=1, head_sha=pin['headSha'], path='.github/workflows/skager-windows-dependencies.yml',
            head_repository=dict(full_name=r.REPOSITORY), status='completed', conclusion='success', event='push')
        self.data[endpoint+'/jobs']=[dict(name='windows-dependencies', status='completed', conclusion='success', run_id=int(pin['runId']), run_attempt=1, head_sha=pin['headSha'])]
    def api(self, endpoint): return self.data[endpoint]
    def pages(self, endpoint): return self.data[endpoint]


class TrustTests(unittest.TestCase):
    def test_exact_failed_attempt_with_successful_prefix_and_sdk_is_accepted(self):
        self.assertEqual(set(r.authenticate(FakeGitHub())), set(r.ARTIFACTS))

    def test_failure_may_not_move_or_be_called_success(self):
        for field,value in [('conclusion','success'),('head_sha','f'*40),('run_attempt',2),('head_branch','other')]:
            gh=FakeGitHub();gh.data[f'{gh.base}/actions/runs/{r.RUN}/attempts/1'][field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):r.authenticate(gh)
        for name in r.REQUIRED_STEPS:
            gh=FakeGitHub();steps=gh.data[f'{gh.base}/actions/runs/{r.RUN}/attempts/1/jobs'][0]['steps']
            next(s for s in steps if s['name']==name)['conclusion']='skipped'
            with self.subTest(step=name),self.assertRaises(ValueError):r.authenticate(gh)
        for index in (0,1,2):
            gh=FakeGitHub();gh.data[f'{gh.base}/actions/runs/{r.RUN}/attempts/1/jobs'][index]['head_sha']='f'*40
            with self.assertRaises(ValueError):r.authenticate(gh)

    def test_artifact_and_sdk_authority_cannot_be_swapped(self):
        for kind,pin in r.ARTIFACTS.items():
            for field,value in [('digest','sha256:'+'0'*64),('expired',True),('name','other')]:
                gh=FakeGitHub();gh.data[f"{gh.base}/actions/artifacts/{pin['artifactId']}"][field]=value
                with self.subTest(kind=kind,field=field),self.assertRaises(ValueError):r.authenticate(gh)
        gh=FakeGitHub();pin=r.ARTIFACTS['sdk'];gh.data[f"{gh.base}/actions/runs/{pin['runId']}/attempts/1"]['conclusion']='failure'
        with self.assertRaises(ValueError):r.authenticate(gh)

    def test_duplicate_folded_fields_and_nonfinite_json_refuse(self):
        for raw in (b'{"a":1,"a":2}',b'{"a":1,"A":2}',b'{"x":{"a":1,"\\u0041":2}}',b'{"x":NaN}'):
            with self.assertRaises(ValueError):r.strict_json(raw)

    def test_zip_traversal_alias_links_and_collisions_refuse(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'bad.zip'
            for names in (('../outside',),('C:/outside',),('x\\y',),('CON.txt',),('.git/config',),('a','A'),('a','a/b')):
                write_zip(path,{n:b'x' for n in names})
                with zipfile.ZipFile(path) as z,self.subTest(names=names),self.assertRaises(ValueError):r.zip_index(z)
            info=r.sealed.archive_entry('link');info.external_attr=(0o120777<<16)
            with zipfile.ZipFile(path,'w') as z:z.writestr(info,b'../outside')
            with zipfile.ZipFile(path) as z,self.assertRaises(ValueError):r.zip_index(z)


    def test_windows_reader_normalization_of_real_raw_zip_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'raw.zip'
            # Exercise Windows ZipInfo behavior even when running this on Linux.
            with patch.object(zipfile.os, 'sep', '\\'):
                for raw, normalized in ((r'x\y','x/y'),('name\x00tail','name')):
                    write_zip(path,{raw:b'inert'})
                    self.assertIn(raw.encode(),path.read_bytes())
                    with zipfile.ZipFile(path) as z:
                        self.assertEqual(z.infolist()[0].orig_filename,raw)
                        self.assertEqual(z.infolist()[0].filename,normalized)
                        with self.assertRaisesRegex(ValueError,'normalized an unsafe raw name'):
                            r.zip_index(z)


class RestoreTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.base=Path(self.temp.name);self.repo=self.base/'repo';self.root=self.repo/'opennav-x';self.root.mkdir(parents=True)
        def git(where,*args): return subprocess.check_output(['git','-C',str(where),*args],stderr=subprocess.DEVNULL)
        self.git=git
        git(self.repo,'init','-q');git(self.repo,'config','user.email','inert@example.invalid');git(self.repo,'config','user.name','Inert')
        (self.repo/'.github/workflows').mkdir(parents=True);(self.repo/'.github/workflows/opennav-baseline.yml').write_text('inert recipe\n')
        (self.root/'app.cpp').write_text('original source\n');git(self.repo,'add','.');git(self.repo,'commit','-qm','inert product')
        commit=git(self.repo,'rev-parse','HEAD').decode().strip()
        upstream=self.root/'build/integration-source';upstream.mkdir(parents=True)
        git(upstream,'init','-q');git(upstream,'config','user.email','inert@example.invalid');git(upstream,'config','user.name','Inert')
        (upstream/'upstream.cpp').write_text('baseline\n');git(upstream,'add','.');git(upstream,'commit','-qm','inert upstream')
        upstream_commit=git(upstream,'rev-parse','HEAD').decode().strip();(upstream/'upstream.cpp').write_text('reviewed patch\n')
        self.patches=[]
        def pin(module,name,value):
            p=patch.object(module,name,value);p.start();self.addCleanup(p.stop)
        pin(r,'COMMIT',commit);pin(r,'PRODUCER',r.sealed.producer(commit,r.RUN,'1'));pin(r.recovery,'PINNED_UPSTREAM',upstream_commit)
        self.identity=dict(commit='f'*40,runId='999',runAttempt='1')
        compiled={}
        for variant in r.sealed.VARIANTS:
            for binary in ('opencpn.exe','opennav-restart.exe'):compiled[f'build/{variant}-install/{binary}']=b'inert; never executed'
            for header in ('config.h','OpenNavBuild.h'):compiled[f'build/{variant}-windows/include/{header}']=f'#define OPENNAV_BUILD_COMMIT "{commit}"\n'.encode()
        compiled[r.sealed.FEEDBACK_MANIFEST]=json.dumps(dict(schema=1,commit=commit,tests=[dict(name=n,path='D:/a/Work/Work/opennav-x/'+p) for n,p in sorted(r.sealed.FEEDBACK_BINARIES.items())])).encode()
        for name in r.sealed.FEEDBACK_BINARIES.values():compiled[name]=b'inert test; never executed'
        for name,data in compiled.items():p=self.root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
        retained=self.base/'retained';original=r.recovery.retain(self.root,retained,r.PRODUCER)
        with zipfile.ZipFile(retained/r.recovery.ARCHIVE) as z:manifest=r.strict_json(z.read(r.recovery.MANIFEST))
        self.manifest=manifest;self.source_files={x['path']:{k:x[k] for k in ('size','sha256')} for x in manifest['files']}
        for name in compiled:(self.root/name).unlink()
        self.compiled={n:record(v) for n,v in compiled.items()}
        late={'downloader-trust-windows/summary.json':b'{"status":"passed"}', 'ocharts-real-host-module/summary.json':b'{"status":"passed"}', 'peer-buffer-windows.log':b'passed inert'}
        updater={'install/'+n:b'inert updater boundary' for n in r.UPDATER_NAMES}
        sources={'openssl-3.5.9.tar.gz':record(b'inert source')};pin(r,'DEPENDENCY_SOURCES',sources)
        sdk={'payload/build/dependency-downloads/'+n:b'inert source' for n in sources}
        sdk['bundle.json']=json.dumps(dict(files={'build/dependency-downloads/'+n:dict(bytes=v['size'],sha256=v['sha256']) for n,v in sources.items()})).encode()
        self.paths={k:self.base/(k+'.zip') for k in r.ARTIFACTS}
        write_zip(self.paths['compiled'],{r.recovery.ARCHIVE:(retained/r.recovery.ARCHIVE).read_bytes(),'receipt.json':(retained/'receipt.json').read_bytes()})
        for kind,values in [('evidence',late),('updater',updater),('sdk',sdk)]:write_zip(self.paths[kind],values)
        artifacts=copy.deepcopy(r.ARTIFACTS)
        for kind,path in self.paths.items():artifacts[kind]['artifactDigest']='sha256:'+r.sealed.sha(path)
        pin(r,'ARTIFACTS',artifacts);pin(r,'INNER_SHA',original['archiveSha256']);pin(r,'MANIFEST_SHA',original['manifestSha256']);pin(r,'SOURCES_SHA',r.object_digest(manifest['sources']))
        pin(r,'COMPILED_FILES_SHA',r.object_digest(self.compiled))
        late_records={'evidence/local/'+n:record(d) for n,d in late.items()};updater_records={'build/updater-qualified/'+n:record(d) for n,d in updater.items()}
        pin(r,'LATE_FILES_SHA',r.object_digest(late_records));pin(r,'UPDATER_FILES_SHA',r.object_digest(updater_records))
        self.all=dict(self.compiled,**late_records,**updater_records,**{'build/dependency-downloads/'+n:v for n,v in sources.items()})
        pin(r,'ALL_FILES_SHA',r.object_digest(self.all));pin(r,'ALL_FILES_COUNT',len(self.all))
        # The binary is intentionally inert; the real verify function is exercised
        # separately against the authenticated native artifact, not launched here.
        self.validator=patch('updater_package.verify_updater_package',return_value=None);self.validator.start();self.addCleanup(self.validator.stop)
        self.receipt=self.base/'restored.json'

    def run_restore(self):return r.restore(self.root,self.paths,self.receipt,self.identity)

    def test_exact_source_and_every_restored_byte_are_bound(self):
        result=self.run_restore();r.validate_receipt(result);r.verify_files(self.root,result['files'])
        self.assertEqual(result['files'],self.all);self.assertEqual(result['producer'],r.PRODUCER)
        self.assertEqual(result['recovery'],self.identity);self.assertEqual(result['originalConclusion'],'failure')
        self.assertEqual(result['qualification'],'not-run');self.assertFalse((self.root/'source').exists())
        for mutate in ('origin','files','source','execution'):
            bad=copy.deepcopy(result)
            if mutate=='origin':bad['producer']['commit']='a'*40
            elif mutate=='files':bad['files'][next(iter(bad['files']))]['sha256']='0'*64
            elif mutate=='source':bad['sourceManifestSha256']='0'*64
            else:bad['recovery']['runId']=r.RUN
            with self.subTest(mutate=mutate),self.assertRaises(ValueError):r.validate_receipt(bad)
        with self.assertRaises(ValueError):self.run_restore()

    def test_product_or_integrated_source_change_refuses_before_copy(self):
        for path in (self.root/'app.cpp',self.root/'build/integration-source/upstream.cpp'):
            before=path.read_bytes();path.write_bytes(b'changed')
            with self.subTest(path=path),self.assertRaises(ValueError):self.run_restore()
            self.assertFalse(self.receipt.exists());self.assertFalse((self.root/'build/production-install/opencpn.exe').exists());path.write_bytes(before)

    def test_preexisting_output_and_redirected_parent_refuse(self):
        target=self.root/'build/production-install/opencpn.exe';target.write_bytes(b'keep me')
        with self.assertRaises(ValueError):self.run_restore()
        self.assertEqual(target.read_bytes(),b'keep me');target.unlink()
        parent=target.parent;parent.rmdir();outside=self.base/'outside';outside.mkdir()
        try:parent.symlink_to(outside,target_is_directory=True)
        except OSError:self.skipTest('Symlink creation unavailable')
        with self.assertRaises(ValueError):self.run_restore()
        self.assertFalse(list(outside.iterdir()))

    def test_wrong_execution_identity_refuses_before_any_restore(self):
        self.identity['runId']=r.RUN
        with self.assertRaises(ValueError):self.run_restore()
        self.assertFalse((self.root/'build/production-install/opencpn.exe').exists())
        self.assertFalse(self.receipt.exists())

    def test_outer_byte_tampering_each_input_refuses_before_mutation(self):
        for kind,path in self.paths.items():
            before=path.read_bytes();path.write_bytes(before+b'tampered')
            with self.subTest(kind=kind),self.assertRaises(ValueError):self.run_restore()
            self.assertFalse(self.receipt.exists());path.write_bytes(before)

    def test_restore_failure_keeps_partial_bytes_without_success_receipt(self):
        original=r.verify_files
        with patch.object(r,'verify_files',side_effect=ValueError('injected final verification failure')):
            with self.assertRaises(ValueError):self.run_restore()
        self.assertFalse(self.receipt.exists());original(self.root,self.all)
        with self.assertRaises(ValueError):self.run_restore()

    def test_source_reference_omission_and_changed_patch_identity_refuse(self):
        for altered in ([],self.manifest['sources'][:2]):
            bad=copy.deepcopy(self.manifest);bad['sources']=altered
            with self.assertRaises(ValueError):r.verify_sources(self.root,bad,self.source_files)
        (self.repo/'.github/workflows/opennav-baseline.yml').write_text('replacement helper workflow\n')
        with self.assertRaises(ValueError):self.run_restore()


if __name__=='__main__':unittest.main()

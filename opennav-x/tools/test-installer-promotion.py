#!/usr/bin/env python3
"""Focused retained-input and mode policy tests; no native acceptance claim."""
import argparse
import ast
import hashlib
import io
import importlib.util
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import tempfile
import unittest
from unittest import mock
import zipfile

import staging_build_inputs as staging_inputs
from hardware_output_policy import require_product_output_policy

SOURCE=Path(__file__).with_name('smoke-installer-windows.py')
TREE=ast.parse(SOURCE.read_text())
FUNCTIONS=ast.Module(body=[node for node in TREE.body if isinstance(node,ast.FunctionDef)
    and node.name in ('arguments','verify_prior_fixtures','archive_members','prepare_retained','prepare_source_engine','prepare_compiled','sha')],type_ignores=[])
ENV=dict(argparse=argparse,Path=Path,PurePosixPath=PurePosixPath,re=re,hashlib=hashlib,
         json=json,os=os,shutil=shutil,zipfile=zipfile,staging_inputs=staging_inputs,require_product_output_policy=require_product_output_policy)
exec(compile(FUNCTIONS,str(SOURCE),'exec'),ENV)
COMMIT='a'*40

def digest(data):return hashlib.sha256(data).hexdigest()
def zipped(entries):
    output=io.BytesIO()
    with zipfile.ZipFile(output,'w') as archive:
        for name,data in entries.items():
            # ZipInfo's constructor normalizes Windows backslashes. Assign
            # afterwards so malformed-member tests retain their actual bytes.
            entry=zipfile.ZipInfo()
            entry.filename=name
            archive.writestr(entry,data)
    return output.getvalue()

class Promotion(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)/'checkout'
        self.root.mkdir();self.inputs=Path(self.temp.name)/'retained';self.inputs.mkdir()
        (self.root/'tools').mkdir()
        (self.root/'tools/test-installer-missing-dll-selftest.ps1').write_text('current test helper')
        self.product={'commit':COMMIT,'test_fixtures':False,'build_purpose':'INSTALLED PRODUCT',
            'xnav_hardware_output_policy':'status-only','executable_sha256':digest(b'product')}
        self.entries={'app/opencpn.exe':b'product','docs/PRODUCT_BUILD.json':json.dumps(self.product).encode()}
        payload=zipped(self.entries)
        manifest={'commit':COMMIT,'payloadSha256':digest(payload),
            'files':[{'path':name,'sha256':digest(data)} for name,data in self.entries.items()]}
        self.support={'build/beta-installer/package.json':json.dumps(manifest).encode(),
            'build/beta-installer/payload.zip':payload,
            'build/production-windows/include/config.h':b'original profile version',
            'build/xnav-install/opencpn.exe':b'fixture',
            'build/xnav-windows/include/config.h':b'fixture profile version',
            'build/xnav-install/wx.dll':b'original fixture dependency'}
        self.engine='opennav-x/installer/windows/Lifecycle.ps1'
        self.source_files={self.engine:b'candidate engine',
            'opennav-x/installer/windows/UpdateSupervisor.ps1':b'candidate supervisor',
            'opennav-x/installer/windows/UpdateTransaction.ps1':b'candidate transaction'}
        self.reference={'productCommit':COMMIT,'files':{name:{'sha256':digest(data)}
                                                     for name,data in self.source_files.items()}}
        self.build_inputs()
    def tearDown(self):self.temp.cleanup()
    def build_inputs(self,source_commit=COMMIT):
        self.reference['productCommit']=source_commit
        files={'SKAGER-Beta2-Setup.exe':b'original setup',
            'SKAGER-Beta2-Portable-Recovery.zip':zipped({'SKAGER-Beta2-Portable-Recovery/'+name:data for name,data in self.entries.items()}),
            'SKAGER-Beta2-Retest-Support.zip':zipped(self.support),
            'SKAGER-Beta2-source.zip':zipped({**self.source_files,'SOURCE_REFERENCE.json':json.dumps(self.reference).encode()})}
        for name,data in files.items():(self.inputs/name).write_bytes(data)
        (self.inputs/'SHA256SUMS.txt').write_text(''.join(digest(data)+'  '+name+'\n' for name,data in files.items()))
    def restore(self):return ENV['prepare_retained'](self.inputs,self.root,COMMIT)
    def test_staging_is_default_and_production_is_explicit(self):
        self.assertEqual(ENV['arguments']([]).mode,'staging')
        self.assertEqual(ENV['arguments'](['--mode','production']).mode,'production')
    def test_tail_requires_authenticated_inputs_and_cannot_fetch_historical_fixtures(self):
        from contextlib import redirect_stderr
        for args in (['--mode','lifecycle-tail'],
                     ['--mode','lifecycle-tail','--retained-package','x','--expected-commit',COMMIT],
                     ['--mode','lifecycle-tail','--compiled-input-receipt','x','--prepared-prior-fixtures']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):ENV['arguments'](args)
        args=ENV['arguments'](['--mode','lifecycle-tail','--compiled-input-receipt','x'])
        self.assertEqual(args.mode,'lifecycle-tail')
        self.assertFalse(args.prepared_prior_fixtures)

    def test_retained_inputs_require_complete_paired_identity(self):
        from contextlib import redirect_stderr
        for args in (['--prepare-only'],['--retained-package','x'],['--expected-commit',COMMIT],
                     ['--retained-package','x','--expected-commit','abc']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):ENV['arguments'](args)
    def test_restores_same_bytes_and_separate_current_helper(self):
        report=self.restore()
        self.assertEqual(report['product_commit'],COMMIT)
        self.assertEqual((self.root/'build/production-install/opencpn.exe').read_bytes(),b'product')
        self.assertEqual((self.root/'build/xnav-install/wx.dll').read_bytes(),b'original fixture dependency')
        self.assertEqual((self.root/'build/retained-source/installer/windows/Lifecycle.ps1').read_bytes(),b'candidate engine')
        self.assertEqual((self.root/'build/retained-source/tools/test-installer-missing-dll-selftest.ps1').read_text(),'current test helper')
    def compiled_fixture(self):
        archive=self.root/'build/developer-preview/SKAGER-Beta2-source.zip'
        archive.parent.mkdir(parents=True)
        shutil.copyfile(self.inputs/archive.name,archive)
        product=self.root/staging_inputs.PACKAGE_ROOT/'docs/PRODUCT_BUILD.json'
        product.parent.mkdir(parents=True);product.write_text(json.dumps(self.product))
        executable=self.root/'build/production-install/opencpn.exe'
        executable.parent.mkdir(parents=True);executable.write_bytes(b'product')
        self.compiled_receipt=self.root/'evidence/local/staging-inputs.json'
        self.compiled_receipt.parent.mkdir(parents=True)
        self.restored=dict(schema=1,kind=staging_inputs.KIND,status='restored',qualification='not-run',
            producer=staging_inputs.producer(COMMIT,'123','1'),harnessCommit='b'*40,
            workspaceRoot=str(self.root.resolve()),sourceArchiveSha256=digest(archive.read_bytes()),
            archiveSha256='c'*64,manifestSha256='d'*64,files=42)
        self.save_compiled_receipt()
        return executable
    def save_compiled_receipt(self):
        self.compiled_receipt.write_text(json.dumps(self.restored))
    def prepare_compiled(self):
        return ENV['prepare_compiled'](self.compiled_receipt,self.root,'b'*40)
    def test_compiled_retest_preserves_product_engine_and_current_helper_without_binary_restore(self):
        executable=self.compiled_fixture()
        before=(executable.read_bytes(),executable.stat().st_mtime_ns)
        report=self.prepare_compiled()
        self.assertEqual(report['product_commit'],COMMIT)
        self.assertEqual((self.root/'build/retained-source/installer/windows/Lifecycle.ps1').read_bytes(),b'candidate engine')
        self.assertEqual((self.root/'build/retained-source/tools/test-installer-missing-dll-selftest.ps1').read_text(),'current test helper')
        self.assertEqual((executable.read_bytes(),executable.stat().st_mtime_ns),before)
        self.assertEqual(report['installer_source_sha256'],digest(b'candidate engine'))
        self.assertEqual(report['installer_source_files_sha256'],{name:digest(data) for name,data in self.source_files.items()})
        for name,data in self.source_files.items():
            self.assertEqual((self.root/'build/retained-source/installer/windows'/PurePosixPath(name).name).read_bytes(),data)
        self.assertEqual(report['compiled_input_producer'],self.restored['producer'])
    def test_compiled_receipt_refuses_wrong_root_harness_status_and_producer(self):
        self.compiled_fixture()
        for key,bad in (('workspaceRoot',str(self.root.parent)),('harnessCommit','e'*40),
                        ('status','sealed'),('qualification','passed'),
                        ('producer',{**self.restored['producer'],'commit':'e'*40})):
            original=self.restored[key]
            with self.subTest(key=key):
                self.restored[key]=bad;self.save_compiled_receipt()
                with self.assertRaises(ValueError):self.prepare_compiled()
                self.assertFalse((self.root/'build/retained-source').exists())
            self.restored[key]=original
    def test_compiled_source_substitution_is_refused_before_engine_extraction(self):
        self.compiled_fixture()
        (self.root/'build/developer-preview/SKAGER-Beta2-source.zip').write_bytes(b'changed archive')
        with self.assertRaisesRegex(ValueError,'archive checksum'):self.prepare_compiled()
        self.assertFalse((self.root/'build/retained-source').exists())
    def test_compiled_source_engine_must_match_its_reference_and_revision(self):
        self.compiled_fixture()
        archive=self.root/'build/developer-preview/SKAGER-Beta2-source.zip'
        for reference in ({**self.reference,'productCommit':'e'*40},
                          {**self.reference,'files':{self.engine:{'sha256':'f'*64}}}):
            archive.write_bytes(zipped({**self.source_files,
                'SOURCE_REFERENCE.json':json.dumps(reference).encode()}))
            self.restored['sourceArchiveSha256']=digest(archive.read_bytes());self.save_compiled_receipt()
            with self.assertRaises(ValueError):self.prepare_compiled()
            self.assertFalse((self.root/'build/retained-source').exists())
    def test_compiled_source_requires_both_original_engine_dependencies_before_extraction(self):
        self.compiled_fixture()
        archive=self.root/'build/developer-preview/SKAGER-Beta2-source.zip'
        for name in self.source_files:
            for mutation in ('missing','changed','case-collision','symlink'):
                with self.subTest(name=name,mutation=mutation):
                    files={**self.source_files,'SOURCE_REFERENCE.json':json.dumps(self.reference).encode()}
                    if mutation=='missing':files.pop(name)
                    elif mutation=='changed':files[name]=b'current helper branch substitution'
                    elif mutation=='case-collision':files[name.upper()]=files[name]
                    data=zipped(files)
                    if mutation=='symlink':
                        output=io.BytesIO()
                        with zipfile.ZipFile(output,'w') as source:
                            for path,value in files.items():
                                entry=zipfile.ZipInfo(path)
                                if path==name:entry.external_attr=0o120777<<16
                                source.writestr(entry,value)
                        data=output.getvalue()
                    archive.write_bytes(data)
                    self.restored['sourceArchiveSha256']=digest(data);self.save_compiled_receipt()
                    with self.assertRaises(ValueError):self.prepare_compiled()
                    self.assertFalse((self.root/'build/retained-source').exists())
    def test_compiled_input_mode_is_mutually_exclusive_with_release_restore(self):
        from contextlib import redirect_stderr
        args=ENV['arguments'](['--compiled-input-receipt','receipt.json'])
        self.assertEqual(args.compiled_input_receipt,Path('receipt.json'))
        for arguments in (['--compiled-input-receipt','receipt','--retained-package','release','--expected-commit',COMMIT],
                          ['--compiled-input-receipt','receipt','--expected-commit',COMMIT],
                          ['--compiled-input-receipt','receipt','--prepare-only']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):ENV['arguments'](arguments)
    def test_changed_setup_refused_before_restore(self):
        (self.inputs/'SKAGER-Beta2-Setup.exe').write_bytes(b'substituted')
        with self.assertRaisesRegex(ValueError,'checksum'):self.restore()
        self.assertFalse((self.root/'build').exists())
    def test_existing_build_is_never_overwritten(self):
        (self.root/'build/production-install').mkdir(parents=True)
        with self.assertRaisesRegex(ValueError,'fresh'):self.restore()
    def test_wrong_source_commit_is_refused(self):
        self.build_inputs(source_commit='b'*40)
        with self.assertRaisesRegex(ValueError,'source revision'):self.restore()
    def test_fixture_output_enabled_product_refused(self):
        self.product['xnav_hardware_output_policy']='enabled'
        self.entries['docs/PRODUCT_BUILD.json']=json.dumps(self.product).encode();self.build_inputs()
        with self.assertRaisesRegex(ValueError,'status-only'):self.restore()
    def test_unexpected_support_path_is_refused(self):
        self.support['tools/replace.py']=b'bad';self.build_inputs()
        with self.assertRaisesRegex(ValueError,'fixture installation only'):self.restore()
    def test_installer_and_recovery_must_identify_same_executable(self):
        self.entries['app/opencpn.exe']=b'substituted';self.build_inputs()
        with self.assertRaisesRegex(ValueError,'executable differs'):self.restore()
    def test_windows_unsafe_archive_names_are_refused(self):
        for name in ('../outside','app\\evil','C:/escape','app/trailing.','/absolute','app/./ambiguous','app/NUL.txt'):
            with self.subTest(name=name),zipfile.ZipFile(io.BytesIO(zipped({name:b'bad'}))) as archive:
                self.assertEqual(archive.infolist()[0].orig_filename,name)
                with self.assertRaisesRegex(ValueError,'Unsafe'):ENV['archive_members'](archive)
    def test_case_collisions_and_symlinks_are_refused(self):
        with zipfile.ZipFile(io.BytesIO(zipped({'app/A':b'a','app/a':b'b'}))) as archive:
            with self.assertRaisesRegex(ValueError,'duplicate'):ENV['archive_members'](archive)
        output=io.BytesIO()
        with zipfile.ZipFile(output,'w') as archive:
            entry=zipfile.ZipInfo('app/link');entry.external_attr=(0o120777<<16)
            archive.writestr(entry,'../../target')
        with zipfile.ZipFile(io.BytesIO(output.getvalue())) as archive:
            with self.assertRaisesRegex(ValueError,'Unsafe'):ENV['archive_members'](archive)

class PreparedPriorFixtures(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.fixtures=[]
        for label,directory,receipt_name,setup_name in (
                ('accepted-beta1','prior-alpha-fixture','installer-prior-release','OpenNavX-Beta1-Setup.exe'),
                ('early-beta2-layout','prior-beta2-layout-fixture','installer-prior-beta2-layout','OpenNavX-Beta2-Setup.exe')):
            setup=self.root/'build'/directory/'setup'/setup_name
            archive=setup.parent.parent/'verified-prior-installer.zip'
            setup.parent.mkdir(parents=True);setup.write_bytes(('inert '+label).encode())
            archive.write_bytes(zipped({setup_name:setup.read_bytes()}))
            lock=dict(commit=COMMIT,runId=123,artifactId=456,setupName=setup_name,
                      setupBytes=setup.stat().st_size,setupSha256=digest(setup.read_bytes()),
                      maximumArchiveBytes=4096,archiveSha256=digest(archive.read_bytes()))
            lock_path=self.root/'tools'/(label+'.lock.json');lock_path.parent.mkdir(exist_ok=True)
            lock_path.write_text(json.dumps(lock))
            receipt=self.root/'evidence/local'/(receipt_name+'.json');receipt.parent.mkdir(parents=True,exist_ok=True)
            receipt.write_text(json.dumps(dict(source=lock,verified=True,scope='inert fixture')))
            self.fixtures.append(dict(setup=setup,archive=archive,lock=lock,lock_path=lock_path,receipt=receipt))

    def verify(self):
        with mock.patch.dict(os.environ,{},clear=True):
            return ENV['verify_prior_fixtures'](self.root)

    def test_both_fixed_archives_setups_and_receipts_revalidated_without_modification(self):
        paths=[p for p in self.root.rglob('*') if p.is_file()]
        before={p:(p.read_bytes(),p.stat().st_mtime_ns) for p in paths}
        report=self.verify()
        self.assertEqual(len(report),2)
        for result,fixture in zip(report,self.fixtures):
            self.assertEqual(result['source'],fixture['lock'])
            self.assertEqual(result['receipt_sha256'],digest(fixture['receipt'].read_bytes()))
            self.assertEqual(result['lock_sha256'],digest(fixture['lock_path'].read_bytes()))
            self.assertEqual(result['setup_sha256'],digest(fixture['setup'].read_bytes()))
        self.assertEqual(before,{p:(p.read_bytes(),p.stat().st_mtime_ns) for p in paths})

    def test_changed_archive_or_setup_in_either_fixture_fails(self):
        for fixture in self.fixtures:
            for key in ('archive','setup'):
                path=fixture[key];original=path.read_bytes()
                for changed in (bytes([original[0]^1])+original[1:],original+b'changed'):
                    with self.subTest(fixture=path.name,kind=key):
                        path.write_bytes(changed)
                        with self.assertRaisesRegex(ValueError,'checksum or size'):self.verify()
                path.write_bytes(original)

    def test_missing_unverified_mismatched_or_duplicate_receipt_fails(self):
        for fixture in self.fixtures:
            path=fixture['receipt'];original=path.read_bytes()
            for receipt in (dict(source=fixture['lock'],verified=False),
                            dict(source=fixture['lock'],verified=1),
                            dict(source={**fixture['lock'],'commit':'b'*40},verified=True)):
                path.write_text(json.dumps(receipt))
                with self.assertRaisesRegex(ValueError,'receipt differs'):self.verify()
            path.write_text('{"verified":true,"verified":true}')
            with self.assertRaisesRegex(ValueError,'Duplicate JSON'):self.verify()
            path.unlink()
            with self.assertRaises(FileNotFoundError):self.verify()
            path.write_bytes(original)

    def test_linked_or_missing_preparation_bytes_fail(self):
        for fixture in self.fixtures:
            for key in ('archive','setup','lock_path','receipt'):
                path=fixture[key];saved=path.with_name(path.name+'.original');path.rename(saved)
                with self.assertRaises(FileNotFoundError):self.verify()
                path.symlink_to(saved)
                with self.assertRaisesRegex(ValueError,'Linked retained'):self.verify()
                path.unlink();saved.rename(path)

    def test_each_credential_refused_before_prepared_input_reads(self):
        for name in ('GH_TOKEN','GITHUB_TOKEN','OPENNAV_ARTIFACT_TOKEN'):
            with mock.patch.dict(os.environ,{name:'inert sentinel'},clear=True), \
                 mock.patch.object(staging_inputs,'plain_file') as read:
                with self.assertRaisesRegex(ValueError,'Remove download credentials'):
                    ENV['verify_prior_fixtures'](self.root)
                read.assert_not_called()

    def test_prepared_option_is_only_for_explicit_compiled_lifecycle(self):
        from contextlib import redirect_stderr
        args=ENV['arguments'](['--mode','production','--compiled-input-receipt','receipt','--prepared-prior-fixtures'])
        self.assertTrue(args.prepared_prior_fixtures)
        for args in (['--prepared-prior-fixtures'],
                     ['--mode','production','--prepared-prior-fixtures'],
                     ['--compiled-input-receipt','receipt','--prepared-prior-fixtures'],
                     ['--mode','production','--retained-package','release','--expected-commit',COMMIT,'--prepared-prior-fixtures']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):ENV['arguments'](args)

    def test_prepared_branch_never_invokes_fetcher_and_preserves_failure(self):
        from types import SimpleNamespace
        branch=next(node for node in ast.walk(TREE) if isinstance(node,ast.If)
                    and ast.unparse(node.test)=="args.mode == 'production'"
                    and any(isinstance(child,ast.If) and ast.unparse(child.test)=='args.prepared_prior_fixtures'
                            for child in node.body))
        for failure in (None,ValueError('original preparation failure')):
            run=mock.Mock();verify=mock.Mock(return_value=['verified'],side_effect=failure)
            environment=dict(args=SimpleNamespace(mode='production',prepared_prior_fixtures=True),
                ROOT=self.root,report={},verify_prior_fixtures=verify,subprocess=SimpleNamespace(run=run))
            statement=compile(ast.Module(body=[branch],type_ignores=[]),str(SOURCE),'exec')
            if failure:
                with self.assertRaisesRegex(ValueError,'original preparation failure'):exec(statement,environment)
            else:
                exec(statement,environment)
                self.assertEqual(environment['report']['prepared_prior_fixtures'],['verified'])
            verify.assert_called_once_with(self.root);run.assert_not_called()


class FunctionalChart(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('chart_render_check',SOURCE.with_name('chart-render-check.py'))
        cls.charts=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.charts)
    def frame(self,land,water):
        # Synthetic unit input only: exercise coverage/contrast rejection, never
        # substitute this frame for a native chart result or design reference.
        return (bytes(land)*640+bytes(water)*640)*800
    def test_arbitrary_distinct_chart_inks_do_not_require_prototype_palette(self):
        result=self.charts.functional(self.frame((60,70,80),(120,130,140)),'XNav','Day','unit')
        self.assertTrue(result['coastline_visible'])
    def test_functional_layout_accepts_other_geometry_and_rejects_obstruction(self):
        frame=dict(x=0,y=0,width=1280,height=800)
        controls=[dict(x=130+i*50,y=100,width=40,height=40,visible=True,enabled=True,label=label)
                  for i,label in enumerate(('Measure','Waypoint','+','−','Follow boat','Layers','North'))]
        controls.extend([dict(x=10,y=y,width=60,height=40,visible=True,enabled=True,label=label)
                         for y,label in ((100,'Chart'),(200,'Settings'))])
        controls.append(dict(x=900,y=720,width=80,height=40,visible=True,enabled=True,label='Source health'))
        display=dict(chart_region=dict(x=100,y=80,width=900,height=570),
            rail_regions=[dict(x=1040,y=80+i*90,width=100,height=60,visible=True,label=str(i)) for i in range(4)],
            interaction_controls=controls,footer_region=dict(x=0,y=700,width=1280,height=80),footer_middle_visible=True)
        self.charts.functional_layout(display,frame,frame)
        display['rail_regions'][0]['x']=150
        with self.assertRaisesRegex(AssertionError,'overlap'):
            self.charts.functional_layout(display,frame,frame)
    def test_route_visibility_requires_actual_stroke_at_projected_samples(self):
        rgb=bytearray(bytes((20,30,40))*1280*800)
        points=[dict(x=200,y=400),dict(x=1000,y=400)]
        with self.assertRaises(AssertionError):self.charts.route_visibility(rgb,points)
        for y in range(399,402):
            for x in range(200,1001):rgb[(y*1280+x)*3:(y*1280+x)*3+3]=bytes((80,180,100))
        self.assertEqual(len(self.charts.route_visibility(rgb,points)['samples']),5)
        for y in range(396,405):
            for x in range(596,605):rgb[(y*1280+x)*3:(y*1280+x)*3+3]=bytes((20,30,40))
        with self.assertRaises(AssertionError):self.charts.route_visibility(rgb,points)

    def test_blank_or_indistinguishable_chart_is_refused(self):
        for land,water in (((20,30,40),(20,30,40)),((20,30,40),(21,31,41))):
            with self.assertRaises(AssertionError):
                self.charts.functional(self.frame(land,water),'XNav','Day','unit')


if __name__=='__main__':unittest.main()

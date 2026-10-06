#!/usr/bin/env python3
"""Inert recovery packaging/qualification boundary tests; no native execution."""
import ast
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import unittest
from unittest import mock

import recovered_staging as recovery
import staging_build_inputs as sealed

spec = importlib.util.spec_from_file_location('input_fixture', Path(__file__).with_name('test-staging-build-inputs.py'))
fixture = importlib.util.module_from_spec(spec); spec.loader.exec_module(fixture)
IDENTITY = dict(commit='b'*40, runId='999', runAttempt='1')
ORIGINAL = sealed.producer(recovery.PRODUCT, '37424595899', '1')


class Recovery(unittest.TestCase):
    def setUp(self):
        self.fixture = fixture.Boundary()
        with mock.patch.object(fixture, 'COMMIT', recovery.PRODUCT):
            self.fixture.setUp()
        self.addCleanup(self.fixture.tearDown)
        self.source, self.root = self.fixture.producer, self.fixture.consumer
        for name in ('windows-peer-cli-receipt.json', 'ais-native-runtime/report.json'):
            path = self.source / 'evidence/local' / name
            data = json.loads(path.read_bytes()); data.update(runId='37424595899', runAttempt='1')
            path.write_text(json.dumps(data))
        product_path = self.source / sealed.PACKAGE_ROOT / 'docs/PRODUCT_BUILD.json'
        product = json.loads(product_path.read_bytes())
        product.update(compiled_ci_run='37424595899',packaging_ci_run=IDENTITY['runId'],packaging_helper_commit=IDENTITY['commit'])
        product_path.write_text(json.dumps(product))
        (self.source/'build/developer-preview/SKAGER-Beta2-Portable-Recovery.zip').write_bytes(fixture.zipped({
            'SKAGER-Beta2-Portable-Recovery/docs/PRODUCT_BUILD.json':json.dumps(product)}))
        names = sealed.inventory(self.source)
        self.origin = dict(producer=ORIGINAL, workspaceRoot=str(self.source), recovery=IDENTITY,
                           files={name:dict(size=(self.source/name).stat().st_size,sha256=sealed.sha(self.source/name)) for name in names
                                  if not name.startswith(('build/developer-preview/', 'build/beta-installer/'))})
        self.origin_path = self.fixture.base / 'origin.json'
        self.origin_path.write_text(json.dumps(self.origin))
        for patch in (mock.patch.object(recovery,'current_identity',return_value=IDENTITY),
                      mock.patch.object(recovery,'validate_origin',side_effect=lambda value:value)):
            patch.start();self.addCleanup(patch.stop)

    def prepare(self):
        return recovery.prepare(self.source, self.origin_path, self.root)

    def validate(self):
        return recovery.validated_receipt(self.root,self.root/recovery.RECEIPT,recovery.PRODUCT,IDENTITY['commit'])

    def test_two_origins_remain_distinct_and_historical_kind_is_not_emitted(self):
        record=self.prepare()
        self.assertEqual(record['producer'],ORIGINAL)
        self.assertEqual(record['packaging'],IDENTITY)
        self.assertEqual(record['kind'],recovery.KIND)
        self.assertNotEqual(record['kind'],sealed.KIND)
        self.assertEqual(record['qualification'],'not-run')
        self.assertEqual(self.validate(),record)
        self.assertFalse((self.root/'build/recovered-staging'/sealed.ARCHIVE).exists())

    def test_package_may_not_claim_recovery_run_compiled_original_application(self):
        path=self.source/sealed.PACKAGE_ROOT/'docs/PRODUCT_BUILD.json'
        data=json.loads(path.read_bytes());data['compiled_ci_run']=IDENTITY['runId']
        path.write_text(json.dumps(data))
        (self.source/'build/developer-preview/SKAGER-Beta2-Portable-Recovery.zip').write_bytes(fixture.zipped({
            'SKAGER-Beta2-Portable-Recovery/docs/PRODUCT_BUILD.json':json.dumps(data)}))
        with self.assertRaisesRegex(ValueError,'conflates'):self.prepare()

    def test_changed_original_byte_prevents_packaging_acceptance(self):
        (self.source/'build/production-install/opencpn.exe').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'retained input changed'): self.prepare()
        self.assertFalse((self.root/recovery.RECEIPT).exists())

    def test_qualification_refuses_changed_payload_origin_or_run(self):
        self.prepare()
        for name in ['build/production-install/opencpn.exe','build/beta-installer/SKAGER-Beta2-Setup.exe',recovery.RESTORE_RECORD]:
            path=self.root/name; previous=path.read_bytes();path.write_bytes(previous+b'changed')
            with self.subTest(name=name),self.assertRaises(ValueError):self.validate()
            path.write_bytes(previous)
        path=self.root/recovery.RECEIPT; previous=path.read_bytes()
        for changes in ({'packaging':dict(IDENTITY,runId='998')},{'harnessCommit':'f'*40},{'producer':dict(ORIGINAL,runAttempt='2')},{'kind':sealed.KIND}):
            record=json.loads(previous);record.update(changes);path.write_text(json.dumps(record))
            with self.subTest(changes=changes),self.assertRaises(ValueError):self.validate()
        path.write_bytes(previous)

    def test_feedback_consumer_accepts_only_valid_recovery_receipt(self):
        self.prepare()
        result=sealed.restored_feedback(self.root,self.root/sealed.FEEDBACK_MANIFEST,self.root/recovery.RECEIPT,recovery.PRODUCT,IDENTITY['commit'])
        self.assertEqual(result['commit'],recovery.PRODUCT)
        self.assertEqual(len(result['tests']),13)
        (self.root/next(iter(sealed.FEEDBACK_BINARIES.values()))).write_bytes(b'altered')
        with self.assertRaises(ValueError):
            sealed.restored_feedback(self.root,self.root/sealed.FEEDBACK_MANIFEST,self.root/recovery.RECEIPT,recovery.PRODUCT,IDENTITY['commit'])

    def test_complete_existing_qualifier_is_required_and_failure_recorded(self):
        self.prepare()
        with mock.patch.dict(os.environ,{},clear=True), mock.patch.object(recovery.subprocess,'run',side_effect=subprocess.CalledProcessError(1,'native')) as run:
            with self.assertRaises(subprocess.CalledProcessError):recovery.qualify(self.root)
        command=run.call_args.args[0]
        self.assertIn(str(self.root/'tools/qualify-staging-windows.ps1'),command)
        self.assertEqual(command[-3:],['-ProductCommit',recovery.PRODUCT,'-CompiledRetest'])
        self.assertEqual(json.loads((self.root/recovery.REPORT).read_bytes())['status'],'failed')

    def test_credentials_or_post_qualification_binary_mutation_never_pass(self):
        self.prepare()
        with mock.patch.dict(os.environ,{'GH_TOKEN':'inert'},clear=True), mock.patch.object(recovery.subprocess,'run') as run:
            with self.assertRaisesRegex(ValueError,'credentials'):recovery.qualify(self.root)
            run.assert_not_called()
        def mutate(*args,**kwargs):(self.root/'build/production-install/opencpn.exe').write_bytes(b'changed')
        with mock.patch.dict(os.environ,{},clear=True), mock.patch.object(recovery.subprocess,'run',side_effect=mutate):
            with self.assertRaisesRegex(ValueError,'retained input changed'):recovery.qualify(self.root)
        self.assertEqual(json.loads((self.root/recovery.REPORT).read_bytes())['status'],'failed')

    def test_pass_records_exact_payloads_without_release_or_physical_acceptance(self):
        self.prepare()
        with mock.patch.dict(os.environ,{},clear=True), mock.patch.object(recovery.subprocess,'run'):
            report=recovery.qualify(self.root)
        self.assertEqual(report['status'],'passed')
        self.assertFalse(report['releaseQualification']);self.assertFalse(report['actualBoat']);self.assertFalse(report['publicAccess'])
        self.assertEqual(report['compiledOrigin'],ORIGINAL)
        self.assertEqual(report['packagingOrigin'],IDENTITY)
        self.assertIn('build/beta-installer/SKAGER-Beta2-Setup.exe',report['files'])

    def test_original_and_qualifying_roots_must_be_distinct_fresh(self):
        with self.assertRaisesRegex(ValueError,'fresh'):recovery.prepare(self.source,self.origin_path,self.source)


class PackagingIdentity(unittest.TestCase):
    def test_product_identity_arguments_do_not_rewrite_workflow_environment(self):
        for name in ('package-preview.py','package-alpha-installer.py'):
            tree=ast.parse(Path(__file__).with_name(name).read_text())
            # Package source selection is explicit; environment writes would relabel
            # the current recovery as the historical producer.
            writes=[node for node in ast.walk(tree) if isinstance(node,ast.Subscript) and isinstance(node.ctx,ast.Store)
                    and isinstance(node.value,ast.Attribute) and node.value.attr=='environ']
            self.assertEqual(writes,[],name)
            source=Path(__file__).with_name(name).read_text()
            self.assertIn("'--product-commit'",source);self.assertIn("'--source-root'",source)


if __name__=='__main__':unittest.main()

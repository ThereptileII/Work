#!/usr/bin/env python3
"""Disposable receipt/archive boundary tests; does not qualify or stage tools."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

spec = importlib.util.spec_from_file_location('palette_qualification', Path(__file__).with_name('qualify-boat-palette-tools.py'))
q = importlib.util.module_from_spec(spec)
spec.loader.exec_module(q)

class QualificationTests(unittest.TestCase):
    def setUp(self):
        self.identity = dict(commit='a'*40, runId='123', runAttempt='1')
        self.files = [dict(name='inert.ps1', sha256=q.digest(b"throw 'never execute'\r\n"), sourceSha256=q.digest(b"throw 'never execute'\n"), size=23, lineEndingsDiffer=True)]
        self.records = [dict(schema=1, gate=gate, status='passed', **self.identity, files=self.files,
                             compositionSha256='b'*64, actualBoat=False, applicationBuild=False) for gate in q.GATES]

    def test_exact_three_gates(self):
        q.validate_gate_set(self.records, self.identity, self.files, 'b'*64)

    def test_refuses_missing_duplicate_or_other_run(self):
        for records in (self.records[:2], [self.records[0]]*3):
            with self.assertRaises(ValueError):
                q.validate_gate_set(records, self.identity, self.files, 'b'*64)
        for key, value in [('commit','c'*40), ('runId','124'), ('runAttempt','2'), ('status','failed'), ('actualBoat',True), ('applicationBuild',True), ('compositionSha256','c'*64)]:
            changed=copy.deepcopy(self.records);changed[0][key]=value
            with self.subTest(key=key), self.assertRaises(ValueError):
                q.validate_gate_set(changed,self.identity,self.files,'b'*64)

    def test_refuses_source_or_tested_byte_drift(self):
        for key in ('sha256','sourceSha256'):
            changed=copy.deepcopy(self.records);changed[1]['files'][0][key]='c'*64
            with self.subTest(key=key), self.assertRaises(ValueError):
                q.validate_gate_set(changed,self.identity,self.files,'b'*64)

    def test_retained_names_and_actual_git_blob_mapping(self):
        self.assertEqual(119,len(q.composition()))
        files=q.inventory()
        self.assertEqual(set(q.composition()),{x['name'] for x in files})
        self.assertNotIn('inspect-fonts.ps1',q.composition())
        for item in files:
            self.assertEqual(item['sha256'],q.digest((q.ROOT/'tools/boat'/item['name']).read_bytes()))

    def test_startup_helper_requires_fresh_native_source_bound_report(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);folder=root/'reports';folder.mkdir()
            source=root/'tools/boat';source.mkdir(parents=True)
            identities=[]
            for name in ('StartupLauncher.ps1','test-startup-launcher.ps1'):
                data=('inert '+name).encode();(source/name).write_bytes(data)
                identities.append(dict(path='tools/boat/'+name,sha256=q.digest(data)))
            for name in q.GATES['policy']:
                if name != 'startup-launcher.json':q.write(folder/name,dict(status='passed'))
            path=folder/'startup-launcher.json'
            record=dict(schema=1,status='passed',environment='native-windows-inert-process',
                        nativeObservation='passed',installedBootstrap='pending',signedOffersAndRollback='pending',
                        checks=['fixture check '+str(i) for i in range(45)],count=45,sourceFiles=identities)
            with patch.object(q,'ROOT',root):
                with self.assertRaises(FileNotFoundError):q.verify_reports('policy',folder)
                q.write(path,record)
                self.assertIn('startup-launcher.json',[r['path'] for r in q.verify_reports('policy',folder)])
                for key,value in [('schema',2),('status','failed'),('environment','linux-portable-contracts'),
                                  ('nativeObservation','pending'),('installedBootstrap','passed'),
                                  ('signedOffersAndRollback','passed'),('checks',[]),('count',44),('sourceFiles',[])]:
                    with self.subTest(field=key):
                        path.write_text(json.dumps(dict(record,**{key:value})))
                        with self.assertRaises(ValueError):q.verify_reports('policy',folder)
                path.write_text(json.dumps(record));(source/'StartupLauncher.ps1').write_bytes(b'changed')
                with self.assertRaisesRegex(ValueError,'exact tested source bytes'):q.verify_reports('policy',folder)

    def test_workflow_runs_focused_startup_before_policy_seal_and_keeps_other_gates(self):
        checkout=Path(q.git('rev-parse','--show-toplevel').decode().strip())
        workflow=(checkout/'.github/workflows/skager-chart-palette-tools.yml').read_text()
        self.assertIn("'review-staging','startup-launcher'",workflow)
        self.assertLess(workflow.index("'startup-launcher'"),workflow.index('--gate policy'))
        self.assertIn('needs: [policy, window, broker]',workflow)
        for gate in q.GATES:self.assertIn('--gate '+gate,workflow)

    def test_window_receipt_requires_all_normal_exits(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);(folder/'chart-palette').mkdir()
            q.write(folder/'native-window-results.json',dict(status='passed',productLaunched=False))
            path=folder/'chart-palette/native-palette-results.json'
            record=dict(status='passed',productLaunched=False,boatTouched=False,physicalOutput=False,cases=[dict(fixtureExitCode=0) for _ in range(16)])
            q.write(path,record);q.verify_reports('window',folder)
            for cases in ([dict(fixtureExitCode=0)]*15,[dict(fixtureExitCode=0)]*15+[dict(fixtureExitCode=7)]):
                path.write_text(json.dumps(dict(record,cases=cases)))
                with self.assertRaises(ValueError):q.verify_reports('window',folder)

    def test_bundle_is_exact_flat_bytes_and_transport_hashes(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);tools=root/'tools/boat';tools.mkdir(parents=True)
            data=b"throw 'never execute'\r\n";(tools/'inert.ps1').write_bytes(data)
            lock=root/'composition.json';q.write(lock,dict(files=['inert.ps1'],testOnly=True))
            evidence=root/'gates';evidence.mkdir()
            records=copy.deepcopy(self.records)
            for record in records:
                record['compositionSha256']=q.digest(lock.read_bytes());record['reports']=[]
                folder=evidence/record['gate'];folder.mkdir();q.write(folder/'gate-receipt.json',record)
            with patch.object(q,'ROOT',root),patch.object(q,'LOCK',lock),patch.object(q,'identity',return_value=self.identity),patch.object(q,'inventory',return_value=self.files),patch.object(q,'verify_reports',return_value=[]):
                q.package(evidence,root/'bundle')
            with zipfile.ZipFile(root/'bundle/review-tools.zip') as archive:
                self.assertEqual(['inert.ps1'],archive.namelist());self.assertEqual(data,archive.read('inert.ps1'))
            receipt=q.read(root/'bundle/qualification.json')
            self.assertEqual(receipt['archiveSha256'],q.digest((root/'bundle/review-tools.zip').read_bytes()))
            self.assertEqual(receipt['manifestSha256'],q.digest((root/'bundle/manifest.json').read_bytes()))
            self.assertFalse(receipt['staged']);self.assertFalse(receipt['actualBoat'])

if __name__ == '__main__':unittest.main()

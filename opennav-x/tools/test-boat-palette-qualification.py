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
    @staticmethod
    def display_record():
        return dict(status='passed', error=None, cleanupErrors=[], productLaunched=False,
                    physicalOutput=False,
                    cases=[dict(action=action, case=case, fixtureExitCode=0)
                           for action, case in sorted(q.DISPLAY_REQUIRED_CASES)],
                    **{field:q.digest((q.ROOT/name).read_bytes())
                       for field,name in q.DISPLAY_SOURCES.items()})

    @staticmethod
    def fixture_git(*args):
        # Report boundary tests use current bytes as a disposable committed input;
        # the separate inventory test checks real tracked HEAD identities.
        return b'' if args[0] == 'rev-parse' else (q.ROOT/args[1].removeprefix('HEAD:')).read_bytes()

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
        self.assertEqual(117,len(q.composition()))
        files=q.inventory()
        self.assertEqual(set(q.composition()),{x['name'] for x in files})
        self.assertNotIn('inspect-fonts.ps1',q.composition())
        for item in files:
            self.assertEqual(item['sha256'],q.digest((q.ROOT/'tools/boat'/item['name']).read_bytes()))

    def test_window_receipt_requires_all_normal_exits(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(q,'git',side_effect=self.fixture_git):
            folder=Path(temp);(folder/'chart-palette').mkdir()
            q.write(folder/'native-window-results.json',dict(status='passed',productLaunched=False))
            q.write(folder/'native-display-results.json', self.display_record())
            path=folder/'chart-palette/native-palette-results.json'
            record=dict(status='passed',productLaunched=False,boatTouched=False,physicalOutput=False,cases=[dict(fixtureExitCode=0) for _ in range(16)])
            q.write(path,record);q.verify_reports('window',folder)
            for cases in ([dict(fixtureExitCode=0)]*15,[dict(fixtureExitCode=0)]*15+[dict(fixtureExitCode=7)]):
                path.write_text(json.dumps(dict(record,cases=cases)))
                with self.assertRaises(ValueError):q.verify_reports('window',folder)

    def test_display_report_requires_actual_complete_clean_cases(self):
        valid = self.display_record()
        with patch.object(q,'git',side_effect=self.fixture_git):
            q.verify_display_report(valid)
            mutations = [('status','failed'),('error','fixture failed'),('cleanupErrors',['cleanup failed']),
                         ('productLaunched',True),('physicalOutput',True),('nativeHelperSha256','0'*64),
                         ('fixtureSha256','0'*64),('cases',valid['cases'][:-1]),
                         ('cases',valid['cases']+[valid['cases'][0]]),('cases',[])]
            for key,value in mutations:
                with self.subTest(key=key,value=value), self.assertRaises(ValueError):
                    q.verify_display_report(dict(valid,**{key:value}))
            for exit_code in (None, False, 7):
                changed=copy.deepcopy(valid);changed['cases'][0]['fixtureExitCode']=exit_code
                with self.subTest(exit_code=exit_code),self.assertRaises(ValueError):
                    q.verify_display_report(changed)

    def test_display_report_rejects_uncommitted_fixture_source(self):
        def changed_git(*args):
            return b'' if args[0]=='rev-parse' else b'unreviewed source'
        with patch.object(q,'git',side_effect=changed_git),self.assertRaises(ValueError):
            q.verify_display_report(self.display_record())

    def test_window_receipt_hashes_display_and_refuses_missing_report(self):
        with tempfile.TemporaryDirectory() as temp,patch.object(q,'git',side_effect=self.fixture_git):
            folder=Path(temp);(folder/'chart-palette').mkdir()
            q.write(folder/'native-window-results.json',dict(status='passed'))
            q.write(folder/'chart-palette/native-palette-results.json',dict(status='passed',cases=[dict(fixtureExitCode=0)]*16))
            with self.assertRaises(FileNotFoundError):q.verify_reports('window',folder)
            display=folder/'native-display-results.json';q.write(display,self.display_record())
            reports=q.verify_reports('window',folder)
            self.assertIn(dict(path='native-display-results.json',sha256=q.digest(display.read_bytes())),reports)

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

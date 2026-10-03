"""Offline contracts for the actual early Poedit prerequisite, no installs."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('gettext_prerequisite',ROOT/'tools/windows_gettext.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)

class GettextPrerequisiteTests(unittest.TestCase):
    def setUp(self):
        self.work=tempfile.TemporaryDirectory();self.addCleanup(self.work.cleanup)
        self.root=Path(self.work.name);self.pf=self.root/'Program Files';self.pf86=self.root/'Program Files (x86)'
        self.bin=self.pf/'Poedit/Gettexttools/bin';self.receipt=self.root/'evidence/gettext.json'
        self.choco=self.root/'chocolatey/bin/choco.exe';self.choco.parent.mkdir(parents=True);self.choco.write_bytes(b'package manager fixture')
        self.environment=patch.dict(os.environ,{'ProgramFiles':str(self.pf),'ProgramFiles(x86)':str(self.pf86),'ChocolateyInstall':str(self.choco.parent.parent)})
        self.environment.start();self.addCleanup(self.environment.stop)
        self.wait=patch.object(g.time,'sleep');self.wait.start();self.addCleanup(self.wait.stop)
    def tools(self,folder=None):
        folder=folder or self.bin;folder.mkdir(parents=True,exist_ok=True)
        for name in g.TOOLS:(folder/name).write_bytes(b'fixture-'+name.encode())
    def native(self,command,timeout,log):
        return {'exitCode':0}, Path(command[0]).stem+' (GNU gettext-tools) 0.26\n'
    def ensure(self,install=False):return g.ensure(self.receipt,install)
    def test_known_pair_requires_no_acquisition(self):
        self.tools()
        with patch.object(g,'native',side_effect=self.native) as run:
            r=self.ensure();self.assertEqual(r['attempts'],[]);self.assertEqual(run.call_count,2)
            self.assertTrue(all(Path(call.args[0][0]).parent==self.bin for call in run.call_args_list))
            self.assertEqual(g.verify(self.receipt),r)
    def test_x86_known_pair_is_supported(self):
        folder=self.pf86/'Poedit/Gettexttools/bin';self.tools(folder)
        with patch.object(g,'native',side_effect=self.native):self.assertEqual(self.ensure()['directory'],str(folder))
    def test_path_lookalike_never_selected(self):
        rogue=self.root/'rogue';self.tools(rogue)
        with patch.dict(os.environ,{'PATH':str(rogue)}),patch.object(g,'native') as run:
            with self.assertRaisesRegex(RuntimeError,'installation was not authorized'):self.ensure()
            run.assert_not_called()
    def test_partial_pair_is_not_usable(self):
        self.tools();(self.bin/'msgmerge.exe').unlink()
        with patch.object(g,'native',side_effect=self.native):
            with self.assertRaisesRegex(RuntimeError,'installation was not authorized'):self.ensure()
    def test_nonzero_version_refused(self):
        self.tools()
        with patch.object(g,'native',return_value=({'exitCode':1},'msgfmt (GNU gettext-tools) 0.26')):
            with self.assertRaisesRegex(RuntimeError,'installation was not authorized'):self.ensure()
    def test_wrong_banner_refused(self):
        self.tools()
        with patch.object(g,'native',return_value=({'exitCode':0},'not gettext')):
            with self.assertRaisesRegex(RuntimeError,'installation was not authorized'):self.ensure()
    def test_retry_recovers_only_after_successful_package_exit(self):
        attempts=[]
        def run(command,timeout,log):
            if Path(command[0])==self.choco:
                attempts.append(command);self.tools()
                return {'exitCode':1 if len(attempts)==1 else 0},'package attempt'
            return self.native(command,timeout,log)
        with patch.object(g,'native',side_effect=run):r=self.ensure(True)
        self.assertEqual([a['exitCode'] for a in r['attempts']],[1,0]);self.assertEqual(r['status'],'passed')
        self.assertIn('--version=3.9.1',attempts[0]);self.assertIn('--source='+g.PROVIDER,attempts[0])
    def test_three_failed_attempts_never_accepted(self):
        with patch.object(g,'native',return_value=({'exitCode':1},'HTTP504')) as run:
            with self.assertRaisesRegex(RuntimeError,'three bounded attempts'):self.ensure(True)
            self.assertEqual(run.call_count,3)
        r=json.loads(self.receipt.read_text());self.assertEqual(r['status'],'failed');self.assertEqual(len(r['attempts']),3)
    def test_success_without_usable_files_is_refused(self):
        with patch.object(g,'native',return_value=({'exitCode':0},'installed')) as run:
            with self.assertRaisesRegex(RuntimeError,'three bounded attempts'):self.ensure(True)
            self.assertEqual(run.call_count,3)
    def test_version_timeout_cannot_trigger_install(self):
        self.tools()
        with patch.object(g,'native',side_effect=g.PrerequisiteDeadline('timed out')) as run:
            with self.assertRaisesRegex(g.PrerequisiteDeadline,'timed out'):self.ensure(True)
            self.assertEqual(run.call_count,1)
    def test_package_timeout_stops_without_retry(self):
        with patch.object(g,'native',side_effect=RuntimeError('subprocess timed out')) as run:
            with self.assertRaisesRegex(RuntimeError,'timed out'):self.ensure(True)
            self.assertEqual(run.call_count,1)
    def test_drift_after_selection_refused(self):
        self.tools()
        with patch.object(g,'native',side_effect=self.native):
            self.ensure();(self.bin/'msgfmt.exe').write_bytes(b'changed')
            with self.assertRaisesRegex(RuntimeError,'identity changed'):g.verify(self.receipt)
    def test_receipt_cannot_retarget_to_path_tool(self):
        self.tools()
        with patch.object(g,'native',side_effect=self.native):self.ensure()
        r=json.loads(self.receipt.read_text());r['directory']=str(self.root/'rogue');self.receipt.write_text(json.dumps(r))
        with self.assertRaisesRegex(RuntimeError,'untrusted directory'):g.verify(self.receipt)
    def test_mutation_during_probe_refused(self):
        self.tools()
        def run(command,timeout,log):
            Path(command[0]).write_bytes(b'drift');return self.native(command,timeout,log)
        with patch.object(g,'native',side_effect=run):
            with self.assertRaisesRegex(RuntimeError,'installation was not authorized'):self.ensure()
    def test_script_orders_gate_before_every_expensive_step(self):
        source=(ROOT/'tools/build-pristine-windows.ps1').read_text()
        gate=source.index("'windows_gettext.py'), 'ensure'")
        for after in ("'test-curl-source-preflight.ps1'",'buildwin\\win_deps.bat',"'build-openssl-windows.ps1'",'--ui'):
            self.assertLess(gate,source.index(after))
        verify=source.index("'windows_gettext.py'), 'verify'");self.assertLess(verify,source.index('    Run cmake'))
        self.assertIn('-DGETTEXT_MSGFMT_EXECUTABLE=$Gettext/msgfmt.exe',source)
        self.assertIn('-DGETTEXT_MSGMERGE_EXECUTABLE=$Gettext/msgmerge.exe',source)
    def test_native_probe_retains_both_streams_and_status(self):
        import sys
        result,output=g.native([sys.executable,'-c','import sys;print("out");print("err",file=sys.stderr);sys.exit(7)'],5,self.root/'probe')
        self.assertEqual(result['exitCode'],7);self.assertEqual(output.strip(),'out')
        self.assertEqual((self.root/'probe.stderr.log').read_text().strip(),'err')
    def test_native_deadline_reaps_owned_process(self):
        import sys
        with self.assertRaisesRegex(RuntimeError,'timed out'):
            g.native([sys.executable,'-c','import time;time.sleep(30)'],.1,self.root/'timeout')
        r=json.loads((self.root/'timeout.json').read_text());self.assertTrue(r['timedOut']);self.assertIsNotNone(r['exitCode'])

if __name__=='__main__':unittest.main()

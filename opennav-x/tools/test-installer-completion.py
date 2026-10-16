#!/usr/bin/env python3
"""Portable installer observation/fixture tests; no native Windows acceptance."""
import ast
from contextlib import contextmanager
import ctypes
import json
import hashlib
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

SOURCE = Path(__file__).with_name('smoke-installer-windows.py')
TREE = ast.parse(SOURCE.read_text())
FUNCTIONS = ast.Module(body=[node for node in TREE.body
                            if isinstance(node, ast.FunctionDef)
                            and node.name in ('engine', 'maintenance', 'installer_fixture')],
                       type_ignores=[])


class Completion(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.result = self.root / 'result.json'
        self.clock = 0.0
        self.publish_at = None
        self.status = 'passed'
        self.report = {'operations': []}
        def sleep(seconds):
            self.clock += seconds
            if self.publish_at is not None and self.clock >= self.publish_at:
                self.result.write_text(json.dumps({'status': self.status}))
        def operation_report(action):
            self.report['operations'].append({'action': action})
            return self.result
        self.environment = dict(
            contextmanager=contextmanager, tempfile=tempfile, shutil=shutil,
            report=self.report, json=json, generation=lambda: self.root,
            operation_report=operation_report, PS=self.root / 'powershell.exe',
            subprocess=SimpleNamespace(list2cmdline=subprocess.list2cmdline,
                                       run=lambda *a, **kw: SimpleNamespace(returncode=0)),
            time=SimpleNamespace(monotonic=lambda: self.clock, sleep=sleep))
        exec(compile(FUNCTIONS, str(SOURCE), 'exec'), self.environment)

    def tearDown(self):
        self.temp.cleanup()

    def test_relocated_cleanup_after_two_minutes_must_finish(self):
        self.publish_at = 153
        result = self.environment['maintenance']('Uninstall')
        self.assertEqual(result['status'], 'passed')
        self.assertGreaterEqual(self.report['operations'][-1]['completion_seconds'], 153)

    def test_bootstrap_exit_without_report_is_not_success(self):
        with self.assertRaisesRegex(AssertionError, '600 seconds'):
            self.environment['maintenance']('Uninstall')
        self.assertLess(self.clock, 601)
        self.assertNotIn('completion_seconds', self.report['operations'][-1])

    def test_failed_durable_result_remains_failure(self):
        self.publish_at, self.status = 153, 'failed'
        with self.assertRaises(AssertionError):
            self.environment['maintenance']('Uninstall')
        self.assertNotIn('completion_seconds', self.report['operations'][-1])

    def test_diagnostics_keeps_its_existing_deadline(self):
        with self.assertRaisesRegex(AssertionError, '120 seconds'):
            self.environment['maintenance']('Diagnostics')
        self.assertLess(self.clock, 121)

    def test_direct_cleanup_after_two_minutes_uses_the_same_bound(self):
        calls=[]
        def run(command, *, timeout, capture_output):
            calls.append(timeout)
            self.assertTrue(capture_output)
            self.assertEqual(command[-4:-2], ['-Action', 'Uninstall'])
            self.assertGreater(timeout, 153)
            self.clock=153
            self.result.write_text('{"status":"passed"}')
            return SimpleNamespace(returncode=0)
        self.environment['subprocess'].run=run
        self.assertEqual(self.environment['engine']('Uninstall')['status'], 'passed')
        self.assertEqual(calls, [600])
        self.assertEqual(self.report['operations'][-1]['completion_seconds'], 153)

    def test_direct_cleanup_timeout_stays_failure_without_retry(self):
        calls=[]
        def run(command, *, timeout, capture_output):
            calls.append(timeout)
            raise subprocess.TimeoutExpired(command, timeout)
        self.environment['subprocess'].run=run
        with self.assertRaises(subprocess.TimeoutExpired):
            self.environment['engine']('Uninstall')
        self.assertEqual(calls, [600])
        self.assertNotIn('completion_seconds', self.report['operations'][-1])

    def test_direct_successful_exit_with_failed_report_is_refused(self):
        self.result.write_text('{"status":"failed"}')
        with self.assertRaises(AssertionError):
            self.environment['engine']('Uninstall')
        self.assertNotIn('completion_seconds', self.report['operations'][-1])

    def test_direct_missing_report_is_not_success(self):
        with self.assertRaises(FileNotFoundError):
            self.environment['engine']('Uninstall')
        self.assertNotIn('completion_seconds', self.report['operations'][-1])

    def test_direct_nonzero_exit_is_refused(self):
        self.result.write_text('{"status":"passed"}')
        self.environment['subprocess'].run=lambda *a, **kw: SimpleNamespace(returncode=1,stdout=b'',stderr=b'failed')
        with self.assertRaises(AssertionError):
            self.environment['engine']('Uninstall')
        self.assertNotIn('completion_seconds', self.report['operations'][-1])

    def test_other_direct_actions_keep_their_existing_deadline(self):
        calls=[]
        def run(command, *, timeout, capture_output):
            calls.append(timeout)
            self.result.write_text('{"status":"passed"}')
            return SimpleNamespace(returncode=0)
        self.environment['subprocess'].run=run
        self.environment['engine']('Diagnostics')
        self.assertEqual(calls, [120])

    def test_failure_preserves_fixture_for_relocated_child(self):
        directory = None
        try:
            with self.assertRaisesRegex(RuntimeError, 'incomplete'):
                with self.environment['installer_fixture']() as directory:
                    Path(directory, 'stock.exe').write_bytes(b'fixture')
                    raise RuntimeError('incomplete')
            self.assertEqual(self.report['retained_failed_fixture'], directory)
            self.assertEqual(Path(directory, 'stock.exe').read_bytes(), b'fixture')
        finally:
            if directory:
                shutil.rmtree(directory)

    def test_success_removes_disposable_fixture(self):
        with self.environment['installer_fixture']() as directory:
            Path(directory, 'stock.exe').write_bytes(b'fixture')
        self.assertFalse(Path(directory).exists())
        self.assertNotIn('retained_failed_fixture', self.report)


class InstallerSelectionPage(unittest.TestCase):
    """Exercise the real observer against captured-page semantics and refusals."""
    def setUp(self):
        self.frame,self.pid,self.title=100,42,'SKAGER Beta 2 Setup'
        # Text and control types from the retained ec51 source page and failed
        # run38158246826 screenshot; the obsolete engine-error phrase is absent.
        self.controls=[
            [101,'Static','OpenCPN detected',True],
            [102,'Static',('OpenCPN must be installed separately. Select its original application. '
                           'Setup verifies the exact version and files before making changes.'),True],
            [103,'Edit',r'C:\private fixture\stock OpenCPN\opencpn.exe',True],
            [104,'ComboBox','Install',True],
            [105,'Button','Browse...',True],
            [106,'Button','Next >',True]]
        self.visible=[(self.frame,self.pid,self.title)]
        self.frame_enabled=True
        self.clock=0.0;self.publish_at=None;self.published=None;self.exit_code=None
        def find(handle):
            return next(control for control in self.controls if control[0]==handle)
        def class_name(handle,buffer,length):
            buffer.value=find(handle)[1]
            return len(buffer.value)
        def enabled(handle):
            return self.frame_enabled if handle==self.frame else find(handle)[3]
        def sleep(seconds):
            self.clock+=seconds
            if self.publish_at is not None and self.clock>=self.publish_at:
                self.controls=self.published
        observer=ast.Module(body=[node for node in TREE.body
            if isinstance(node,ast.FunctionDef) and node.name in
            ('installer_selection_ready','wait_installer_selection')],type_ignores=[])
        self.environment=dict(ctypes=ctypes,
            ui=SimpleNamespace(windows=lambda pid:[item for item in self.visible if item[1]==pid],
                IsWindowEnabled=enabled,GetClassNameW=class_name,
                children=lambda frame:[(row[0],row[2]) for row in self.controls],
                control_text=lambda handle:find(handle)[2]),
            time=SimpleNamespace(monotonic=lambda:self.clock,sleep=sleep))
        exec(compile(observer,str(SOURCE),'exec'),self.environment)
        self.process=SimpleNamespace(pid=self.pid,poll=lambda:self.exit_code)

    def ready(self):
        return self.environment['installer_selection_ready'](self.frame,self.pid,self.title)

    def wait(self):
        return self.environment['wait_installer_selection'](self.process,self.frame,self.title)

    def test_captured_current_page_passes_without_obsolete_error_caption(self):
        self.assertFalse(any('Select the original installed' in row[2] for row in self.controls))
        self.assertTrue(self.ready())

    def test_owned_setup_title_alone_does_not_make_welcome_ready(self):
        self.controls=[[101,'Static','Install SKAGER',True],[106,'Button','Next >',True]]
        self.assertFalse(self.ready())

    def test_foreign_same_title_and_controls_are_not_owned_evidence(self):
        self.visible=[(self.frame,99,self.title)]
        self.assertFalse(self.ready())

    def test_replaced_frame_handle_is_not_the_original_owned_window(self):
        self.visible=[(999,self.pid,self.title)]
        self.assertFalse(self.ready())

    def test_disabled_frame_behind_modal_is_not_ready(self):
        self.frame_enabled=False
        self.assertFalse(self.ready())

    def test_install_caption_elsewhere_cannot_fake_selected_action(self):
        self.controls[3][2]='Update'
        self.controls.append([107,'Static','Install',True])
        self.assertFalse(self.ready())

    def test_missing_or_duplicate_page_identity_refused(self):
        original=[row[:] for row in self.controls]
        for identifier in (101,102):
            for duplicate in (False,True):
                with self.subTest(identifier=identifier,duplicate=duplicate):
                    self.controls=[row[:] for row in original if row[0]!=identifier]
                    if duplicate:
                        control=next(row for row in original if row[0]==identifier)
                        self.controls.extend([control[:],[identifier+1000,*control[1:]]])
                    self.assertFalse(self.ready())

    def test_missing_duplicate_or_disabled_path_and_action_controls_refused(self):
        original=[row[:] for row in self.controls]
        for identifier in (103,104):
            for defect in ('missing','duplicate','disabled'):
                with self.subTest(identifier=identifier,defect=defect):
                    self.controls=[row[:] for row in original]
                    control=next(row for row in self.controls if row[0]==identifier)
                    if defect=='missing':self.controls.remove(control)
                    elif defect=='duplicate':self.controls.append([identifier+1000,*control[1:]])
                    else:control[3]=False
                    self.assertFalse(self.ready())

    def test_delayed_real_page_publication_uses_existing_deadline(self):
        self.published=self.controls
        self.controls=[[101,'Static','Install SKAGER',True],[106,'Button','Next >',True]]
        self.publish_at=.4
        self.wait()
        self.assertGreaterEqual(self.clock,.4)
        self.assertLess(self.clock,10)

    def test_missing_page_fails_at_existing_ten_second_bound(self):
        self.controls=[[101,'Static','Install SKAGER',True],[106,'Button','Next >',True]]
        with self.assertRaisesRegex(RuntimeError,'Installer selection page did not open'):
            self.wait()
        self.assertGreaterEqual(self.clock,10)
        self.assertLess(self.clock,10.2)

    def test_process_exit_cannot_supply_selection_evidence(self):
        self.exit_code=0
        with self.assertRaisesRegex(AssertionError,'Installer exited before its selection page'):
            self.wait()
        self.assertEqual(self.clock,0)


class PackagedStartupCompletion(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.profile = self.root / 'profile'; self.profile.mkdir()
        (self.root / 'app').mkdir()
        self.executable = self.root / 'app/opencpn.exe'
        self.executable.write_bytes(b'inert file; never execute')
        self.log = self.profile / 'opencpn.log'
        self.ready = b'------- OpenCPN version fixture\nOnInitTimer...Finalize Canvases\n'
        self.log.write_bytes(self.ready)  # The old completed run is not readiness.
        self.clock = 0.0; self.publish_at = None; self.rotation = False
        self.closed = []; self.exit_waits = []; self.close_failure = False
        self.report = {'operations': []}; self.owned = set()
        def sleep(seconds):
            self.clock += seconds
            if self.publish_at is not None and self.clock >= self.publish_at:
                self.log.write_bytes((b'rotated\n' if self.rotation else self.ready) + self.ready)
        source = SOURCE.with_name('test-packaged-updater-windows.py')
        spec = importlib.util.spec_from_file_location('packaged_completion_fixture', source)
        self.helper = importlib.util.module_from_spec(spec); spec.loader.exec_module(self.helper)
        self.time = SimpleNamespace(monotonic=lambda: self.clock, sleep=sleep)
        self.clock_patch = mock.patch.object(self.helper, 'time', self.time); self.clock_patch.start()
        nested = next(node for node in ast.walk(ast.parse(source.read_text()))
                      if isinstance(node, ast.FunctionDef) and node.name == 'launch_and_close')
        def wait_exit(handle, timeout_ms):
            self.exit_waits.append(timeout_ms)
            if self.close_failure:
                raise RuntimeError('original close failure')
        parent = SimpleNamespace(pid=1, wait=lambda **kw: 0, poll=lambda: 0)
        environment = dict(
            generation=lambda identifier: self.root, profile=self.profile,
            evidence=self.root, report=self.report, owned=self.owned,
            stock=self.root / 'stock/opencpn.exe', stock_before={},
            inventory=lambda path: {}, navigation_matches=lambda: True,
            startup_log=self.helper.startup_log, wait_startup_ready=self.helper.wait_startup_ready,
            _startup=self.helper._startup, hashlib=hashlib, time=self.time,
            process_image=lambda pid: self.executable,
            subprocess=SimpleNamespace(Popen=lambda *args, **kw: parent, STDOUT=subprocess.STDOUT),
            ui=SimpleNamespace(windows=lambda: [], wait_window=lambda *args, **kw: (10, 2),
                               IsWindowEnabled=lambda handle: True, monitor_process=lambda pid: 20,
                               close=lambda handle: self.closed.append(self.clock), wait_clean_exit=wait_exit))
        exec(compile(ast.Module(body=[nested], type_ignores=[]), str(source), 'exec'), environment)
        self.launch = environment['launch_and_close']

    def tearDown(self):
        self.clock_patch.stop(); self.temp.cleanup()

    def test_visible_window_and_exited_launcher_wait_for_fresh_deferred_init(self):
        self.publish_at = 6
        self.launch(['inert'], 'fixture', 'restored-startup', False)
        self.assertEqual(len(self.closed), 1)
        self.assertGreaterEqual(self.closed[0], 6.6)
        self.assertEqual(self.exit_waits, [30000])
        self.assertEqual(self.report['operations'][0]['status'], 'passed')
        self.assertFalse(self.owned)

    def test_rotated_fresh_log_uses_existing_readiness_policy(self):
        self.publish_at = 6; self.rotation = True
        self.launch(['inert'], 'fixture', 'rotated-startup', False)
        self.assertGreaterEqual(self.closed[0], 6.6)
        self.assertEqual(self.exit_waits, [30000])

    def test_stale_ready_log_never_closes_or_claims_startup(self):
        with self.assertRaisesRegex(RuntimeError, 'fresh startup'):
            self.launch(['inert'], 'fixture', 'incomplete-startup', False)
        self.assertFalse(self.closed); self.assertFalse(self.exit_waits)
        self.assertLess(self.clock, 46)
        record = self.report['operations'][0]
        self.assertEqual(record['lastStage'], 'fresh-startup-readiness')
        self.assertFalse(record['startupLog']['freshInitialization'])
        self.assertIn(2, self.owned)  # Existing outer failure cleanup owns child.

    def test_close_failure_stays_failure_with_bounded_actual_log(self):
        self.publish_at = 6; self.close_failure = True
        with self.assertRaisesRegex(RuntimeError, 'original close failure'):
            self.launch(['inert'], 'fixture', 'failed-close', False)
        self.assertEqual(self.exit_waits, [30000]); self.assertEqual(len(self.closed), 1)
        record = self.report['operations'][0]
        self.assertEqual(record['lastStage'], 'normal-close')
        self.assertEqual(record['status'], 'failed')
        self.assertTrue(record['startupLog']['freshInitialization'])
        self.assertEqual((self.root / record['startupLog']['path']).read_bytes(), self.log.read_bytes())
        self.assertEqual(record['startupLog']['sha256'], hashlib.sha256(self.log.read_bytes()).hexdigest())


class InstallerFinishLaunch(unittest.TestCase):
    """Execute the real Finish call site against the retained checked control."""
    def setUp(self):
        self.frame,self.pid,self.child=100,42,101
        self.controls=[(self.child,'Open SKAGER')]
        self.visible=[(self.frame,self.pid,'SKAGER Beta 2 Setup')]
        self.kind='Button';self.style=3;self.enabled=True;self.checked=1
        self.ignore_uncheck=False;self.recheck_during_capture=False
        self.replace_during_capture=False;self.detached=False
        self.sets=[];self.finish=[]
        def class_name(handle,buffer,length): buffer.value=self.kind
        def message(handle,message,wparam,lparam):
            self.assertEqual(handle,self.child)
            if self.detached:return 0  # Destroyed HWND must not prove unchecked.
            if message==0x00F0:return self.checked
            self.assertEqual((message,wparam,lparam),(0x00F1,0,0))
            self.sets.append(handle)
            if not self.ignore_uncheck:self.checked=0
            return 0
        def capture(*args,**kwargs):
            if self.recheck_during_capture:self.checked=1
            if self.replace_during_capture:
                self.controls=[(102,'Open SKAGER')];self.detached=True;self.checked=1
        def owner(handle,output):
            ctypes.cast(output,ctypes.POINTER(ctypes.c_ulong)).contents.value=self.pid
        self.environment=dict(ctypes=ctypes,h=self.frame,p=SimpleNamespace(pid=self.pid),
            EVIDENCE=Path('inert-evidence'),report={'screenshots':[]},
            time=SimpleNamespace(sleep=lambda seconds:None),
            press=lambda identifier:self.finish.append((identifier,self.checked)),
            ui=SimpleNamespace(user=None,W=SimpleNamespace(DWORD=ctypes.c_ulong),
                GetWindowThreadProcessId=owner,IsChild=lambda frame,child:child in [h for h,_ in self.controls],
                windows=lambda pid:self.visible,
                IsWindowEnabled=lambda handle:self.enabled,children=lambda frame:self.controls,
                GetClassNameW=class_name,declare=lambda *args:lambda handle,index:self.style,
                SendMessageW=message,capture=capture))
        helpers=[n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name in
                 ('installer_finish_launch_checkbox','disable_installer_finish_launch')]
        self.assertEqual(len(helpers),2)
        exec(compile(ast.Module(body=helpers,type_ignores=[]),str(SOURCE),'exec'),self.environment)
        wizard=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='wizard')
        install=next(n for n in wizard.body if isinstance(n,ast.If) and isinstance(n.test,ast.Name) and n.test.id=='install')
        start=next(i for i,n in enumerate(install.body) if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='launch_choice')
        end=next(i for i in range(start,len(install.body)) if isinstance(install.body[i],ast.Expr) and isinstance(install.body[i].value,ast.Call) and isinstance(install.body[i].value.func,ast.Name) and install.body[i].value.func.id=='press')
        self.sequence=compile(ast.Module(body=install.body[start:end+1],type_ignores=[]),str(SOURCE),'exec')

    def run_finish(self): exec(self.sequence,self.environment)

    def test_retained_checked_open_skager_is_unchecked_once_before_finish(self):
        self.run_finish()
        self.assertEqual(self.sets,[self.child]);self.assertEqual(self.finish,[(1,0)])
        self.assertEqual(self.environment['report']['finish_launch'],[
            dict(label='Open SKAGER',control=self.child,checked_before=True,checked_after=False)])

    def test_already_unchecked_stays_unchecked(self):
        self.checked=0;self.run_finish()
        self.assertEqual(self.sets,[self.child]);self.assertEqual(self.finish,[(1,0)])

    def test_unknown_missing_or_ambiguous_caption_refuses_before_finish(self):
        for controls in ([],[(self.child,'Launch SKAGER Beta 2')],[(self.child,'Open SKAGER'),(102,'Open SKAGER')]):
            with self.subTest(controls=controls):
                self.controls=controls
                with self.assertRaisesRegex(AssertionError,'one exact Open SKAGER'):self.run_finish()
                self.assertEqual(self.sets,[]);self.assertEqual(self.finish,[])

    def test_wrong_owner_disabled_noncheckbox_and_unknown_state_refuse(self):
        for field,value in (('visible',[(self.frame,99,'SKAGER Beta 2 Setup')]),
                            ('enabled',False),('kind','Static'),('style',0),('checked',2)):
            with self.subTest(field=field):
                original=getattr(self,field);setattr(self,field,value)
                with self.assertRaises(AssertionError):self.run_finish()
                self.assertEqual(self.sets,[]);self.assertEqual(self.finish,[])
                setattr(self,field,original)

    def test_failed_or_later_reversed_uncheck_never_presses_finish(self):
        for field in ('ignore_uncheck','recheck_during_capture'):
            with self.subTest(field=field):
                self.checked=1;self.sets=[];setattr(self,field,True)
                with self.assertRaisesRegex(AssertionError,'checkbox'):self.run_finish()
                self.assertEqual(self.sets,[self.child]);self.assertEqual(self.finish,[])
                setattr(self,field,False)

    def test_original_obsolete_caption_silently_leaves_launch_enabled(self):
        # Original failure sequence: the retained current caption does not
        # match, no assertion fires, and Finish sees the checked launch option.
        exec("for child,caption in ui.children(h):\n"
             "    if caption.replace('&','')=='Launch SKAGER Beta 2':\n"
             "        ui.SendMessageW(child,0x00F1,0,0)\n"
             "press(1)\n",self.environment)
        self.assertEqual(self.sets,[]);self.assertEqual(self.finish,[(1,1)])

    def test_replaced_control_with_zero_reply_from_old_handle_refuses_before_finish(self):
        self.replace_during_capture=True
        with self.assertRaisesRegex(AssertionError,'replaced before Finish'):self.run_finish()
        self.assertEqual(self.sets,[self.child]);self.assertEqual(self.finish,[])
        self.assertEqual(self.environment['ui'].SendMessageW(self.child,0x00F0,0,0),0)
        self.assertEqual(self.checked,1)


if __name__ == '__main__':
    unittest.main()

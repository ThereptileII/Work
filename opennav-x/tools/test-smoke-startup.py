#!/usr/bin/env python3
"""Inert snapshot tests: no application, display or profile changes."""
import argparse
import ast
import ctypes
import contextlib
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from smoke_startup import defer_boat_setup, native_setup_window, native_setup_observation


def snapshot(labels, tick=10):
    return {'runtime': {'ui_update': {'ticks': tick}, 'display': {
        'interaction_controls': [dict(label=label, visible=True, enabled=label!='Back',
                                      x=10, y=20, width=140, height=48) for label in labels]}}}


SETUP = ['Field: Vessel name', 'Field: Draft · metres', 'Field: Chart safety depth · metres',
         'Later', 'Back', 'Continue']


class StartupTests(unittest.TestCase):
    def test_explicit_later_waits_for_new_closed_publication(self):
        records=iter([snapshot(SETUP), snapshot(SETUP), snapshot([], 10), snapshot([], 11)])
        clicks=[]
        with patch('smoke_startup.time.sleep'):
            result=defer_boat_setup(lambda:next(records), clicks.append)
        self.assertEqual([c['label'] for c in clicks], ['Later'])
        self.assertEqual(result,dict(status='deferred-with-Later',before_ticks=10,after_ticks=11))

    def test_existing_profile_or_unrelated_later_is_untouched(self):
        for labels in ([], ['Later'], ['Auto', 'Standby']):
            clicks=[]
            self.assertEqual(defer_boat_setup(lambda:snapshot(labels),clicks.append),dict(status='not-present'))
            self.assertEqual(clicks,[])

    def test_partial_ambiguous_disabled_or_clipped_setup_refuses(self):
        cases=[snapshot(SETUP[:-1]),snapshot(SETUP+['Later']),snapshot(SETUP),snapshot(SETUP)]
        cases[2]['runtime']['display']['interaction_controls'][3]['enabled']=False
        cases[3]['runtime']['display']['interaction_controls'][3]['height']=20
        for record in cases:
            clicks=[]
            with self.assertRaises(AssertionError):defer_boat_setup(lambda:record,clicks.append)
            self.assertEqual(clicks,[])

    def test_native_dialog_handles_missing_text_fields_and_waits_for_disappearance(self):
        # Native 0db evidence has the setup buttons but no wxTextCtrl entries.
        records=iter([snapshot(['Later','Back','Continue']), snapshot([],11), snapshot([],12)])
        bounds=dict(x=0,y=0,width=660,height=620)
        windows=iter([bounds,bounds,None])
        clicks=[]
        with patch('smoke_startup.time.sleep'):
            result=defer_boat_setup(lambda:next(records),clicks.append,
                                    native_window=lambda:next(windows))
        self.assertEqual([c['label'] for c in clicks],['Later'])
        self.assertEqual(result['after_ticks'],12)

    def test_native_initial_actions_must_be_complete_unique_and_contained(self):
        cases=[snapshot(['Later','Continue']),snapshot(['Later','Back','Continue','Later']),
               snapshot(['Later','Back','Continue']),snapshot(['Later','Back','Continue']),
               snapshot(['Later','Back','Continue'])]
        cases[2]['runtime']['display']['interaction_controls'][1]['enabled']=True
        cases[3]['runtime']['display']['interaction_controls'][2]['enabled']=False
        cases[4]['runtime']['display']['interaction_controls'][0]['x']=1000
        for record in cases:
            clicks=[]
            with self.assertRaises(AssertionError):
                defer_boat_setup(lambda:record,clicks.append,
                    native_window=lambda:dict(x=0,y=0,width=660,height=620))
            self.assertEqual(clicks,[])

    def test_setup_actions_without_fields_or_native_witness_refuse(self):
        for witness in (None,lambda:None):
            clicks=[]
            with self.assertRaisesRegex(AssertionError,'lack a verified'):
                defer_boat_setup(lambda:snapshot(['Later','Back','Continue']),clicks.append,
                                native_window=witness)
            self.assertEqual(clicks,[])

    def test_native_dialog_witness_requires_exact_title_pid_class_and_bounds(self):
        class Rect(ctypes.Structure):
            _fields_=[(name,ctypes.c_long) for name in ('left','top','right','bottom')]
        rows=[(20,7,'Boat Setup & Sensor Check')]
        classname=['#32770']
        def read_class(handle,buffer,length):
            self.assertEqual(handle,20);buffer.value=classname[0];return len(buffer.value)
        def read_rect(handle,pointer):
            self.assertEqual(handle,20)
            rect=pointer._obj;rect.left=100;rect.top=90;rect.right=760;rect.bottom=710
            return True
        def windows(pid):
            self.assertEqual(pid,7);return rows
        ui=SimpleNamespace(C=ctypes,W=SimpleNamespace(RECT=Rect),windows=windows,
                           GetClassNameW=read_class,GetWindowRect=read_rect)
        self.assertEqual(native_setup_window(ui,7),dict(x=100,y=90,width=660,height=620))
        classname[0]='wxWindowNR'
        with self.assertRaises(AssertionError):native_setup_window(ui,7)
        classname[0]='#32770'
        rows[:]=[(20,8,'Boat Setup & Sensor Check')]
        with self.assertRaises(AssertionError):native_setup_window(ui,7)
        rows[:]=[(20,7,'Boat Setup & Sensor Check')]*2
        with self.assertRaises(AssertionError):native_setup_window(ui,7)
        rows[:]=[(20,7,'Another Later dialog')]
        self.assertIsNone(native_setup_window(ui,7))

    def test_opt_in_observer_records_success_without_another_click(self):
        before=snapshot(SETUP);after=snapshot([],11)
        records=iter([before,after]);observed=[];clicks=[]
        result=defer_boat_setup(lambda:next(records),clicks.append,
            observe=lambda phase,sample,target:observed.append((phase,sample,target)))
        self.assertEqual(result['status'],'deferred-with-Later')
        self.assertEqual([phase for phase,_,_ in observed],['before-click','after-click'])
        self.assertIs(observed[0][1],before);self.assertIs(observed[1][1],after)
        self.assertEqual(clicks,[observed[0][2]])

    def test_opt_in_observer_retains_timeout_without_retry_or_false_success(self):
        before=snapshot(SETUP);after=snapshot(SETUP,11)
        records=iter([before,after]);observed=[];clicks=[]
        with patch('smoke_startup.time.monotonic',side_effect=[0,0,2]),patch('smoke_startup.time.sleep'):
            with self.assertRaisesRegex(AssertionError,'did not close'):
                defer_boat_setup(lambda:next(records),clicks.append,timeout=1,
                    observe=lambda phase,sample,target:observed.append((phase,sample,target)))
        self.assertEqual([phase for phase,_,_ in observed],['before-click','after-click','timeout'])
        self.assertIs(observed[-1][1],after);self.assertEqual(len(clicks),1)

    def test_native_observer_only_reads_focus_capture_hit_and_owned_windows(self):
        class Rect(ctypes.Structure):
            _fields_=[(name,ctypes.c_long) for name in ('left','top','right','bottom')]
        class Point(ctypes.Structure):
            _fields_=[('x',ctypes.c_long),('y',ctypes.c_long)]
        declared=[]
        def declare(dll,name,*signature):
            declared.append(name)
            if name=='GetForegroundWindow':return lambda:20
            if name=='GetGUIThreadInfo':
                def info(thread,pointer):
                    value=pointer._obj;value.focus=21;value.capture=21;value.flags=0
                    return True
                return info
            if name=='GetCursorPos':
                def cursor(pointer):pointer._obj.x=80;pointer._obj.y=44;return True
                return cursor
            raise AssertionError('Observer requested unexpected API '+name)
        def class_name(handle,buffer,length):buffer.value='#32770' if handle==20 else 'wxWindowNR';return 8
        def owner(handle,pointer):pointer._obj.value=7;return 11
        def bounds(handle,pointer):
            rect=pointer._obj;rect.left=10;rect.top=20;rect.right=150;rect.bottom=68
            return True
        ui=SimpleNamespace(C=ctypes,W=SimpleNamespace(RECT=Rect,POINT=Point,DWORD=ctypes.c_ulong,
                                                     HWND=ctypes.c_void_p,BOOL=ctypes.c_int),
            user=object(),declare=declare,GetClassNameW=class_name,GetWindowThreadProcessId=owner,
            GetWindowRect=bounds,text=lambda h:'Boat Setup & Sensor Check' if h==20 else 'Later',
            IsWindowVisible=lambda h:True,IsWindowEnabled=lambda h:True,
            WindowFromPoint=lambda p:21,windows=lambda pid:[(20,pid,'Boat Setup & Sensor Check')],
            children=lambda h:[(21,'Later')])
        result=native_setup_observation(ui,7,dict(x=10,y=20,width=140,height=48))
        self.assertEqual(declared,['GetForegroundWindow','GetGUIThreadInfo','GetCursorPos'])
        self.assertEqual(result['foreground']['handle'],20)
        self.assertEqual(result['foreground_thread']['focus']['handle'],21)
        self.assertEqual(result['foreground_thread']['capture']['handle'],21)
        self.assertEqual(result['target_hit']['pid'],7)
        self.assertEqual(result['target_midpoint'],[80,44])
        self.assertEqual(result['cursor'],[80,44])
        self.assertEqual([c['title'] for c in result['later_controls']],['Later'])

    def test_setup_only_argument_contract_before_any_fixture_io(self):
        tree=ast.parse(Path(__file__).with_name('smoke-navigation.py').read_text())
        start=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and
                   any(isinstance(t,ast.Name) and t.id=='parser' for t in n.targets))
        end=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and
                 any(isinstance(t,ast.Name) and t.id=='route_standard' for t in n.targets))
        parser_code=compile(ast.Module(body=tree.body[start:end],type_ignores=[]),'navigation-arguments','exec')
        valid=['probe','--setup-only','--instruments','--trace-setup-pointer']
        with patch('sys.argv',valid):
            ns={'argparse':argparse,'sys':SimpleNamespace(platform='win32'),'__doc__':'Probe'}
            exec(parser_code,ns)
            self.assertTrue(ns['args'].setup_only)
        for platform,argv in [('win32',['probe','--setup-only']),
                              ('win32',['probe','--setup-only','--instruments']),
                              ('win32',['probe','--setup-only','--objects','--trace-setup-pointer']),
                              ('linux',valid)]:
            with patch('sys.argv',argv),contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as failure:
                    exec(parser_code,{'argparse':argparse,'sys':SimpleNamespace(platform=platform),'__doc__':'Probe'})
            self.assertEqual(failure.exception.code,2)

    def run_setup_only_exit(self,status,exit_code):
        tree=ast.parse(Path(__file__).with_name('smoke-navigation.py').read_text())
        main=next(n for n in tree.body if isinstance(n,ast.Try) and n.finalbody)
        block=next(n for n in main.body if isinstance(n,ast.If) and isinstance(n.test,ast.Attribute)
                   and n.test.attr=='setup_only')
        events=[]
        report={'first_start_setup':{'status':status}}
        ns=dict(args=SimpleNamespace(setup_only=True),report=report,handle=99,
                stop=SimpleNamespace(set=lambda:events.append('stop-input')),
                thread=SimpleNamespace(join=lambda **kw:events.append('join-input')),
                ui=SimpleNamespace(close=lambda h:events.append(('close',h))),
                app=SimpleNamespace(wait=lambda **kw:exit_code))
        return compile(ast.Module(body=[block],type_ignores=[]),'navigation-probe-exit','exec'),ns,events

    def test_setup_only_success_is_explicit_and_exits_before_navigation(self):
        code,ns,events=self.run_setup_only_exit('deferred-with-Later',0)
        with self.assertRaises(SystemExit) as completed:exec(code,ns)
        self.assertEqual(completed.exception.code,0)
        self.assertEqual(ns['report']['result'],'startup pointer probe passed')
        self.assertEqual(ns['report']['normal_exit'],0)
        self.assertEqual(events,['stop-input','join-input',('close',99)])

    def test_setup_only_missing_sheet_or_unclean_exit_never_reports_pass(self):
        for status,exit_code in [('not-present',0),('deferred-with-Later',1)]:
            code,ns,events=self.run_setup_only_exit(status,exit_code)
            with self.assertRaises(AssertionError):exec(code,ns)
            self.assertNotIn('result',ns['report'])
            if status=='not-present':self.assertEqual(events,[])

    def test_unclosed_sheet_does_not_bypass_layout_gate(self):
        clicks=[]
        with self.assertRaisesRegex(AssertionError,'did not close'):
            defer_boat_setup(lambda:snapshot(SETUP),clicks.append,timeout=0)
        self.assertEqual(len(clicks),1)


if __name__=='__main__':unittest.main()

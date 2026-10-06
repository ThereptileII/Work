#!/usr/bin/env python3
"""Inert snapshot tests: no application, display or profile changes."""
import ctypes
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from smoke_startup import defer_boat_setup, native_setup_window


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

    def test_native_creation_waits_for_advanced_initial_publication_before_click(self):
        # Even actions appearing in the same old tick cannot qualify input.
        records=iter([snapshot([]),snapshot(['Later','Back','Continue']),
                      snapshot(['Later','Back','Continue'],11),snapshot([],12)])
        bounds=dict(x=0,y=0,width=660,height=620)
        windows=iter([bounds,bounds,bounds,None])
        observations=[];clicks=[]
        def read():
            value=next(records);observations.append(value);return value
        def click(target):
            self.assertEqual(observations[-1]['runtime']['ui_update']['ticks'],11)
            clicks.append(target)
        with patch('smoke_startup.time.sleep'):
            result=defer_boat_setup(read,click,native_window=lambda:next(windows))
        self.assertEqual(len(clicks),1)
        self.assertEqual(result,dict(status='deferred-with-Later',before_ticks=11,after_ticks=12))

    def test_native_never_published_controls_timeout_without_input(self):
        clicks=[]
        with patch('smoke_startup.time.monotonic',side_effect=[0,0,2]),patch('smoke_startup.time.sleep'):
            with self.assertRaisesRegex(AssertionError,'not published'):
                defer_boat_setup(lambda:snapshot([]),clicks.append,timeout=1,
                    native_window=lambda:dict(x=0,y=0,width=660,height=620))
        self.assertEqual(clicks,[])

    def test_delayed_wrong_step_ambiguous_hidden_or_clipped_controls_still_refuse(self):
        cases=[snapshot(['Later','Back','Continue'],11) for _ in range(4)]
        cases[0]['runtime']['display']['interaction_controls'][1]['enabled']=True
        cases[1]['runtime']['display']['interaction_controls'].append(
            dict(cases[1]['runtime']['display']['interaction_controls'][0]))
        cases[2]['runtime']['display']['interaction_controls'][0]['visible']=False
        cases[3]['runtime']['display']['interaction_controls'][0]['x']=1000
        for invalid in cases:
            records=iter([snapshot([]),invalid]);clicks=[]
            with patch('smoke_startup.time.sleep'),self.assertRaises(AssertionError):
                defer_boat_setup(lambda:next(records),clicks.append,
                    native_window=lambda:dict(x=0,y=0,width=660,height=620))
            self.assertEqual(clicks,[])

    def test_native_window_change_during_publication_wait_refuses(self):
        records=iter([snapshot([]),snapshot(['Later','Back','Continue'],11)])
        windows=iter([dict(x=0,y=0,width=660,height=620),None]);clicks=[]
        with patch('smoke_startup.time.sleep'),self.assertRaisesRegex(AssertionError,'changed before'):
            defer_boat_setup(lambda:next(records),clicks.append,native_window=lambda:next(windows))
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

    def test_unclosed_sheet_does_not_bypass_layout_gate(self):
        clicks=[]
        with self.assertRaisesRegex(AssertionError,'did not close'):
            defer_boat_setup(lambda:snapshot(SETUP),clicks.append,timeout=0)
        self.assertEqual(len(clicks),1)


if __name__=='__main__':unittest.main()

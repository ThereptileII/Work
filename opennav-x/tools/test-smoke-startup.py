#!/usr/bin/env python3
"""Inert snapshot tests: no application, display or profile changes."""
import unittest
from unittest.mock import patch
from smoke_startup import defer_boat_setup


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

    def test_unclosed_sheet_does_not_bypass_layout_gate(self):
        clicks=[]
        with self.assertRaisesRegex(AssertionError,'did not close'):
            defer_boat_setup(lambda:snapshot(SETUP),clicks.append,timeout=0)
        self.assertEqual(len(clicks),1)


if __name__=='__main__':unittest.main()

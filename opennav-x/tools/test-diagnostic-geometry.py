#!/usr/bin/env python3
"""Portable regression for the native DPI test's layout-observation barrier."""
import copy
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('diagnostic_geometry', Path(__file__).with_name('diagnostic-geometry.py'))
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


class LayoutObservation(unittest.TestCase):
    def setUp(self):
        # Native 6160d3e4 150% failure capture: all current controls/rail fit.
        self.expected = {'Menu': (1166, 51, 1262, 123),
                         'Navigation': (17, 711, 185, 783),
                         'System': (1094, 711, 1262, 783)}
        self.record = {'runtime': {'ui_update': {'ticks': '12'}, 'display': {
            'interaction_controls': [dict(label=label, x=r[0], y=r[1],
                                          width=r[2]-r[0], height=r[3]-r[1],
                                          visible=True, enabled=True)
                                     for label, r in self.expected.items()],
            'rail_regions': [{'label': 'sog', 'x': 1065, 'y': 129, 'width': 204,
                              'height': 143, 'visible': True}]}}}

    def test_exact_current_native_geometry_after_publication_passes(self):
        self.assertTrue(geometry.matches_native_controls(self.record, self.expected, 8))

    def test_even_matching_older_snapshot_waits_for_new_publication(self):
        self.assertFalse(geometry.matches_native_controls(self.record, self.expected, 12))
        self.assertFalse(geometry.matches_native_controls(self.record, self.expected, 13))

    def test_newer_tick_with_old_frame_position_still_refuses(self):
        old = copy.deepcopy(self.record)
        old['runtime']['display']['interaction_controls'][0]['x'] += 64
        self.assertFalse(geometry.matches_native_controls(old, self.expected, 8))
        self.assertTrue(geometry.matches_native_controls(self.record, self.expected, 8))

    def test_wrong_size_or_vertical_layout_is_not_a_current_observation(self):
        for field in ('x', 'y', 'width', 'height'):
            with self.subTest(field=field):
                changed = copy.deepcopy(self.record)
                changed['runtime']['display']['interaction_controls'][1][field] += 1
                self.assertFalse(geometry.matches_native_controls(changed, self.expected, 8))

    def test_missing_duplicate_and_hidden_chrome_refuse(self):
        for kind in ('missing', 'duplicate', 'hidden'):
            with self.subTest(kind=kind):
                changed = copy.deepcopy(self.record)
                controls = changed['runtime']['display']['interaction_controls']
                if kind == 'missing':
                    controls.pop()
                elif kind == 'duplicate':
                    controls.append(copy.deepcopy(controls[0]))
                else:
                    controls[0]['visible'] = False
                self.assertFalse(geometry.matches_native_controls(changed, self.expected, 8))

    def test_missing_or_malformed_observation_refuses(self):
        self.assertFalse(geometry.matches_native_controls({}, self.expected, 8))
        self.record['runtime']['ui_update']['ticks'] = 'invalid'
        self.assertFalse(geometry.matches_native_controls(self.record, self.expected, 8))

    def test_empty_native_reference_cannot_pass_vacuously(self):
        self.assertFalse(geometry.matches_native_controls(self.record, {}, 8))

    def test_scrolled_row_pairs_offscreen_bounds_without_asserting_visibility(self):
        row=self.record['runtime']['display']['interaction_controls'][0]
        row['visible']=False
        self.assertFalse(geometry.matches_native_controls(self.record,self.expected,8))
        self.assertTrue(geometry.matches_native_controls(self.record,self.expected,8,require_visible=False))
        row['y']+=24
        self.assertFalse(geometry.matches_native_controls(self.record,self.expected,8,require_visible=False))

    def test_offscreen_observation_still_rejects_old_ticks_and_duplicate_rows(self):
        self.assertFalse(geometry.matches_native_controls(self.record,self.expected,12,require_visible=False))
        controls=self.record['runtime']['display']['interaction_controls']
        controls.append(copy.deepcopy(controls[0]))
        self.assertFalse(geometry.matches_native_controls(self.record,self.expected,8,require_visible=False))

    def test_real_rail_clipping_is_not_filtered_by_observation_barrier(self):
        # The exact rejected 6160 rail sample must reach the unchanged bounds
        # assertion if it recurs with CURRENT chrome. Do not poll until it fits.
        region = self.record['runtime']['display']['rail_regions'][0]
        region.update(x=1129, width=204, height=122)
        self.assertTrue(geometry.matches_native_controls(self.record, self.expected, 8))
        with self.assertRaises(AssertionError):
            assert 0 <= region['x'] < region['x']+region['width'] <= 1280


if __name__ == '__main__':
    unittest.main()

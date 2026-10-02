"""Native-DPI harness targeting regressions; no simulated touch acceptance.

Load only the pure path selector, not the Windows smoke script's startup code.
The recorded failure rectangles establish that the old fixed gesture started
inside a native field. Synthetic hit tests exercise fail-closed path selection;
only a Windows replay can prove native gesture delivery and actual scrolling.
"""
import ast
from pathlib import Path
import unittest

SOURCE = Path(__file__).resolve().parents[1] / 'tools/preferences-touch.py'
tree = ast.parse(SOURCE.read_text())
selector = next(n for n in tree.body
                if isinstance(n, ast.FunctionDef) and n.name == 'preferences_pan_path')
namespace = {}
exec(compile(ast.Module(body=[selector], type_ignores=[]), str(SOURCE), 'exec'), namespace)
select = namespace['preferences_pan_path']

# Exact physical rectangles from retained FFE dpi-results.json, 125% / 120 DPI.
DRAWER = (545, 128, 1058, 733)
BATTERY_FIELD = (590, 683, 1012, 711)
LOWER_ACTION = (574, 1067, 1028, 1157)
# A synthetic body viewport, not a recorded native HWND measurement.
BODY = (546, 240, 1057, 732)


def contains(rect, x, y):
    return rect[0] <= x < rect[2] and rect[1] <= y < rect[3]


class PreferencesPan(unittest.TestCase):
    def test_retained_old_start_hits_edit_geometry(self):
        x = DRAWER[0] + (DRAWER[2] - DRAWER[0]) // 2
        y = DRAWER[3] - 50
        self.assertEqual((x, y), (801, 683))
        self.assertTrue(contains(BATTERY_FIELD, x, y))

    def test_body_target_avoids_recorded_edit(self):
        def body_hit(x, y):
            return contains(BODY, x, y) and not contains(BATTERY_FIELD, x, y)
        start, end = select(BODY, LOWER_ACTION, 125, body_hit)
        self.assertTrue(body_hit(*start) and body_hit(*end))
        self.assertLess(end[1], start[1])
        self.assertEqual(start[1] - end[1], 200)

    def test_obscured_left_gutter_uses_verified_right(self):
        start, end = select(BODY, LOWER_ACTION, 125,
                            lambda x, y: x > (BODY[0] + BODY[2]) // 2)
        self.assertGreater(start[0], (BODY[0] + BODY[2]) // 2)
        self.assertEqual(start[0], end[0])

    def test_field_or_other_child_is_not_scroll_body(self):
        with self.assertRaisesRegex(AssertionError, 'unobscured'):
            select(BODY, LOWER_ACTION, 125, lambda x, y: False)

    def test_end_must_also_hit_body(self):
        with self.assertRaisesRegex(AssertionError, 'unobscured'):
            select(BODY, LOWER_ACTION, 125, lambda x, y: y > 680)

    def test_action_above_viewport_reverses_gesture(self):
        start, end = select(BODY, (574, 150, 1028, 220), 125, lambda x, y: True)
        self.assertGreater(end[1], start[1])
        self.assertTrue(contains(BODY, *start) and contains(BODY, *end))

    def test_short_body_refuses_gesture(self):
        with self.assertRaisesRegex(AssertionError, 'usable'):
            select((546, 240, 1057, 290), LOWER_ACTION, 125, lambda x, y: True)

    def test_visible_action_requires_no_pan(self):
        with self.assertRaisesRegex(AssertionError, 'needs no pan'):
            select(BODY, (574, 300, 1028, 390), 125, lambda x, y: True)

    def test_existing_dpi_scales_keep_bounded_body_paths(self):
        for scale in (100, 125, 150):
            with self.subTest(scale=scale):
                start, end = select(BODY, LOWER_ACTION, scale, lambda x, y: True)
                self.assertEqual(start[1] - end[1], round(160 * scale / 100))
                self.assertTrue(contains(BODY, *start) and contains(BODY, *end))


if __name__ == '__main__':
    unittest.main()

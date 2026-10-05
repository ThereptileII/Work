"""Regression for the retained FFE hidden-ProductPanel Diagnostics collision.

Extract only the pure selector: importing smoke-preview would launch an app.
"""
import ast
import copy
from pathlib import Path
import unittest

source = Path(__file__).resolve().parents[1] / 'tools/smoke-preview.py'
tree = ast.parse(source.read_text())
selector = next(node for node in tree.body
                if isinstance(node, ast.FunctionDef) and node.name == 'preferences_target')
namespace = {}
exec(compile(ast.Module(body=[selector], type_ignores=[]), str(source), 'exec'), namespace)
select = namespace['preferences_target']

# Exact relevant controls and drawer from FFE preview-logs/opennav-diagnostics.json.
RECORD = {'ui_page': 'Settings', 'runtime': {'display': {
    'drawer': {'height': 674, 'y': 80, 'width': 432, 'x': 648},
    'interaction_controls': [
        {'height': 52, 'y': 261, 'visible': False, 'width': 487,
         'accessible_name': 'Diagnostics', 'x': 591, 'label': 'Diagnostics', 'enabled': True},
        {'height': 72, 'y': 664, 'visible': True, 'width': 386,
         'accessible_name': 'Diagnostics', 'x': 671, 'label': 'Diagnostics', 'enabled': True},
    ]}}}


class PreferencesSelection(unittest.TestCase):
    def test_recorded_hidden_page_collision(self):
        controls = RECORD['runtime']['display']['interaction_controls']
        # The old label-only uniqueness assertion fails on these actual rows.
        self.assertEqual(len([r for r in controls if r['label'] == 'Diagnostics']), 2)
        target, usable = select(RECORD, 'Diagnostics')
        self.assertIs(target, controls[1])
        self.assertTrue(usable)

    def test_duplicate_visible_actions_remain_failure(self):
        record = copy.deepcopy(RECORD)
        controls = record['runtime']['display']['interaction_controls']
        controls.append(copy.deepcopy(controls[1]))
        with self.assertRaises(AssertionError): select(record, 'Diagnostics')

    def test_below_fold_action_requires_scroll(self):
        record = copy.deepcopy(RECORD)
        target = record['runtime']['display']['interaction_controls'][1]
        target.update(y=820, visible=False)
        observed, usable = select(record, 'Diagnostics')
        self.assertIs(observed, target)
        self.assertFalse(usable)

    def test_disabled_drawer_action_cannot_select_hidden_page(self):
        record = copy.deepcopy(RECORD)
        record['runtime']['display']['interaction_controls'][1]['enabled'] = False
        with self.assertRaises(AssertionError): select(record, 'Diagnostics')

    def test_clipped_visible_record_is_not_clickable(self):
        record = copy.deepcopy(RECORD)
        record['runtime']['display']['interaction_controls'][1]['y'] = 720
        self.assertFalse(select(record, 'Diagnostics')[1])


if __name__ == '__main__':
    unittest.main()

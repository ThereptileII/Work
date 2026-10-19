#!/usr/bin/env python3
"""Check native-name selection without starting the navigation fixture."""
import ast
import copy
import ctypes
from ctypes import wintypes
from pathlib import Path
import unittest


source = Path(__file__).with_name('smoke-navigation.py')
tree = ast.parse(source.read_text(encoding='utf-8'))
function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                and node.name == 'require_native_waypoint_name')
namespace = {}
exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
require_name = namespace['require_native_waypoint_name']


class NativeUi:
    C, W = ctypes, wintypes

    def __init__(self):
        self.nodes = {
            1: dict(parent=0, caption='SKAGER / OpenCPN', native_class='wxWindow',
                    rect=(0, 0, 1280, 800)),
            2: dict(parent=1, caption='SKAGER product page: Waypoint detail', native_class='wxWindow',
                    rect=(80, 68, 1094, 766)),
            3: dict(parent=2, caption='', native_class='Edit', rect=(100, 200, 600, 248),
                    value='ALPHA TEST UI edited'),
            # A matching caption must never hide a failed edit or cancellation.
            4: dict(parent=1, caption='ALPHA TEST UI edited', native_class='Static',
                    rect=(0, 0, 50, 20)),
        }
        for node in self.nodes.values():
            node.update(pid=42, visible=True, enabled=True)
        self.reads = []

    def IsChild(self, owner, window):
        while window in self.nodes:
            window = self.nodes[window]['parent']
            if window == owner:
                return True
        return False

    def children(self, owner):
        return [(handle, node['caption']) for handle, node in self.nodes.items()
                if node['visible'] and self.IsChild(owner, handle)]

    def GetWindowThreadProcessId(self, handle, result):
        result._obj.value = self.nodes[handle]['pid']
        return 10

    def IsWindowVisible(self, handle):
        return self.nodes[handle]['visible']

    def IsWindowEnabled(self, handle):
        return self.nodes[handle]['enabled']

    def GetWindowRect(self, handle, result):
        result._obj.left, result._obj.top, result._obj.right, result._obj.bottom = self.nodes[handle]['rect']
        return True

    def GetClassNameW(self, handle, result, capacity):
        result.value = self.nodes[handle]['native_class']
        return len(result.value)

    def control_text(self, handle):
        self.reads.append(handle)
        return self.nodes[handle].get('value', self.nodes[handle]['caption'])


class NativeWaypointName(unittest.TestCase):
    def setUp(self):
        self.ui = NativeUi()
        self.field = dict(label='Field: Waypoint name', accessible_name='Waypoint name',
                          x=100, y=200, width=500, height=48, visible=True, enabled=True)
        self.record = dict(ui_page='Waypoint detail', runtime=dict(display=dict(interaction_controls=[self.field])))

    def read(self):
        return require_name(self.ui, 1, 42, self.record, 'ALPHA TEST UI edited')

    def test_reads_owned_edit_buffer_despite_empty_cached_caption(self):
        self.assertEqual(self.read()['value'], 'ALPHA TEST UI edited')
        self.assertEqual(self.ui.reads, [3])

    def test_exact_native_page_edit_witness_handles_unpublished_msw_field(self):
        self.record['runtime']['display']['interaction_controls'] = []
        self.assertFalse(self.read()['diagnostic_field_published'])
        self.assertEqual(self.ui.reads, [3])

    def test_matching_unrelated_caption_cannot_hide_wrong_value(self):
        for value in ('ALPHA TEST edited', '', 'Deleted'):
            with self.subTest(value=value):
                self.ui.nodes[3]['value'] = value
                with self.assertRaises(AssertionError):
                    self.read()

    def test_unselected_page_is_refused(self):
        self.record['ui_page'] = 'Route detail'
        with self.assertRaises(AssertionError):
            self.read()

    def test_missing_or_duplicate_page_is_refused(self):
        for duplicate in (False, True):
            with self.subTest(duplicate=duplicate):
                self.ui = NativeUi()
                if duplicate:
                    self.ui.nodes[5] = copy.deepcopy(self.ui.nodes[2])
                else:
                    self.ui.nodes[2]['caption'] = 'SKAGER product page: Route detail'
                with self.assertRaises(AssertionError):
                    self.read()

    def test_missing_duplicate_or_non_edit_field_is_refused(self):
        for kind in ('missing', 'duplicate', 'wrong-class'):
            with self.subTest(kind=kind):
                self.ui = NativeUi()
                if kind == 'missing':
                    self.ui.nodes[3]['parent'] = 1
                elif kind == 'duplicate':
                    self.ui.nodes[5] = copy.deepcopy(self.ui.nodes[3])
                else:
                    self.ui.nodes[3]['native_class'] = 'Static'
                with self.assertRaises(AssertionError):
                    self.read()

    def test_owner_or_enabled_state_must_match_for_frame_page_and_edit(self):
        for handle in (1, 2, 3):
            for key, value in (('pid', 99), ('visible', False), ('enabled', False)):
                with self.subTest(handle=handle, key=key):
                    self.ui = NativeUi()
                    self.ui.nodes[handle][key] = value
                    with self.assertRaises(AssertionError):
                        self.read()

    def test_clipped_edit_is_refused(self):
        self.ui.nodes[3]['rect'] = (70, 200, 600, 248)
        with self.assertRaises(AssertionError):
            self.read()

    def test_published_field_must_be_unique_and_match_native_identity_geometry(self):
        for key, value in (('label', 'Field: Other'), ('accessible_name', 'Other'),
                           ('x', 101), ('height', 49), ('visible', False), ('enabled', False)):
            with self.subTest(key=key):
                field = dict(self.field, **{key: value})
                self.record['runtime']['display']['interaction_controls'] = [field]
                with self.assertRaises(AssertionError):
                    self.read()
        self.record['runtime']['display']['interaction_controls'] = [self.field, dict(self.field)]
        with self.assertRaises(AssertionError):
            self.read()


if __name__ == '__main__':
    unittest.main()

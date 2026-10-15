#!/usr/bin/env python3
"""Portable checks of the native mode smoke's actual palette interaction block."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


def palette_block(source=None):
    tree = ast.parse(source if source is not None else
                     Path(__file__).with_name('smoke-modes-windows.py').read_text())
    body = next(node.body for node in tree.body if isinstance(node, ast.Try))
    capture = next(index for index, node in enumerate(body)
                   if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
                   and isinstance(node.value.func, ast.Name)
                   and node.value.func.id == 'capture_chart')
    # Include the source-state barrier preceding the first capture, and stop
    # before the unrelated zoom/simulation checks. No Windows import is needed.
    previous = body[capture-1]
    start = capture - 1 if (isinstance(previous, ast.Expr) and
                           isinstance(previous.value, ast.Call) and
                           isinstance(previous.value.func, ast.Name) and
                           previous.value.func.id == 'data') else capture
    stop = next(index for index, node in enumerate(body)
                if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
                and isinstance(node.value.func, ast.Attribute)
                and node.value.func.attr == 'click_text')
    return compile(ast.Module(body=body[start:stop], type_ignores=[]),
                   'smoke-modes-windows.py:palette', 'exec')


class ModePaletteInput(unittest.TestCase):
    def run_palette(self, initial='Day', blocked=None, wrong=None, pointer_error=False):
        self.light = initial
        self.actions = []
        self.captures = []
        transitions = {'Day': 'Dusk', 'Dusk': 'Night', 'Night': 'Day'}

        def data(predicate):
            record = {'runtime': {'display': {'light': self.light}}}
            if not predicate(record):
                raise AssertionError('Mode-cycle diagnostic state did not arrive')
            return record

        def pointer(pid, label):
            self.assertEqual(pid, 17)
            self.actions.append(label)
            if pointer_error:
                raise RuntimeError('Pointer target is covered')
            self.assertEqual(label, self.light)
            if label != blocked:
                self.light = wrong or transitions[label]

        def capture(name, style='XNav', light='Day'):
            self.assertEqual(style, 'XNav')
            self.assertEqual(light, self.light)
            self.captures.append((name, light))

        # Model the observed failure: direct HWND messages can return without
        # activating the palette, whereas checked pointer input changes it.
        ui = SimpleNamespace(pointer_text=pointer, cycle_light=lambda pid: None)
        exec(palette_block(), dict(ui=ui, pid=17, data=data, capture_chart=capture))

    def test_real_pointer_visits_all_modes_and_returns_to_day(self):
        self.run_palette()
        self.assertEqual(self.actions, ['Day', 'Dusk', 'Night'])
        self.assertEqual(self.light, 'Day')
        self.assertEqual(self.captures, [('01-xnav-unavailable.png', 'Day'),
                                        ('02-xnav-dusk.png', 'Dusk'),
                                        ('03-xnav-night.png', 'Night')])

    def test_wrong_initial_mode_refuses_before_capture_or_input(self):
        with self.assertRaisesRegex(AssertionError, 'diagnostic state'):
            self.run_palette(initial='Night')
        self.assertEqual((self.actions, self.captures), ([], []))

    def test_missing_transition_still_fails_at_each_destination(self):
        for mode, count in [('Day', 1), ('Dusk', 2), ('Night', 3)]:
            with self.subTest(mode=mode):
                with self.assertRaisesRegex(AssertionError, 'diagnostic state'):
                    self.run_palette(blocked=mode)
                self.assertEqual(len(self.actions), count)

    def test_skipped_dusk_is_not_accepted_as_a_successful_click(self):
        with self.assertRaisesRegex(AssertionError, 'diagnostic state'):
            self.run_palette(wrong='Night')
        self.assertEqual(self.actions, ['Day'])
        self.assertEqual(len(self.captures), 1)

    def test_pointer_refusal_propagates_without_another_input_path(self):
        with self.assertRaisesRegex(RuntimeError, 'covered'):
            self.run_palette(pointer_error=True)
        self.assertEqual(self.actions, ['Day'])
        self.assertEqual(len(self.captures), 1)


if __name__ == '__main__':
    unittest.main()

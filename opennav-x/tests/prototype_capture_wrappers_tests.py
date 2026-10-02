#!/usr/bin/env python3
"""Inert wrapper contracts: no native client, desktop, rendering or builds."""
import importlib.util
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools/prototype' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


settings = load('capture-ais-component')
horizon = load('capture-horizon-component')


def emitted_names(source):
    return re.findall(r'Capture\("([^"]+)"\)', (ROOT / 'tests' / source).read_text())


class CaptureWrapperTests(unittest.TestCase):
    def test_current_settings_state_machine_is_accepted(self):
        names = emitted_names('settings_drawer_test.cpp')
        self.assertEqual(len(names), 14)
        settings.validate_capture_names('settings', names, 14)
        self.assertEqual(settings.capture_theme('settings', 'display-applied-125-chart'), 'night')
        self.assertEqual(settings.capture_theme('settings', 'display-150-chart-night'), 'night')
        for theme in ('day', 'dusk', 'night'):
            self.assertEqual(settings.capture_theme('settings', 'display-' + theme), theme)
            self.assertEqual(settings.capture_theme('ais', 'ais-' + theme), theme)

    def test_settings_rejects_missing_unknown_and_duplicate_states(self):
        names = emitted_names('settings_drawer_test.cpp')
        for bad in (names[:-1], names + ['unknown-night'], names[:-1] + [names[0]],
                    names[:-1] + ['unknown-night']):
            with self.subTest(names=bad), self.assertRaises(AssertionError):
                settings.validate_capture_names('settings', bad, 14)
        with self.assertRaises(KeyError):
            settings.capture_theme('settings', 'unreviewed-night')

    def test_current_horizon_state_machine_keeps_all_canonical_states(self):
        names = emitted_names('horizon_test.cpp')
        self.assertEqual(len(names), 15)
        self.assertEqual(horizon.validate_capture_names(names),
                         {'prototype-fixture-day', 'prototype-fixture-dusk', 'prototype-fixture-night'})
        self.assertGreaterEqual(horizon.DESKTOP_SIZE[0], 1920)
        self.assertGreaterEqual(horizon.DESKTOP_SIZE[1], 1080)
        horizon.validate_capture_size('prototype-large-desktop-1920', (1920, 1080))

    def test_horizon_rejects_missing_duplicate_unknown_or_cropped_evidence(self):
        names = emitted_names('horizon_test.cpp')
        for bad in (names[:-1], names + ['unknown'], names[:-1] + [names[0]],
                    [n for n in names if n != 'prototype-fixture-night']):
            with self.subTest(names=bad), self.assertRaises(AssertionError):
                horizon.validate_capture_names(bad)
        for size in ((1440, 900), (1280, 800), (1920, 1079), (1921, 1080)):
            with self.subTest(size=size), self.assertRaises(AssertionError):
                horizon.validate_capture_size('prototype-large-desktop-1920', size)
        with self.assertRaises(AssertionError):
            horizon.exact_geometry([88, 896, 1612, 150], [88, 896, 1612, 148], 'wrong height')


if __name__ == '__main__':
    unittest.main()

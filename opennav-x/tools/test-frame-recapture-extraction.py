#!/usr/bin/env python3
"""Portable checks of the exact production extraction used by native focus tests."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('recapture', ROOT / 'tools/test-frame-recapture-windows.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Extraction(unittest.TestCase):
    def test_only_setup_guard_changes(self):
        result = module.extract(ROOT)
        self.assertEqual(set(result), {'original', 'guarded'})
        self.assertEqual(result['original']['shell-transient.inc'],
                         result['guarded']['shell-transient.inc'].replace(module.GUARD, ''))
        for name in ('frame-recapture.inc', 'integration-transient.inc'):
            self.assertEqual(result['original'][name], result['guarded'][name])
        self.assertIn('!dialog->IsModal()', result['original']['shell-transient.inc'])
        self.assertIn('shell && IsXNav()', result['original']['integration-transient.inc'])

    def test_missing_or_duplicate_method_refused(self):
        for source in ('', 'void F() {}\nvoid F() {}'):
            with self.assertRaises(ValueError):
                module.method(source, 'void F()')

    def test_real_button_sources_are_compiled(self):
        cmake = (ROOT / 'tests/frame_recapture_windows/CMakeLists.txt').read_text()
        self.assertIn('../../src/ui/Controls.cpp', cmake)
        self.assertIn('XNAV_ENABLE_TEST_FIXTURES=0', cmake)
        self.assertNotIn('Shell.cpp', cmake)


if __name__ == '__main__':
    unittest.main()

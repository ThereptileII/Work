#!/usr/bin/env python3
"""Portable parser checks; actual Windows resource APIs remain a native gate."""
import importlib.util
from pathlib import Path
import struct
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('brand', ROOT / 'tools/verify-skager-pe-brand.py')
brand = importlib.util.module_from_spec(spec)
spec.loader.exec_module(brand)


class BrandResourceTests(unittest.TestCase):
    def setUp(self):
        self.frames = brand.ico_frames((ROOT / 'resources/branding/skager.ico').read_bytes())
        self.ids = dict(enumerate(self.frames.values(), 1))
        self.group = struct.pack('<HHH', 0, 1, len(self.frames)) + b''.join(
            struct.pack('<BBBBHHIH', size % 256, size % 256, 0, 0, 1, 32, len(data), ident)
            for ident, (size, data) in enumerate(self.frames.items(), 1))

    def test_all_approved_frames(self):
        self.assertEqual(sorted(brand.check_group(self.group, self.ids.__getitem__, self.frames)),
                         [16, 20, 24, 32, 40, 48, 64, 128, 256])

    def test_one_changed_pixel_byte_rejected(self):
        data = bytearray(self.ids[1]); data[-1] ^= 1; self.ids[1] = bytes(data)
        with self.assertRaisesRegex(ValueError, 'differs from approved frame bytes'):
            brand.check_group(self.group, self.ids.__getitem__, self.frames)

    def test_missing_frame_rejected(self):
        group = struct.pack('<HHH', 0, 1, len(self.frames) - 1) + self.group[6:-14]
        with self.assertRaisesRegex(ValueError, 'differ from required'):
            brand.check_group(group, self.ids.__getitem__, self.frames)

    def test_source_metadata(self):
        self.assertEqual(brand.expected_metadata((ROOT / 'src/integration/Skager.rc.in').read_text()),
                         {'ProductName': 'SKAGER', 'FileDescription': 'SKAGER navigation based on OpenCPN'})
        self.assertEqual(brand.expected_metadata((ROOT / 'installer/windows/AlphaSetup.nsi').read_text(), True),
                         {'ProductName': 'SKAGER Beta 2', 'FileDescription': 'Version-gated SKAGER Beta setup'})


if __name__ == '__main__':
    unittest.main()

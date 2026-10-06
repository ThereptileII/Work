#!/usr/bin/env python3
"""Portable parser checks; actual Windows resource APIs remain a native gate."""
import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('brand', ROOT / 'tools/verify-skager-pe-brand.py')
brand = importlib.util.module_from_spec(spec)
spec.loader.exec_module(brand)


def fixed_bytes(file=(5, 12, 4, 0), product=(0, 4, 0, 1)):
    return struct.pack('<13I', 0xFEEF04BD, 0x00010000,
                       file[0] << 16 | file[1], file[2] << 16 | file[3],
                       product[0] << 16 | product[1], product[2] << 16 | product[3],
                       0, 0, 0x40004, 1, 0, 0, 0)


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

    def test_exact_application_and_setup_version_contracts(self):
        for version, fourth in [('0.4.0-beta2', 0), ('0.4.0-beta2.1', 1), ('0.4.0-beta2.65535', 65535)]:
            product = (0, 4, 0, fourth)
            for setup in (False, True):
                with self.subTest(version=version, setup=setup):
                    expected = brand.expected_versions(version, setup)
                    self.assertEqual(expected['strings'], {'ProductVersion': version,
                                     'FileVersion': version if setup else '5,12,4'})
                    file = product if setup else (5, 12, 4, 0)
                    self.assertEqual(brand.check_fixed_version(fixed_bytes(file, product), expected['fixed']),
                                     {'file': file, 'product': product})

    def test_numeric_versions_reject_wrong_identity_and_malformed_data(self):
        expected = brand.expected_versions('0.4.0-beta2.1')['fixed']
        for data in [fixed_bytes(product=(0, 4, 0, 0)), fixed_bytes(file=(5, 12, 3, 0)),
                     fixed_bytes()[:-1], fixed_bytes() + b'\0',
                     b'\0' * 4 + fixed_bytes()[4:], fixed_bytes()[:4] + b'\0' * 4 + fixed_bytes()[8:]]:
            with self.subTest(data=data), self.assertRaises(ValueError):
                brand.check_fixed_version(data, expected)
        with self.assertRaises(ValueError):
            brand.expected_versions('0.4.0-beta2.01')

    def test_inspection_requires_all_strings_and_numeric_versions(self):
        metadata = {'ProductName': 'SKAGER', 'FileDescription': 'Fixture'}
        version = brand.expected_versions('0.4.0-beta2.1')
        values = {key: [value] for key, value in {**metadata, **version['strings']}.items()}
        test = self
        class InertResources:
            numeric = fixed_bytes()
            closed = False
            def __init__(self, path): pass
            def close(self): InertResources.closed = True
            def entries(self, kind): return [(1, 1033)]
            def read(self, kind, name, language):
                return test.group if kind == 14 else test.ids[name] if kind == 3 else b'inert version resource'
            def version(self, data, field): return values[field]
            def fixed_version(self, data): return self.numeric
        with tempfile.TemporaryDirectory() as directory, patch.object(brand, 'Resources', InertResources):
            path = Path(directory) / 'inert.exe';path.write_bytes(b'never executed')
            result = brand.inspect(path, self.frames, metadata, version)
            self.assertEqual(result['versions'][0]['fixed'], version['fixed'])
            for field in ('ProductName', 'FileDescription', 'ProductVersion', 'FileVersion'):
                original = values[field]
                for wrong in [[], ['wrong'], original + ['wrong']]:
                    with self.subTest(field=field, wrong=wrong), self.assertRaises(ValueError):
                        values[field] = wrong
                        brand.inspect(path, self.frames, metadata, version)
                    self.assertTrue(InertResources.closed)
                values[field] = original
            InertResources.numeric = fixed_bytes(product=(0, 4, 0, 2))
            with self.assertRaisesRegex(ValueError, 'Numeric PE version mismatch'):
                brand.inspect(path, self.frames, metadata, version)
            self.assertTrue(InertResources.closed)


if __name__ == '__main__':
    unittest.main()

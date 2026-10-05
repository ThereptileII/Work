#!/usr/bin/env python3
"""Canonical Beta 2 sequence and exact source/resource identity; no builds."""
from pathlib import Path
import tempfile
import unittest

from product_version import read_product_version, validate_product_version, windows_product_version


class ProductVersionTests(unittest.TestCase):
    def test_historical_and_increasing_successors_have_exact_resource_identity(self):
        for version, fourth in [('0.4.0-beta2', 0), ('0.4.0-beta2.1', 1),
                                ('0.4.0-beta2.2', 2), ('0.4.0-beta2.65535', 65535)]:
            self.assertEqual(validate_product_version(version), version)
            self.assertEqual(windows_product_version(version), (0, 4, 0, fourth))

    def test_ambiguous_out_of_series_or_unrepresentable_versions_refuse(self):
        for version in ['0.4.0-beta2.0', '0.4.0-beta2.01', '0.4.0-beta2.65536',
                        '0.4.0-beta2.1+build', '0.4.0-beta2.1.2', '0.4.0',
                        '0.4.1-beta2', '0.4.0-beta3', ' 0.4.0-beta2', None, 1]:
            with self.subTest(version=version), self.assertRaises(ValueError):
                validate_product_version(version)

    def test_header_is_authority_and_ambiguous_or_other_edition_refuses(self):
        with tempfile.TemporaryDirectory() as temporary:
            header = Path(temporary)/'Version.h'
            text = ('#pragma once\nnamespace opennav::application {\n'
                    'inline constexpr char Version[] = "0.4.0-beta2.2";\n'
                    'inline constexpr char Edition[] = "Beta 2";\n}\n')
            header.write_text(text)
            self.assertEqual(read_product_version(header), '0.4.0-beta2.2')
            for changed in [text.replace('Beta 2', 'Production'), text+text,
                            text.replace('Version[]', 'Unknown[]')]:
                header.write_text(changed)
                with self.assertRaises(ValueError): read_product_version(header)


if __name__ == '__main__':
    unittest.main()

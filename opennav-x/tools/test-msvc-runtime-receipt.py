#!/usr/bin/env python3
"""Inert packaging fixtures; native Windows version-resource checks remain required."""
import base64
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

import msvc_runtime_receipt as receipt


def pe(machine=0x14c):
    data = bytearray(128)
    data[:2] = b'MZ'
    struct.pack_into('<I', data, 0x3c, 64)
    data[64:68] = b'PE\0\0'
    struct.pack_into('<H', data, 68, machine)
    return bytes(data)


class RuntimeReceiptTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.vs = self.root / 'Visual Studio'
        self.runtime = self.vs / 'VC/Redist/MSVC/14.44.35211/x86/Microsoft.VC143.CRT'
        self.runtime.mkdir(parents=True)
        self.app = self.root / 'app'
        self.app.mkdir()
        for name in ('vcruntime140.dll', 'msvcp140.dll', 'concrt140.dll'):
            (self.runtime / name).write_bytes(pe() + name.encode())
        self.product_id = 'Microsoft.VisualStudio.Product.Enterprise'
        self.catalog = {'id': 'VisualStudio/17.14.0+12345.1',
                        'buildVersion': '17.14.12345.1',
                        'productDisplayVersion': '17.14.0', 'productName': 'Visual Studio',
                        'productSemanticVersion': '17.14.0+12345.1',
                        'productPreReleaseMilestoneSuffix': ''}
        self.versions = {'fileVersion': '14.44.35211.0', 'productVersion': '14.44.35211.0'}
        version_patch = patch.object(receipt, 'windows_file_versions', return_value=self.versions)
        self.version_reader = version_patch.start()
        self.addCleanup(version_patch.stop)

    def copy(self, **identity):
        return receipt.copy_msvc_runtime(self.runtime, self.app, self.vs, '17.14.12345.1',
                                         identity.get('product_id', self.product_id),
                                         identity.get('catalog', self.catalog))

    def test_complete_sorted_receipt_is_repeatable_and_read_only(self):
        first = self.copy()
        self.assertEqual(first, self.copy())
        self.assertEqual(first['schemaVersion'], 2)
        self.assertEqual(first['visualStudio']['productId'], self.product_id)
        self.assertEqual(first['visualStudio']['catalog'], self.catalog)
        self.catalog['productName'] = 'Changed input after receipt'
        self.assertEqual(first['visualStudio']['catalog']['productName'], 'Visual Studio')
        self.assertEqual([f['name'] for f in first['files']],
                         ['concrt140.dll', 'msvcp140.dll', 'vcruntime140.dll'])
        self.assertEqual(first['redistributable']['version'], '14.44.35211')
        self.assertEqual(first['visualStudio']['installationDirectory'], str(self.vs))
        for record in first['files']:
            self.assertEqual((self.app / record['name']).read_bytes(),
                             (self.runtime / record['name']).read_bytes())
            self.assertEqual(record['sha256'], hashlib.sha256((self.app / record['name']).read_bytes()).hexdigest())
            self.assertEqual(record['fileVersion'], '14.44.35211.0')
        output = self.root / 'MSVC_RUNTIME_RECEIPT.json'
        receipt.write_runtime_receipt(first, output)
        self.addCleanup(output.chmod, 0o644)
        self.assertEqual(json.loads(output.read_text()), first)
        self.assertFalse(output.stat().st_mode & stat.S_IWUSR)
        self.assertIn('unresolved', first['legalStatus'])
        with self.assertRaises(FileExistsError):
            receipt.write_runtime_receipt(first, output)

    def test_missing_source_cannot_be_satisfied_by_stale_installed_dll(self):
        (self.runtime / 'msvcp140.dll').unlink()
        (self.app / 'msvcp140.dll').write_bytes(pe())
        with self.assertRaisesRegex(ValueError, 'required DLL missing'):
            self.copy()

    def test_wrong_vs_selection_rejected(self):
        with self.assertRaisesRegex(ValueError, 'selected Visual Studio'):
            receipt.copy_msvc_runtime(self.runtime, self.app, self.root / 'other', '17.14.1',
                                      self.product_id, self.catalog)

    def test_64_bit_source_rejected(self):
        (self.runtime / 'msvcp140.dll').write_bytes(pe(0x8664))
        with self.assertRaisesRegex(ValueError, 'Win32/x86'):
            self.copy()

    def test_missing_version_rejected(self):
        self.version_reader.side_effect = ValueError('MSVC file version missing')
        with self.assertRaisesRegex(ValueError, 'version missing'):
            self.copy()

    def test_copy_corruption_rejected(self):
        real_copy = shutil.copy2
        def corrupt(source, target):
            real_copy(source, target)
            target.write_bytes(target.read_bytes() + b'corrupted')
        with patch.object(receipt.shutil, 'copy2', side_effect=corrupt):
            with self.assertRaisesRegex(ValueError, 'equality failed'):
                self.copy()

    def test_source_change_during_copy_rejected(self):
        real_copy = shutil.copy2
        def change(source, target):
            source.write_bytes(source.read_bytes() + b'changed')
            real_copy(source, target)
        with patch.object(receipt.shutil, 'copy2', side_effect=change):
            with self.assertRaisesRegex(ValueError, 'equality failed'):
                self.copy()

    def test_copied_version_mismatch_rejected(self):
        self.version_reader.side_effect = lambda p: (
            {'fileVersion': '1.2.3.4', 'productVersion': '1.2.3.4'}
            if p.parent == self.app else self.versions)
        with self.assertRaisesRegex(ValueError, 'equality failed'):
            self.copy()

    @unittest.skipIf(os.name == 'nt', 'symlink fixture needs Unix permissions')
    def test_source_symlink_rejected(self):
        (self.runtime / 'msvcp140.dll').unlink()
        (self.runtime / 'msvcp140.dll').symlink_to(self.runtime / 'concrt140.dll')
        with self.assertRaisesRegex(ValueError, 'redirected'):
            self.copy()

    def test_actual_product_edition_is_retained_without_entitlement_claim(self):
        for edition in ('Community', 'Professional', 'Enterprise', 'BuildTools'):
            with self.subTest(edition=edition):
                product_id = 'Microsoft.VisualStudio.Product.' + edition
                value = self.copy(product_id=product_id, catalog={})
                self.assertEqual(value['visualStudio']['productId'], product_id)
                self.assertEqual(value['visualStudio']['catalog'], {})
                self.assertEqual(value['legalStatus'],
                                 'redistribution-terms-unresolved-review-required')
                self.assertNotIn('licensed', json.dumps(value).casefold())

    def test_bad_product_identity_rejected_before_inspection_or_copy(self):
        for product_id in (None, True, '', 'Enterprise',
                           'Microsoft.VisualStudio.Product.',
                           'Microsoft.VisualStudio.Product.Enterprise\n',
                           'Microsoft.VisualStudio.Product.' + 'E' * 65):
            with self.subTest(product_id=product_id):
                with self.assertRaisesRegex(ValueError, 'product identity'):
                    self.copy(product_id=product_id)
                self.version_reader.assert_not_called()
                self.assertEqual(list(self.app.iterdir()), [])

    def test_bad_catalog_rejected_before_inspection_or_copy(self):
        for catalog in (None, [], {'licenseKey': 'not retained'}, {'id': None},
                        {'id': True}, {'id': {}}, {'id': 'x' * 257},
                        {'id': 'é' * 129}, {'id': 'bad\x00identity'},
                        {'id': 'bad\nidentity'}):
            with self.subTest(catalog=catalog):
                with self.assertRaisesRegex(ValueError, 'catalog identity'):
                    self.copy(catalog=catalog)
                self.version_reader.assert_not_called()
                self.assertEqual(list(self.app.iterdir()), [])


class VisualStudioCatalogTransportTests(unittest.TestCase):
    @staticmethod
    def encoded(text):
        return base64.b64encode(text.encode('utf-8')).decode('ascii')

    def test_catalog_transport_preserves_exact_available_identity_text(self):
        catalog = {field: 'identity' for field in receipt.VISUAL_STUDIO_CATALOG_FIELDS}
        catalog.update(productName='Visual Studio Édition',
                       productPreReleaseMilestoneSuffix='',
                       productMilestoneIsPreRelease='False')
        encoded = self.encoded(json.dumps(catalog, ensure_ascii=False))
        self.assertEqual(receipt.parse_visual_studio_catalog_base64(encoded), catalog)
        self.assertEqual(receipt.parse_visual_studio_catalog_base64('e30='), {})

    def test_invalid_json_catalog_schema_is_closed(self):
        for text in ('null', '[]', 'true', '{',
                     '{"id":"first","id":"second"}',
                     '{"licensingIdentity":"unexpected"}',
                     '{"id":{"nested":"value"}}', '{"id":17}',
                     '{"id":false}', '{"id":"' + 'x' * 257 + '"}',
                     json.dumps({'id': 'bad\tidentity'})):
            with self.subTest(text=text[:80]):
                with self.assertRaises(ValueError):
                    receipt.parse_visual_studio_catalog_base64(self.encoded(text))

    def test_catalog_transport_is_bounded_and_strict(self):
        for raw in (None, b'e30=', '', '*not-base64*', 'e30=\n', '/w==',
                    self.encoded(' ' * 4097), 'A' * 5465):
            with self.subTest(raw=str(raw)[:80]):
                with self.assertRaises(ValueError):
                    receipt.parse_visual_studio_catalog_base64(raw)


class NativeVersionResourceTests(unittest.TestCase):
    @unittest.skipUnless(os.name == 'nt', 'native Windows version API check')
    def test_python_executable_has_numeric_fixed_versions(self):
        versions = receipt.windows_file_versions(Path(sys.executable))
        for key in ('fileVersion', 'productVersion'):
            self.assertRegex(versions[key], r'^\d+\.\d+\.\d+\.\d+$')
            self.assertNotEqual(versions[key], '0.0.0.0')


if __name__ == '__main__':
    unittest.main()

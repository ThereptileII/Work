#!/usr/bin/env python3
"""Inert packaging fixtures; native Windows version-resource checks remain required."""
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
        self.versions = {'fileVersion': '14.44.35211.0', 'productVersion': '14.44.35211.0'}
        version_patch = patch.object(receipt, 'windows_file_versions', return_value=self.versions)
        self.version_reader = version_patch.start()
        self.addCleanup(version_patch.stop)

    def copy(self):
        return receipt.copy_msvc_runtime(self.runtime, self.app, self.vs, '17.14.12345.1')

    def test_complete_sorted_receipt_is_repeatable_and_read_only(self):
        first = self.copy()
        self.assertEqual(first, self.copy())
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
            receipt.copy_msvc_runtime(self.runtime, self.app, self.root / 'other', '17.14.1')

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


class NativeVersionResourceTests(unittest.TestCase):
    @unittest.skipUnless(os.name == 'nt', 'native Windows version API check')
    def test_python_executable_has_numeric_fixed_versions(self):
        versions = receipt.windows_file_versions(Path(sys.executable))
        for key in ('fileVersion', 'productVersion'):
            self.assertRegex(versions[key], r'^\d+\.\d+\.\d+\.\d+$')
            self.assertNotEqual(versions[key], '0.0.0.0')


if __name__ == '__main__':
    unittest.main()

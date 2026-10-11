#!/usr/bin/env python3
"""Functional receipt checks with inert DLL bytes and an archive-decoder seam.

The seam implements only the external 7z listing/stdout contract using ZIP;
receipt generation, disk/hash checks and all failure decisions run unchanged.
This does not claim native 7z or Windows execution.
"""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import warnings
import zipfile

import wx_runtime_receipt as wx


class WxRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.install = self.root / 'install'
        self.install.mkdir()
        self.archive = self.root / wx.ARCHIVE_NAME
        self.lock = self.root / 'windows-wx.lock.json'
        self.members = [('lib/vc14x_dll/' + name, ('inert ' + name).encode())
                        for name in sorted(wx.REQUIRED | {'wxmsw32u_adv_vc14x.dll'})]
        self.save_archive()
        for name, data in self.members:
            if name.endswith('_adv_vc14x.dll'):
                continue  # The receipt represents installed members, not every archive member.
            (self.install / Path(name).name).write_bytes(data)
        self.decoder = patch.object(wx.subprocess, 'run', side_effect=self.decode_archive)
        self.decoder.start()
        self.addCleanup(self.decoder.stop)

    def save_archive(self):
        with zipfile.ZipFile(self.archive, 'w') as archive:
            for name, data in self.members:
                archive.writestr(name, data)
        self.archive_record = {'file': wx.ARCHIVE_NAME, 'url': wx.ARCHIVE_URL,
                               **wx._record(self.archive)}
        self.lock.write_text(json.dumps({'version': wx.VERSION, 'abi': wx.ABI,
                                        'archives': [self.archive_record]}))

    def decode_archive(self, command, **kwargs):
        self.assertEqual(command[0], '7z')
        self.assertEqual(kwargs['timeout'], 60)
        position = command.index('--')
        with zipfile.ZipFile(command[position + 1]) as archive:
            if command[1] == 'l':
                listing = ''.join('Path = ' + item.filename.replace('/', '\\') +
                                  '\r\nSize = ' + str(item.file_size) +
                                  '\r\nAttributes = A\r\n\r\n'
                                  for item in archive.infolist())
                data = listing.encode('utf-8')
            else:
                self.assertEqual(command[1:6], ['x', '-so', '-y', '-bd', '-spd'])
                data = archive.read(command[position + 2])
        return subprocess.CompletedProcess(command, 0, data, b'')

    def create(self):
        return wx.write_runtime_receipt(self.install, self.lock, self.archive)

    def verify(self):
        return wx.verify_runtime_receipt(self.install, self.lock, self.archive)

    def write_receipt(self, document):
        (self.install / wx.RECEIPT_NAME).write_text(json.dumps(document))

    def test_deterministic_subset_and_archive_member_mapping(self):
        receipt = self.create()
        self.assertEqual(receipt, self.verify())
        self.assertEqual(receipt['archive'], self.archive_record)
        self.assertEqual(len(receipt['files']), 2)
        for record in receipt['files']:
            self.assertEqual(record['member'], 'lib/vc14x_dll/' + record['path'])
            self.assertEqual(record['sha256'], hashlib.sha256(
                (self.install / record['path']).read_bytes()).hexdigest())
        original = (self.install / wx.RECEIPT_NAME).read_bytes()
        self.create()
        self.assertEqual(original, (self.install / wx.RECEIPT_NAME).read_bytes())

    def test_missing_receipt_and_installed_dll_refused(self):
        with self.assertRaises(FileNotFoundError):
            self.verify()
        self.create()
        (self.install / 'wxbase32u_vc14x.dll').unlink()
        with self.assertRaisesRegex(ValueError, 'Required installed'):
            self.verify()

    def test_tampered_dll_refused_even_with_updated_receipt_hash(self):
        receipt = self.create()
        record = receipt['files'][0]
        path = self.install / record['path']
        path.write_bytes(b'changed DLL')
        record.update(wx._record(path))
        self.write_receipt(receipt)
        with self.assertRaisesRegex(ValueError, 'differs from archive member'):
            self.verify()

    def test_tampered_archive_or_incorrect_locked_size_refused(self):
        self.create()
        self.archive.write_bytes(self.archive.read_bytes() + b'tampered')
        with self.assertRaisesRegex(ValueError, 'differs from lock'):
            self.verify()
        self.save_archive()
        lock = json.loads(self.lock.read_text())
        lock['archives'][0]['bytes'] += 1
        self.lock.write_text(json.dumps(lock))
        with self.assertRaisesRegex(ValueError, 'differs from lock'):
            self.verify()

    def test_forged_missing_duplicate_or_extra_receipt_mapping_refused(self):
        receipt = self.create()
        variants = []
        changed = copy.deepcopy(receipt)
        changed['files'][0]['member'] = 'lib/vc14x_dll/wxmsw32u_adv_vc14x.dll'
        variants.append(changed)
        changed = copy.deepcopy(receipt)
        changed['files'].pop()
        variants.append(changed)
        changed = copy.deepcopy(receipt)
        changed['files'].append(changed['files'][0])
        variants.append(changed)
        changed = copy.deepcopy(receipt)
        changed['unexpected'] = 'not verified'
        variants.append(changed)
        changed = copy.deepcopy(receipt)
        changed['schema'] = True
        variants.append(changed)
        for changed in variants:
            with self.subTest(changed=changed):
                self.write_receipt(changed)
                with self.assertRaisesRegex(ValueError, 'receipt differs'):
                    self.verify()

    def test_duplicate_json_keys_refused(self):
        self.create()
        path = self.install / wx.RECEIPT_NAME
        path.write_text(path.read_text().replace('"schema": 1', '"schema": 1, "schema": 1'))
        with self.assertRaisesRegex(ValueError, 'duplicate JSON key'):
            self.verify()

    def test_added_installed_dll_needs_receipt_mapping(self):
        self.create()
        name, data = self.members[1]
        self.assertTrue(name.endswith('_adv_vc14x.dll'))
        (self.install / Path(name).name).write_bytes(data)
        with self.assertRaisesRegex(ValueError, 'receipt differs'):
            self.verify()

    def test_duplicate_installed_basename_refused(self):
        nested = self.install / 'plugins'
        nested.mkdir()
        name = 'wxbase32u_vc14x.dll'
        (nested / name).write_bytes((self.install / name).read_bytes())
        with self.assertRaisesRegex(ValueError, 'Duplicate installed'):
            self.create()

    def test_unknown_or_debug_wx_dll_refused(self):
        for name in ('wxunknown.dll', 'wxbase32ud_vc14x.dll'):
            with self.subTest(name=name):
                path = self.install / name
                path.write_bytes(b'inert')
                with self.assertRaisesRegex(ValueError, 'Unexpected installed'):
                    self.create()
                path.unlink()

    def test_valid_wx_name_without_archive_member_refused(self):
        (self.install / 'wxmsw32u_stc_vc14x.dll').write_bytes(b'unmapped DLL')
        with self.assertRaisesRegex(ValueError, 'has no archive member'):
            self.create()

    def test_duplicate_archive_members_and_traversal_refused(self):
        original = list(self.members)
        for name in (original[0][0], 'lib/vc14x_dll/../bad.dll'):
            with self.subTest(name=name):
                self.members = original + [(name, b'inert')]
                with warnings.catch_warnings():
                    warnings.simplefilter('ignore', UserWarning)
                    self.save_archive()
                with self.assertRaises(ValueError):
                    self.create()

    def test_missing_or_duplicate_locked_release_archive_refused(self):
        for records in ([], [self.archive_record, self.archive_record]):
            with self.subTest(records=records):
                self.lock.write_text(json.dumps({'version': wx.VERSION, 'abi': wx.ABI,
                                                'archives': records}))
                with self.assertRaises(ValueError):
                    self.create()

    def test_symlink_runtime_and_receipt_refused(self):
        path = self.install / 'wxbase32u_vc14x.dll'
        original = path.read_bytes()
        path.unlink()
        target = self.root / 'other.dll'
        target.write_bytes(original)
        try:
            path.symlink_to(target)
        except OSError:
            self.skipTest('Symlinks unavailable on this host')
        with self.assertRaisesRegex(ValueError, 'Unsafe path'):
            self.create()
        path.unlink()
        path.write_bytes(original)
        receipt = self.create()
        output = self.install / wx.RECEIPT_NAME
        output.unlink()
        other = self.root / 'other.json'
        other.write_text(json.dumps(receipt))
        output.symlink_to(other)
        with self.assertRaises(ValueError):
            self.verify()

    def test_archive_tool_failure_or_truncated_member_refused(self):
        with patch.object(wx.subprocess, 'run', return_value=
                          subprocess.CompletedProcess([], 2, b'', b'failure')):
            with self.assertRaisesRegex(ValueError, 'inspection failed'):
                self.create()
        decoder = self.decode_archive

        def truncated(command, **kwargs):
            result = decoder(command, **kwargs)
            if command[1] == 'x':
                result.stdout = result.stdout[:-1]
            return result

        with patch.object(wx.subprocess, 'run', side_effect=truncated):
            with self.assertRaisesRegex(ValueError, 'differs from archive member'):
                self.create()

    def test_invalid_listing_refused(self):
        for listing in (b'Path = x\nPath = y\n', b'not a technical listing\n', b''):
            with self.subTest(listing=listing), self.assertRaises(ValueError):
                wx._member_listing(listing)


if __name__ == '__main__':
    unittest.main()

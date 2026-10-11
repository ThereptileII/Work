"""Portable malformed-layout controls for the inert fixture constructor only."""
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest import mock

import forwarder_fixtures as fixtures


def seed():
    """Small PE32 data model; never presented as a Windows-loadable image."""
    raw = bytearray(0x800)
    def put(at, value, size=4):
        struct.pack_into('<H' if size == 2 else '<I', raw, at, value)
    raw[:2] = b'MZ'
    put(0x3c, 0x80)
    raw[0x80:0x84] = b'PE\0\0'
    put(0x84, 0x14c, 2)
    put(0x86, 2, 2)
    put(0x94, 224, 2)
    put(0x96, 0x2000, 2)
    put(0x98, 0x10b, 2)
    put(0xd4, 0x200)  # SizeOfHeaders, used by the production reader.
    put(0xf4, 16)
    put(0xf8, 0x2000)
    put(0xfc, 0x300)
    for at, rva, offset, flags in ((0x178, 0x1000, 0x200, 0x60000020),
                                   (0x1a0, 0x2000, 0x400, 0x40000040)):
        put(at + 8, 0x200 if rva == 0x1000 else 0x400)
        put(at + 12, rva)
        put(at + 16, 0x200 if rva == 0x1000 else 0x400)
        put(at + 20, offset)
        put(at + 36, flags)
    # Distinct EAT slots alias the same inert local placeholder address.
    put(0x40c, 0x2160)
    raw[0x560:0x56a] = b'seed.dll\0\0'
    put(0x410, 209)
    put(0x414, 3)
    put(0x418, 3)
    put(0x41c, 0x2040)
    put(0x420, 0x2050)
    put(0x424, 0x2060)
    for i, value in enumerate((0x1000, 0x1004, 0x1000)):
        put(0x440 + i * 4, value)
    for i, (name, rva) in enumerate(((fixtures.RESERVATION, 0x2080),
                                    (b'fixture_value', 0x2110),
                                    (b'forwarded_probe', 0x2130))):
        put(0x450 + i * 4, rva)
        put(0x460 + i * 2, i, 2)
        at = 0x400 + rva - 0x2000
        raw[at:at + len(name) + 1] = name + b'\0'
    return bytes(raw)


class ConstructorTests(unittest.TestCase):
    def rejected(self, raw, match):
        with self.assertRaisesRegex(ValueError, match):
            fixtures.construct(bytes(raw), 'fixture_forward_clean_name.dll')

    def test_nine_exact_targets_and_only_two_bounded_deltas(self):
        original = seed()
        for name, target in fixtures.FORWARDERS.items():
            with self.subTest(name=name):
                changed, receipt = fixtures.construct(original, name)
                image = fixtures.Image(changed)
                probe = next(e for e in image.entries if e[0] == b'forwarded_probe')
                self.assertEqual(image.string(probe[3]), target.encode())
                self.assertEqual(len(changed), len(original))
                allowed = set(range(receipt['eatFileOffset'], receipt['eatFileOffset'] + 4))
                allowed.update(range(receipt['reservationFileOffset'],
                                     receipt['reservationFileOffset'] + receipt['reservationBytes']))
                deltas = {i for i, (a, b) in enumerate(zip(original, changed)) if a != b}
                self.assertTrue(deltas and deltas <= allowed)
                self.assertEqual(original[0x200:0x400], changed[0x200:0x400])
                self.assertNotEqual(receipt['beforeSha256'], receipt['afterSha256'])
                self.rejected(changed, 'exact unmodified')

    def test_unknown_file_refused(self):
        with self.assertRaisesRegex(ValueError, 'not allowlisted'):
            fixtures.construct(seed(), 'vendor.dll')

    def test_old_placeholder_alias_and_index_required(self):
        raw = bytearray(seed())
        struct.pack_into('<I', raw, 0x448, 0x1004)
        self.rejected(raw, 'aliases differ')
        raw = bytearray(seed())
        struct.pack_into('<H', raw, 0x462, 2)
        self.rejected(raw, 'EAT index is shared')

    def test_existing_forwarder_refused(self):
        raw = bytearray(seed())
        struct.pack_into('<I', raw, 0x444, 0x2110)
        self.rejected(raw, 'already has forwarded')

    def test_missing_reservation_or_name_terminator_refused(self):
        raw = bytearray(seed())
        raw[0x480 + len(fixtures.SLOT_PREFIX)] = ord('Y')
        self.rejected(raw, 'exact unmodified')
        raw = bytearray(seed())
        raw[0x530:0x700] = b'A' * (0x700 - 0x530)
        self.rejected(raw, 'string bounds')

    def test_export_interval_must_hold_entire_target_and_tables(self):
        raw = bytearray(seed())
        struct.pack_into('<I', raw, 0xfc, 0x90)
        self.rejected(raw, 'string bounds|outside export interval')

    def test_table_name_overlap_refused(self):
        raw = bytearray(seed())
        struct.pack_into('<I', raw, 0x420, 0x2040)
        self.rejected(raw, 'overlap|ASCII|ordinal')
        raw = bytearray(seed())
        struct.pack_into('<I', raw, 0x450, 0x2000)
        self.rejected(raw, 'ASCII|overlap')

    def test_import_directory_or_dll_name_overlap_refused(self):
        raw = bytearray(seed())
        struct.pack_into('<II', raw, 0x100, 0x2080, 40)
        self.rejected(raw, 'other data directory overlaps')
        raw = bytearray(seed())
        struct.pack_into('<I', raw, 0x40c, 0x2080)
        self.rejected(raw, 'overlaps DLL name')

    def test_raw_and_virtual_section_overlap_refused(self):
        raw = bytearray(seed())
        struct.pack_into('<I', raw, 0x1b4, 0x300)
        self.rejected(raw, 'raw sections overlap')
        raw = bytearray(seed())
        struct.pack_into('<I', raw, 0x1ac, 0x1100)
        self.rejected(raw, 'virtual sections overlap')

    def test_export_code_or_writable_owner_refused(self):
        for flags in (0x60000040, 0xc0000040):
            raw = bytearray(seed())
            struct.pack_into('<I', raw, 0x1c4, flags)
            self.rejected(raw, 'read-only non-code')

    def test_placeholder_must_point_to_code(self):
        raw = bytearray(seed())
        struct.pack_into('<I', raw, 0x440, 0x2300)
        struct.pack_into('<I', raw, 0x448, 0x2300)
        self.rejected(raw, 'fixture code')

    def test_architecture_checksum_signature_and_truncation_refused(self):
        for offset, value, size, message in ((0x84, 0x8664, 2, 'I386'),
                                            (0xd8, 1, 4, 'checksum'),
                                            (0x118, 1, 4, 'signed')):
            raw = bytearray(seed())
            struct.pack_into('<H' if size == 2 else '<I', raw, offset, value)
            self.rejected(raw, message)
        self.rejected(seed()[:0x500], 'outside bytes')

    def test_all_inputs_validate_before_write_and_failed_receipt_retained(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            for name in fixtures.FORWARDERS:
                (directory / name).write_bytes(seed())
            last = directory / list(fixtures.FORWARDERS)[-1]
            last.write_bytes(b'bad')
            receipt = directory / 'receipt.json'
            with self.assertRaises(ValueError):
                fixtures.patch_fixtures(directory, receipt)
            result = json.loads(receipt.read_text())
            self.assertEqual(result['status'], 'failed')
            self.assertIn('error', result)
            self.assertTrue(all(not item['written'] for item in result['fixtures']))
            for name in list(fixtures.FORWARDERS)[:-1]:
                self.assertEqual((directory / name).read_bytes(), seed())

    def test_fixed_files_only_and_repeat_retains_failed_receipt(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            for name in fixtures.FORWARDERS:
                (directory / name).write_bytes(seed())
            (directory / 'untouched.dll').write_bytes(b'not a fixture')
            receipt = directory / 'receipt.json'
            result = fixtures.patch_fixtures(directory, receipt)
            self.assertEqual(result['status'], 'passed')
            self.assertEqual(len(result['fixtures']), 9)
            self.assertTrue(all(item['written'] for item in result['fixtures']))
            self.assertEqual((directory / 'untouched.dll').read_bytes(), b'not a fixture')
            with self.assertRaises(ValueError):
                fixtures.patch_fixtures(directory, receipt)
            self.assertEqual(json.loads(receipt.read_text())['status'], 'failed')

    def test_partial_write_failure_is_fatal_and_receipted(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            for name in fixtures.FORWARDERS:
                (directory / name).write_bytes(seed())
            receipt = directory / 'receipt.json'
            original_write = Path.write_bytes
            refused = list(fixtures.FORWARDERS)[1]
            def fail_one(path, data):
                if path.name == refused:
                    raise OSError('injected fixture write failure')
                return original_write(path, data)
            with mock.patch.object(Path, 'write_bytes', fail_one):
                with self.assertRaisesRegex(OSError, 'injected'):
                    fixtures.patch_fixtures(directory, receipt)
            report = json.loads(receipt.read_text())
            self.assertEqual(report['status'], 'failed')
            self.assertIn('injected', report['error'])
            self.assertEqual([r['written'] for r in report['fixtures']], [True] + [False] * 8)
            self.assertEqual((directory / refused).read_bytes(), seed())


if __name__ == '__main__':
    unittest.main(verbosity=2)

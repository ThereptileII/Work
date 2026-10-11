#!/usr/bin/env python3
"""Data-only receipt tests; no NSIS, application execution or native acceptance."""
import ast
import hashlib
import json
import os
from pathlib import Path
import stat
import struct
import tempfile
import unittest
from unittest import mock
import warnings
import zipfile

import installer_package_receipt as receipt

COMMIT = 'a' * 40
VERSION = '0.4.0-beta2.4'


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def pe():
    raw = bytearray(128)
    raw[:2] = b'MZ'
    struct.pack_into('<I', raw, 60, 64)
    raw[64:68] = b'PE\0\0'
    struct.pack_into('<H', raw, 68, 0x14c)
    struct.pack_into('<H', raw, 88, 0x10b)
    return bytes(raw)


class Receipt(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.recovery = self.root / receipt.PREFIX
        self.installer = self.root / 'beta-installer'
        self.recovery.mkdir()
        self.installer.mkdir()
        self.files = {
            'app/opencpn.exe': pe(), 'app/opennav-restart.exe': b'inert restart helper',
            'app/skager-start.exe': b'inert startup launcher', 'app/plugins/inert_pi.dll': b'inert DLL bytes; never loaded',
            'app/OPENNAV_PORTABLE_PREVIEW': b'recovery marker',
            'docs/PRODUCT_BUILD.json': json.dumps({
                'version': VERSION, 'commit': COMMIT, 'test_fixtures': False,
                'build_purpose': 'INSTALLED PRODUCT', 'xnav_hardware_output_policy': 'status-only',
                'executable_sha256': sha(pe()), 'restart_helper_sha256': sha(b'inert restart helper'),
                'startup_launcher_sha256': sha(b'inert startup launcher'), 'xnav_manual_control_contract': 0,
                'commissioning_restart_protocol': 1, 'update_startup_health': 1}).encode(),
            'docs/notices.txt': b'fixture notice\r\n', 'profile/opencpn.conf': b'isolated fixture profile',
            'logs/README.txt': b'fixture log directory', 'Run-SKAGER.cmd': b'inert launcher text',
        }
        for name, raw in self.files.items():
            path = self.recovery / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)
        self.package = {
            'schema': 1, 'commit': COMMIT, 'version': VERSION, 'payloadSha256': '0' * 64,
            'supportedOpenCpn': [{'version': '5.12.4', 'arch': 'x86',
                'executableSha256': 'b' * 64, 'upstreamCommit': 'c' * 40,
                'integrationPackage': 'SKAGER-Beta2-Setup.exe', 'qualificationCommit': COMMIT,
                'qualificationRun': 'https://example.invalid/retained-fixture',
                'qualificationEvidence': 'inert fixture', 'stockSetupSha256': 'd' * 64}],
            'files': [{'path': name, 'sha256': sha(raw)} for name, raw in self.files.items()
                      if name.startswith(('app/', 'docs/')) and name != receipt.MARKER],
            'qualification': 'candidate',
        }
        self.selected_compatibility = json.loads(json.dumps(self.package['supportedOpenCpn']))
        self.payload = self.installer / 'payload.zip'
        with zipfile.ZipFile(self.payload, 'w', zipfile.ZIP_DEFLATED) as output:
            for entry in self.package['files']:
                output.writestr(entry['path'], self.files[entry['path']])
        self.package['payloadSha256'] = sha(self.payload.read_bytes())
        self.write_package()
        self.setup = self.installer / 'SKAGER-Beta2-Setup.exe'
        self.setup.write_bytes(b'inert Setup fixture; never executed')
        self.setup.with_suffix('.exe.sha256').write_text(sha(self.setup.read_bytes()) + '  ' + self.setup.name + '\r\n')
        self.original_recovery()

    def tearDown(self):
        self.temp.cleanup()

    def write_package(self):
        # Deliberately retain CRLF and key/record order: exact raw bytes matter.
        raw = (json.dumps(self.package, indent=2).replace('\n', '\r\n') + '\r\n').encode()
        (self.installer / 'package.json').write_bytes(raw)

    def original_recovery(self):
        manifest = {p.relative_to(self.recovery).as_posix(): sha(p.read_bytes())
                    for p in self.recovery.rglob('*') if p.is_file() and p.name != receipt.INVENTORY}
        (self.recovery / receipt.INVENTORY).write_text(json.dumps(manifest, indent=2) + '\n')
        self.archive = self.root / (receipt.PREFIX + '.zip')
        with zipfile.ZipFile(self.archive, 'w', zipfile.ZIP_DEFLATED) as output:
            for path in sorted(self.recovery.rglob('*')):
                if path.is_file():
                    output.write(path, receipt.PREFIX + '/' + path.relative_to(self.recovery).as_posix())
        self.checksum = self.archive.with_suffix('.zip.sha256')
        self.checksum.write_text(sha(self.archive.read_bytes()) + '  ' + self.archive.name + '\n')

    def finish(self):
        return receipt.retain_installer_package(self.recovery, self.installer, COMMIT, VERSION, self.selected_compatibility)

    def snapshot(self):
        return {p.relative_to(self.root).as_posix(): p.read_bytes() for p in self.root.rglob('*')
                if p.is_file() and not any(part.startswith('.installer-receipt-') for part in p.parts)}

    def refuses_unchanged(self):
        before = self.snapshot()
        with self.assertRaises((ValueError, OSError, zipfile.BadZipFile)):
            self.finish()
        self.assertEqual(self.snapshot(), before)
        self.assertFalse((self.recovery / receipt.SIDECAR).exists())

    def test_exact_sidecar_and_payload_unchanged_with_complete_archive_inventory(self):
        before = {name: (self.recovery / name).read_bytes() for name in self.files}
        payload = self.payload.read_bytes()
        package = (self.installer / 'package.json').read_bytes()
        result = self.finish()
        self.assertEqual((self.recovery / receipt.SIDECAR).read_bytes(), package)
        self.assertEqual(self.payload.read_bytes(), payload)
        self.assertEqual((self.installer / 'package.json').read_bytes(), package)
        for name, raw in before.items():
            self.assertEqual((self.recovery / name).read_bytes(), raw)
        manifest = json.loads((self.recovery / receipt.INVENTORY).read_bytes())
        self.assertEqual(manifest, {**{name: sha(raw) for name, raw in before.items()}, receipt.SIDECAR: sha(package)})
        with zipfile.ZipFile(self.archive) as archive:
            self.assertIsNone(archive.testzip())
            self.assertEqual(set(archive.namelist()), {receipt.PREFIX + '/' + name for name in manifest} | {receipt.PREFIX + '/' + receipt.INVENTORY})
            for name, digest in manifest.items():
                self.assertEqual(sha(archive.read(receipt.PREFIX + '/' + name)), digest)
            self.assertEqual(archive.read(receipt.PREFIX + '/' + receipt.SIDECAR), package)
        with zipfile.ZipFile(self.payload) as payload_zip:
            self.assertNotIn(receipt.SIDECAR, payload_zip.namelist())
            self.assertNotIn('docs/installer/package.json', payload_zip.namelist())
        self.assertEqual(self.checksum.read_text(), sha(self.archive.read_bytes()) + '  ' + self.archive.name + '\n')
        self.assertEqual(result['packageSha256'], sha(package))
        self.assertEqual(result['payloadSha256'], sha(payload))
        self.assertTrue(result['appDocsUnchanged'])
        self.assertFalse(result['launchAuthority'])
        with self.assertRaises(ValueError):
            self.finish()

    def test_package_identity_schema_and_hash_refusals(self):
        for key, bad in [('schema', True), ('schema', 2), ('commit', 'b' * 40),
                         ('version', '0.4.0-beta2.5'), ('payloadSha256', '0' * 64),
                         ('qualification', 'approved'), ('files', []), ('supportedOpenCpn', [])]:
            with self.subTest(key=key, bad=bad):
                old = self.package[key]
                self.package[key] = bad
                self.write_package()
                self.refuses_unchanged()
                self.package[key] = old
        self.write_package()

    def test_supported_identity_set_must_match_selected_source(self):
        self.package['supportedOpenCpn'][0]['executableSha256'] = 'f' * 64
        self.write_package()
        self.refuses_unchanged()

    def test_package_duplicate_casefold_json_and_extra_fields(self):
        path = self.installer / 'package.json'
        raw = path.read_bytes()
        for altered in [raw.replace(b'"schema": 1', b'"schema": 1,"Schema":1'),
                        raw.replace(b'"schema": 1', b'"schema": 1,"unexpected":1')]:
            path.write_bytes(altered)
            self.refuses_unchanged()

    def test_payload_list_missing_extra_alias_and_hash_mismatch(self):
        original = list(self.package['files'])
        alterations = [original[:-1], original + [original[0]],
            original + [{'path': 'app/OPENCpn.exe', 'sha256': original[0]['sha256']}],
            [{'path': 'app/opencpn.exe', 'sha256': '0' * 64}] + original[1:],
            original + [{'path': receipt.SIDECAR, 'sha256': 'a' * 64}]]
        for files in alterations:
            with self.subTest(files=files):
                self.package['files'] = files
                self.write_package()
                self.refuses_unchanged()

    def test_unsafe_record_paths_and_ancestor_case_aliases(self):
        for path in ['../outside', '/app/a', 'app/../a', 'app/a\\b', 'app/C:foo',
                     'app/NUL.dll', 'app/COM¹.dll', 'app/AUX .dll', 'app/a.', 'app/a ',
                     'app//a', 'app/a\x00b', 'app/CONIN$', 'app/COM0.dll', 'app/unicodé.txt',
                     'app/' + 'a' * 237, 'docs/installer/package.json']:
            with self.subTest(path=path):
                self.package['files'].append({'path': path, 'sha256': 'a' * 64})
                self.write_package()
                self.refuses_unchanged()
                self.package['files'].pop()
        with self.assertRaises(ValueError):
            receipt._aliases(['app/Foo/a', 'app/foo/b'])
        with self.assertRaises(ValueError):
            receipt._aliases(['app/file', 'app/file/child'])

    def test_payload_actual_extra_duplicate_and_symlink_entries(self):
        raw = self.payload.read_bytes()
        for kind in ['extra', 'duplicate', 'symlink', 'traversal']:
            with self.subTest(kind=kind):
                self.payload.write_bytes(raw)
                with warnings.catch_warnings():
                    warnings.simplefilter('ignore', UserWarning)
                    with zipfile.ZipFile(self.payload, 'a') as archive:
                        name = {'extra': 'app/extra.dll', 'duplicate': 'app/opencpn.exe',
                                'symlink': 'app/link.dll', 'traversal': 'app/../outside'}[kind]
                        info = zipfile.ZipInfo(name)
                        if kind == 'symlink':
                            info.external_attr = (stat.S_IFLNK | 0o777) << 16
                        archive.writestr(info, b'not allowed')
                self.package['payloadSha256'] = sha(self.payload.read_bytes())
                self.write_package()
                self.refuses_unchanged()

    def test_payload_corrupt_crc_refuses_even_with_new_outer_payload_hash(self):
        with zipfile.ZipFile(self.payload) as archive:
            entry = archive.getinfo('app/opencpn.exe')
        raw = bytearray(self.payload.read_bytes())
        name_length, extra_length = struct.unpack_from('<HH', raw, entry.header_offset + 26)
        offset = entry.header_offset + 30 + name_length + extra_length
        raw[offset] ^= 0x40
        self.payload.write_bytes(raw)
        self.package['payloadSha256'] = sha(raw)
        self.write_package()
        self.refuses_unchanged()

    def test_product_build_identity_and_actual_executable_refusals(self):
        path = self.recovery / 'docs/PRODUCT_BUILD.json'
        original = json.loads(path.read_bytes())
        for key, value in [('commit', 'b' * 40), ('version', '0.4.0-beta2.5'),
                           ('test_fixtures', True), ('build_purpose', 'TEST'),
                           ('executable_sha256', '0' * 64), ('restart_helper_sha256', '0' * 64),
                           ('startup_launcher_sha256', '0' * 64), ('commissioning_restart_protocol', 0),
                           ('update_startup_health', 0), ('xnav_manual_control_contract', None),
                           ('xnav_hardware_output_policy', 'unrestricted')]:
            with self.subTest(key=key):
                build = dict(original, **{key: value})
                path.write_text(json.dumps(build))
                self.synchronize_payload_and_recovery()
                self.refuses_unchanged()
        path.write_text(json.dumps(original))
        exe = self.recovery / 'app/opencpn.exe'
        for raw in [b'not PE', pe().replace(b'PE\0\0', b'XX\0\0'), pe()[:68] + b'\x64\x86' + pe()[70:]]:
            with self.subTest(raw=raw):
                exe.write_bytes(raw)
                build = dict(original, executable_sha256=sha(raw))
                path.write_text(json.dumps(build))
                self.synchronize_payload_and_recovery()
                self.refuses_unchanged()

    def synchronize_payload_and_recovery(self):
        with zipfile.ZipFile(self.payload, 'w', zipfile.ZIP_DEFLATED) as output:
            for entry in self.package['files']:
                raw = (self.recovery / entry['path']).read_bytes()
                entry['sha256'] = sha(raw)
                output.writestr(entry['path'], raw)
        self.package['payloadSha256'] = sha(self.payload.read_bytes())
        self.write_package()
        self.original_recovery()

    def test_recovery_actual_extra_missing_changed_and_manifest_corruption(self):
        (self.recovery / 'app/extra.dll').write_bytes(b'extra')
        self.refuses_unchanged()
        (self.recovery / 'app/extra.dll').unlink()
        file = self.recovery / 'docs/notices.txt'
        file.write_bytes(b'changed')
        self.refuses_unchanged()
        file.unlink()
        self.refuses_unchanged()
        file.write_bytes(self.files['docs/notices.txt'])
        (self.recovery / receipt.INVENTORY).write_text('{}')
        self.refuses_unchanged()

    def test_recovery_archive_and_checksum_mismatch(self):
        self.checksum.write_text('0' * 64 + '  ' + self.archive.name + '\n')
        self.refuses_unchanged()
        self.checksum.write_text(sha(self.archive.read_bytes()) + '  ' + self.archive.name + '\n')
        with zipfile.ZipFile(self.archive, 'a') as archive:
            archive.writestr(receipt.PREFIX + '/extra.txt', b'extra')
        self.checksum.write_text(sha(self.archive.read_bytes()) + '  ' + self.archive.name + '\n')
        self.refuses_unchanged()

    def test_plain_file_and_ancestor_link_refusals(self):
        outside = self.root / 'outside'
        outside.mkdir()
        target = self.recovery / 'docs/notices.txt'
        target.unlink()
        target.symlink_to(self.recovery / 'app/opencpn.exe')
        self.refuses_unchanged()
        target.unlink()
        os.link(self.recovery / 'app/opencpn.exe', target)
        self.refuses_unchanged()
        target.unlink()
        target.write_bytes(self.files['docs/notices.txt'])
        alias = self.root / 'linked'
        alias.symlink_to(self.recovery, target_is_directory=True)
        with self.assertRaises(ValueError):
            receipt.retain_installer_package(alias, self.installer, COMMIT, VERSION, self.selected_compatibility)
        alias.unlink()
        (self.recovery / 'empty-link').symlink_to(outside, target_is_directory=True)
        self.refuses_unchanged()

    def test_case_aliased_tree_and_preexisting_empty_installer_refuse(self):
        (self.recovery / 'APP').mkdir()
        self.refuses_unchanged()
        (self.recovery / 'APP').rmdir()
        (self.recovery / 'Installer').mkdir()
        self.refuses_unchanged()
        (self.recovery / 'Installer').rmdir()
        (self.recovery / 'installer').mkdir()
        self.refuses_unchanged()

    def test_bounded_metadata_files_archive_and_entry_count(self):
        for name, limit in [('MAX_METADATA', 16), ('MAX_FILE', 16), ('MAX_ARCHIVE', 16),
                            ('MAX_TOTAL', 16), ('MAX_FILES', 1), ('MAX_PRODUCT', 16)]:
            with self.subTest(bound=name), mock.patch.object(receipt, name, limit):
                self.refuses_unchanged()

    def test_missing_or_changed_setup_refuses(self):
        self.setup.write_bytes(b'changed Setup')
        self.refuses_unchanged()
        self.setup.unlink()
        self.refuses_unchanged()

    def test_staging_write_failure_preserves_all_original_outputs(self):
        with mock.patch.object(receipt, '_write', side_effect=OSError('injected disk full')):
            self.refuses_unchanged()

    def test_publication_failure_restores_all_prior_outputs(self):
        rename = Path.rename
        for filename in [receipt.INVENTORY, self.archive.name, self.checksum.name]:
            with self.subTest(filename=filename):
                def fail_staged(source, target):
                    if source.name == filename and source.parent.name.startswith('.installer-receipt-'):
                        raise OSError('injected publication failure')
                    return rename(source, target)
                with mock.patch.object(Path, 'rename', fail_staged):
                    self.refuses_unchanged()

    def test_late_input_mutation_refuses_staged_publication(self):
        read = receipt._read
        mutated = False
        def change(path, *args, **kwargs):
            nonlocal mutated
            value = read(path, *args, **kwargs)
            if Path(path).name == self.archive.name and Path(path).parent.name.startswith('.installer-receipt-') and not mutated:
                mutated = True
                (self.recovery / 'docs/notices.txt').write_bytes(b'late mutation')
            return value
        before_archive = self.archive.read_bytes()
        with mock.patch.object(receipt, '_read', change), self.assertRaises(ValueError):
            self.finish()
        self.assertEqual(self.archive.read_bytes(), before_archive)
        self.assertFalse((self.recovery / receipt.SIDECAR).exists())

    def test_final_invocation_is_after_nsis_and_setup_checksum(self):
        source = Path(__file__).with_name('package-alpha-installer.py').read_text()
        tree = ast.parse(source)
        calls = [(node.lineno, node) for node in ast.walk(tree) if isinstance(node, ast.Call)]
        final = [line for line, call in calls if isinstance(call.func, ast.Name) and call.func.id == 'retain_installer_package']
        self.assertEqual(len(final), 1)
        checksum_line = source[:source.index("(setup.with_suffix")].count('\n') + 1
        self.assertGreater(final[0], checksum_line)
        self.assertNotIn("'installer/package.json'", source[:source.index('commit=os.environ')])


if __name__ == '__main__':
    unittest.main()

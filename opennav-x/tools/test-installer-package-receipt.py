#!/usr/bin/env python3
"""Data-only receipt tests; no NSIS, application execution or native acceptance."""
import ast
import hashlib
import json
import os
from pathlib import Path
import stat
import struct
import sys
import tempfile
import types
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
        self.helper_source = self.root / 'source/installer/windows'
        self.helper_source.mkdir(parents=True)
        self.helpers = {name: b'\xef\xbb\xbf# inert producer fixture; never executed\r\n# ' + name.encode() + b'\r\n'
                        for name in receipt.ENGINE_HELPERS}
        for name, raw in self.helpers.items():
            (self.helper_source / name).write_bytes(raw)
        self.engine_hashes = receipt.snapshot_installer_helpers(self.helper_source, self.installer)
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
        return receipt.retain_installer_package(self.recovery, self.installer, COMMIT, VERSION, self.selected_compatibility, self.engine_hashes)

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
        setup = self.setup.read_bytes()
        result = self.finish()
        self.assertEqual((self.recovery / receipt.SIDECAR).read_bytes(), package)
        self.assertEqual(self.payload.read_bytes(), payload)
        self.assertEqual((self.installer / 'package.json').read_bytes(), package)
        self.assertEqual(self.setup.read_bytes(), setup)
        for name, raw in before.items():
            self.assertEqual((self.recovery / name).read_bytes(), raw)
        manifest = json.loads((self.recovery / receipt.INVENTORY).read_bytes())
        engines = {'installer/engine/' + name: sha(raw) for name, raw in self.helpers.items()}
        self.assertEqual(manifest, {**{name: sha(raw) for name, raw in before.items()}, receipt.SIDECAR: sha(package), **engines})
        for name, raw in self.helpers.items():
            self.assertEqual((self.recovery / 'installer/engine' / name).read_bytes(), raw)
            self.assertEqual((self.installer / 'engine' / name).read_bytes(), raw)
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
        self.assertEqual(result['engineSha256'], self.engine_hashes)
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
            receipt.retain_installer_package(alias, self.installer, COMMIT, VERSION, self.selected_compatibility, self.engine_hashes)
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

    def test_helper_snapshot_missing_empty_changed_alias_extra_and_links_refuse(self):
        engine = self.installer / 'engine'
        path = engine / 'Lifecycle.ps1'
        for kind in ['missing', 'empty', 'changed', 'alias', 'extra', 'symlink', 'hardlink']:
            with self.subTest(kind=kind):
                path.unlink()
                if kind in {'alias', 'extra'}:
                    path.write_bytes(self.helpers[path.name])
                    (engine / ('LIFECYCLE.ps1' if kind == 'alias' else 'unexpected.ps1')).write_bytes(b'extra')
                elif kind == 'symlink':
                    path.symlink_to(self.helper_source / path.name)
                elif kind == 'hardlink':
                    os.link(self.helper_source / path.name, path)
                elif kind != 'missing':
                    path.write_bytes(b'' if kind == 'empty' else b'mutated after NSIS')
                self.refuses_unchanged()
                for child in engine.iterdir():
                    child.unlink()
                for name, raw in self.helpers.items():
                    (engine / name).write_bytes(raw)

    def test_helper_snapshot_requires_original_exact_hash_set_and_bound(self):
        original = self.engine_hashes
        for bad in [{}, dict(original, unexpected='a' * 64), dict(original, **{'Lifecycle.ps1': '0' * 64})]:
            with self.subTest(hashes=bad):
                self.engine_hashes = bad
                self.refuses_unchanged()
        self.engine_hashes = original
        with mock.patch.object(receipt, 'MAX_ENGINE', 4):
            self.refuses_unchanged()

    def test_source_snapshot_refuses_missing_alias_and_preexisting_namespace(self):
        source = self.helper_source
        lifecycle = source / 'Lifecycle.ps1'
        lifecycle.unlink()
        output = self.root / 'missing-output'
        output.mkdir()
        with self.assertRaises(OSError):
            receipt.snapshot_installer_helpers(source, output)
        self.assertFalse((output / 'engine').exists())
        lifecycle.write_bytes(self.helpers[lifecycle.name])
        alias = source / 'LIFECYCLE.ps1'
        alias.write_bytes(b'alias')
        with self.assertRaises(ValueError):
            receipt.snapshot_installer_helpers(source, output)
        alias.unlink()
        (output / 'Engine').mkdir()
        with self.assertRaises(ValueError):
            receipt.snapshot_installer_helpers(source, output)

    def test_late_helper_mutation_refuses_publication(self):
        read = receipt._read
        def change(path, *args, **kwargs):
            value = read(path, *args, **kwargs)
            if Path(path).name == self.archive.name and Path(path).parent.name.startswith('.installer-receipt-'):
                (self.installer / 'engine/Lifecycle.ps1').write_bytes(b'late mutation')
            return value
        archive = self.archive.read_bytes()
        with mock.patch.object(receipt, '_read', change), self.assertRaises(ValueError):
            self.finish()
        self.assertEqual(self.archive.read_bytes(), archive)
        self.assertFalse((self.recovery / 'installer').exists())

    def run_recipe_with_inert_compiler(self, mutation=None):
        # Execute the real producer recipe with a data-only NSIS stand-in. No
        # compiler/app/script is run. Inspect its actual /D inputs at invocation.
        source_root = self.root / 'source'
        (self.helper_source / 'compatibility.json').write_text(json.dumps({
            'openNavVersion': VERSION, 'supportedOpenCpn': self.selected_compatibility}))
        (self.recovery / 'docs/SKAGER-Beta2-Release-Notes.md').write_bytes(b'fixture release notes')
        self.original_recovery()
        output = self.root / 'compiled-installer'
        program_files = self.root / 'program-files'
        compiler = program_files / 'NSIS/makensis.exe'
        compiler.parent.mkdir(parents=True)
        compiler.write_bytes(b'inert compiler path fixture; never executed')
        fake_os = types.ModuleType('os')
        fake_os.__dict__.update(vars(os))
        fake_os.name = 'nt'
        fake_os.environ = dict(os.environ, GITHUB_SHA=COMMIT, **{'ProgramFiles(x86)': str(program_files)})
        calls = []
        def compile_setup(arguments, **kwargs):
            if arguments[0] != str(compiler):
                return
            calls.append(arguments)
            defines = dict(arg[2:].split('=', 1) for arg in arguments if arg.startswith('/D'))
            for define, name in [('ENGINE', 'Lifecycle.ps1'), ('UPDATE_TRANSACTION', 'UpdateTransaction.ps1'),
                                 ('UPDATE_SUPERVISOR', 'UpdateSupervisor.ps1')]:
                self.assertEqual(Path(defines[define]), output / 'engine' / name)
                self.assertEqual(Path(defines[define]).read_bytes(), self.helpers[name])
            Path(defines['OUTPUT']).write_bytes(b'inert compiled Setup; never executed')
            if mutation:
                mutation(output)
        recipe = Path(__file__).with_name('package-alpha-installer.py')
        with mock.patch.dict(sys.modules, {'os': fake_os}), \
                mock.patch.object(sys, 'argv', [str(recipe), '--preview', str(self.recovery), '--output', str(output)]), \
                mock.patch('product_version.read_product_version', return_value=VERSION), \
                mock.patch('subprocess.run', side_effect=compile_setup), mock.patch('builtins.print'):
            exec(compile(recipe.read_bytes(), str(recipe), 'exec'), {'__file__': str(source_root / 'tools/package-alpha-installer.py')})
        self.assertEqual(len(calls), 1)
        return output

    def test_real_recipe_passes_exact_snapshots_and_retains_them_after_source_changes(self):
        def change_source(_):
            (self.helper_source / 'Lifecycle.ps1').write_bytes(b'new source after compiler input')
        output = self.run_recipe_with_inert_compiler(change_source)
        for name, raw in self.helpers.items():
            self.assertEqual((output / 'engine' / name).read_bytes(), raw)
            self.assertEqual((self.recovery / 'installer/engine' / name).read_bytes(), raw)

    def test_real_recipe_refuses_helper_mutation_after_compiler(self):
        def mutate(output):
            (output / 'engine/UpdateTransaction.ps1').write_bytes(b'mutated compiler input')
        with self.assertRaisesRegex(ValueError, 'snapshot changed'):
            self.run_recipe_with_inert_compiler(mutate)
        self.assertFalse((self.recovery / 'installer').exists())
        self.assertFalse((self.root / 'compiled-installer/SKAGER-Beta2-Setup.exe.sha256').exists())

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

#!/usr/bin/env python3
"""Disposable packaging checks; no native build, network, or product execution."""
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
from unittest import mock
import zipfile

import windows_icon

SPEC = importlib.util.spec_from_file_location('update_package', Path(__file__).with_name('package-update-launcher.py'))
package = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(package)


def pe32(path):
    content = bytearray(128)
    content[:2] = b'MZ'
    struct.pack_into('<I', content, 60, 64)
    content[64:68] = b'PE\0\0'
    struct.pack_into('<H', content, 68, 0x14c)
    struct.pack_into('<H', content, 88, 0x10b)
    path.write_bytes(content)


class PackageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='skager-source-fixture-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()

    def directory_alias(self, alias, target):
        if os.name == 'nt':
            # Junctions need no symlink privilege on the native Windows runner.
            command = Path(os.environ['SystemRoot']) / 'System32/cmd.exe'
            subprocess.run([str(command), '/d', '/c', 'mklink', '/J', str(alias), str(target)],
                           check=True, capture_output=True, timeout=10)
        else:
            alias.symlink_to(target, target_is_directory=True)

    def test_clean_source_guard_reports_bounded_paths_without_relaxation(self):
        with mock.patch.object(package, 'run', return_value=''):
            package.require_clean_sources('before')
        status = '?? upstream/OpenCPN/\n M tools/update-verifier/go.sum\n' + ('?? ' + 'x' * 300 + '\n') * 15
        with mock.patch.object(package, 'run', return_value=status) as run:
            with self.assertRaises(ValueError) as failure:
                package.require_clean_sources('before')
        detail = str(failure.exception)
        self.assertIn('upstream/OpenCPN/', detail)
        self.assertIn(' M tools/update-verifier/go.sum', detail)
        self.assertIn('additionalEntries=5', detail)
        self.assertLess(len(detail), 3500)
        self.assertNotIn('\n', detail)
        self.assertEqual(run.call_args.args[0][-2:], ['--', '.'])

    def test_upstream_notice_bytes_and_inventory_are_preserved(self):
        notice = self.root / 'LICENSE'
        notice.write_bytes(b'Original copyright\r\nOriginal license\r\n')
        inventory = package.Inventory()
        inventory.add(notice, 'dependency/LICENSE')
        archive = self.root / 'source.zip'
        result = inventory.write(archive, {'schema': 1})
        self.assertEqual(result, package.digest(archive))
        with zipfile.ZipFile(archive) as content:
            self.assertEqual(content.read('dependency/LICENSE'), notice.read_bytes())
            reference = json.loads(content.read('SOURCE_REFERENCE.json'))
            self.assertEqual(reference['files']['dependency/LICENSE']['sha256'], package.digest(notice))
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            inventory.write(archive, {})

    def test_source_mutation_and_bounds_rejected(self):
        source = self.root / 'source.go'
        source.write_bytes(b'original')
        inventory = package.Inventory()
        inventory.add(source, 'module/source.go')
        source.write_bytes(b'modified')
        with self.assertRaisesRegex(ValueError, 'changed'):
            inventory.write(self.root / 'source.zip', {})
        with mock.patch.object(package, 'MAX_SOURCE_BYTES', 3):
            with self.assertRaisesRegex(ValueError, 'budget'):
                package.Inventory().add(source, 'module/source.go')
        with mock.patch.object(package, 'MAX_FILES', 1):
            inventory = package.Inventory()
            inventory.add(source, 'one')
            with self.assertRaisesRegex(ValueError, 'budget'):
                inventory.add(source, 'two')
        for name in ('../escape', '/absolute', 'C:/drive', 'a\\b'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                package.Inventory().add(source, name)

    def test_redirected_source_refused(self):
        source = self.root / 'outside'
        source.write_text('private', encoding='utf-8')
        link = self.root / 'linked'
        try:
            link.symlink_to(source)
        except OSError:
            self.skipTest('Host cannot create disposable symlink')
        with self.assertRaisesRegex(ValueError, 'reparse'):
            package.Inventory().add(link, 'module/link')

    def test_provisioned_alias_does_not_allow_source_or_output_aliases(self):
        source = self.root / 'real-source'
        source.mkdir()
        (source / 'source.go').write_text('package fixture\n', encoding='utf-8')
        alias = self.root / 'provisioned-alias'
        self.directory_alias(alias, source)
        self.assertEqual(package.provisioned_source_root(alias, 'GOROOT'), source)
        with self.assertRaises(ValueError) as failure:
            package.Inventory().tree(alias, 'module')
        self.assertIn('Corresponding-source tree', str(failure.exception))
        self.assertIn('component=' + repr(str(alias)), str(failure.exception))
        with self.assertRaises(ValueError) as failure:
            package.prepare_outputs(alias)
        self.assertIn('Package install directory', str(failure.exception))
        self.assertIn('component=' + repr(str(alias)), str(failure.exception))
        # A link beneath a canonical root remains forbidden, including a child
        # directory that os.walk would otherwise silently omit from the bundle.
        cache = self.root / 'cache'
        cache.mkdir()
        interior = cache / 'module'
        self.directory_alias(interior, source)
        canonical = package.provisioned_source_root(cache, 'GOMODCACHE')
        with self.assertRaisesRegex(ValueError, 'reparse'):
            package.module_cache_source(interior / 'source.go', canonical)
        with self.assertRaisesRegex(ValueError, 'reparse'):
            package.Inventory().tree(canonical, 'modules')
        install = self.root / 'install'
        (install / 'opennav').mkdir(parents=True)
        output_alias = install / 'opennav/third-party'
        self.directory_alias(output_alias, source)
        with self.assertRaises(ValueError) as failure:
            package.prepare_outputs(install)
        self.assertIn('Package source-bundle output', str(failure.exception))
        self.assertIn('component=' + repr(str(output_alias)), str(failure.exception))
        self.assertEqual(list(source.iterdir()), [source / 'source.go'])

    def test_module_inputs_stay_below_canonical_cache(self):
        cache = self.root / 'cache'
        cache.mkdir()
        module = cache / 'module'
        module.mkdir()
        source = module / 'go.mod'
        source.write_text('module fixture\n', encoding='utf-8')
        self.assertEqual(package.module_cache_source(source, cache), source)
        for path, error in ((self.root / 'outside', 'outside'), (cache, 'whole'),
                            (cache / 'module/../escape', 'traversal'), (Path('relative'), 'absolute')):
            with self.subTest(path=path), self.assertRaisesRegex(ValueError, error):
                package.module_cache_source(path, cache)
        # The first download may create the cache, but no output path receives
        # the provisioned-root allowance.
        alias = self.root / 'cache-parent'
        self.directory_alias(alias, cache)
        missing = alias / 'new-cache'
        self.assertEqual(package.provisioned_source_root(missing, 'GOMODCACHE', allow_missing=True),
                         cache / 'new-cache')
        with self.assertRaisesRegex(ValueError, 'Unrecognized'):
            package.provisioned_source_root(alias, 'Package install directory')

    def test_existing_output_is_preserved(self):
        output = self.root / 'skager-start.exe'
        output.write_bytes(b'preserve')
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            package.prepare_outputs(self.root)
        self.assertEqual(output.read_bytes(), b'preserve')
        output.unlink()
        (self.root / 'opennav/third-party/updater').mkdir(parents=True)
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            package.prepare_outputs(self.root)

    def test_pe32_and_module_stream_validation(self):
        exe = self.root / 'fixture.exe'
        pe32(exe)
        package.validate_pe32(exe)
        data = bytearray(exe.read_bytes())
        struct.pack_into('<H', data, 68, 0x8664)
        exe.write_bytes(data)
        with self.assertRaisesRegex(ValueError, 'x86'):
            package.validate_pe32(exe)
        self.assertEqual(len(package.json_stream('{"Path":"a"}\n{"Path":"b"}')), 2)
        with mock.patch.object(package, 'MAX_MODULES', 1):
            with self.assertRaisesRegex(ValueError, 'oversized'):
                package.json_stream('{} {}')

    def test_native_platform_gate_precedes_side_effects(self):
        with mock.patch.object(package, 'is_native_windows', return_value=False), mock.patch.object(package, 'run') as runner:
            with self.assertRaisesRegex(ValueError, 'native Windows'):
                package.package(self.root, 'a' * 40, 'go')
            runner.assert_not_called()
        self.assertEqual(list(self.root.iterdir()), [])

    def test_source_is_complete_before_build_and_record_has_relative_bundle(self):
        self.package_fixture()

    def test_setup_go_junctions_use_only_canonical_compiler_and_source_roots(self):
        self.package_fixture(redirected_roots=True)

    def test_missing_broker_producer_recipe_is_refused(self):
        self.package_fixture(omitted_source='tools/package-update-broker.py')

    def test_missing_broker_verifier_source_is_refused(self):
        self.package_fixture(omitted_source='tools/broker_package.py')

    def test_missing_provisioner_producer_recipe_is_refused(self):
        self.package_fixture(omitted_source='tools/package-update-provisioner.py')

    def test_missing_provisioner_verifier_source_is_refused(self):
        self.package_fixture(omitted_source='tools/provisioner_package.py')

    def test_missing_baseline_handoff_producer_source_is_refused(self):
        self.package_fixture(omitted_source='tools/package-baseline-handoff.py')

    def test_missing_baseline_handoff_verifier_source_is_refused(self):
        self.package_fixture(omitted_source='tools/baseline_handoff_package.py')

    def test_missing_imported_updater_helper_is_refused(self):
        self.package_fixture(omitted_source='tools/updater_package.py')

    def test_missing_icon_source_or_recipe_is_refused(self):
        for source in package.ICON_INPUTS:
            with self.subTest(source=source):
                # Each complete packaging fixture needs its own directory.
                with tempfile.TemporaryDirectory(dir=self.root) as directory:
                    with mock.patch.object(self, 'root', Path(directory)):
                        self.package_fixture(omitted_source=source)

    def test_icon_embedding_failure_does_not_publish_launcher(self):
        self.package_fixture(icon_failure=True)

    def package_fixture(self, redirected_roots=False, omitted_source=None, icon_failure=False):
        repo = self.root / 'repo'
        module = repo / 'tools/update-verifier'
        module.mkdir(parents=True)
        tracked = ['LICENSE', 'tools/package-update-launcher.py', 'tools/package-update-broker.py',
                   'tools/broker_package.py', 'tools/package-update-provisioner.py',
                        'tools/provisioner_package.py', 'tools/baseline_handoff_package.py',
                        'tools/package-baseline-handoff.py', 'tools/updater_package.py', 'tools/update-verifier/go.mod',
                   'tools/update-verifier/go.sum', 'tools/update-verifier/cmd/skager-start/main.go', *package.ICON_INPUTS]
        if omitted_source is not None:
            tracked.remove(omitted_source)
        for name in tracked:
            target = repo / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text('exact committed source\n', encoding='utf-8')
        for name in ('resources/branding/skager.ico', 'resources/branding/provenance.json'):
            if name in tracked:
                (repo / name).write_bytes((package.ROOT / name).read_bytes())
        goroot = self.root / 'go'
        (goroot / 'src/runtime').mkdir(parents=True)
        (goroot / 'src/runtime/runtime.go').write_text('package runtime\n', encoding='utf-8')
        (goroot / 'LICENSE').write_bytes(b'Go upstream license\r\n')
        (goroot / 'VERSION').write_text(package.GO_VERSION + '\n', encoding='utf-8')
        dependency = self.root / 'module-cache/dependency'
        dependency.mkdir(parents=True)
        for name, data in [('LICENSE', 'Exact upstream license\n'), ('go.mod', 'module public.example/dependency\n'), ('go.sum', 'dependency checksum\n'), ('lib.go', 'package dependency\n')]:
            (dependency / name).write_text(data, encoding='utf-8')
        install = self.root / 'install'
        install.mkdir()
        go = goroot / 'bin/go.exe'
        go.parent.mkdir()
        go.write_bytes(b'never executed')
        selected_go = go
        reported_goroot = goroot
        reported_cache = dependency.parent
        if redirected_roots:
            reported_goroot = self.root / 'setup-go-junction'
            reported_cache = self.root / 'module-cache-junction'
            self.directory_alias(reported_goroot, goroot)
            self.directory_alias(reported_cache, dependency.parent)
            selected_go = reported_goroot / 'bin/go.exe'
        commit = 'a' * 40
        events = []

        def run(command, directory, environment=None):
            events.append(command[1:3])
            if command[0] == 'git':
                if command[1] == 'rev-parse':
                    return commit
                if command[1] == 'status':
                    return ''
                if command[1] == 'ls-files':
                    return '\0'.join(tracked) + '\0'
            self.assertEqual(Path(command[0]), go)
            if command[1] == 'env':
                return json.dumps({'GOVERSION': package.GO_VERSION, 'GOROOT': str(reported_goroot), 'GOHOSTOS': 'windows', 'GOMOD': str(module / 'go.mod'), 'GOMODCACHE': str(reported_cache)})
            self.assertEqual(environment['GOROOT'], str(goroot))
            self.assertEqual(environment['GOMODCACHE'], str(dependency.parent))
            if command[1:3] == ['mod', 'download']:
                return json.dumps({'Path': 'public.example/dependency', 'Version': 'v1.0.0', 'Dir': str(dependency),
                                   'GoMod': str(dependency / 'go.mod'), 'Sum': 'h1:source', 'GoModSum': 'h1:module'})
            if command[1:3] == ['mod', 'verify']:
                return 'all modules verified'
            if command[1] == 'list':
                return json.dumps({'Main': True, 'Path': package.MAIN_MODULE, 'Dir': str(module)}) + '\n' + json.dumps({'Path': 'public.example/dependency', 'Version': 'v1.0.0'})
            if command[1] == 'build':
                binary = Path(command[command.index('-o') + 1])
                archive = binary.parent / 'updater/updater-source.zip'
                with zipfile.ZipFile(archive) as source:
                    self.assertEqual(source.read('modules/public.example/dependency@v1.0.0/LICENSE'), (dependency / 'LICENSE').read_bytes())
                    self.assertEqual(source.read(package.GO_VERSION + '/LICENSE'), (goroot / 'LICENSE').read_bytes())
                    self.assertIn('project/tools/update-verifier/go.mod', source.namelist())
                    self.assertEqual(source.read('project/tools/package-update-broker.py'),
                                     (repo / 'tools/package-update-broker.py').read_bytes())
                    self.assertEqual(source.read('project/tools/broker_package.py'),
                                     (repo / 'tools/broker_package.py').read_bytes())
                    self.assertEqual(source.read('project/tools/updater_package.py'),
                                     (repo / 'tools/updater_package.py').read_bytes())
                    self.assertIn('provisioning/go.sum', source.namelist())
                    for name in package.ICON_INPUTS:
                        self.assertEqual(source.read('project/' + name), (repo / name).read_bytes())
                    self.assertEqual(json.loads(source.read('SOURCE_REFERENCE.json'))['build'], package.ICON_BUILD_RECIPE)
                self.assertGreaterEqual(events.count(['mod', 'verify']), 2)
                self.assertIn('-mod=readonly', command)
                self.assertIn('-trimpath', command)
                self.assertEqual(environment['CGO_ENABLED'], '0')
                self.assertEqual(environment['GOARCH'], '386')
                pe32(binary)
                return ''
            if command[1:3] == ['version', '-m']:
                return f'private-producer-path: {package.GO_VERSION}\n\tpath\t{package.MAIN_MODULE}/cmd/skager-start\n\tbuild\tCGO_ENABLED=0\n\tbuild\tGOARCH=386\n\tbuild\tGOOS=windows\n\tbuild\tvcs.revision={commit}\n\tbuild\tvcs.modified=false\n'
            self.fail('Unexpected external command in fixture')

        def embed_icon(binary, icon):
            self.assertIn(['build', '-mod=readonly'], events)
            self.assertNotIn(['version', '-m'], events)
            self.assertEqual(icon, (repo / 'resources/branding/skager.ico').read_bytes())
            if icon_failure:
                raise ValueError('Icon resource failure')
            # Changing inert fixture bytes proves the final digest is calculated
            # after resource embedding, not from the unbranded Go output.
            binary.write_bytes(binary.read_bytes() + b'fixture-icon-resources')

        with mock.patch.object(package, 'ROOT', repo), mock.patch.object(package, 'is_native_windows', return_value=True), mock.patch.object(package, 'run', side_effect=run), mock.patch.object(package, 'embed_icon', side_effect=embed_icon) as embed:
            if omitted_source is not None:
                with self.assertRaisesRegex(ValueError, 'must be committed'):
                    package.package(install, commit, str(selected_go))
                self.assertFalse(any(event[0] == 'build' for event in events))
                return
            if icon_failure:
                with self.assertRaisesRegex(ValueError, 'Icon resource failure'):
                    package.package(install, commit, str(selected_go))
                self.assertEqual(list(install.iterdir()), [])
                return
            result = package.package(install, commit, str(selected_go))
            embed.assert_called_once()
        self.assertEqual(result['sourceBundle']['archive'], 'opennav/third-party/updater/updater-source.zip')
        self.assertEqual(result['sourceBundle']['path'], 'third-party-sources/skager-updater-source.zip')
        self.assertNotIn('private-producer-path', result['buildInfo'])
        self.assertEqual(result['binary']['sha256'], package.digest(install / 'skager-start.exe'))
        self.assertTrue((install / 'skager-start.exe').read_bytes().endswith(b'fixture-icon-resources'))
        self.assertEqual(result['sourceBundle']['sha256'], package.digest(install / result['sourceBundle']['archive']))
        self.assertFalse((install / 'updater-trust.json').exists())


class CoffPreservationTests(unittest.TestCase):
    """Model the native .rsrc insertion that left Go's COFF file offset stale."""
    PE = 128
    TABLE = PE + 24 + 224
    OLD_SYMBOLS = 2560
    NEW_SYMBOLS = 3072

    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix='skager-coff-')
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / 'inert.exe'

    def fixture(self, relocated=False):
        strings = b'long_function_name\0long_data_name\0'
        symbols = b''.join(struct.pack('<8sIhHBB', struct.pack('<II', 0, name), 0,
                                       section, 0x20, 2, 0)
                           for name, section in ((4, 1), (23, 2)))
        symbols += struct.pack('<I', 4 + len(strings)) + strings
        sections = [(b'.text', b'inert code', 0x60000020),
                    (b'.data', b'preserved Go build identity', 0xc0000040),
                    (b'.reloc', b'inert relocation', 0x42000040),
                    (b'.symtab', symbols, 0x42000000)]
        if relocated:
            sections.insert(2, (b'.rsrc', b'\x7f' * 512, 0x40000040))
        content = bytearray(1024 + len(sections) * 512)
        content[:2] = b'MZ'
        struct.pack_into('<I', content, 60, self.PE)
        content[self.PE:self.PE + 4] = b'PE\0\0'
        struct.pack_into('<HHIIIHH', content, self.PE + 4, 0x14c, len(sections), 0,
                         self.OLD_SYMBOLS, 2, 224, 2)
        struct.pack_into('<H', content, self.PE + 24, 0x10b)
        for index, (name, payload, flags) in enumerate(sections):
            offset = 1024 + index * 512
            struct.pack_into('<8sIIIIIIHHI', content, self.TABLE + index * 40,
                             name, len(payload), (index + 1) * 4096, 512, offset,
                             0, 0, 0, 0, flags)
            content[offset:offset + len(payload)] = payload
        return content

    def snapshot(self):
        self.path.write_bytes(self.fixture())
        return windows_icon._snapshot_coff(self.path)

    def test_native_resource_transaction_repairs_only_the_coff_pointer(self):
        self.path.write_bytes(self.fixture())
        shifted = self.fixture(relocated=True)
        expected = bytearray(shifted)
        struct.pack_into('<I', expected, self.PE + 12, self.NEW_SYMBOLS)
        kernel = mock.Mock()
        kernel.BeginUpdateResourceW.return_value = 1
        kernel.UpdateResourceW.return_value = 1

        def commit(handle, discard):
            self.assertEqual((handle, discard), (1, False))
            self.path.write_bytes(shifted)
            return 1

        def readback(path, icon):
            self.assertEqual(path.read_bytes(), expected)

        kernel.EndUpdateResourceW.side_effect = commit
        icon = (package.ROOT / 'resources/branding/skager.ico').read_bytes()
        with mock.patch.object(windows_icon, '_native_file', side_effect=lambda p: p), \
                mock.patch.object(windows_icon.ctypes, 'WinDLL', return_value=kernel, create=True), \
                mock.patch.object(windows_icon, 'verify_icon', side_effect=readback) as verify:
            windows_icon.embed_icon(self.path, icon)
        verify.assert_called_once_with(self.path, icon)
        self.assertEqual(kernel.UpdateResourceW.call_count, 10)
        self.assertEqual(self.path.read_bytes(), expected)

    def test_unchanged_or_already_corrected_pointer_preserves_every_byte(self):
        original = self.snapshot()
        for relocated in (False, True):
            content = self.fixture(relocated)
            if relocated:
                struct.pack_into('<I', content, self.PE + 12, self.NEW_SYMBOLS)
            self.path.write_bytes(content)
            windows_icon._restore_coff_pointer(self.path, original)
            self.assertEqual(self.path.read_bytes(), content)

    def test_unexpected_native_mutations_refuse_before_any_repair(self):
        original = self.snapshot()
        symtab = self.TABLE + 4 * 40
        cases = [
            ('changed symbols', self.NEW_SYMBOLS, b'X'),
            ('changed strings', self.NEW_SYMBOLS + 40, b'X'),
            ('changed padding', self.NEW_SYMBOLS + 511, b'X'),
            ('changed count', self.PE + 16, struct.pack('<I', 3)),
            ('unexpected pointer', self.PE + 12, struct.pack('<I', 1111)),
            ('missing section', symtab, b'.missing'),
            ('duplicate section', self.TABLE + 2 * 40, b'.symtab\0'),
            ('changed size', symtab + 8, struct.pack('<I', 123)),
            ('changed referenced index', self.TABLE, b'.wrong\0\0'),
            ('changed referenced RVA', self.TABLE + 12, struct.pack('<I', 8192)),
            ('out of bounds', symtab + 20, struct.pack('<I', 999999)),
            ('overlap', symtab + 20, struct.pack('<I', 2560)),
        ]
        for name, offset, replacement in cases:
            with self.subTest(name=name):
                content = self.fixture(True)
                content[offset:offset + len(replacement)] = replacement
                self.path.write_bytes(content)
                with self.assertRaises(ValueError):
                    windows_icon._restore_coff_pointer(self.path, original)
                self.assertEqual(self.path.read_bytes(), content)

    def test_malformed_original_tables_are_rejected(self):
        cases = [(self.PE + 12, struct.pack('<I', self.OLD_SYMBOLS - 1)),
                 (self.PE + 16, struct.pack('<I', 1000000)),
                 (self.OLD_SYMBOLS + 36, struct.pack('<I', 0)),
                 (self.OLD_SYMBOLS + 36, struct.pack('<I', 1000000)),
                 (self.OLD_SYMBOLS + 12, struct.pack('<h', 99)),
                 (self.OLD_SYMBOLS + 17, b'\x01')]
        for offset, replacement in cases:
            with self.subTest(offset=offset, replacement=replacement):
                content = self.fixture()
                content[offset:offset + len(replacement)] = replacement
                self.path.write_bytes(content)
                with self.assertRaises(ValueError):
                    windows_icon._snapshot_coff(self.path)
                self.assertEqual(self.path.read_bytes(), content)

    def test_symbol_reference_to_a_reindexed_section_is_rejected(self):
        before = self.fixture()
        struct.pack_into('<h', before, self.OLD_SYMBOLS + 12, 3)
        self.path.write_bytes(before)
        original = windows_icon._snapshot_coff(self.path)
        after = self.fixture(True)
        struct.pack_into('<h', after, self.NEW_SYMBOLS + 12, 3)
        self.path.write_bytes(after)
        with self.assertRaisesRegex(ValueError, 'referenced COFF section index'):
            windows_icon._restore_coff_pointer(self.path, original)
        self.assertEqual(self.path.read_bytes(), after)

    def test_absent_coff_tables_stay_absent(self):
        before = self.fixture()
        struct.pack_into('<II', before, self.PE + 12, 0, 0)
        self.path.write_bytes(before)
        original = windows_icon._snapshot_coff(self.path)
        self.assertIsNone(original)
        windows_icon._restore_coff_pointer(self.path, original)
        self.assertEqual(self.path.read_bytes(), before)
        self.path.write_bytes(self.fixture(True))
        with self.assertRaisesRegex(ValueError, 'introduced COFF symbols'):
            windows_icon._restore_coff_pointer(self.path, original)


if __name__ == '__main__':
    unittest.main()

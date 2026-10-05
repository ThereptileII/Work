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

    def package_fixture(self, redirected_roots=False):
        repo = self.root / 'repo'
        module = repo / 'tools/update-verifier'
        module.mkdir(parents=True)
        tracked = ['LICENSE', 'tools/package-update-launcher.py', 'tools/update-verifier/go.mod',
                   'tools/update-verifier/go.sum', 'tools/update-verifier/cmd/skager-start/main.go']
        for name in tracked:
            target = repo / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text('exact committed source\n', encoding='utf-8')
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
                    self.assertIn('provisioning/go.sum', source.namelist())
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

        with mock.patch.object(package, 'ROOT', repo), mock.patch.object(package, 'is_native_windows', return_value=True), mock.patch.object(package, 'run', side_effect=run):
            result = package.package(install, commit, str(selected_go))
        self.assertEqual(result['sourceBundle']['archive'], 'opennav/third-party/updater/updater-source.zip')
        self.assertEqual(result['sourceBundle']['path'], 'third-party-sources/skager-updater-source.zip')
        self.assertNotIn('private-producer-path', result['buildInfo'])
        self.assertEqual(result['binary']['sha256'], package.digest(install / 'skager-start.exe'))
        self.assertEqual(result['sourceBundle']['sha256'], package.digest(install / result['sourceBundle']['archive']))
        self.assertFalse((install / 'updater-trust.json').exists())


if __name__ == '__main__':
    unittest.main()

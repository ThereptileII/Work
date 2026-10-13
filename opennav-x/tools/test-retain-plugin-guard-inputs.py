#!/usr/bin/env python3
"""Portable inert-file tests. These never load DLLs or authenticate test bytes."""
import importlib.util
import json
import os
import stat
import shutil
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

spec = importlib.util.spec_from_file_location('capture_plugin_inputs', Path(__file__).with_name('retain-plugin-guard-inputs.py'))
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)
COMMIT = '1' * 40


class RecordIdentityTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / 'fixture.exe'
        self.path.write_bytes(b'inert executable')

    def windows_stats(self):
        info = self.path.lstat()
        fields = ('st_dev', 'st_ino', 'st_mode', 'st_size', 'st_mtime_ns',
                  'st_ctime_ns', 'st_nlink', 'st_file_attributes')
        path = SimpleNamespace(**{n: getattr(info, n, 0) for n in fields})
        path.st_mode |= 0o111
        opened = SimpleNamespace(**vars(path))
        opened.st_mode &= ~0o111
        opened.st_ctime_ns += 1000
        return path, opened

    def test_record_distinct_stat_sources_keep_complete_baselines(self):
        path, opened = self.windows_stats()
        real_lstat = Path.lstat
        with mock.patch.object(Path, 'lstat', lambda p: path if p == self.path else real_lstat(p)), \
                mock.patch.object(capture.os, 'fstat', return_value=opened):
            record, identity = capture.record(self.root, self.path.name)
        self.assertEqual(identity, capture.stamp(path))
        self.assertEqual(record['sha256'], capture.receipt._digest(self.path))

    def test_record_common_identity_and_reparse_mismatch_refuse(self):
        for field in ('st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_nlink',
                      'st_mode', 'st_file_attributes'):
            with self.subTest(field=field):
                path, opened = self.windows_stats()
                setattr(opened, field, stat.S_IFDIR | 0o600 if field == 'st_mode'
                        else 0x400 if field == 'st_file_attributes' else getattr(opened, field) + 1)
                real_lstat = Path.lstat
                with mock.patch.object(Path, 'lstat', lambda p: path if p == self.path else real_lstat(p)), \
                        mock.patch.object(capture.os, 'fstat', return_value=opened):
                    with self.assertRaisesRegex(ValueError, 'Source changed before read'):
                        capture.record(self.root, self.path.name)

    def test_record_each_stat_source_metadata_mutation_refuses(self):
        for source in ('path', 'descriptor'):
            for field in ('st_mode', 'st_ctime_ns', 'st_file_attributes'):
                with self.subTest(source=source, field=field):
                    path, opened = self.windows_stats()
                    changed_path, changed_opened = SimpleNamespace(**vars(path)), SimpleNamespace(**vars(opened))
                    changed = changed_path if source == 'path' else changed_opened
                    setattr(changed, field, getattr(changed, field) ^ 1)
                    calls = 0
                    real_lstat = Path.lstat
                    def descriptor_stat(fd):
                        nonlocal calls
                        calls += 1
                        return opened if calls == 1 else changed_opened
                    def path_stat(p):
                        return (changed_path if calls >= 2 else path) if p == self.path else real_lstat(p)
                    with mock.patch.object(Path, 'lstat', path_stat), \
                            mock.patch.object(capture.os, 'fstat', side_effect=descriptor_stat):
                        with self.assertRaisesRegex(ValueError, 'Source changed (during|after) read'):
                            capture.record(self.root, self.path.name)


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root = self.base / 'producer'
        self.root.mkdir()
        self.output = self.base / 'captured'
        self.expected = {}
        self.write(capture.stage.PREFIX['curl'] + '/include/curl/curl.h', b'curl header')
        self.write(capture.stage.PREFIX['curl'] + '/lib/libcurl.lib', b'curl import')
        for name in ('archive.h', 'archive_entry.h'):
            self.write(capture.stage.CACHE + '/include/' + name, name.encode())
        for name in ('archive.lib', 'libarchive.lib'):
            self.write(capture.stage.CACHE + '/' + name, name.encode())
        self.write(capture.WX + '/include/wx/wx.h', b'wx header')
        self.write(capture.WX + '/lib/vc14x_dll/wxbase32u.lib', b'wx import')
        self.write(capture.WX + '/lib/vc14x_dll/mswu/wx/setup.h', b'wx setup')
        self.cache = capture.BUILD + '/CMakeCache.txt'
        self.cache_text = (
            'CMAKE_GENERATOR_PLATFORM:INTERNAL=Win32\n' +
            'LibArchive_LIBRARY:FILEPATH=' + str(self.root / capture.stage.CACHE / 'libarchive.lib') + '\n' +
            'LibArchive_INCLUDE_DIR:PATH=' + str(self.root / capture.stage.CACHE / 'include') + '\n' +
            'wxWidgets_ROOT_DIR:PATH=' + str(self.root / capture.WX) + '\n' +
            'wxWidgets_LIB_DIR:PATH=' + str(self.root / capture.WX / 'lib/vc14x_dll') + '\n' +
            'wxWidgets_CONFIGURATION:STRING=mswu\n')
        self.write(self.cache, self.cache_text.encode())
        self.write(capture.BUILD + '/include/OpenNavBuild.h', ('#define OPENNAV_BUILD_COMMIT "' + COMMIT + '"\n').encode())
        self.dependencies = {name: {'kernel32.dll'} for name in capture.runner.DLLS}
        self.dependencies['archive.dll'].add('liblzma.dll')
        self.dependencies['liblzma.dll'] = {'api-ms-win-core-file-l1-1-0.dll', 'kernel32.dll'}
        for name in self.dependencies:
            self.write(capture.INSTALL + '/' + name, name.encode())
        for kind, name in (('curl', 'libcurl.dll'), ('openssl', 'libssl-3.dll'),
                           ('openssl', 'libcrypto-3.dll'), ('zlib', 'zlib1.dll')):
            self.write(capture.stage.PREFIX[kind] + '/bin/' + name, name.encode())
        for name in [capture.stage.PREFIX['curl'] + '/include/curl/curl.h',
                     capture.stage.PREFIX['curl'] + '/lib/libcurl.lib'] + [
                capture.stage.PREFIX[kind] + '/bin/' + dll for kind, dll in (
                    ('curl', 'libcurl.dll'), ('openssl', 'libssl-3.dll'),
                    ('openssl', 'libcrypto-3.dll'), ('zlib', 'zlib1.dll'))]:
            self.expected[name] = capture.record(self.root, name)[0]

    def write(self, name, content):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
        return path

    def inspect(self, path):
        return self.dependencies[path.name.lower()]

    def run_capture(self, inspect=None):
        return capture.capture(self.root, self.output, COMMIT, self.expected,
                               inspect or self.inspect, {'testOnly': True})

    def refuse(self, pattern=None, inspect=None):
        with self.assertRaisesRegex((ValueError, OSError), pattern or '.'):
            self.run_capture(inspect)
        self.assertFalse((self.output / 'inventory.json').exists())

    def test_closed_inventory_and_runner_compatibility(self):
        self.write(capture.INSTALL + '/unused.dll', b'do not retain')
        self.write(capture.INSTALL + '/profile/private.key', b'do not retain')
        self.write(capture.WX + '/lib/vc14x_dll/private.pdb', b'do not retain')
        result = self.run_capture()
        inventory = self.output / 'inventory.json'
        data = capture.runner.retained_inputs(self.output / 'data', inventory,
                                             capture.receipt._digest(inventory), COMMIT)
        self.assertEqual(set(data), {'schema', 'candidateCommit', 'inputs', 'runtime', 'files'})
        self.assertEqual(set(data['runtime']), set(capture.runner.DLLS) | {'liblzma.dll'})
        self.assertTrue(data['inputs']['archiveImport'].endswith('/libarchive.lib'))
        self.assertNotIn('sdk/archive/lib/archive.lib', data['files'])
        self.assertFalse(any('private' in n or 'unused' in n for n in data['files']))
        capture.runner.bind_tls(data, self.expected)
        self.assertTrue(result['requiresIndependentInventoryPin'])
        self.assertFalse(result['nativeAcceptance'] or result['productAcceptance'] or result['boatAcceptance'])
        self.assertEqual(result['candidateCommit'], COMMIT)
        self.assertEqual(result['sources']['runtime/liblzma.dll'], capture.INSTALL + '/liblzma.dll')
        self.assertEqual(capture.receipt._inventory(self.output / 'data', ['sdk', 'runtime']), data['files'])
        self.assertFalse((self.output / 'data/inventory.json').exists())
        with self.assertRaisesRegex(ValueError, 'external reviewed hash'):
            capture.runner.retained_inputs(self.output / 'data', inventory, '0' * 64, COMMIT)

    def test_capture_accepts_distinct_stable_descriptor_ctime(self):
        real_fstat = os.fstat
        def descriptor_stat(fd):
            info = real_fstat(fd)
            fields = ('st_dev', 'st_ino', 'st_mode', 'st_size', 'st_mtime_ns',
                      'st_ctime_ns', 'st_nlink', 'st_file_attributes')
            opened = SimpleNamespace(**{n: getattr(info, n, 0) for n in fields})
            opened.st_mode &= ~0o111
            opened.st_ctime_ns += 1000
            return opened
        with mock.patch.object(capture.os, 'fstat', side_effect=descriptor_stat):
            self.run_capture()
        self.assertTrue((self.output / 'inventory.json').is_file())

    def test_copy_descriptor_mutation_or_replacement_refuses(self):
        for field in ('st_ctime_ns', 'st_mode', 'st_file_attributes', 'st_ino'):
            with self.subTest(field=field):
                real_open, real_fstat = Path.open, os.fstat
                copying, calls = False, 0
                def open_path(path, mode='r', *args, **kwargs):
                    nonlocal copying, calls
                    if mode == 'xb' and self.output / 'data' in path.parents:
                        copying, calls = True, 0
                    return real_open(path, mode, *args, **kwargs)
                def descriptor_stat(fd):
                    nonlocal calls
                    info = real_fstat(fd)
                    if not copying:
                        return info
                    calls += 1
                    fields = ('st_dev', 'st_ino', 'st_mode', 'st_size', 'st_mtime_ns',
                              'st_ctime_ns', 'st_nlink', 'st_file_attributes')
                    opened = SimpleNamespace(**{n: getattr(info, n, 0) for n in fields})
                    if field == 'st_ino' or calls == 2:
                        setattr(opened, field, getattr(opened, field) ^ 1)
                    return opened
                try:
                    with mock.patch.object(Path, 'open', open_path), \
                            mock.patch.object(capture.os, 'fstat', side_effect=descriptor_stat):
                        self.refuse('Source (changed during|replaced before) copy')
                finally:
                    if self.output.exists():
                        shutil.rmtree(self.output)

    def test_archive_selection_is_exact_with_both_imports_present(self):
        selected = str(self.root / capture.stage.CACHE / 'libarchive.lib')
        alternate = str(self.root / capture.stage.CACHE / 'archive.lib')
        self.assertIn(selected, self.cache_text)
        self.write(self.cache, self.cache_text.replace(selected, alternate).encode())
        result = self.run_capture()
        self.assertIn('sdk/archive/lib/archive.lib', result['sources'])
        self.assertNotIn('sdk/archive/lib/libarchive.lib', result['sources'])

    def test_missing_and_duplicate_cmake_selection_refused(self):
        for text in (self.cache_text.replace('LibArchive_LIBRARY:FILEPATH=', 'Other:FILEPATH='),
                     self.cache_text + 'LibArchive_LIBRARY:UNINITIALIZED=bad\n'):
            with self.subTest(text=text):
                self.write(self.cache, text.encode())
                self.refuse('CMake selection')
                self.output.rmdir()

    def test_wrong_platform_wx_and_archive_roots_refused(self):
        wx_root = str(self.root / capture.WX)
        wrong_wx_root = str(self.root / capture.stage.CACHE)
        archive = str(self.root / capture.stage.CACHE / 'libarchive.lib')
        header = str(self.root / capture.stage.CACHE / 'include/archive.h')
        for text in (self.cache_text.replace('=Win32', '=x64'),
                     self.cache_text.replace('=mswu', '=mswud'),
                     self.cache_text.replace('wxWidgets_ROOT_DIR:PATH=' + wx_root,
                                             'wxWidgets_ROOT_DIR:PATH=' + wrong_wx_root),
                     self.cache_text.replace(archive, header)):
            with self.subTest(text=text):
                self.assertNotEqual(text, self.cache_text)
                self.write(self.cache, text.encode())
                self.refuse()
                self.output.rmdir()

    def test_stale_compiled_header_refused(self):
        self.write(capture.BUILD + '/include/OpenNavBuild.h', ('#define OPENNAV_BUILD_COMMIT "' + '2' * 40 + '"\n').encode())
        self.refuse('source commit')

    def test_authenticated_tls_bytes_cannot_be_replaced(self):
        self.write(capture.INSTALL + '/libcurl.dll', b'replacement')
        self.refuse('authenticated producer')

    def test_extra_unauthenticated_curl_header_refused(self):
        self.write(capture.stage.PREFIX['curl'] + '/include/curl/extra.h', b'extra')
        self.refuse('curl header inventory')

    def test_missing_runtime_refused(self):
        (self.root / capture.INSTALL / 'archive.dll').unlink()
        self.refuse('runtime missing')

    def test_retained_b0_libcurl_system_ldap_import_closure(self):
        # Imports observed in the retained B0 production libcurl.dll, not a
        # claim that these inert fixture bytes are the production DLL.
        self.dependencies['libcurl.dll'] = {
            'api-ms-win-crt-convert-l1-1-0.dll', 'api-ms-win-crt-environment-l1-1-0.dll',
            'api-ms-win-crt-filesystem-l1-1-0.dll', 'api-ms-win-crt-heap-l1-1-0.dll',
            'api-ms-win-crt-math-l1-1-0.dll', 'api-ms-win-crt-multibyte-l1-1-0.dll',
            'api-ms-win-crt-runtime-l1-1-0.dll', 'api-ms-win-crt-stdio-l1-1-0.dll',
            'api-ms-win-crt-string-l1-1-0.dll', 'api-ms-win-crt-time-l1-1-0.dll',
            'api-ms-win-crt-utility-l1-1-0.dll', 'crypt32.dll', 'iphlpapi.dll',
            'kernel32.dll', 'libcrypto-3.dll', 'libssl-3.dll', 'normaliz.dll',
            'vcruntime140.dll', 'wldap32.dll', 'ws2_32.dll', 'zlib1.dll'}
        result = self.run_capture()
        self.assertIn('wldap32.dll', result['runtimeImports']['libcurl.dll'])
        inventory = json.loads((self.output / 'inventory.json').read_text())
        self.assertNotIn('wldap32.dll', inventory['runtime'])

    def test_system_ldap_import_does_not_admit_unknown_or_local_impostor(self):
        self.dependencies['libcurl.dll'].update({'wldap32.dll', 'unapproved.dll'})
        self.refuse('Unresolved production runtime import: unapproved.dll')
        self.output.rmdir()
        self.dependencies['libcurl.dll'].remove('unapproved.dll')
        self.write(capture.INSTALL + '/WLDAP32.dll', b'local impostor')
        self.refuse('System LDAP DLL shadowed')

    def test_retained_runtime_cannot_supply_system_ldap_impostor(self):
        self.run_capture()
        data_root = self.output / 'data'
        (data_root / 'runtime/wldap32.dll').write_bytes(b'retained impostor')
        manifest = self.output / 'inventory.json'
        data = json.loads(manifest.read_text())
        data['runtime']['wldap32.dll'] = 'runtime/wldap32.dll'
        data['files']['runtime/wldap32.dll'] = capture.record(data_root, 'runtime/wldap32.dll')[0]
        manifest.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'System LDAP DLL cannot be supplied'):
            capture.runner.retained_inputs(data_root, manifest,
                                           capture.receipt._digest(manifest), COMMIT)

    def test_unresolved_import_and_nonallowlisted_driver_refused(self):
        for dependency in ('missing.dll', 'private.drv'):
            self.dependencies['archive.dll'] = {dependency}
            self.write(capture.INSTALL + '/private.drv', b'driver')
            self.refuse('Unresolved')
            self.output.rmdir()

    def test_empty_import_output_refused(self):
        self.dependencies['archive.dll'] = set()
        self.refuse('dumpbin dependencies')

    def test_sdk_private_key_refused(self):
        self.write(capture.WX + '/include/wx/private.key', b'private')
        self.refuse('Unexpected SDK input type')

    def test_pinned_wx_implementation_includes_and_release_config_retained(self):
        # Real names in the locked 3.2.8 headers/Dev archives. These files are
        # SDK includes/configuration, not separately compiled source targets.
        selected = {capture.WX + '/include/wx/' + name: 'sdk/wx/include/wx/' + name
                    for name in ('arrimpl.cpp', 'listimpl.cpp', 'thrimpl.cpp')}
        selected[capture.WX + '/lib/vc14x_dll/mswu/build.cfg'] = \
            'sdk/wx/lib/vc14x_dll/mswu/build.cfg'
        contents = {name: ('pinned SDK member ' + name).encode() for name in selected}
        for name, content in contents.items():
            self.write(name, content)
        result = self.run_capture()
        data = capture.runner.retained_inputs(self.output / 'data', self.output / 'inventory.json',
                                             capture.receipt._digest(self.output / 'inventory.json'), COMMIT)
        for name, target in selected.items():
            self.assertEqual(result['sources'][target], name)
            self.assertEqual((self.output / 'data' / target).read_bytes(), contents[name])
            self.assertEqual(data['files'][target], capture.record(self.root, name)[0])

    def test_cpp_and_cfg_exceptions_cannot_escape_exact_wx_tree(self):
        for name in (
                capture.WX + '/include/wx/foreign.cpp',
                capture.WX + '/include/wx/nested/arrimpl.cpp',
                capture.WX + '/include/arrimpl.cpp',
                capture.stage.PREFIX['curl'] + '/include/curl/arrimpl.cpp',
                capture.WX + '/include/wx/build.cfg',
                capture.WX + '/lib/vc14x_dll/mswu/foreign.cfg',
                capture.WX + '/lib/vc14x_dll/mswu/wx/build.cfg',
                capture.WX + '/lib/vc14x_dll/mswu/wx/arrimpl.cpp',
                capture.stage.PREFIX['curl'] + '/include/curl/build.cfg'):
            with self.subTest(name=name):
                path = self.write(name, b'not an approved SDK member')
                self.refuse('Unexpected SDK input type')
                path.unlink()
                self.output.rmdir()

    def test_file_and_directory_case_aliases_refused(self):
        if os.name == 'nt':
            self.skipTest('Case-sensitive negative fixture is portable-host only')
        for name in (capture.WX + '/include/wx/WX.h', capture.WX + '/include/WX/other.h',
                     capture.INSTALL + '/ARCHIVE.DLL'):
            path = self.write(name, b'alias')
            self.refuse('aliased')
            path.unlink()
            if path.parent.name == 'WX':
                path.parent.rmdir()
            self.output.rmdir()

    def test_selected_symlink_refused(self):
        path = self.root / capture.INSTALL / 'archive.dll'
        path.unlink()
        try:
            path.symlink_to(self.root / capture.INSTALL / 'liblzma.dll')
        except OSError:
            self.skipTest('Symlink fixture unavailable')
        self.refuse()

    def test_selected_hardlink_refused(self):
        path = self.root / capture.INSTALL / 'archive.dll'
        path.unlink()
        os.link(self.root / capture.INSTALL / 'liblzma.dll', path)
        self.refuse('Hardlinked')

    def test_source_mutation_during_inspection_refused(self):
        calls = 0
        def mutate(path):
            nonlocal calls
            calls += 1
            if calls == len(self.dependencies) + 1:
                self.write(capture.WX + '/include/wx/wx.h', b'changed')
            return self.inspect(path)
        self.refuse('changed during capture', mutate)

    def test_same_hash_source_replacement_during_capture_refused(self):
        calls = 0
        def replace(path):
            nonlocal calls
            calls += 1
            if calls == len(self.dependencies) + 1:
                source = self.root / capture.WX / 'include/wx/wx.h'
                content = source.read_bytes()
                source.unlink()
                source.write_bytes(content)
            return self.inspect(path)
        self.refuse('changed during capture', replace)

    def test_limits_refuse_before_output_inventory(self):
        for constant, value in (('MAX_FILE_BYTES', 2), ('MAX_FILES', 2),
                                ('MAX_TOTAL_BYTES', 2), ('MAX_RUNTIME', 2), ('MAX_ENTRIES', 1)):
            with self.subTest(limit=constant), mock.patch.object(capture, constant, value):
                self.refuse('limit|oversized')
            self.output.rmdir()

    def test_output_reuse_and_inside_source_refused(self):
        self.run_capture()
        before = (self.output / 'inventory.json').read_bytes()
        with self.assertRaises(FileExistsError):
            self.run_capture()
        self.assertEqual((self.output / 'inventory.json').read_bytes(), before)
        with self.assertRaisesRegex(ValueError, 'outside producer'):
            capture.capture(self.root, self.root / 'capture', COMMIT, self.expected, self.inspect, {})

    def test_source_path_and_runtime_reparse_refused(self):
        original = capture.receipt._plain_path
        def reparse(root, name):
            if name == capture.INSTALL + '/archive.dll':
                raise ValueError('reparse point refused')
            return original(root, name)
        with mock.patch.object(capture.receipt, '_plain_path', side_effect=reparse):
            self.refuse('reparse')

    def test_archive_header_case_alias_refused(self):
        if os.name == 'nt':
            self.skipTest('Case-sensitive negative fixture is portable-host only')
        self.write(capture.stage.CACHE + '/include/ARCHIVE.H', b'alias')
        self.refuse('Case-aliased source parent')

    def test_changed_retained_copy_refused(self):
        original = capture.record
        def mutate(root, name):
            if root == self.output / 'data':
                (root / name).write_bytes(b'changed retained copy')
            return original(root, name)
        with mock.patch.object(capture, 'record', side_effect=mutate):
            self.refuse('Retained bytes differ')

    def test_final_producer_authentication_precedes_publication(self):
        def refuse():
            raise ValueError('Producer authentication changed')
        with self.assertRaisesRegex(ValueError, 'authentication changed'):
            capture.capture(self.root, self.output, COMMIT, self.expected, self.inspect, {}, refuse)
        self.assertFalse((self.output / 'inventory.json').exists())
        self.assertFalse((self.output / 'capture.json').exists())

    def test_native_host_guard_refuses_local_run(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(ValueError, 'GitHub-hosted Windows'):
                capture.runner.host_guard()

    def test_report_size_is_bounded(self):
        with mock.patch.object(capture.receipt, 'MAX_RECEIPT_BYTES', 16):
            self.refuse('report byte limit')

    def compiler_fixture(self):
        tool = self.base / 'compiler/Hostx64/x86/dumpbin.exe'
        tool.parent.mkdir(parents=True)
        tool.write_bytes(b'inert inspector fixture')
        self.write(capture.BUILD + '/CMakeFiles/3.31/CMakeCXXCompiler.cmake',
                   ('set(CMAKE_CXX_COMPILER "' + str(tool.with_name('cl.exe')) + '")\n').encode())
        return tool

    def test_real_inspector_selection_and_dependents_parser(self):
        tool = self.compiler_fixture()
        inspector, metadata = capture.dumpbin_inspector(self.root)
        stdout = b'  KERNEL32.dll\r\n  api-ms-win-core-file-l1-1-0.dll\r\n  not-an-import\r\n'
        with mock.patch.object(capture.native, 'win32') as pe, \
                mock.patch.object(capture.subprocess, 'run', return_value=SimpleNamespace(stdout=stdout, stderr=b'')) as run:
            selected = self.root / capture.INSTALL / 'archive.dll'
            self.assertEqual(inspector(selected), {'kernel32.dll', 'api-ms-win-core-file-l1-1-0.dll'})
            pe.assert_called_once_with(selected)
            self.assertEqual(run.call_args.args[0], [str(tool), '/DEPENDENTS', str(selected)])
            self.assertEqual(run.call_args.kwargs['timeout'], 30)
        self.assertEqual(metadata['path'], str(tool))
        self.assertEqual(metadata['sha256'], capture.receipt._digest(tool))

    def test_inspector_tool_mutation_refused(self):
        tool = self.compiler_fixture()
        inspector, _ = capture.dumpbin_inspector(self.root)
        def inspect_only(*args, **kwargs):
            tool.write_bytes(b'changed inspector')
            return SimpleNamespace(stdout=b'  kernel32.dll\n', stderr=b'')
        with mock.patch.object(capture.native, 'win32'), mock.patch.object(capture.subprocess, 'run', side_effect=inspect_only):
            with self.assertRaisesRegex(ValueError, 'dumpbin changed'):
                inspector(self.root / capture.INSTALL / 'archive.dll')

    def test_ambiguous_or_non_x86_compiler_refused(self):
        tool = self.compiler_fixture()
        path = self.root / capture.BUILD / 'CMakeFiles/3.31/CMakeCXXCompiler.cmake'
        selected = str(tool.with_name('cl.exe'))
        wrong = str(tool.parent.parent / 'x64/cl.exe')
        self.assertIn(selected, path.read_text())
        path.write_text(path.read_text().replace(selected, wrong))
        with self.assertRaisesRegex(ValueError, 'MSVC x86'):
            capture.dumpbin_inspector(self.root)
        self.write(capture.BUILD + '/CMakeFiles/3.30/CMakeCXXCompiler.cmake', b'old compiler')
        with self.assertRaisesRegex(ValueError, 'ambiguous'):
            capture.dumpbin_inspector(self.root)

    def test_cli_rechecks_authenticated_dependency_authority(self):
        authority = {'mode': 'cross-run-bundle', 'producer': 'fixture-only'}
        argv = ['capture', '--root', str(self.root), '--producer-commit', COMMIT,
                '--producer-run-id', '123', '--producer-run-attempt', '1', '--architecture', 'Win32',
                '--dependency-bundle', str(self.base / 'bundle'),
                '--dependency-bundle-provenance', str(self.base / 'external-provenance.json'),
                '--output', str(self.output)]
        with mock.patch.object(capture.sys, 'argv', argv), mock.patch.object(capture, 'host_guard'), \
                mock.patch.object(capture, 'dumpbin_inspector', return_value=(self.inspect, {})), \
                mock.patch.object(capture.native, 'dependency_inputs', side_effect=[
                    (self.expected, authority), (self.expected, {'mode': 'changed'})]) as authenticate:
            with self.assertRaisesRegex(ValueError, 'Dependency authority changed'):
                capture.main()
            self.assertEqual(authenticate.call_count, 2)
            self.assertEqual(authenticate.call_args.args,
                             (self.root, self.base / 'bundle', self.base / 'external-provenance.json'))
        self.assertFalse((self.output / 'inventory.json').exists())

    def test_host_commit_run_attempt_and_job_are_mandatory(self):
        environment = {'GITHUB_SHA': COMMIT, 'GITHUB_RUN_ID': '123', 'GITHUB_RUN_ATTEMPT': '1',
                       'GITHUB_JOB': 'windows-integration', 'GITHUB_WORKSPACE': str(self.root.parent)}
        with mock.patch.object(capture.runner, 'host_guard'), mock.patch.object(capture, 'ROOT', self.root), \
                mock.patch.dict(os.environ, environment, clear=True), \
                mock.patch.object(capture.subprocess, 'check_output', return_value=COMMIT + '\n'), \
                mock.patch.object(capture.subprocess, 'run'):
            # Source root basename is fixed by the actual CI contract.
            producer = self.root.parent / 'opennav-x'
            self.root.rename(producer)
            with mock.patch.object(capture, 'ROOT', producer):
                capture.host_guard(producer, COMMIT, '123', '1')
                for key, wrong in (('GITHUB_SHA', '2' * 40), ('GITHUB_RUN_ID', '124'),
                                   ('GITHUB_RUN_ATTEMPT', '2'), ('GITHUB_JOB', 'other')):
                    with self.subTest(key=key), mock.patch.dict(os.environ, {key: wrong}):
                        with self.assertRaisesRegex(ValueError, 'exact native producer'):
                            capture.host_guard(producer, COMMIT, '123', '1')


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3
"""Portable fake-runner producer checks; no compiler, network or broker execution."""
import base64
import importlib.util
import json
import os
from pathlib import Path
import struct
import tempfile
import unittest
from unittest import mock
import zipfile

SPEC = importlib.util.spec_from_file_location('broker_producer', Path(__file__).with_name('package-update-broker.py'))
producer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(producer)
launcher = producer.launcher


def pe32(path):
    content = bytearray(512)
    content[:2] = b'MZ'
    struct.pack_into('<I', content, 60, 64)
    content[64:68] = b'PE\0\0'
    struct.pack_into('<H', content, 68, 0x14c)
    struct.pack_into('<H', content, 84, 224)
    struct.pack_into('<H', content, 88, 0x10b)
    struct.pack_into('<H', content, 156, 2)
    path.write_bytes(content)


class BrokerProducerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='broker-producer-fixture-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.repo = self.root / 'repo'
        self.module = self.repo / 'tools/update-verifier'
        self.tracked = ['LICENSE', 'tools/package-update-launcher.py', 'tools/package-update-broker.py',
                        'tools/broker_package.py', 'tools/package-update-provisioner.py',
                        'tools/provisioner_package.py', 'tools/updater_package.py',
                        'tools/update-verifier/go.mod', 'tools/update-verifier/go.sum',
                        'tools/update-verifier/cmd/skager-update-broker/main.go']
        for name in self.tracked:
            path = self.repo / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(('committed fixture: ' + name + '\n').encode())
        self.goroot = self.root / 'go'
        (self.goroot / 'bin').mkdir(parents=True)
        self.go = self.goroot / 'bin/go.exe'
        self.go.write_bytes(b'never executed')
        (self.goroot / 'src/runtime').mkdir(parents=True)
        (self.goroot / 'src/runtime/runtime.go').write_text('package runtime\n')
        (self.goroot / 'VERSION').write_text(launcher.GO_VERSION + '\n')
        (self.goroot / 'LICENSE').write_bytes(b'upstream Go license\r\n')
        self.cache = self.root / 'module-cache'
        self.dependency = self.cache / 'dependency'
        self.dependency.mkdir(parents=True)
        for name in ('LICENSE', 'go.mod', 'lib.go'):
            (self.dependency / name).write_text('exact dependency ' + name + '\n')
        checksum = 'h1:' + base64.b64encode(b'a' * 32).decode()
        self.reference = {'schema': 1, 'productCommit': 'a' * 40, 'goVersion': launcher.GO_VERSION,
                          'mainModule': 'tools/update-verifier', 'modules': [
                              {'path': 'public.example/dependency', 'version': 'v1.0.0',
                               'sum': checksum, 'goModSum': checksum,
                               'sourcePrefix': 'modules/public.example/dependency@v1.0.0'}],
                          'build': 'CGO_ENABLED=0 GOOS=windows GOARCH=386 GO386=sse2 go build '
                                   '-mod=readonly -trimpath -buildvcs=true -ldflags=-H=windowsgui ./cmd/skager-start'}
        self.commit = self.reference['productCommit']
        self.modules = [{'Main': True, 'Path': launcher.MAIN_MODULE, 'Dir': str(self.module)},
                        {'Path': 'public.example/dependency', 'Version': 'v1.0.0', 'Sum': checksum,
                         'GoModSum': checksum, 'Dir': str(self.dependency), 'GoMod': str(self.dependency / 'go.mod')}]
        # Actual go list from a committed module can omit unused transitive
        # module sums and cache locations. The retained full provisioning sum
        # resolves them only in the separate offline audit module.
        self.actual_modules = [dict(self.modules[0]), {'Path': 'public.example/dependency', 'Version': 'v1.0.0'}]
        self.audit_directory = None
        self.install = self.root / 'broker-install'
        self.install.mkdir()
        self.archive = self.install / producer.updater.SOURCE_RELATIVE
        self.archive.parent.mkdir(parents=True)
        self.events = []
        self.dirty = False
        self.on_build = None
        self.info = self.metadata()
        with mock.patch.object(producer, 'ROOT', self.repo), mock.patch.object(launcher, 'run', side_effect=self.fake_run):
            inventory = producer.source_inventory(self.module, self.goroot, self.cache, self.modules, self.reference)
        provision = self.root / 'provisioning-go.sum'
        self.provision_sum = b'locked provisioning checksum\n'
        provision.write_bytes(self.provision_sum)
        inventory.add(provision, 'provisioning/go.sum')
        inventory.write(self.archive, self.reference)
        pe32(self.install / 'skager-start.exe')
        self.launcher_record = {'schema': 1, 'productCommit': self.commit, 'goVersion': launcher.GO_VERSION,
                                'binary': {'path': 'skager-start.exe', 'sha256': launcher.digest(self.install / 'skager-start.exe')},
                                'sourceBundle': {'archive': producer.updater.SOURCE_RELATIVE,
                                                 'path': producer.updater.BUNDLE_PATH, 'sha256': launcher.digest(self.archive),
                                                 'reference': self.reference},
                                'buildInfo': producer.build_info(self.metadata(), self.commit).replace(producer.BINARY, 'skager-start.exe').replace('/cmd/skager-update-broker', '/cmd/skager-start')}
        self.record_path = self.archive.parent / 'build.json'
        self.record_path.write_text(json.dumps(self.launcher_record))
        self.events.clear()

    def metadata(self):
        settings = {'-buildmode': 'exe', '-compiler': 'gc', '-trimpath': 'true',
                    'CGO_ENABLED': '0', 'GOARCH': '386', 'GOOS': 'windows', 'GO386': 'sse2',
                    'vcs': 'git', 'vcs.revision': self.commit, 'vcs.modified': 'false', 'vcs.time': '2026-10-12T12:00:00Z'}
        return ('private-path/' + producer.BINARY + ': ' + launcher.GO_VERSION + '\n\tpath\t' + launcher.MAIN_MODULE +
                '/cmd/skager-update-broker\n\tmod\t' + launcher.MAIN_MODULE + '\t(devel)\t\n' +
                ''.join('\tbuild\t' + key + '=' + value + '\n' for key, value in settings.items()))

    def fake_run(self, command, directory, environment=None):
        self.events.append(command)
        if command[0] == 'git':
            if command[1] == 'rev-parse':
                return self.commit
            if command[1] == 'status':
                return ' M tools/update-verifier/go.sum\n' if self.dirty else ''
            if command[1] == 'ls-files':
                return '\0'.join(self.tracked) + '\0'
        self.assertEqual(Path(command[0]), self.go)
        self.assertEqual(environment['CGO_ENABLED'], '0')
        self.assertEqual(environment['GOARCH'], '386')
        self.assertEqual(environment['GO386'], 'sse2')
        self.assertEqual(environment['GOTOOLCHAIN'], 'local')
        self.assertEqual(environment['GOFLAGS'], '-mod=readonly')
        if command[1] == 'env':
            return json.dumps({'GOVERSION': launcher.GO_VERSION, 'GOROOT': str(self.goroot), 'GOHOSTOS': 'windows',
                               'GOMOD': str(self.module / 'go.mod'), 'GOMODCACHE': str(self.cache)})
        self.assertEqual(environment['GOROOT'], str(self.goroot))
        self.assertEqual(environment['GOMODCACHE'], str(self.cache))
        self.assertEqual(environment['GOPROXY'], 'off')
        self.assertEqual(environment['GOSUMDB'], 'off')
        if command[1:3] == ['mod', 'verify']:
            self.assertEqual(Path(directory).name, 'source-audit')
            return 'all modules verified'
        if command[1] == 'list':
            if Path(directory) == self.module:
                return '\n'.join(json.dumps(item) for item in self.actual_modules)
            self.audit_directory = Path(directory)
            self.assertEqual(self.audit_directory.name, 'source-audit')
            self.assertEqual((self.audit_directory / 'go.mod').read_bytes(), (self.module / 'go.mod').read_bytes())
            self.assertEqual((self.audit_directory / 'go.sum').read_bytes(), self.provision_sum)
            audited = [dict(item, Dir=str(directory)) if item.get('Main') else item for item in self.modules]
            return '\n'.join(json.dumps(item) for item in audited)
        if command[1] == 'build':
            self.assertEqual(Path(directory), self.module)
            self.assertEqual(command[-1], './cmd/skager-update-broker')
            for flag in ('-mod=readonly', '-trimpath', '-buildvcs=true', '-ldflags=-H=windowsgui'):
                self.assertIn(flag, command)
            pe32(Path(command[command.index('-o') + 1]))
            if self.on_build:
                self.on_build()
            return ''
        if command[1:3] == ['version', '-m']:
            return self.info
        self.fail('Unexpected command: ' + repr(command))

    def package(self):
        with mock.patch.object(producer, 'ROOT', self.repo), mock.patch.object(launcher, 'ROOT', self.repo), \
                mock.patch.object(launcher, 'is_native_windows', return_value=True), \
                mock.patch.object(launcher, 'run', side_effect=self.fake_run), \
                mock.patch.object(launcher.Inventory, 'write', side_effect=AssertionError('No duplicate source archive')):
            return producer.package(self.install, self.commit, str(self.go))

    def test_native_gate_has_no_output_or_runner_side_effects(self):
        with mock.patch.object(launcher, 'is_native_windows', return_value=False), mock.patch.object(launcher, 'run') as run:
            with self.assertRaisesRegex(ValueError, 'native Windows'):
                producer.package(self.install, self.commit, str(self.go))
            run.assert_not_called()
        self.assertFalse((self.install / producer.BINARY).exists())

    def test_build_reuses_complete_source_and_exclusively_publishes_fixed_outputs(self):
        before = {path: path.read_bytes() for path in (self.archive, self.record_path, self.install / 'skager-start.exe')}
        result = self.package()
        self.assertEqual(result['sourceBundle'], self.launcher_record['sourceBundle'])
        self.assertEqual(result['build'], producer.BUILD_RECIPE)
        self.assertEqual(result['binary'], {'path': producer.BINARY, 'sha256': launcher.digest(self.install / producer.BINARY)})
        self.assertEqual(json.loads((self.install / producer.RECORD_RELATIVE).read_text()), result)
        self.assertNotIn('private-path', result['buildInfo'])
        self.assertEqual(sum(command[1:3] == ['mod', 'verify'] for command in self.events), 2)
        self.assertFalse(any(command[1:3] == ['mod', 'download'] for command in self.events))
        self.assertEqual(before, {path: path.read_bytes() for path in before})
        self.assertFalse((self.install / 'update-trust.json').exists())
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            self.package()

    def test_missing_optional_committed_graph_metadata_uses_bound_offline_audit(self):
        self.assertNotIn('Sum', self.actual_modules[1])
        self.assertNotIn('GoModSum', self.actual_modules[1])
        self.assertNotIn('Dir', self.actual_modules[1])
        committed_sum = (self.module / 'go.sum').read_bytes()
        result = self.package()
        self.assertEqual(result['sourceBundle'], self.launcher_record['sourceBundle'])
        self.assertEqual((self.module / 'go.sum').read_bytes(), committed_sum)
        self.assertIsNotNone(self.audit_directory)
        self.assertFalse(self.audit_directory.exists())

    def test_private_audit_graph_version_drift_refuses(self):
        self.actual_modules[1]['Version'] = 'v2.0.0'
        with self.assertRaisesRegex(ValueError, 'audit graph differs'):
            self.package()
        self.assertFalse(any(command[1] == 'build' for command in self.events))

    def test_private_audit_input_mutation_during_build_refuses(self):
        self.on_build = lambda: (self.audit_directory / 'go.sum').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'audit inputs changed'):
            self.package()
        self.assertFalse((self.install / producer.BINARY).exists())

    def test_missing_audited_cache_location_refuses_without_fallback(self):
        del self.modules[1]['Dir']
        with self.assertRaisesRegex(ValueError, 'Audited broker cache locations unavailable'):
            self.package()
        self.assertFalse(any(command[1:3] == ['mod', 'download'] for command in self.events))

    def test_audit_checksum_absence_refuses_without_fallback(self):
        del self.modules[1]['GoModSum']
        with self.assertRaisesRegex(ValueError, 'module graph differs'):
            self.package()
        self.assertFalse(any(command[1] == 'build' for command in self.events))

    def test_current_module_must_match_retained_audit_module(self):
        (self.module / 'go.mod').write_bytes(b'changed module')
        with self.assertRaisesRegex(ValueError, 'audit module differs'):
            self.package()
        self.assertFalse(any(command[1:3] == ['mod', 'verify'] for command in self.events))

    def test_retained_provisioning_checksum_tamper_refuses_before_audit(self):
        with zipfile.ZipFile(self.archive) as source:
            contents = {name: source.read(name) for name in source.namelist()}
        value = contents['provisioning/go.sum']
        contents['provisioning/go.sum'] = bytes([value[0] ^ 1]) + value[1:]
        with zipfile.ZipFile(self.archive, 'w') as source:
            for name, data in contents.items():
                source.writestr(name, data)
        self.launcher_record['sourceBundle']['sha256'] = launcher.digest(self.archive)
        self.record_path.write_text(json.dumps(self.launcher_record))
        with self.assertRaisesRegex(ValueError, 'offline audit input hash differs'):
            self.package()
        self.assertFalse(any(command[1:3] == ['mod', 'verify'] for command in self.events))

    def test_default_trust_refuses_before_build(self):
        (self.install / 'update-trust.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, 'trust configuration'):
            self.package()
        self.assertFalse(any(command[1] == 'build' for command in self.events))

    def test_changed_checkout_source_refuses_before_build(self):
        (self.repo / 'tools/update-verifier/cmd/skager-update-broker/main.go').write_text('changed')
        with self.assertRaisesRegex(ValueError, 'Actual broker build input differs'):
            self.package()
        self.assertFalse(any(command[1] == 'build' for command in self.events))

    def test_missing_build_script_refuses(self):
        self.tracked.remove('tools/broker_package.py')
        with self.assertRaisesRegex(ValueError, 'complete broker source'):
            self.package()

    def test_missing_provisioner_scripts_refuse_incomplete_source_closure(self):
        for name in ('tools/package-update-provisioner.py', 'tools/provisioner_package.py'):
            with self.subTest(name=name):
                self.tracked.remove(name)
                with self.assertRaisesRegex(ValueError, 'complete broker source'):
                    self.package()
                self.tracked.append(name)

    def test_missing_imported_updater_helper_refuses(self):
        self.tracked.remove('tools/updater_package.py')
        with self.assertRaisesRegex(ValueError, 'complete broker source'):
            self.package()

    def test_trimpath_metadata_omits_linker_flags_and_gui_is_observed(self):
        self.assertNotIn('-ldflags=', self.info)
        result = self.package()
        self.assertNotIn('-ldflags=', result['buildInfo'])
        self.assertIn('-ldflags=-H=windowsgui', result['build'])
        producer.validate_broker_pe(self.install / producer.BINARY)

    def test_console_pe_refuses_publication_without_linker_metadata(self):
        def console_binary():
            command = next(command for command in reversed(self.events) if command[1] == 'build')
            binary = Path(command[command.index('-o') + 1])
            data = bytearray(binary.read_bytes())
            struct.pack_into('<H', data, 156, 3)
            binary.write_bytes(data)
        self.on_build = console_binary
        with self.assertRaisesRegex(ValueError, 'GUI PE subsystem'):
            self.package()
        self.assertFalse((self.install / producer.BINARY).exists())

    def test_unexpected_linker_metadata_refuses(self):
        for flags in ('-H=windows', '-H=windowsgui'):
            with self.subTest(flags=flags):
                self.info = self.metadata() + '\tbuild\t-ldflags=' + flags + '\n'
                with self.assertRaisesRegex(ValueError, 'Actual broker Go build identity'):
                    self.package()

    def test_truncated_or_short_optional_pe_header_refuses(self):
        binary = self.root / 'malformed.exe'
        pe32(binary)
        binary.write_bytes(binary.read_bytes()[:150])
        with self.assertRaisesRegex(ValueError, 'Truncated broker PE optional header'):
            producer.validate_broker_pe(binary)
        pe32(binary)
        data = bytearray(binary.read_bytes())
        struct.pack_into('<H', data, 84, 26)
        binary.write_bytes(data)
        with self.assertRaisesRegex(ValueError, 'optional header size'):
            producer.validate_broker_pe(binary)

    def test_changed_module_graph_refuses(self):
        self.modules[1]['Sum'] = 'h1:' + base64.b64encode(b'b' * 32).decode()
        with self.assertRaisesRegex(ValueError, 'module graph differs'):
            self.package()

    def test_module_cache_escape_refuses(self):
        self.modules[1]['Dir'] = str(self.repo)
        with self.assertRaisesRegex(ValueError, 'outside'):
            self.package()

    def test_dirty_checkout_refuses_before_compiler(self):
        self.dirty = True
        with self.assertRaisesRegex(ValueError, 'committed source'):
            self.package()
        self.assertFalse(any(command[0] != 'git' for command in self.events))

    def test_unexpected_archive_entry_refuses_even_with_updated_outer_hash(self):
        with zipfile.ZipFile(self.archive, 'a') as source:
            source.writestr('unexpected', b'unexpected')
        self.launcher_record['sourceBundle']['sha256'] = launcher.digest(self.archive)
        self.record_path.write_text(json.dumps(self.launcher_record))
        with self.assertRaisesRegex(ValueError, 'source file declarations differ'):
            self.package()

    def test_archive_member_hash_refuses_self_consistent_outer_hash(self):
        with zipfile.ZipFile(self.archive) as source:
            contents = {name: source.read(name) for name in source.namelist()}
        contents['project/tools/broker_package.py'] = b'changed archived source'
        with zipfile.ZipFile(self.archive, 'w') as source:
            for name, data in contents.items():
                source.writestr(name, data)
        self.launcher_record['sourceBundle']['sha256'] = launcher.digest(self.archive)
        self.record_path.write_text(json.dumps(self.launcher_record))
        with self.assertRaisesRegex(ValueError, 'source size differs|member hash differs'):
            self.package()

    def test_selected_compiler_outside_goroot_refuses(self):
        outside = self.root / 'different-go.exe'
        outside.write_bytes(b'never executed')
        original = launcher.provisioned_source_root
        def provision(path, role, **kwargs):
            if role == 'Go compiler':
                return outside
            return original(path, role, **kwargs)
        # The fake env response is still from a Windows Go 1.27.1; identity
        # must also bind the canonical selected compiler to that GOROOT.
        with mock.patch.object(launcher, 'provisioned_source_root', side_effect=provision), \
                mock.patch.object(self, 'go', outside), self.assertRaisesRegex(ValueError, 'outside pinned GOROOT'):
            self.package()

    def test_replaced_module_refuses(self):
        self.modules[1]['Replace'] = {'Dir': 'outside'}
        with self.assertRaisesRegex(ValueError, 'Replaced'):
            self.package()

    def test_mutating_compiler_source_during_build_refuses_publication(self):
        self.on_build = lambda: (self.goroot / 'src/runtime/runtime.go').write_text('changed')
        with self.assertRaisesRegex(ValueError, 'Actual broker build input differs'):
            self.package()
        self.assertFalse((self.install / producer.BINARY).exists())

    def test_dirty_after_build_refuses_publication(self):
        self.on_build = lambda: setattr(self, 'dirty', True)
        with self.assertRaisesRegex(ValueError, 'committed source'):
            self.package()

    def test_actual_metadata_mismatch_refuses_publication(self):
        self.info = self.info.replace('GOARCH=386', 'GOARCH=amd64')
        with self.assertRaisesRegex(ValueError, 'Actual broker Go build identity'):
            self.package()
        self.assertFalse((self.install / producer.BINARY).exists())

    def test_duplicate_actual_metadata_is_rejected(self):
        self.info += '\tbuild\tGOARCH=386\n'
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            self.package()

    def test_racing_output_is_preserved(self):
        binary = self.install / producer.BINARY
        self.on_build = lambda: binary.write_bytes(b'preserve racing output')
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            self.package()
        self.assertEqual(binary.read_bytes(), b'preserve racing output')

    def test_partial_publication_is_retained_and_refuses_retry(self):
        link = os.link
        calls = []
        def publish(source, destination):
            calls.append(Path(destination))
            if len(calls) == 2:
                raise OSError('inert exclusive publication fault')
            link(source, destination)
        with mock.patch.object(producer.os, 'link', side_effect=publish), self.assertRaises(OSError):
            self.package()
        self.assertTrue((self.install / producer.RECORD_RELATIVE).is_file())
        self.assertFalse((self.install / producer.BINARY).exists())
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            self.package()


if __name__ == '__main__':
    unittest.main()

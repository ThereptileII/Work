#!/usr/bin/env python3
"""Strict copied provisioner checks using inert PE/source fixtures only."""
import copy
import importlib.util
import json
from pathlib import Path
import struct
import unittest
from unittest import mock
import zipfile

import broker_package as broker
import provisioner_package as provisioner
import updater_package as updater

spec = importlib.util.spec_from_file_location('provisioner_fixture_support',
                                              Path(__file__).with_name('test-package-update-provisioner.py'))
fixtures = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixtures)


class ProvisionerPackageTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.ProvisionerProducerTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.fixture.package()
        f = self.fixture.fixture
        self.app, self.commit = f.install, f.commit
        self.binary = self.app / provisioner.BINARY
        self.record_path = self.app / provisioner.RECORD_RELATIVE
        self.record = json.loads(self.record_path.read_text())
        self.original_binary = self.binary.read_bytes()
        self.module = broker._embedded_info(self.binary, provisioner.PROVISIONER)[1]

    def record_write(self):
        self.record_path.write_text(json.dumps(self.record))

    def binary_write(self, module=None, version=updater.GO_VERSION):
        module = self.module if module is None else module
        data = bytearray(fixtures.pe_fixtures.pe(module, version))
        struct.pack_into('<H', data, 156, 3)
        self.binary.write_bytes(data)
        self.record['binary']['sha256'] = updater._hash(self.binary)
        self.record['buildInfo'] = provisioner.BINARY + ': ' + version + '\n' + ''.join(
            '\t' + line + '\n' for line in module.splitlines())
        self.record_write()

    def verify(self):
        return provisioner.verify_provisioner_package(self.app, self.commit)

    def test_exact_console_descriptor_and_no_side_effects(self):
        before = {p: p.read_bytes() for p in self.app.rglob('*') if p.is_file()}
        with mock.patch('subprocess.run', side_effect=AssertionError('No execution')):
            self.assertEqual(self.verify(), updater.verify_updater_package(self.app, self.commit))
        self.assertEqual(before, {p: p.read_bytes() for p in before})

    def test_missing_malformed_or_relabelled_prerequisite_refuses(self):
        original = copy.deepcopy(self.record)
        for mutate in (lambda r: r.pop('brokerPrerequisite'),
                       lambda r: r['brokerPrerequisite']['binary'].update(path='helper.exe'),
                       lambda r: r['brokerPrerequisite']['binary'].update(sha256='b' * 64),
                       lambda r: r['brokerPrerequisite']['buildRecord'].update(sha256='b' * 64),
                       lambda r: r['brokerPrerequisite'].update(extra='ignored')):
            self.record = copy.deepcopy(original)
            mutate(self.record)
            self.record_write()
            with self.assertRaises(ValueError):
                self.verify()

    def test_changed_valid_broker_record_refuses_old_provisioner_binding(self):
        path = self.app / broker.RECORD_RELATIVE
        path.write_text(json.dumps(json.loads(path.read_text()), indent=4))
        broker.verify_broker_package(self.app, self.commit)
        with self.assertRaisesRegex(ValueError, 'prerequisite descriptor changed'):
            self.verify()

    def test_missing_or_mixed_broker_prerequisite_refuses(self):
        path = self.app / broker.RECORD_RELATIVE
        record = json.loads(path.read_text())
        record['productCommit'] = 'b' * 40
        path.write_text(json.dumps(record))
        with self.assertRaisesRegex(ValueError, 'commit'):
            self.verify()
        path.unlink()
        with self.assertRaises(FileNotFoundError):
            self.verify()

    def test_recipe_schema_commit_and_source_descriptor_refuse(self):
        original = copy.deepcopy(self.record)
        for mutate in (lambda r: r.update(schema=True), lambda r: r.update(productCommit='b' * 40),
                       lambda r: r.update(build=broker.BUILD_RECIPE),
                       lambda r: r.update(build=provisioner.BUILD_RECIPE + ' -ldflags=-H=windowsgui'),
                       lambda r: r['sourceBundle'].update(sha256='b' * 64),
                       lambda r: r['binary'].update(path='arbitrary-helper.exe'), lambda r: r.update(extra=True)):
            self.record = copy.deepcopy(original)
            mutate(self.record)
            self.record_write()
            with self.assertRaises(ValueError):
                self.verify()

    def test_wrong_console_subsystem_architecture_and_changed_binary_refuse(self):
        for offset, value in ((68, 0x8664), (88, 0x20b), (156, 2)):
            self.binary_write()
            data = bytearray(self.binary.read_bytes())
            struct.pack_into('<H', data, offset, value)
            self.binary.write_bytes(data)
            self.record['binary']['sha256'] = updater._hash(self.binary)
            self.record_write()
            with self.assertRaisesRegex(ValueError, 'console Win32'):
                self.verify()
        self.binary_write()
        self.binary.write_bytes(self.binary.read_bytes() + b'mutated')
        with self.assertRaisesRegex(ValueError, 'changed'):
            self.verify()

    def test_forged_record_cannot_replace_actual_metadata(self):
        self.record['buildInfo'] = self.record['buildInfo'].replace(self.commit, 'b' * 40)
        self.record_write()
        with self.assertRaisesRegex(ValueError, 'embedded'):
            self.verify()

    def test_self_consistent_wrong_embedded_identity_refuses(self):
        for before, after in (('vcs.modified=false', 'vcs.modified=true'), (self.commit, 'b' * 40),
                              ('GOARCH=386', 'GOARCH=amd64'), ('CGO_ENABLED=0', 'CGO_ENABLED=1'),
                              ('GO386=sse2', 'GO386=softfloat'),
                              ('/cmd/skager-update-provisioner', '/cmd/skager-update-broker')):
            self.binary_write(self.module.replace(before, after))
            with self.assertRaisesRegex(ValueError, 'build settings'):
                self.verify()
        self.binary_write(version='go1.27.0')
        with self.assertRaisesRegex(ValueError, 'compiler'):
            self.verify()
        self.binary_write(self.module + 'build\t-ldflags=-H=windowsgui\n')
        with self.assertRaisesRegex(ValueError, 'build settings'):
            self.verify()

    def test_extra_compiler_flags_refuse_even_with_matching_record(self):
        for setting in ('-tags=extra', '-gcflags=all=-N', '-race=true', 'GOEXPERIMENT=unexpected'):
            with self.subTest(setting=setting):
                self.binary_write(self.module + 'build\t' + setting + '\n')
                with self.assertRaisesRegex(ValueError, 'build settings'):
                    self.verify()

    def test_mapped_metadata_required_overlay_marker_insufficient(self):
        data = bytearray(self.original_binary)
        data[512:526] = bytes(14)
        data.extend(self.original_binary[512:])
        self.binary.write_bytes(data)
        self.record['binary']['sha256'] = updater._hash(self.binary)
        self.record_write()
        with self.assertRaisesRegex(ValueError, 'missing or ambiguous'):
            self.verify()

    def rewrite_source(self, transform):
        archive = self.app / updater.SOURCE_RELATIVE
        with zipfile.ZipFile(archive) as source:
            contents = {name: source.read(name) for name in source.namelist()}
        transform(contents)
        with zipfile.ZipFile(archive, 'w') as source:
            for name, data in contents.items():
                source.writestr(name, data)
        bundle = copy.deepcopy(self.record['sourceBundle'])
        bundle['sha256'] = updater._hash(archive)
        for path in (archive.parent / 'build.json', self.app / broker.RECORD_RELATIVE, self.record_path):
            record = json.loads(path.read_text())
            record['sourceBundle'] = copy.deepcopy(bundle)
            path.write_text(json.dumps(record))
        self.record['sourceBundle'] = bundle
        # Rebind only the record hash so malformed source reaches the verifier
        # under test; do not verify or execute a prerequisite in this mutator.
        self.record['brokerPrerequisite']['buildRecord']['sha256'] = updater._hash(self.app / broker.RECORD_RELATIVE)
        self.record_write()

    def test_required_provisioner_main_and_packaging_source_cannot_be_omitted(self):
        with zipfile.ZipFile(self.app / updater.SOURCE_RELATIVE) as source:
            original = {name: source.read(name) for name in source.namelist()}
        for name in sorted(provisioner.REQUIRED_SOURCES | broker.REQUIRED_SOURCES):
            with self.subTest(omitted=name):
                def omit(contents):
                    contents.pop(name)
                    reference = json.loads(contents['SOURCE_REFERENCE.json'])
                    reference['files'].pop(name)
                    contents['SOURCE_REFERENCE.json'] = json.dumps(reference).encode()
                self.rewrite_source(omit)
                try:
                    with self.assertRaisesRegex(ValueError, 'sources missing'):
                        self.verify()
                finally:
                    self.rewrite_source(lambda contents: contents.update(original))

    def test_imported_registration_members_are_hash_checked(self):
        with zipfile.ZipFile(self.app / updater.SOURCE_RELATIVE) as source:
            original = {name: source.read(name) for name in source.namelist()}
        for name in sorted(broker.REGISTRATION_SOURCES):
            with self.subTest(tampered=name):
                self.rewrite_source(lambda contents: contents.update({name: b'X' * len(contents[name])}))
                try:
                    with self.assertRaisesRegex(ValueError, 'bytes differ'):
                        self.verify()
                finally:
                    self.rewrite_source(lambda contents: contents.update(original))

    def test_any_archived_module_go_bytes_are_hash_checked(self):
        # The producer fixture has the complete module. Add an unrelated Go
        # source with a claimed digest to prove every retained Go member is
        # checked, including code outside the enrollment package.
        def append(contents):
            member = broker.PREFIX + 'another/imported.go'
            data = b'package changed\n'
            reference = json.loads(contents['SOURCE_REFERENCE.json'])
            reference['files'][member] = {'bytes': len(data), 'sha256': 'b' * 64}
            contents[member] = data
            contents['SOURCE_REFERENCE.json'] = json.dumps(reference).encode()
        self.rewrite_source(append)
        with self.assertRaisesRegex(ValueError, 'bytes differ'):
            self.verify()

    def test_duplicate_json_and_linked_binary_refuse(self):
        text = self.record_path.read_text()
        self.record_path.write_text('{"schema":1,' + text[1:])
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            self.verify()
        self.record_write()
        other = self.app / 'elsewhere.exe'
        self.binary.rename(other)
        self.binary.symlink_to(other)
        with self.assertRaisesRegex(ValueError, 'reparse'):
            self.verify()


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3
"""Inert operator-kit orchestration tests; real verifiers, mocked build seams."""
from contextlib import ExitStack
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import unittest
from unittest import mock
import zipfile


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


entry = load('protected_operator_kit', 'package-protected-operator-kit.py')
fixtures = load('operator_kit_baseline_fixtures', 'test_baseline_handoff_package.py')


class OperatorKitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Existing inert PE/source fixtures exercise the actual package verifiers.
        # Only expensive compiler commands are replaced by the established fake.
        cls.fixture = fixtures.BaselineHandoffPackageTests()
        cls.fixture.setUp()
        cls.fixture.package()
        cls.addClassCleanup(cls.fixture.doCleanups)
        cls.full_install = cls.fixture.install
        cls.commit = cls.fixture.commit

    def setUp(self):
        import tempfile
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / 'original-launcher'
        self.source.mkdir()
        self.components = {}
        for name in entry.updater.TRANSFER_FILES:
            target = self.source / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(self.full_install / name, target)
        for component in entry.COMPONENTS:
            self.components[component.binary] = (self.full_install / component.binary).read_bytes()
            self.components[component.record_relative] = (self.full_install / component.record_relative).read_bytes()
        self.output = self.root / 'operator-kit.zip'
        self.events = []
        self.on_build = None

    def build(self, component):
        def produce(install, commit, go):
            self.assertEqual(commit, self.commit)
            self.assertEqual(go, 'inert-go')
            self.assertNotEqual(install, self.source)
            self.events.append(component.binary)
            # The original source remains launcher-only throughout each build.
            self.assertFalse((self.source / component.binary).exists())
            for name in (component.binary, component.record_relative):
                target = install / name
                target.parent.mkdir(parents=True, exist_ok=True)
                with target.open('xb') as output:
                    output.write(self.components[name])
            if self.on_build:
                self.on_build(install, component)
        return produce

    def package(self, **overrides):
        with ExitStack() as stack:
            stack.enter_context(mock.patch.object(entry.broker_producer.launcher, 'is_native_windows', return_value=True))
            for producer, component in zip((entry.broker_producer, entry.provisioner_producer, entry.handoff_producer),
                                           entry.COMPONENTS):
                build = AssertionError('Reuse mode must never invoke a producer') if overrides.get('components') else self.build(component)
                stack.enter_context(mock.patch.object(producer, 'package', side_effect=build))
            return entry.package(overrides.get('source', self.source), overrides.get('output', self.output),
                                 overrides.get('commit', self.commit), 'inert-go',
                                 components_install=overrides.get('components'))

    def test_exact_deterministic_inventory_chain_and_immutable_original(self):
        original = entry._source_snapshot(self.source, self.commit)
        def extra(install, component):
            if component == entry.COMPONENTS[-1]:
                (install / 'app').mkdir()
                (install / 'app' / 'opencpn.exe').write_bytes(b'NOT-AN-OPERATOR-TOOL')
                (install / 'credentials.txt').write_bytes(b'INERT-EXTRA-DO-NOT-COPY')
        self.on_build = extra
        result = self.package()
        self.assertEqual(self.events, [c.binary for c in entry.COMPONENTS])
        self.assertEqual(entry._source_snapshot(self.source, self.commit), original)
        self.assertEqual(result['sha256'], hashlib.sha256(self.output.read_bytes()).hexdigest())
        self.assertEqual(result['bytes'], self.output.stat().st_size)
        self.assertEqual(self.output.stat().st_nlink, 1)
        with zipfile.ZipFile(self.output) as kit:
            self.assertEqual(set(kit.namelist()), set(entry.PAYLOAD_LIMITS) | {entry.MANIFEST})
            self.assertEqual([n for n in kit.namelist() if n.endswith('.exe')], sorted(c.binary for c in entry.COMPONENTS))
            manifest = json.loads(kit.read(entry.MANIFEST))
            self.assertEqual(manifest['productCommit'], self.commit)
            for name, record in manifest['files'].items():
                self.assertEqual(record, {'bytes': len(kit.read(name)),
                                         'sha256': hashlib.sha256(kit.read(name)).hexdigest()})
            self.assertEqual(kit.read(entry.updater.SOURCE_RELATIVE),
                             (self.source / entry.updater.SOURCE_RELATIVE).read_bytes())
            with zipfile.ZipFile(self.source / entry.updater.SOURCE_RELATIVE) as source:
                self.assertEqual(kit.read(entry.LICENSE), source.read('project/LICENSE'))
                self.assertEqual(kit.read(entry.REFERENCE), source.read('SOURCE_REFERENCE.json'))
        second = self.root / 'second.zip'
        self.package(output=second)
        self.assertEqual(self.output.read_bytes(), second.read_bytes())

    def component_install(self):
        path = self.root / 'original-components'
        shutil.copytree(self.full_install, path)
        return path

    def test_reuse_verified_components_never_builds_and_preserves_both_originals(self):
        components = self.component_install()
        original = entry._source_snapshot(self.source, self.commit)
        before = entry._components_snapshot(components, self.commit, original)
        self.package()
        self.events.clear()
        reused = self.root / 'reused.zip'
        self.package(components=components, output=reused)
        self.assertEqual(self.events, [])
        self.assertEqual(reused.read_bytes(), self.output.read_bytes())
        self.assertEqual(entry._source_snapshot(self.source, self.commit), original)
        self.assertEqual(entry._components_snapshot(components, self.commit, original), before)

    def test_reuse_component_commit_mismatch_and_extra_entries_refuse(self):
        components = self.component_install()
        record = components / entry.COMPONENTS[0].record_relative
        original = record.read_bytes()
        data = json.loads(original)
        data['productCommit'] = 'b' * 40
        record.write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            self.package(components=components)
        record.write_bytes(original)
        extra = components / 'profile.ini'
        extra.write_bytes(b'INERT-UNRELATED-STATE')
        with self.assertRaisesRegex(ValueError, 'Unexpected'):
            self.package(components=components)
        self.assertEqual(self.events, [])
        self.assertFalse(self.output.exists())

    def test_reuse_requires_original_launcher_and_archive_bytes(self):
        components = self.component_install()
        for name in ('skager-start.exe', entry.updater.SOURCE_RELATIVE):
            path = components / name
            original = path.read_bytes()
            path.write_bytes(original + b'CHANGED')
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.package(components=components)
            path.write_bytes(original)
        self.assertFalse(self.output.exists())

    def test_reuse_valid_same_commit_alternate_launcher_is_not_exact_original(self):
        components = self.component_install()
        binary = components / 'skager-start.exe'
        binary.write_bytes(binary.read_bytes() + b'INERT-PE-OVERLAY')
        path = components / 'opennav/third-party/updater/build.json'
        record = json.loads(path.read_bytes())
        record['binary']['sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
        path.write_text(json.dumps(record))
        # Same commit and a valid individual chain still cannot substitute for
        # the separately supplied original launcher producer's exact bytes.
        entry._verify_components(components, self.commit, entry.updater.verify_updater_package(self.source, self.commit))
        with self.assertRaisesRegex(ValueError, 'exact original launcher/source'):
            self.package(components=components)
        self.assertEqual(self.events, [])
        self.assertFalse(self.output.exists())

    def test_reuse_component_mutation_or_replacement_during_transfer_refuses(self):
        components = self.component_install()
        copy = entry._copy_component_files
        path = components / entry.COMPONENTS[-1].binary
        original = path.read_bytes()
        for identical in (False, True):
            def change(source, install, snapshot):
                copy(source, install, snapshot)
                replacement = self.root / 'replacement-component.exe'
                replacement.write_bytes(original if identical else original + b'CHANGED')
                os.replace(replacement, path)
            with self.subTest(identical=identical), mock.patch.object(entry, '_copy_component_files', side_effect=change):
                with self.assertRaises(ValueError):
                    self.package(components=components)
            path.write_bytes(original)
        self.assertFalse(self.output.exists())
        self.assertEqual(self.events, [])

    def test_reuse_components_links_and_output_overlap_refuse(self):
        components = self.component_install()
        linked = self.root / 'linked-components'
        linked.symlink_to(components, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'link'):
            self.package(components=linked)
        with self.assertRaisesRegex(ValueError, 'separate'):
            self.package(components=components, output=components / 'kit.zip')
        path = components / entry.COMPONENTS[0].binary
        os.link(path, self.root / 'hardlinked-component.exe')
        with self.assertRaisesRegex(ValueError, 'single-link'):
            self.package(components=components)
        self.assertFalse(self.output.exists())

    def test_native_gate_before_source_reads_or_builds(self):
        with mock.patch.object(entry.broker_producer.launcher, 'is_native_windows', return_value=False), \
                mock.patch.object(entry, '_source_snapshot') as read:
            with self.assertRaisesRegex(ValueError, 'native Windows'):
                entry.package(self.source, self.output, self.commit)
            read.assert_not_called()
        self.assertFalse(self.output.exists())

    def test_explicit_commit_mismatch_refuses_before_build(self):
        for commit in ('', self.commit.upper(), 'b' * 40):
            with self.subTest(commit=commit), self.assertRaises(ValueError):
                self.package(commit=commit)
        self.assertEqual(self.events, [])
        self.assertFalse(self.output.exists())

    def test_source_mutation_during_build_refuses_publication(self):
        def change(install, component):
            if component == entry.COMPONENTS[-1]:
                with (self.source / 'skager-start.exe').open('ab') as target:
                    target.write(b'CHANGED')
        self.on_build = change
        with self.assertRaisesRegex(ValueError, 'changed'):
            self.package()
        self.assertFalse(self.output.exists())

    def test_identical_byte_original_replacement_is_still_refused(self):
        def replace(install, component):
            if component == entry.COMPONENTS[-1]:
                path = self.source / 'skager-start.exe'
                replacement = self.root / 'replacement.exe'
                replacement.write_bytes(path.read_bytes())
                os.replace(replacement, path)
        self.on_build = replace
        with self.assertRaisesRegex(ValueError, 'original launcher producer changed'):
            self.package()
        self.assertFalse(self.output.exists())

    def test_component_or_source_archive_mutation_refuses(self):
        for name in (entry.COMPONENTS[0].binary, entry.updater.SOURCE_RELATIVE):
            with self.subTest(name=name):
                def change(install, component):
                    if component == entry.COMPONENTS[-1]:
                        with (install / name).open('ab') as target:
                            target.write(b'CHANGED')
                self.on_build = change
                with self.assertRaises(ValueError):
                    self.package()
                self.assertFalse(self.output.exists())

    def test_existing_and_racing_outputs_are_preserved(self):
        self.output.write_bytes(b'EXISTING')
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            self.package()
        self.assertEqual(self.output.read_bytes(), b'EXISTING')
        self.assertEqual(self.events, [])
        self.output.unlink()
        def race(install, component):
            if component == entry.COMPONENTS[-1]:
                self.output.write_bytes(b'RACING')
        self.on_build = race
        with self.assertRaises(FileExistsError):
            self.package()
        self.assertEqual(self.output.read_bytes(), b'RACING')

    def test_symlink_hardlink_and_source_output_overlap_refuse(self):
        linked = self.root / 'linked-source'
        linked.symlink_to(self.source, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'link'):
            self.package(source=linked)
        with self.assertRaisesRegex(ValueError, 'separate'):
            self.package(output=self.source / 'kit.zip')
        output_link = self.root / 'linked-output.zip'
        output_link.symlink_to(self.output)
        with self.assertRaisesRegex(ValueError, 'link'):
            self.package(output=output_link)
        os.link(self.source / 'skager-start.exe', self.root / 'hardlink.exe')
        with self.assertRaisesRegex(ValueError, 'single-link'):
            self.package()
        self.assertEqual(self.events, [])

    def test_original_extra_state_and_total_bound_refuse(self):
        extra = self.source / 'profile.ini'
        extra.write_bytes(b'INERT-USER-STATE')
        with self.assertRaisesRegex(ValueError, 'Unexpected'):
            self.package()
        self.assertEqual(self.events, [])
        extra.unlink()
        with mock.patch.object(entry, 'MAX_KIT_BYTES', 10), self.assertRaisesRegex(ValueError, 'total bound'):
            self.package()
        self.assertFalse(self.output.exists())

    def test_payload_mutation_during_archiving_refuses_actual_readback(self):
        write = entry._write_archive
        def change(install, archive, manifest):
            path = install / entry.COMPONENTS[0].binary
            data = path.read_bytes()
            path.write_bytes(bytes([data[0] ^ 1]) + data[1:])
            return write(install, archive, manifest)
        with mock.patch.object(entry, '_write_archive', side_effect=change), self.assertRaises(ValueError):
            self.package()
        self.assertFalse(self.output.exists())


if __name__ == '__main__':
    unittest.main()

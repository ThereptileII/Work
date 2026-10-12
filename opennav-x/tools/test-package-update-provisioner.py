#!/usr/bin/env python3
"""Inert provisioner producer checks: fake compiler, no execution or network."""
import copy
import importlib.util
import json
from pathlib import Path
import struct
import unittest
from unittest import mock

import broker_package as broker
import provisioner_package as provisioner


def load(name, file):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(file))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


entry = load('provisioner_producer', 'package-update-provisioner.py')
shared = entry.producer
launcher = shared.launcher
fixtures = load('broker_producer_fixtures', 'test-package-update-broker.py')
pe_fixtures = load('broker_metadata_fixtures', 'test-broker-package.py')


class ProvisionerProducerTests(unittest.TestCase):
    def setUp(self):
        # Reuse the established fake compiler/cache/source closure fixture. Its
        # assertions still govern all shared audit commands; only the console
        # component build is intercepted below. Neither PE fixture is executed.
        self.fixture = fixtures.BrokerProducerTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        f = self.fixture
        for name in provisioner.REQUIRED_SOURCES | broker.REQUIRED_SOURCES:
            name = name.removeprefix('project/')
            if name not in f.tracked:
                f.tracked.append(name)
                path = f.repo / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(('inert corresponding source: ' + name).encode())
        with mock.patch.object(shared, 'ROOT', f.repo), mock.patch.object(launcher, 'run', side_effect=self.fake_run):
            inventory = shared.source_inventory(f.module, f.goroot, f.cache, f.modules, f.reference,
                                                component=provisioner.PROVISIONER)
        provision = f.root / 'provisioning-go.sum'
        inventory.add(provision, 'provisioning/go.sum')
        f.archive.unlink()
        inventory.write(f.archive, f.reference)
        f.launcher_record['sourceBundle']['sha256'] = launcher.digest(f.archive)
        f.record_path.write_text(json.dumps(f.launcher_record))
        self.broker_binary = f.install / broker.BINARY
        self.broker_record_path = f.install / broker.RECORD_RELATIVE
        self.broker_record_path.parent.mkdir(parents=True)
        self.broker_module = ''.join(line[1:] + '\n' for line in f.metadata().splitlines()[1:])
        self.broker_binary.write_bytes(pe_fixtures.pe(self.broker_module))
        self.broker_record = {'schema': 1, 'productCommit': f.commit, 'goVersion': launcher.GO_VERSION,
            'binary': {'path': broker.BINARY, 'sha256': launcher.digest(self.broker_binary)},
            'sourceBundle': copy.deepcopy(f.launcher_record['sourceBundle']),
            'buildInfo': f.metadata().replace('private-path/', ''), 'build': broker.BUILD_RECIPE}
        self.broker_record_path.write_text(json.dumps(self.broker_record))
        f.info = f.metadata().replace(broker.BINARY, provisioner.BINARY).replace('/cmd/skager-update-broker',
                                                                               '/cmd/skager-update-provisioner')
        f.events.clear()
        self.binary_transform = None

    def fake_run(self, command, directory, environment=None):
        f = self.fixture
        if command[1] == 'build':
            f.events.append(command)
            self.assertEqual(Path(command[0]), f.go)
            self.assertEqual(Path(directory), f.module)
            self.assertEqual(command[1:5], ['build', '-mod=readonly', '-trimpath', '-buildvcs=true'])
            self.assertEqual(command[-1], './cmd/skager-update-provisioner')
            self.assertFalse(any(arg.startswith('-ldflags') for arg in command))
            self.assertEqual(environment['GOPROXY'], 'off')
            self.assertEqual(environment['GOSUMDB'], 'off')
            self.assertEqual(environment['CGO_ENABLED'], '0')
            self.assertEqual(environment['GOARCH'], '386')
            self.assertEqual(environment['GO386'], 'sse2')
            module = ''.join(line[1:] + '\n' for line in f.info.splitlines()[1:])
            data = bytearray(pe_fixtures.pe(module))
            struct.pack_into('<H', data, 156, 3)
            if self.binary_transform:
                data = self.binary_transform(data)
            Path(command[command.index('-o') + 1]).write_bytes(data)
            if f.on_build:
                f.on_build()
            return ''
        return f.fake_run(command, directory, environment)

    def package(self):
        f = self.fixture
        with mock.patch.object(shared, 'ROOT', f.repo), mock.patch.object(launcher, 'ROOT', f.repo), \
                mock.patch.object(launcher, 'is_native_windows', return_value=True), \
                mock.patch.object(launcher, 'run', side_effect=self.fake_run), \
                mock.patch.object(launcher.Inventory, 'write', side_effect=AssertionError('Retain original archive')):
            return entry.package(f.install, f.commit, str(f.go))

    def assert_unpublished(self):
        self.assertFalse((self.fixture.install / provisioner.BINARY).exists())
        self.assertFalse((self.fixture.install / provisioner.RECORD_RELATIVE).exists())

    def test_console_build_fixed_recipe_retained_prerequisites_and_strict_verifier(self):
        f = self.fixture
        before = {p: p.read_bytes() for p in f.install.rglob('*') if p.is_file()}
        result = self.package()
        self.assertEqual(result['build'], provisioner.BUILD_RECIPE)
        self.assertEqual(result['brokerPrerequisite'], broker.broker_prerequisite(f.install, f.commit))
        self.assertEqual(provisioner.verify_provisioner_package(f.install, f.commit),
                         shared.updater.verify_updater_package(f.install, f.commit))
        self.assertEqual(before, {p: p.read_bytes() for p in before})
        self.assertFalse(any(command[1:3] == ['mod', 'download'] for command in f.events))
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            self.package()

    def test_native_gate_precedes_all_runners_and_prerequisite_reads(self):
        f = self.fixture
        with mock.patch.object(launcher, 'is_native_windows', return_value=False), \
                mock.patch.object(launcher, 'run') as run, mock.patch.object(broker, 'broker_prerequisite') as prerequisite:
            with self.assertRaisesRegex(ValueError, 'native Windows'):
                entry.package(f.install, f.commit, str(f.go))
            run.assert_not_called()
            prerequisite.assert_not_called()
        self.assert_unpublished()

    def test_dirty_and_mixed_checkout_sources_refuse(self):
        f = self.fixture
        f.dirty = True
        with self.assertRaisesRegex(ValueError, 'committed source'):
            self.package()
        f.dirty = False
        (f.repo / 'tools/provisioner_package.py').write_bytes(b'mixed source')
        with self.assertRaisesRegex(ValueError, 'Actual broker build input differs'):
            self.package()
        self.assertFalse(any(c[1] == 'build' for c in f.events))
        self.assert_unpublished()

    def test_clean_commit_switch_during_build_refuses(self):
        f = self.fixture
        f.on_build = lambda: setattr(f, 'commit', 'b' * 40)
        with self.assertRaisesRegex(ValueError, 'commit changed'):
            self.package()
        self.assert_unpublished()

    def test_missing_provisioner_source_refuses_before_build(self):
        f = self.fixture
        for name in sorted(provisioner.REQUIRED_SOURCES):
            if not name.endswith('.go'):
                continue
            with self.subTest(omitted=name):
                tracked = name.removeprefix('project/')
                f.tracked.remove(tracked)
                f.events.clear()
                try:
                    with self.assertRaisesRegex(ValueError, 'complete broker source'):
                        self.package()
                    self.assertFalse(any(c[1] == 'build' for c in f.events))
                    self.assert_unpublished()
                finally:
                    f.tracked.append(tracked)

    def test_changed_valid_broker_record_during_build_refuses(self):
        self.fixture.on_build = lambda: self.broker_record_path.write_text(json.dumps(self.broker_record, indent=2))
        with self.assertRaisesRegex(ValueError, 'broker prerequisite changed'):
            self.package()
        self.assert_unpublished()

    def test_changed_valid_broker_binary_during_build_refuses(self):
        def change():
            module = self.broker_module.replace('12:00:00Z', '12:00:01Z')
            self.broker_binary.write_bytes(pe_fixtures.pe(module))
            self.broker_record['binary']['sha256'] = launcher.digest(self.broker_binary)
            self.broker_record['buildInfo'] = broker.BINARY + ': ' + launcher.GO_VERSION + '\n' + ''.join(
                '\t' + line + '\n' for line in module.splitlines())
            self.broker_record_path.write_text(json.dumps(self.broker_record))
        self.fixture.on_build = change
        with self.assertRaisesRegex(ValueError, 'broker prerequisite changed'):
            self.package()
        self.assert_unpublished()

    def test_wrong_architecture_and_gui_subsystem_refuse_publication(self):
        for offset, value in ((68, 0x8664), (156, 2)):
            with self.subTest(value=value):
                def transform(data):
                    struct.pack_into('<H', data, offset, value)
                    return data
                self.binary_transform = transform
                with self.assertRaisesRegex(ValueError, 'x86|console PE subsystem'):
                    self.package()
                self.assert_unpublished()

    def test_compiler_text_cannot_forge_actual_embedded_metadata(self):
        def transform(data):
            old, new = b'vcs.modified=false', b'vcs.modified=other'
            return data.replace(old, new)
        self.binary_transform = transform
        with self.assertRaisesRegex(ValueError, 'embedded'):
            self.package()
        self.assert_unpublished()

    def test_default_trust_is_refused_before_build(self):
        (self.fixture.install / 'update-trust.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, 'trust configuration'):
            self.package()
        self.assert_unpublished()


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3
"""Portable inert producer and verifier checks; no native execution or network."""
import copy
import importlib.util
import json
from pathlib import Path
import struct
import unittest
from unittest import mock

import baseline_handoff_package as handoff
import broker_package as broker
import provisioner_package as provisioner


def load(name, file):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(file))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


entry = load('baseline_handoff_producer', 'package-baseline-handoff.py')
shared = entry.producer
launcher = shared.launcher
provisioner_fixtures = load('baseline_provisioner_fixtures', 'test-package-update-provisioner.py')
pe_fixtures = load('baseline_pe_fixtures', 'test-broker-package.py')


class BaselineHandoffPackageTests(unittest.TestCase):
    def setUp(self):
        self.prior = provisioner_fixtures.ProvisionerProducerTests()
        self.prior.setUp()
        self.addCleanup(self.prior.doCleanups)
        f = self.prior.fixture
        # Extend the original retained source archive BEFORE provisioner
        # packaging. Every prerequisite then points to this exact same archive.
        for name in handoff.REQUIRED_SOURCES:
            tracked = name.removeprefix('project/')
            if tracked not in f.tracked:
                f.tracked.append(tracked)
                path = f.repo / tracked
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(('inert corresponding source: ' + name).encode())
        with mock.patch.object(shared, 'ROOT', f.repo), mock.patch.object(launcher, 'run', side_effect=self.prior.fake_run):
            inventory = shared.source_inventory(f.module, f.goroot, f.cache, f.modules, f.reference,
                                                component=handoff.BASELINE_HANDOFF)
        inventory.add(f.root / 'provisioning-go.sum', 'provisioning/go.sum')
        f.archive.unlink()
        inventory.write(f.archive, f.reference)
        f.launcher_record['sourceBundle']['sha256'] = launcher.digest(f.archive)
        f.record_path.write_text(json.dumps(f.launcher_record))
        self.prior.broker_record['sourceBundle'] = copy.deepcopy(f.launcher_record['sourceBundle'])
        self.prior.broker_record_path.write_text(json.dumps(self.prior.broker_record))
        self.prior.package()
        self.provisioner_path = f.install / provisioner.RECORD_RELATIVE
        self.provisioner_binary = f.install / provisioner.BINARY
        self.install, self.commit = f.install, f.commit
        self.info = f.metadata().replace(broker.BINARY, handoff.BINARY).replace(
            '/cmd/skager-update-broker', '/cmd/skager-baseline-package').replace('private-path/', '')
        f.info = self.info
        f.events.clear()
        self.binary_transform = None

    def fake_run(self, command, directory, environment=None):
        f = self.prior.fixture
        if command[1] == 'build':
            f.events.append(command)
            self.assertEqual(command[1:5], ['build', '-mod=readonly', '-trimpath', '-buildvcs=true'])
            self.assertEqual(command[-1], './cmd/skager-baseline-package')
            self.assertFalse(any(arg.startswith('-ldflags') for arg in command))
            self.assertEqual(environment['GOOS'], 'windows')
            self.assertEqual(environment['GOARCH'], '386')
            self.assertEqual(environment['GO386'], 'sse2')
            self.assertEqual(environment['GOPROXY'], 'off')
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
        f = self.prior.fixture
        with mock.patch.object(shared, 'ROOT', f.repo), mock.patch.object(launcher, 'ROOT', f.repo), \
                mock.patch.object(launcher, 'is_native_windows', return_value=True), \
                mock.patch.object(launcher, 'run', side_effect=self.fake_run), \
                mock.patch.object(launcher.Inventory, 'write', side_effect=AssertionError('Retain source archive')):
            return entry.package(f.install, f.commit, str(f.go))

    def assert_unpublished(self):
        self.assertFalse((self.install / handoff.BINARY).exists())
        self.assertFalse((self.install / handoff.RECORD_RELATIVE).exists())

    def test_exact_console_same_commit_chain_and_no_overwrite(self):
        before = {p: p.read_bytes() for p in self.install.rglob('*') if p.is_file()}
        record = self.package()
        self.assertEqual(record['build'], handoff.BUILD_RECIPE)
        self.assertEqual(record['brokerPrerequisite'], handoff.provisioner_prerequisite(self.install, self.commit))
        self.assertEqual(handoff.verify_baseline_handoff_package(self.install, self.commit),
                         shared.updater.verify_updater_package(self.install, self.commit))
        self.assertEqual(before, {p: p.read_bytes() for p in before})
        self.assertFalse(any(c[1:3] == ['mod', 'download'] for c in self.prior.fixture.events))
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            self.package()

    def test_native_gate_before_any_runner_or_prerequisite(self):
        with mock.patch.object(launcher, 'is_native_windows', return_value=False), \
                mock.patch.object(launcher, 'run') as run, \
                mock.patch.object(handoff, 'provisioner_prerequisite') as prerequisite:
            with self.assertRaisesRegex(ValueError, 'native Windows'):
                entry.package(self.install, self.commit, 'go')
            run.assert_not_called()
            prerequisite.assert_not_called()
        self.assert_unpublished()

    def test_mixed_source_and_missing_baseline_main_refuse_before_build(self):
        f = self.prior.fixture
        source = f.repo / 'tools/update-verifier/baselinepackage/package.go'
        source.write_bytes(b'changed after retained archive')
        with self.assertRaisesRegex(ValueError, 'Actual broker build input differs'):
            self.package()
        self.assertFalse(any(c[1] == 'build' for c in f.events))
        self.assert_unpublished()
        source.write_bytes(('inert corresponding source: ' + broker.PREFIX + 'baselinepackage/package.go').encode())
        f.tracked.remove('tools/update-verifier/cmd/skager-baseline-package/main.go')
        with self.assertRaisesRegex(ValueError, 'complete broker source'):
            self.package()
        self.assert_unpublished()

    def test_missing_packaging_recipe_refuses_before_build(self):
        f = self.prior.fixture
        f.tracked.remove('tools/package-baseline-handoff.py')
        with self.assertRaisesRegex(ValueError, 'complete broker source'):
            self.package()
        self.assertFalse(any(c[1] == 'build' for c in f.events))
        self.assert_unpublished()

    def test_provisioner_change_during_build_refuses_publication(self):
        f = self.prior.fixture
        original = self.provisioner_path.read_text()
        f.on_build = lambda: self.provisioner_path.write_text(json.dumps(json.loads(original), indent=4))
        with self.assertRaisesRegex(ValueError, 'prerequisite changed'):
            self.package()
        self.assert_unpublished()

    def test_wrong_pe_identity_refuses_publication(self):
        for offset, value in ((68, 0x8664), (156, 2)):
            with self.subTest(offset=offset):
                self.binary_transform = lambda data: (struct.pack_into('<H', data, offset, value) or data)
                with self.assertRaisesRegex(ValueError, 'x86|console PE subsystem'):
                    self.package()
                self.assert_unpublished()

    def test_verifier_refuses_relabelled_or_changed_prerequisite(self):
        self.package()
        record_path = self.install / handoff.RECORD_RELATIVE
        record = json.loads(record_path.read_text())
        record['brokerPrerequisite']['binary']['path'] = 'unapproved.exe'
        record_path.write_text(json.dumps(record))
        with self.assertRaisesRegex(ValueError, 'prerequisite descriptor changed'):
            handoff.verify_baseline_handoff_package(self.install, self.commit)


if __name__ == '__main__':
    unittest.main()

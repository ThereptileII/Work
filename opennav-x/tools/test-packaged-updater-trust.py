"""Inert checks for exact packaged trust and actual legacy initialization."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('packaged_trust_fixture', Path(__file__).with_name('test-packaged-updater-windows.py'))
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


class TrustTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.app = self.root / 'app'
        self.app.mkdir()
        self.root_raw = b'{ "signed": { "version": 1 } }'
        self.raw = b'{"bootstrapRoot":' + self.root_raw + b',"metadataUrl":"https://inert.invalid","channel":"beta"}\n'
        self.pin = hashlib.sha256(self.raw).hexdigest()
        self.product = dict(commit='a' * 40, update_trust=dict(schema=1, productCommit='a' * 40, channel='beta',
                            sourcePath='opennav-x/installer/windows/staging-update-trust.json',
                            sourceSha256=self.pin, configSha256=self.pin))
        self.path = self.app / 'update-trust.json'
        self.path.write_bytes(self.raw)

    def initialize(self, generation='a' * 32, version=1):
        identity = helper.packaged_trust_identity(self.app, self.product)
        (self.root / 'update-artifacts').mkdir(exist_ok=True)
        channel = self.root / 'update-trust/beta'
        channel.mkdir(parents=True, exist_ok=True)
        cache = channel / generation / 'metadata'
        cache.mkdir(parents=True)
        root = self.root_raw if version == 1 else json.dumps({'signed': {'version': version}}).encode()
        (cache / 'root.json').write_bytes(root)
        (self.root / 'update-trust-initialized-beta').write_bytes(json.dumps(
            dict(schema=1, channel='beta', configSha256=self.pin), separators=(',', ':')).encode())
        state = dict(schema=1, **{k: identity[k] for k in ('bootstrapSha256', 'metadataUrl', 'channel')},
                     generation=generation, metadata={'root.json': hashlib.sha256(root).hexdigest()})
        (channel / 'state.json').write_text(json.dumps(state))
        return identity

    def test_exact_selected_original_trust_and_intentional_initialization_are_retained(self):
        before = self.path.read_bytes()
        identity = self.initialize()
        self.assertEqual(identity['state'], 'packaged-public-trust')
        self.assertEqual(identity['configSha256'], self.pin)
        self.assertEqual(identity['bootstrapSha256'], hashlib.sha256(self.root_raw).hexdigest())
        self.assertNotEqual(identity['bootstrapSha256'], hashlib.sha256(json.dumps(json.loads(self.root_raw)).encode()).hexdigest())
        self.assertEqual(self.path.read_bytes(), before)
        original = helper.legacy_trust_observation(self.root, identity, initial=True)
        self.assertEqual(original['state'], 'initialized-original-identity')
        self.assertTrue(helper.legacy_trust_preserved(original, helper.legacy_trust_observation(self.root, identity)))

    def test_changed_missing_undeclared_mixed_commit_and_source_pins_refuse(self):
        for key, value in (('schema', True), ('productCommit', 'b' * 40), ('channel', 'stable'),
                           ('sourcePath', 'unreviewed.json'), ('sourceSha256', '0' * 64),
                           ('configSha256', self.pin.upper())):
            bad = copy.deepcopy(self.product)
            bad['update_trust'][key] = value
            with self.subTest(key=key), self.assertRaises(AssertionError):
                helper.packaged_trust_identity(self.app, bad)
        self.path.write_bytes(self.raw + b' ')
        with self.assertRaises(AssertionError):
            helper.packaged_trust_identity(self.app, self.product)
        with self.assertRaises(AssertionError):
            helper.packaged_trust_identity(self.app, {'commit': self.product['commit']})
        self.path.unlink()
        with self.assertRaises(FileNotFoundError):
            helper.packaged_trust_identity(self.app, self.product)
        self.assertEqual(helper.packaged_trust_identity(self.app, {'commit': self.product['commit']}), {'state': 'unconfigured'})

    def test_linked_or_oversized_public_trust_refuses(self):
        other = self.root / 'original.json'
        self.path.rename(other)
        self.path.symlink_to(other)
        with self.assertRaises(AssertionError):
            helper.packaged_trust_identity(self.app, self.product)
        self.path.unlink()
        self.path.hardlink_to(other)
        with self.assertRaises(AssertionError):
            helper.packaged_trust_identity(self.app, self.product)
        self.path.unlink()
        self.path.write_bytes(b'x' * ((576 << 10) + 1))
        with self.assertRaises(AssertionError):
            helper.packaged_trust_identity(self.app, self.product)

    def test_unconfigured_packages_refuse_any_legacy_initialization_output_without_reset(self):
        for name in ('update-trust', 'update-artifacts', 'update-trust-initialized-beta', 'update-trust-initialized-stable'):
            path = self.root / name
            path.write_bytes(b'original state must survive')
            with self.subTest(name=name), self.assertRaises(AssertionError):
                helper.legacy_trust_observation(self.root, {'state': 'unconfigured'})
            self.assertEqual(path.read_bytes(), b'original state must survive')
            path.unlink()
        (self.root / 'update-trust').symlink_to(self.root / 'missing')
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, {'state': 'unconfigured'})

    def test_legitimate_cache_refresh_advances_but_downgrade_or_replaced_marker_refuses(self):
        identity = self.initialize()
        before = helper.legacy_trust_observation(self.root, identity)
        # Real refresh atomically replaces state.json and selects a new cache;
        # the original marker/store directory objects remain intact.
        self.initialize('b' * 32, 2)
        after = helper.legacy_trust_observation(self.root, identity)
        self.assertTrue(helper.legacy_trust_preserved(before, after))
        self.assertFalse(helper.legacy_trust_preserved(after, before))
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, identity, initial=True)
        marker = self.root / 'update-trust-initialized-beta'
        original = marker.read_bytes()
        marker.rename(self.root / 'original-marker-retained')
        marker.write_bytes(original)
        self.assertFalse(helper.legacy_trust_preserved(after, helper.legacy_trust_observation(self.root, identity)))

    def test_self_consistent_reserialized_initial_root_refuses(self):
        identity = self.initialize()
        channel = self.root / 'update-trust/beta'
        root = json.dumps(json.loads(self.root_raw)).encode()
        (channel / ('a' * 32) / 'metadata/root.json').write_bytes(root)
        state_path = channel / 'state.json'
        state = json.loads(state_path.read_bytes())
        state['metadata']['root.json'] = hashlib.sha256(root).hexdigest()
        state_path.write_text(json.dumps(state))
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, identity, initial=True)

    def test_missing_corrupt_changed_identity_pending_and_metadata_refuse(self):
        identity = self.initialize()
        channel = self.root / 'update-trust/beta'
        state_path = channel / 'state.json'
        original = state_path.read_bytes()
        for key, value in (('schema', True), ('bootstrapSha256', '0' * 64), ('metadataUrl', 'https://other.invalid'),
                           ('channel', 'stable'), ('generation', '../escape')):
            state = json.loads(original)
            state[key] = value
            state_path.write_text(json.dumps(state))
            with self.subTest(key=key), self.assertRaises(AssertionError):
                helper.legacy_trust_observation(self.root, identity)
        state_path.write_bytes(original)
        pending = channel / 'refresh.pending'
        pending.write_bytes(b'uncertain original refresh')
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, identity)
        self.assertEqual(pending.read_bytes(), b'uncertain original refresh')
        pending.unlink()
        metadata = channel / ('a' * 32) / 'metadata/root.json'
        metadata.write_bytes(b'changed')
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, identity)
        metadata.unlink()
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, identity)

    def test_public_trust_duplicate_fields_and_changed_once_marker_refuse(self):
        with self.assertRaises(AssertionError):
            helper.public_trust_identity(self.raw.replace(b'"channel":"beta"', b'"channel":"beta","channel":"beta"'))
        identity = self.initialize()
        marker = self.root / 'update-trust-initialized-beta'
        marker.write_bytes(marker.read_bytes() + b'\n')
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, identity)


if __name__ == '__main__':
    unittest.main()

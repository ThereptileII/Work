#!/usr/bin/env python3
"""Fail-closed harness contracts; no application, display or peer is launched."""
import importlib.util
import hashlib
import json
from pathlib import Path
import tempfile
import types
import unittest
import zipfile
from unittest.mock import Mock, patch


spec = importlib.util.spec_from_file_location(
    "peer_two", Path(__file__).with_name("smoke-peer-two-instances.py"))
peer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(peer)


class PeerTwoContracts(unittest.TestCase):
    def retained_fixture(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name).resolve()
        product, harness = 'a' * 40, 'b' * 40
        exe = root / 'build/xnav-install/opencpn.exe'
        exe.parent.mkdir(parents=True)
        exe.write_bytes(b'inert retained executable; never launched')
        build = root / 'build/xnav-windows'
        build.mkdir()
        identity = peer.staging_inputs.producer(product, '123', '1')
        manifest = json.dumps(dict(producer=identity, files=[dict(
            path='build/xnav-install/opencpn.exe', size=exe.stat().st_size,
            sha256=peer.staging_inputs.sha(exe))])).encode()
        archive = root / 'build/retained-staging-download/STAGING_BUILD_INPUTS.zip'
        archive.parent.mkdir()
        with zipfile.ZipFile(archive, 'w') as out:
            out.writestr(peer.staging_inputs.MANIFEST, manifest)
        receipt = root / 'evidence/local/staging-inputs.json'
        receipt.parent.mkdir(parents=True)
        record = dict(schema=1, kind=peer.staging_inputs.KIND, producer=identity,
                      status='restored', qualification='not-run', workspaceRoot=str(root),
                      harnessCommit=harness, archiveSha256=peer.staging_inputs.sha(archive),
                      manifestSha256=hashlib.sha256(manifest).hexdigest())
        receipt.write_text(json.dumps(record))
        return root, exe, build, product, harness, receipt, record, archive

    def test_same_commit_default_and_explicit_identity_pass(self):
        commit = 'a' * 40
        for requested in (None, commit):
            self.assertEqual(peer.product_identity(Path('unused'), Path('unused'), requested,
                                                   None, commit), (commit, None))
        for requested in ('b' * 40, '', 'a' * 8):
            with self.subTest(requested=requested), self.assertRaises(ValueError):
                peer.product_identity(Path('unused'), Path('unused'), requested, None, commit)

    def test_retained_producer_passes_with_separate_harness_identity(self):
        root, exe, build, product, harness, receipt, _, _ = self.retained_fixture()
        expected, digest = peer.product_identity(exe, build, product, receipt, harness, root)
        self.assertEqual(expected, product)
        self.assertEqual(digest, peer.staging_inputs.sha(receipt))
        peer.require_build({'build_commit': product}, expected)
        with self.assertRaises(AssertionError):
            peer.require_build({'build_commit': harness}, expected)

    def test_retained_receipt_mismatches_are_refused(self):
        root, exe, build, product, harness, receipt, record, _ = self.retained_fixture()
        for mutation in (dict(schema=True), dict(status='sealed'), dict(qualification='passed'),
                         dict(kind='other'), dict(harnessCommit=product), dict(workspaceRoot='/other'),
                         dict(producer=peer.staging_inputs.producer('c' * 40, '123', '1')),
                         dict(archiveSha256='bad'), dict(manifestSha256='0' * 64)):
            receipt.write_text(json.dumps({**record, **mutation}))
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                peer.product_identity(exe, build, product, receipt, harness, root)

    def test_missing_malformed_and_linked_receipts_are_refused(self):
        root, exe, build, product, harness, receipt, record, _ = self.retained_fixture()
        for raw in ('{broken', '[]', '{"producer": {}, "producer": {}}'):
            receipt.write_text(raw)
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                peer.product_identity(exe, build, product, receipt, harness, root)
        receipt.unlink()
        with self.assertRaises(FileNotFoundError):
            peer.product_identity(exe, build, product, receipt, harness, root)
        target = receipt.with_name('other.json')
        target.write_text(json.dumps(record))
        with self.assertRaises((ValueError, FileNotFoundError)):
            peer.product_identity(exe, build, product, target, harness, root)
        try:
            receipt.symlink_to(target)
        except OSError:
            return  # Windows may deny unprivileged symlink creation.
        with self.assertRaises(ValueError):
            peer.product_identity(exe, build, product, receipt, harness, root)

    def test_retained_executable_and_paths_must_match_authenticated_manifest(self):
        root, exe, build, product, harness, receipt, _, archive = self.retained_fixture()
        with self.assertRaises(ValueError):
            peer.product_identity(exe, root / 'other-build', product, receipt, harness, root)
        exe.write_bytes(b'changed retained executable')
        with self.assertRaises(ValueError):
            peer.product_identity(exe, build, product, receipt, harness, root)
        with zipfile.ZipFile(archive, 'w') as out:
            out.writestr(peer.staging_inputs.MANIFEST, b'{}')
        with self.assertRaises(ValueError):
            peer.product_identity(exe, build, product, receipt, harness, root)

    def test_only_exact_binary_identity_is_accepted(self):
        expected = "a" * 40
        peer.require_build({"build_commit": expected}, expected)
        for diagnostic in ({}, {"build_commit": ""}, {"build_commit": "b" * 40},
                           {"build_commit": expected[:8]}):
            with self.subTest(diagnostic=diagnostic), self.assertRaises(AssertionError):
                peer.require_build(diagnostic, expected)

    def test_windows_refuses_non_disposable_hosts(self):
        for env in ({}, {"GITHUB_ACTIONS": "true"},
                    {"GITHUB_ACTIONS": "true", "RUNNER_ENVIRONMENT": "self-hosted",
                     "RUNNER_TEMP": "unused"}):
            with patch.object(peer, "os", types.SimpleNamespace(name="nt", environ=env)):
                with self.assertRaises(RuntimeError):
                    peer.native_temp_root()

    def test_all_supported_outputs_reject_each_sentinel(self):
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary)
            outputs = [profile / name for name in ("stdout", "stderr", "opencpn.log")]
            for target in outputs:
                for secret in peer.SECRETS:
                    for output in outputs:
                        output.write_bytes(b"ordinary output\n")
                    target.write_bytes(b"before " + secret + b" after\n")
                    with self.subTest(output=target.name), self.assertRaises(AssertionError):
                        peer.assert_no_secrets(profile, outputs[0], outputs[1])
                    sanitized = peer.redact(target.read_bytes())
                    self.assertFalse(any(value in sanitized for value in peer.SECRETS))
                    self.assertIn(b"before [REDACTED] after", sanitized)
                    target.write_bytes(sanitized)
                    peer.assert_no_secrets(profile, outputs[0], outputs[1])
            outputs[0].unlink()
            with self.assertRaises(RuntimeError):
                peer.assert_no_secrets(profile, outputs[0], outputs[1])

    def test_malformed_startup_evidence_is_not_retried_as_a_race(self):
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary)
            (profile / "opencpn.log").write_text("OnInitTimer...Finalize Canvases")
            (profile / "opennav-diagnostics.json").write_text("{broken")
            process = Mock(pid=123)
            process.poll.return_value = None
            with self.assertRaises(json.JSONDecodeError):
                peer.wait_ready(process, profile)

    def test_terminated_process_cannot_supply_startup_evidence(self):
        process = Mock(pid=123, returncode=1)
        process.poll.return_value = 1
        with tempfile.TemporaryDirectory() as temporary, self.assertRaises(RuntimeError):
            peer.wait_ready(process, Path(temporary))


if __name__ == "__main__":
    unittest.main(verbosity=2)

#!/usr/bin/env python3
"""Fail-closed harness contracts; no application, display or peer is launched."""
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import Mock, patch


spec = importlib.util.spec_from_file_location(
    "peer_two", Path(__file__).with_name("smoke-peer-two-instances.py"))
peer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(peer)


class PeerTwoContracts(unittest.TestCase):
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

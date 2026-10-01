#!/usr/bin/env python3
"""Focused disposable-fixture tests for the peer-sharing boundary."""

from __future__ import annotations

import ctypes
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from peer_boundary import (  # noqa: E402
    PeerBoundary,
    PeerBoundaryError,
    _CLIENT_KEY,
    _KEY_BYTES,
    _CERT_BYTES,
    _SERVER_KEY,
    _Tcp6Row,
    _TcpRow,
    linux_listeners,
    windows_listeners,
)


ROOT = Path(__file__).resolve().parents[1]
PREPARE = ROOT / "tools" / "prepare-test-profile.py"


class PeerBoundaryTests(unittest.TestCase):
    def prepared_profile(self):
        temp = tempfile.TemporaryDirectory(prefix="peer-boundary-")
        root = Path(temp.name)
        build = root / "build"
        (build / "include").mkdir(parents=True)
        (build / "include" / "config.h").write_text(
            '#define VERSION_FULL "5.12.4-test"\n'
            '#define VERSION_DATE "2026-10-01"\n', encoding="utf-8"
        )
        profile = root / "profile"
        subprocess.run(
            [sys.executable, str(PREPARE), "--build", str(build),
             "--profile", str(profile)],
            check=True, capture_output=True, text=True,
        )
        self.addCleanup(temp.cleanup)
        return profile

    def test_constructor_seeds_and_preserves_actual_prepared_profile(self):
        profile = self.prepared_profile()
        self.assertEqual(
            (profile / "OPENNAV_TEST_PROFILE").read_bytes(),
            b"Disposable disconnected UI test.\n",
        )
        boundary = PeerBoundary(profile)
        self.assertEqual(boundary.assert_preserved(), {"profile_preserved": True})
        config = (profile / "opencpn.conf").read_text(encoding="utf-8")
        self.assertIn("[Settings/RESTClient]\nServerKeys=" + _CLIENT_KEY, config)
        self.assertIn("[Settings/RestServer]\nServerKeys=" + _SERVER_KEY, config)

    def test_existing_credential_and_certificate_refuse_without_mutation(self):
        profile = self.prepared_profile()
        config = (profile / "opencpn.conf").read_bytes()
        # The profile generator leaves this section absent, so this is one
        # existing key, rather than an intentionally ambiguous duplicate.
        (profile / "opencpn.conf").write_bytes(
            config + b"[Settings/RESTClient]\nServerKeys=preexisting\n"
        )
        before = (profile / "opencpn.conf").read_bytes()
        with self.assertRaises(PeerBoundaryError):
            PeerBoundary(profile)
        self.assertEqual((profile / "opencpn.conf").read_bytes(), before)

        profile = self.prepared_profile()
        cert = profile / "cert.pem"
        cert.write_bytes(b"existing certificate\n")
        before_config = (profile / "opencpn.conf").read_bytes()
        with self.assertRaises(PeerBoundaryError):
            PeerBoundary(profile)
        self.assertEqual(cert.read_bytes(), b"existing certificate\n")
        self.assertEqual((profile / "opencpn.conf").read_bytes(), before_config)

    def test_duplicate_credential_is_rejected(self):
        profile = self.prepared_profile()
        config = (profile / "opencpn.conf").read_bytes()
        config += b"[Settings/RESTClient]\nServerKeys=one\nServerKeys=two\n"
        (profile / "opencpn.conf").write_bytes(config)
        with self.assertRaisesRegex(PeerBoundaryError, "duplicate"):
            PeerBoundary(profile)

    def test_tampering_with_cert_key_or_credentials_fails_closed(self):
        for filename, replacement in (
            ("cert.pem", b"tampered certificate\n"),
            ("key.pem", b"tampered key\n"),
        ):
            with self.subTest(filename=filename):
                boundary = PeerBoundary(self.prepared_profile())
                (boundary.profile / filename).write_bytes(replacement)
                with self.assertRaises(PeerBoundaryError):
                    boundary.assert_preserved()

        boundary = PeerBoundary(self.prepared_profile())
        config = (boundary.profile / "opencpn.conf").read_text(encoding="utf-8")
        config = config.replace(
            "ServerKeys=" + _CLIENT_KEY, "ServerKeys=changed", 1
        )
        (boundary.profile / "opencpn.conf").write_text(config, encoding="utf-8")
        with self.assertRaises(PeerBoundaryError):
            boundary.assert_preserved()

    def test_native_row_layouts_are_explicit(self):
        self.assertEqual(ctypes.sizeof(_TcpRow), 24)
        self.assertEqual(ctypes.sizeof(_Tcp6Row), 56)

    def test_ipv6_listener_is_owned_and_rejected(self):
        if not socket.has_ipv6:
            self.skipTest("Host has no IPv6 support")
        listener = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        self.addCleanup(listener.close)
        listener.bind(("::1", 0))
        listener.listen()
        port = listener.getsockname()[1]
        self.assertIn(port, self._reader()(os.getpid()))
        boundary = PeerBoundary(self.prepared_profile())
        with patch("peer_boundary._PORTS", frozenset((port,))):
            with self.assertRaises(PeerBoundaryError):
                boundary.observe(os.getpid())

    @staticmethod
    def _reader():
        return windows_listeners if os.name == "nt" else linux_listeners

    def _child_listener(self, directory: Path):
        ready = directory / "listener.port"
        code = (
            "import socket,sys,time,os\n"
            "s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)\n"
            "s.bind(('127.0.0.1',0)); s.listen()\n"
            "with open(sys.argv[1]+'.new','w') as f: f.write(str(s.getsockname()[1]))\n"
            "os.replace(sys.argv[1]+'.new',sys.argv[1])\n"
            "time.sleep(30)\n"
        )
        child = subprocess.Popen(
            [sys.executable, "-c", code, str(ready)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        for _ in range(30):
            if ready.is_file():
                return child, int(ready.read_text())
            if child.poll() is not None:
                break
            time.sleep(0.1)
        child.terminate()
        child.wait(timeout=3)
        self.fail("bounded child listener startup failed")

    def test_current_platform_listener_ownership_and_closed_pid(self):
        profile = self.prepared_profile()
        boundary = PeerBoundary(profile)
        own = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        own.bind(("127.0.0.1", 0))
        own.listen()
        own_port = own.getsockname()[1]
        if own_port in (8443, 8444):
            own.close()
            self.skipTest("ephemeral port selected a protected peer port")
        child, child_port = self._child_listener(Path(profile).parent)
        try:
            reader = self._reader()
            own_ports = reader(os.getpid())
            self.assertIn(own_port, own_ports)
            self.assertNotIn(child_port, own_ports)
            self.assertIn(child_port, reader(child.pid))
            report = boundary.observe(os.getpid())
            self.assertIn(own_port, report["owned_tcp_listener_ports"])
            # Exercise rejection with a real owned socket without reserving
            # service ports or touching another process on the test host.
            with patch("peer_boundary._PORTS", frozenset((own_port,))):
                with self.assertRaises(PeerBoundaryError):
                    boundary.observe(os.getpid())
        finally:
            own.close()
            child.terminate()
            child.wait(timeout=3)
        with self.assertRaises(PeerBoundaryError):
            reader(child.pid)


if __name__ == "__main__":
    unittest.main(verbosity=2)

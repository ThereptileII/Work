#!/usr/bin/env python3
"""Prove the source artifact can recreate its exact patched curl test script."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest import mock
import zipfile

import source_package


ROOT = Path(__file__).resolve().parents[1]
LOCK = json.loads((ROOT / "tools/windows-curl.lock.json").read_text(encoding="utf-8"))
HELPER = ROOT / "tools/patch-curl-test-openssl.py"
SPEC = importlib.util.spec_from_file_location("patch_curl_test_openssl", HELPER)
assert SPEC and SPEC.loader
patcher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(patcher)


class CurlPatchSourcePackageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        if not hasattr(cls, "curl_archive"):
            raise RuntimeError("test runner must supply --archive for locked curl source")
        archive = cls.curl_archive
        if archive.is_symlink() or not archive.is_file():
            raise RuntimeError("--archive must be the regular locked curl source archive")
        digest = hashlib.sha256()
        size = 0
        with archive.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                size += len(block)
                digest.update(block)
        if size != LOCK["bytes"] or digest.hexdigest() != LOCK["sha256"]:
            raise RuntimeError("--archive differs from the locked curl 8.22.0 source")

    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="curl-source-package-")
        self.addCleanup(self.temporary.cleanup)
        self.checkout = Path(self.temporary.name) / "checkout"
        self.product = self.checkout / "opennav-x"
        self.product.mkdir(parents=True)
        self._git(self.checkout, "init", "-q")
        self._git(self.checkout, "config", "user.name", "Source Package Test")
        self._git(self.checkout, "config", "user.email", "source-package@example.invalid")
        self._write(self.checkout / ".github/workflows/opennav-baseline.yml", "name: fixture\n")
        self._write(self.product / ".gitignore", "build/\n")
        self._write(self.product / "tools/patch-curl-test-openssl.py", HELPER.read_bytes())
        for name in (
            "opencpn-5.12.4-xnav.patch",
            "opencpn-5.12.4-regression-tests.patch",
            "opencpn-5.12.4-ais-transport.patch",
            "opencpn-5.12.4-chart-presentation.patch",
            "opencpn-5.12.4-maintained-curl.patch",
            "opencpn-5.12.4-download-trust.patch",
            "opencpn-5.12.4-wxcurl-trust.patch",
            "opencpn-5.12.4-peer-response-buffer.patch",
            "opencpn-5.12.4-peer-unavailable.patch",
        ):
            self._write(self.product / "patches" / name, b"fixture patch bytes\n")
        self._git(self.checkout, "add", ".")
        self._git(self.checkout, "commit", "-qm", "source package fixture")
        self.commit = self._git(self.checkout, "rev-parse", "HEAD").strip()

        self.upstream = self.product / "build/integration-source"
        self.upstream.mkdir(parents=True)
        self._write(self.upstream / "upstream.cpp", "pinned fixture\n")
        self._git(self.upstream, "init", "-q")
        self._git(self.upstream, "config", "user.name", "Upstream Fixture")
        self._git(self.upstream, "config", "user.email", "upstream@example.invalid")
        self._git(self.upstream, "add", ".")
        self._git(self.upstream, "commit", "-qm", "pinned source fixture")
        pin = self._git(self.upstream, "rev-parse", "HEAD").strip()
        self.upstream_pin = mock.patch.object(source_package, "PINNED_UPSTREAM", pin)
        self.upstream_pin.start()
        self.addCleanup(self.upstream_pin.stop)

    @staticmethod
    def _write(path: Path, content: bytes | str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(content, str):
            path.write_text(content, encoding="utf-8")
        else:
            path.write_bytes(content)

    @staticmethod
    def _git(path: Path, *args: str) -> str:
        return subprocess.check_output(["git", "-C", str(path), *args], text=True)

    def test_packaged_original_archive_and_helper_recreate_patched_genserv(self) -> None:
        source_zip = self.checkout / "corresponding-source.zip"
        helper_hash = hashlib.sha256(HELPER.read_bytes()).hexdigest()
        archive_hash = LOCK["sha256"]
        references = source_package.create_source_archive(
            self.product,
            self.commit,
            source_zip,
            [{
                "archive": self.curl_archive,
                "path": "third-party-sources/" + LOCK["archive"],
                "sha256": archive_hash,
                "reference": {
                    "library": "curl",
                    "version": LOCK["version"],
                    "url": LOCK["url"],
                },
            }],
        )
        bundled_path = "third-party-sources/" + LOCK["archive"]
        helper_path = "opennav-x/tools/patch-curl-test-openssl.py"
        with zipfile.ZipFile(source_zip) as package:
            self.assertIsNone(package.testzip())
            package_reference = json.loads(package.read("SOURCE_REFERENCE.json"))
            self.assertEqual(package_reference, references)
            self.assertEqual(package.read(bundled_path), self.curl_archive.read_bytes())
            self.assertEqual(package.read(helper_path), HELPER.read_bytes())
            self.assertEqual(references["files"][bundled_path]["sha256"], archive_hash)
            self.assertEqual(references["files"][helper_path]["sha256"], helper_hash)
            bundled_record = references["bundledDependencySources"][0]
            self.assertEqual(bundled_record["path"], bundled_path)
            self.assertEqual(bundled_record["sha256"], archive_hash)
            self.assertEqual(bundled_record["bytes"], LOCK["bytes"])

            extracted = Path(self.temporary.name) / "artifact"
            helper_copy = extracted / helper_path
            packaged_archive = extracted / bundled_path
            helper_copy.parent.mkdir(parents=True)
            packaged_archive.parent.mkdir(parents=True)
            helper_copy.write_bytes(package.read(helper_path))
            packaged_archive.write_bytes(package.read(bundled_path))

        with tarfile.open(packaged_archive, "r:xz") as curl:
            member_name = f"curl-{LOCK['version']}/tests/certs/genserv.pl"
            member = curl.getmember(member_name)
            self.assertTrue(member.isfile())
            original = curl.extractfile(member).read()
        self.assertEqual(hashlib.sha256(original).hexdigest(), patcher.ORIGINAL_SHA256)

        generated_source = Path(self.temporary.name) / "curl/tests/certs/genserv.pl"
        generated_source.parent.mkdir(parents=True)
        generated_source.write_bytes(original)
        receipt_path = Path(self.temporary.name) / "genserv-patch-receipt.json"
        completed = subprocess.run(
            [sys.executable, str(helper_copy), "--source", str(generated_source),
             "--evidence", str(receipt_path)],
            check=True, capture_output=True, text=True,
        )
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        self.assertEqual(json.loads(completed.stdout), receipt)
        self.assertEqual(receipt["beforeSha256"], patcher.ORIGINAL_SHA256)
        self.assertEqual(receipt["afterSha256"], patcher.PATCHED_SHA256)
        self.assertEqual(hashlib.sha256(generated_source.read_bytes()).hexdigest(),
                         patcher.PATCHED_SHA256)


if __name__ == "__main__":
    cli = argparse.ArgumentParser(add_help=False)
    cli.add_argument("--archive", required=True, type=Path,
                     help="verified curl-8.22.0.tar.xz from the locked source cache")
    parsed, remaining = cli.parse_known_args()
    CurlPatchSourcePackageTests.curl_archive = parsed.archive
    sys.argv = [sys.argv[0], *remaining]
    unittest.main()

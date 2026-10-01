#!/usr/bin/env python3
"""Test the exact, hash-guarded curl executable and subprocess patch."""

from __future__ import annotations

import argparse
import importlib.util
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HELPER_PATH = ROOT / "tools/patch-curl-test-openssl.py"
SPEC = importlib.util.spec_from_file_location("patch_curl_test_openssl", HELPER_PATH)
assert SPEC and SPEC.loader
patcher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(patcher)


class PatchCurlTestOpenSSLTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        if not hasattr(cls, "source_fixture"):
            raise RuntimeError("test runner must supply --source from the locked curl archive")
        if cls.source_fixture.is_symlink() or not cls.source_fixture.is_file():
            raise RuntimeError("--source must be a regular locked curl tests/certs/genserv.pl")
        data = cls.source_fixture.read_bytes()
        if patcher._sha256(data) != patcher.ORIGINAL_SHA256:
            raise RuntimeError("--source does not match locked curl 8.22.0 genserv.pl bytes")

    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)
        self.source = self.work / "tests/certs/genserv.pl"
        self.source.parent.mkdir(parents=True)
        shutil.copyfile(self.source_fixture, self.source)

    def test_patches_only_the_locked_source_and_records_hashes(self) -> None:
        receipt = patcher.patch_source(self.source)
        self.assertEqual(receipt["state"], "patched")
        self.assertEqual(receipt["beforeSha256"], patcher.ORIGINAL_SHA256)
        self.assertEqual(receipt["afterSha256"], patcher.PATCHED_SHA256)
        data = self.source.read_bytes()
        self.assertIn(patcher.NEW_SELECTION, data)
        self.assertNotIn(patcher.OLD_SELECTION, data)
        self.assertEqual(data.count(patcher.NEW_SELECTION), 1)
        self.assertEqual(data.count(patcher.NEW_REDIR), 1)
        self.assertNotIn(patcher.OLD_REDIR, data)

    def test_exact_expected_source_is_idempotent(self) -> None:
        first = patcher.patch_source(self.source)
        before = self.source.read_bytes()
        second = patcher.patch_source(self.source)
        self.assertEqual(first["afterSha256"], patcher.PATCHED_SHA256)
        self.assertEqual(second["state"], "already-patched")
        self.assertEqual(second["beforeSha256"], patcher.PATCHED_SHA256)
        self.assertEqual(second["afterSha256"], patcher.PATCHED_SHA256)
        self.assertEqual(self.source.read_bytes(), before)

    def test_unexpected_or_partially_modified_source_is_refused_unchanged(self) -> None:
        self.source.write_bytes(self.source.read_bytes().replace(b"openssl 1.0.2+", b"openssl 1.0.1+"))
        before = self.source.read_bytes()
        with self.assertRaisesRegex(ValueError, "neither the locked original nor reviewed patch"):
            patcher.patch_source(self.source)
        self.assertEqual(self.source.read_bytes(), before)

    def test_symlink_source_is_refused(self) -> None:
        link = self.work / "linked-genserv.pl"
        link.symlink_to(self.source)
        with self.assertRaisesRegex(ValueError, "regular, non-symlink"):
            patcher.patch_source(link)

    @unittest.skipUnless(shutil.which("perl"), "Perl is needed for the subprocess regression")
    def test_redir_closes_input_and_handles_large_output_without_pipes(self) -> None:
        script = self.work / "redir-test.pl"
        script.write_bytes(
            b"use strict; use warnings; use File::Spec; use IPC::Open3;\n"
            + patcher.NEW_REDIR
            + b"redir('>output.bin', '2>', $^X, '-e', "
              b"'binmode STDOUT; binmode STDERR; exit 7 if defined(<STDIN>); "
              b"print STDOUT q(O) x 131072; print STDERR q(E) x 131072;');\n"
            + b"redir($^X, '-e', 'print STDOUT q(out); print STDERR q(err);');\n"
        )
        result = subprocess.run(
            ["perl", str(script)], cwd=self.work, capture_output=True,
            timeout=10, check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr.decode(errors="replace"))
        self.assertEqual((self.work / "output.bin").read_bytes(), b"O" * 131072)
        self.assertEqual(result.stdout, b"out")
        self.assertEqual(result.stderr, b"err")


if __name__ == "__main__":
    cli = argparse.ArgumentParser(add_help=False)
    cli.add_argument("--source", required=True, type=Path,
                     help="genserv.pl extracted from the locked curl 8.22.0 archive")
    parsed, remaining = cli.parse_known_args()
    PatchCurlTestOpenSSLTests.source_fixture = parsed.source
    sys.argv = [sys.argv[0], *remaining]
    unittest.main()

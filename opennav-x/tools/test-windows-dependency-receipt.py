#!/usr/bin/env python3
"""Adversarial tests for the same-job native dependency receipt boundary."""

from __future__ import annotations

import hashlib
import io
import json
import os
from pathlib import Path
import contextlib
import tempfile
import unittest
from unittest import mock

import windows_dependency_receipt as receipt


class DependencyReceiptTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="dependency-receipt-test-")
        self.base = Path(self.temp.name)
        self.root = self.base / "workspace"
        self.root.mkdir()
        self.prefix = self.root / "build" / "dependency"
        self.prefix.mkdir(parents=True)
        (self.prefix / "one.dll").write_bytes(b"native-one")
        (self.prefix / "one.lib").write_bytes(b"import-lib")
        digest = hashlib.sha256(b"input").hexdigest()
        self.context = {
            "runId": "36850600318",
            "runAttempt": "2",
            "job": "windows-dependencies",
            "githubSha": "a" * 40,
            "architecture": "Win32",
            "inputs": {"tools/build.ps1": digest},
            "toolchain": {"msvc": digest},
        }
        self.roots = ["build/dependency"]
        self.receipt_path = self.base / "receipt.json"

    def tearDown(self):
        self.temp.cleanup()

    def capture(self):
        receipt.capture(self.root, self.receipt_path, self.context, self.roots)

    def verify(self, *, root=None, context=None, roots=None):
        receipt.verify(root or self.root, self.receipt_path,
                       context if context is not None else self.context,
                       roots if roots is not None else self.roots)

    def test_capture_verify_round_trip_binds_context_and_all_file_bytes(self):
        self.capture()
        document = json.loads(self.receipt_path.read_text())
        expected_paths = {"build/dependency/one.dll", "build/dependency/one.lib"}
        self.assertEqual(set(document["files"]), expected_paths)
        for relative in expected_paths:
            actual = (self.root / relative).read_bytes()
            self.assertEqual(document["files"][relative], {
                "bytes": len(actual), "sha256": hashlib.sha256(actual).hexdigest()
            })
        self.verify()

    def test_verification_rejects_changed_missing_and_added_files(self):
        self.capture()
        (self.prefix / "one.dll").write_bytes(b"changed bytes")
        with self.assertRaises(receipt.ReceiptError):
            self.verify()

        (self.prefix / "one.dll").write_bytes(b"native-one")
        (self.prefix / "one.lib").unlink()
        with self.assertRaises(receipt.ReceiptError):
            self.verify()

        (self.prefix / "one.lib").write_bytes(b"import-lib")
        (self.prefix / "extra.pdb").write_bytes(b"unlisted output")
        with self.assertRaises(receipt.ReceiptError):
            self.verify()

    def test_verification_rejects_a_different_workspace_even_with_identical_bytes(self):
        self.capture()
        other = self.base / "copied-workspace"
        (other / "build" / "dependency").mkdir(parents=True)
        for name in ("one.dll", "one.lib"):
            (other / "build" / "dependency" / name).write_bytes(
                (self.prefix / name).read_bytes())
        with self.assertRaises(receipt.ReceiptError):
            self.verify(root=other)

    def test_capture_refuses_symlinked_root_or_ancestor(self):
        self.capture()
        root_link = self.base / "root-link"
        root_link.symlink_to(self.root, target_is_directory=True)
        with self.assertRaises((receipt.ReceiptError, OSError)):
            receipt.capture(root_link, self.receipt_path, self.context, self.roots)

        parent_link = self.base / "ancestor-link"
        parent_link.symlink_to(self.base, target_is_directory=True)
        aliased_root = parent_link / "workspace"
        with self.assertRaises((receipt.ReceiptError, OSError)):
            receipt.capture(aliased_root, self.receipt_path, self.context, self.roots)

    def test_capture_and_verify_refuse_symlinks_inside_inventory(self):
        outside = self.base / "outside.dll"
        outside.write_bytes(b"must not be inventoried through a link")
        link = self.prefix / "linked.dll"
        try:
            link.symlink_to(outside)
        except (OSError, NotImplementedError) as error:
            self.skipTest(f"symlinks unavailable: {error}")
        with self.assertRaises(receipt.ReceiptError):
            self.capture()

        link.unlink()
        self.capture()
        link.symlink_to(outside)
        with self.assertRaises(receipt.ReceiptError):
            self.verify()

    def test_verify_refuses_a_symlinked_receipt(self):
        self.capture()
        target = self.base / "real-receipt.json"
        self.receipt_path.rename(target)
        try:
            self.receipt_path.symlink_to(target)
        except (OSError, NotImplementedError) as error:
            self.skipTest(f"symlinks unavailable: {error}")
        with self.assertRaises(receipt.ReceiptError):
            self.verify()

    def test_receipt_paths_and_roots_reject_escape_and_windows_aliases(self):
        self.capture()
        document = json.loads(self.receipt_path.read_text())
        document["files"]["../outside"] = document["files"].pop("build/dependency/one.lib")
        self.receipt_path.write_text(json.dumps(document))
        with self.assertRaises(receipt.ReceiptError):
            self.verify()

        for unsafe_roots in (["../outside"], ["C:/outside"],
                             ["build/dependency", "build/dependency."]):
            with self.subTest(roots=unsafe_roots):
                with self.assertRaises(receipt.ReceiptError):
                    receipt.capture(self.root, self.receipt_path, self.context,
                                    sorted(unsafe_roots))

        # Root identity validation rejects case-only aliases before consulting
        # the filesystem. Windows cannot materialize a second directory with
        # different case, so this must stay a pure path-list assertion.
        with self.assertRaises(receipt.ReceiptError):
            receipt.capture(self.root, self.receipt_path, self.context,
                            ["build/DEPENDENCY", "build/dependency"])

        alias_dir = self.root / "case-folded-files"
        alias_dir.mkdir()
        lower = alias_dir / "same.dll"
        upper = alias_dir / "SAME.dll"
        lower.write_bytes(b"first")
        if not upper.exists():
            upper.write_bytes(b"different case alias")
        if lower.samefile(upper):
            # A Windows filesystem aliases these spellings to one file. Supply
            # both directory entries so _inventory's case-fold guard still
            # sees the collision on that platform.
            walker = mock.patch.object(
                receipt.os, "walk",
                return_value=iter([(str(alias_dir), [], ["same.dll", "SAME.dll"])]),
            )
        else:
            walker = mock.patch.object(receipt.os, "walk", wraps=receipt.os.walk)
        with walker, self.assertRaises(receipt.ReceiptError):
            receipt.capture(self.root, self.receipt_path, self.context,
                            ["case-folded-files"])

    def test_inventory_rejects_windows_stream_and_reserved_names(self):
        unsafe_names = ["stream:payload.dll", "trailing-dot.", "trailing-space ",
                        "CON.dll", "NUL"]
        for name in unsafe_names:
            with self.subTest(name=name):
                # These components are invalid on Windows and may be created
                # as alternate streams, normalized names or devices instead
                # of ordinary files. Inject only the directory listing; the
                # real inventory calls _plain_path, whose guard must reject
                # each raw name before any filesystem lookup.
                walker = mock.patch.object(
                    receipt.os, "walk",
                    return_value=iter([(str(self.prefix), [], [name])]),
                )
                with walker, self.assertRaises(receipt.ReceiptError):
                    receipt.capture(self.root, self.receipt_path, self.context,
                                    self.roots)

        # Verification must independently reject unsafe raw JSON keys, even
        # where the platform filesystem cannot represent those names.
        for relative in (
            "build/dependency/stream:payload.dll",
            "build/dependency/trailing-dot.",
            "build/dependency/trailing-space ",
            "build/dependency/CON.dll",
            "build/dependency/NUL",
        ):
            with self.subTest(receipt_path=relative):
                self.capture()
                document = json.loads(self.receipt_path.read_text())
                record = document["files"].pop("build/dependency/one.lib")
                document["files"][relative] = record
                self.receipt_path.write_text(json.dumps(document), encoding="utf-8")
                with self.assertRaises(receipt.ReceiptError):
                    self.verify()

    def test_receipt_rejects_malformed_unknown_and_duplicate_json_fields(self):
        self.capture()
        self.receipt_path.write_text("{not json", encoding="utf-8")
        with self.assertRaises((receipt.ReceiptError, ValueError)):
            self.verify()

        # Recreate a valid receipt before testing a well-formed but unknown field.
        self.capture()
        value = json.loads(self.receipt_path.read_text())
        value["unreviewed"] = True
        self.receipt_path.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaises(receipt.ReceiptError):
            self.verify()

    def test_cli_context_parser_rejects_duplicate_json_keys(self):
        context_path = self.base / "context.json"
        raw = json.dumps(self.context, separators=(",", ":"))
        raw = raw.replace('"runId":"36850600318"',
                          '"runId":"36850600318","runId":"36850600318"')
        context_path.write_text(raw, encoding="utf-8")
        errors = io.StringIO()
        with contextlib.redirect_stderr(errors):
            result = receipt.main([
                "capture", "--root", str(self.root), "--receipt",
                str(self.receipt_path), "--context", str(context_path),
                "--path", self.roots[0],
            ])
        self.assertEqual(result, 1)
        self.assertIn("duplicate JSON key", errors.getvalue())

        self.capture()
        raw = self.receipt_path.read_text(encoding="utf-8")
        raw = raw.replace('"schemaVersion":1', '"schemaVersion":1,"schemaVersion":1', 1)
        self.receipt_path.write_text(raw, encoding="utf-8")
        with self.assertRaises(receipt.ReceiptError):
            self.verify()

    def test_context_must_be_complete_canonical_and_from_same_job(self):
        invalid = []
        missing = dict(self.context)
        missing.pop("inputs")
        invalid.append(missing)
        unknown = dict(self.context)
        unknown["extra"] = "no"
        invalid.append(unknown)
        bad_digest = dict(self.context)
        bad_digest["inputs"] = {"tools/build.ps1": "A" * 64}
        invalid.append(bad_digest)
        bad_run = dict(self.context)
        bad_run["runId"] = "invalid/run"
        invalid.append(bad_run)
        bad_attempt = dict(self.context)
        bad_attempt["runAttempt"] = "0"
        invalid.append(bad_attempt)
        wrong_arch = dict(self.context)
        wrong_arch["architecture"] = "x64"
        invalid.append(wrong_arch)

        for context in invalid:
            with self.subTest(context=context):
                with self.assertRaises(receipt.ReceiptError):
                    receipt.capture(self.root, self.receipt_path, context, self.roots)

        self.capture()
        for field in ("runId", "runAttempt", "job", "githubSha"):
            changed = dict(self.context)
            changed[field] = "b" * 40 if field == "githubSha" else "other"
            with self.subTest(receipt_context=field):
                with self.assertRaises(receipt.ReceiptError):
                    self.verify(context=changed)

    def test_root_count_context_count_and_oversized_receipt_limits(self):
        for roots in ([f"path-{n}" for n in range(13)],):
            with self.assertRaises(receipt.ReceiptError):
                receipt.capture(self.root, self.receipt_path, self.context, roots)

        for field in ("inputs", "toolchain"):
            context = dict(self.context)
            context[field] = {f"file-{n}": "0" * 64 for n in range(129)}
            with self.subTest(context_inventory=field):
                with self.assertRaises(receipt.ReceiptError):
                    receipt.capture(self.root, self.receipt_path, context, self.roots)

        self.receipt_path.write_bytes(b" " * (receipt.MAX_RECEIPT_BYTES + 1))
        with self.assertRaises(receipt.ReceiptError):
            self.verify()

    def test_inventory_rejects_more_than_the_supported_file_count(self):
        # Create one beyond the documented ceiling. This checks that oversized
        # inventories fail closed without building a heavyweight native output.
        bulk = self.root / "bulk"
        bulk.mkdir()
        for index in range(receipt.MAX_FILES + 1):
            (bulk / f"f{index:05}.txt").touch()
        with self.assertRaises(receipt.ReceiptError):
            receipt.capture(self.root, self.receipt_path, self.context,
                            ["bulk", "build/dependency"])

    def test_inventory_bounds_total_entries_before_materializing_large_tree(self):
        bulk = self.root / "bounded-tree"
        bulk.mkdir()
        # Keep file count at MAX_FILES while putting total filesystem entries
        # (files + directories) above MAX_ENTRIES.
        for index in range(receipt.MAX_FILES):
            (bulk / f"f{index:05}.bin").touch()
        for index in range(receipt.MAX_ENTRIES - receipt.MAX_FILES):
            (bulk / f"d{index:05}").mkdir()
        with self.assertRaises(receipt.ReceiptError):
            receipt.capture(self.root, self.receipt_path, self.context,
                            ["bounded-tree"])

    def test_walk_errors_do_not_silently_create_partial_inventory(self):
        sentinel = self.root / "sentinel.dll"
        sentinel.write_bytes(b"independent inventoried file")

        def partial_walk(anchor, *args, **kwargs):
            onerror = kwargs.get("onerror")
            if onerror:
                onerror(PermissionError("simulated unreadable nested directory"))
            # Model os.walk's default behavior when no error callback is
            # supplied: it silently omits the unreadable dependency tree.
            return iter(())

        with mock.patch.object(receipt.os, "walk", side_effect=partial_walk):
            with self.assertRaises((receipt.ReceiptError, OSError)):
                receipt.capture(self.root, self.receipt_path, self.context,
                                ["build/dependency", "sentinel.dll"])


if __name__ == "__main__":
    unittest.main(verbosity=2)

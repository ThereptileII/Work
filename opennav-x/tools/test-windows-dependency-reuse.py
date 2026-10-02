#!/usr/bin/env python3
"""Refusal tests for the disabled same-job Windows dependency wrapper."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import tempfile
import unittest
from unittest import mock

import windows_dependency_evidence as evidence
import windows_dependency_reuse as reuse


class SameJobReuseReceiptTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="same-job-reuse-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.original = self.root / evidence.DEFAULT_ZLIB_SOURCE_VERIFICATION
        self.original.parent.mkdir(parents=True)
        self.original.write_text(json.dumps({"mode": "build", "status": "verified"}), encoding="utf-8")
        for name in reuse.INPUTS:
            if name == evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION:
                continue
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(("fixed " + name).encode())
        for name in reuse.PRODUCER_FACTS.values():
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
        for kind, name in reuse.PRODUCER_FACTS.items():
            (self.root / name).write_text(
                json.dumps({"schemaVersion": 1, "kind": kind, "tools": {"cl.exe": "fixture"}}),
                encoding="utf-8",
            )
        for name in evidence.PREFIXES.values():
            prefix = self.root / name
            prefix.mkdir(parents=True)
            (prefix / "output.dll").write_bytes(b"verified prefix bytes")
        self.environ = mock.patch.dict(os.environ, {
            "GITHUB_ACTIONS": "true", "GITHUB_RUN_ID": "1001", "GITHUB_RUN_ATTEMPT": "1",
            "GITHUB_JOB": "windows-integration", "GITHUB_SHA": "a" * 40,
        }, clear=True)
        self.environ.start()
        self.addCleanup(self.environ.stop)
        git = mock.patch.object(reuse.subprocess, "run", return_value=mock.Mock(stdout="a" * 40 + "\n"))
        git.start()
        self.addCleanup(git.stop)
        verifier = mock.patch.object(reuse.evidence, "verify_dependency_evidence", side_effect=self._evidence)
        self.verifier = verifier.start()
        self.addCleanup(verifier.stop)

    def _evidence(self, root, zlib_source_verification=evidence.DEFAULT_ZLIB_SOURCE_VERIFICATION):
        record = json.loads((Path(root) / zlib_source_verification).read_text(encoding="utf-8"))
        if record.get("mode") != "build" or record.get("status") != "verified":
            raise ValueError("zlib source evidence not from successful build")
        return {"manifests": {}, "logs": {}}

    def capture(self):
        reuse.capture_first_success(self.root, fixture_success=True)

    def test_first_success_round_trip_and_later_preflight_overwrite(self):
        self.capture()
        self.assertTrue((self.root / reuse.RECEIPT).is_file())
        self.assertEqual(self.verifier.call_count, 2)
        reuse.verify_same_job(self.root)
        self.original.write_text(json.dumps({"mode": "source-only", "status": "verified"}))
        reuse.verify_same_job(self.root)

    def test_zlib_facts_from_actual_producer_paths_are_consumed_and_bound(self):
        # Derive the fixture locations from the producer, independently of the
        # consumer map used by setUp. Refuse an unfamiliar declaration shape.
        producer = Path(__file__).with_name("build-zlib-windows.ps1").read_text()
        directories = re.findall(r"(?m)^\$Evidence = Join-Path \$Root '([^']+)'$", producer)
        self.assertEqual(len(directories), 1)
        for kind, variable in (("zlib-parent", "ParentFacts"), ("zlib-child", "ChildFacts")):
            (self.root / reuse.PRODUCER_FACTS[kind]).unlink()
            filenames = re.findall(
                rf"(?m)^\${variable} = Join-Path \$Evidence '([^']+)'$", producer)
            self.assertEqual(len(filenames), 1)
            relative = directories[0] + "/" + filenames[0]
            self.assertEqual(reuse.PRODUCER_FACTS[kind], relative)
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps({"schemaVersion": 1, "kind": kind}), encoding="utf-8")
        self.capture()
        reuse.verify_same_job(self.root)
        for kind in ("zlib-parent", "zlib-child"):
            with self.subTest(kind=kind):
                path = self.root / reuse.PRODUCER_FACTS[kind]
                original = path.read_bytes()
                try:
                    path.write_bytes(original + b" ")
                    with self.assertRaisesRegex(ValueError, "job identity"):
                        reuse.verify_same_job(self.root)
                finally:
                    path.write_bytes(original)

    def test_checkout_root_workflow_is_bound_by_commit_not_nested_path(self):
        workflow = ".github/workflows/opennav-baseline.yml"
        self.assertNotIn(workflow, reuse.INPUTS)
        self.assertFalse((self.root / workflow).exists())
        self.capture()
        reuse.verify_same_job(self.root)

    def test_missing_success_assertion_or_failed_evidence_never_writes_receipt(self):
        with self.assertRaisesRegex(ValueError, "fixture-gate confirmation"):
            reuse.capture_first_success(self.root, fixture_success=False)
        self.assertFalse((self.root / reuse.RECEIPT).exists())
        self.original.write_text(json.dumps({"mode": "source-only", "status": "verified"}))
        with self.assertRaisesRegex(ValueError, "successful build"):
            self.capture()
        self.assertFalse((self.root / reuse.RECEIPT).exists())
        self.assertFalse((self.root / evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION).exists())

    def test_changed_prefix_bytes_or_extra_file_refuse_reuse(self):
        self.capture()
        prefix = self.root / evidence.PREFIXES["curl"]
        (prefix / "output.dll").write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "inventory differs"):
            reuse.verify_same_job(self.root)
        (prefix / "output.dll").write_bytes(b"verified prefix bytes")
        (prefix / "extra.dll").write_bytes(b"unlisted")
        with self.assertRaisesRegex(ValueError, "inventory differs"):
            reuse.verify_same_job(self.root)

    def test_changed_producer_fact_log_or_input_refuses_reuse(self):
        self.capture()
        for relative in (
            next(iter(reuse.PRODUCER_FACTS.values())),
            "evidence/local/windows-curl-native-output.log",
            "tools/build-curl-windows.ps1",
            "tools/windows-curl-environment.ps1",
            "tools/windows-curl-import-layout.cmake",
            "tools/test-curl-source-preflight.ps1",
            "evidence/local/windows-curl-source-preflight/source-analysis.json",
            "evidence/local/windows-curl-source-preflight/test1119.stderr.txt",
            "build/dependency-downloads/zlib-1.3.2.tar.gz",
        ):
            with self.subTest(relative=relative):
                path = self.root / relative
                original = path.read_bytes()
                try:
                    path.write_bytes(original + b" ")
                    with self.assertRaisesRegex(ValueError, "job identity"):
                        reuse.verify_same_job(self.root)
                finally:
                    path.write_bytes(original)

    def test_missing_fact_and_tampered_preserved_source_refuse_reuse(self):
        self.capture()
        preserved = self.root / evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION
        preserved.write_text(json.dumps({"mode": "source-only", "status": "verified"}))
        with self.assertRaisesRegex(ValueError, "job identity|inventory differs"):
            reuse.verify_same_job(self.root)
        preserved.write_text(json.dumps({"mode": "build", "status": "verified"}))
        (self.root / next(iter(reuse.PRODUCER_FACTS.values()))).unlink()
        with self.assertRaises((ValueError, OSError)):
            reuse.verify_same_job(self.root)

    def test_different_job_attempt_commit_and_non_ci_context_refuse(self):
        self.capture()
        for field, value in (("GITHUB_RUN_ATTEMPT", "2"), ("GITHUB_RUN_ID", "1002"),
                             ("GITHUB_SHA", "b" * 40), ("GITHUB_JOB", "other-job"),
                             ("GITHUB_ACTIONS", "false")):
            with self.subTest(field=field), mock.patch.dict(os.environ, {field: value}):
                with self.assertRaises(ValueError):
                    reuse.verify_same_job(self.root)

    def test_capture_never_replaces_existing_receipt_or_preserved_source(self):
        self.capture()
        with self.assertRaisesRegex(ValueError, "already exists"):
            self.capture()

    def test_late_capture_failure_removes_preserved_copy_without_receipt(self):
        fact = self.root / next(iter(reuse.PRODUCER_FACTS.values()))
        fact.write_text(json.dumps({"schemaVersion": True, "kind": "openssl-parent"}))
        with self.assertRaisesRegex(ValueError, "producer-time native tool facts"):
            self.capture()
        self.assertFalse((self.root / evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION).exists())
        fact.unlink()
        with self.assertRaises((ValueError, OSError)):
            self.capture()
        self.assertFalse((self.root / evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION).exists())
        self.assertFalse((self.root / reuse.RECEIPT).exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)

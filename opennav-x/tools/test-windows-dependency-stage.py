#!/usr/bin/env python3
"""Fixed cache-map and refusal tests for same-job Windows restaging."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import windows_dependency_receipt as receipt
import windows_dependency_stage as stage


class StageTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="dependency-stage-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        files = {}
        names = set()
        for kind, subdir in stage.HEADER_TREES:
            names.add(f"{stage.PREFIX[kind]}/{subdir}/nested/test.h")
            names.add(f"{stage.PREFIX[kind]}/{subdir}/public.h")
        names.update(f"{stage.PREFIX[kind]}/{source}" for kind, source, _ in stage.FILES)
        for name in sorted(names):
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            data = name.encode()
            path.write_bytes(data)
            files[name] = {"bytes": len(data), "sha256": receipt._digest(path)}
        self.expected = files
        receipt_path = self.root / stage.reuse.RECEIPT
        receipt_path.parent.mkdir(parents=True, exist_ok=True)
        receipt_path.write_text(json.dumps({"files": files}), encoding="utf-8")
        self.cache = self.root / stage.CACHE
        (self.cache / "include/openssl").mkdir(parents=True)
        (self.cache / "include/openssl/stale.h").write_bytes(b"stock")
        (self.cache / "include/curl").mkdir(parents=True)
        (self.cache / "include/curl/stale.h").write_bytes(b"stock")
        (self.cache / "ssleay32.dll").write_bytes(b"stock legacy")
        (self.cache / "unrelated.bin").write_bytes(b"keep")
        verifier = mock.patch.object(stage.reuse, "verify_same_job")
        self.verify = verifier.start()
        self.addCleanup(verifier.stop)

    def test_restage_exact_headers_files_and_manifests(self):
        stage.stage_verified_cache(self.root)
        self.verify.assert_called_once()
        for kind, subdir in stage.HEADER_TREES:
            source = f"{stage.PREFIX[kind]}/{subdir}"
            target = f"{stage.CACHE}/{subdir}"
            self.assertEqual(
                {name.replace(source, target, 1): record
                 for name, record in self.expected.items() if name.startswith(source + "/")},
                receipt._inventory(self.root, [target]),
            )
            self.assertFalse((self.root / target / "stale.h").exists())
        for kind, source, destination in stage.FILES:
            self.assertEqual((self.root / stage.PREFIX[kind] / source).read_bytes(),
                             (self.cache / destination).read_bytes())
        self.assertFalse((self.cache / "ssleay32.dll").exists())
        self.assertEqual((self.cache / "unrelated.bin").read_bytes(), b"keep")

    def test_missing_or_changed_receipt_source_refuses(self):
        source = self.root / stage.PREFIX["curl"] / "include/curl/public.h"
        source.write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "header source inventory differs"):
            stage.stage_verified_cache(self.root)
        self.assertTrue((self.cache / "include/openssl/stale.h").exists())

    def test_receipt_refusal_and_fixed_file_change_leave_cache_unstaged(self):
        self.verify.side_effect = ValueError("same-job identity differs")
        with self.assertRaisesRegex(ValueError, "same-job identity"):
            stage.stage_verified_cache(self.root)
        self.assertTrue((self.cache / "include/openssl/stale.h").exists())
        self.verify.side_effect = None
        (self.root / stage.PREFIX["openssl"] / "lib/libssl.lib").write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "fixed source inventory differs"):
            stage.stage_verified_cache(self.root)
        self.assertTrue((self.cache / "include/openssl/stale.h").exists())

    def test_legacy_cache_link_and_copy_growth_refuse_without_temp_file(self):
        (self.cache / "include/openssl/stale.h").unlink()
        (self.cache / "include/openssl/stale.h").symlink_to(self.cache / "unrelated.bin")
        with self.assertRaisesRegex(ValueError, "unsafe existing cache entry"):
            stage.stage_verified_cache(self.root)
        (self.cache / "include/openssl/stale.h").unlink()
        name = f"{stage.PREFIX['zlib']}/include/zlib.h"
        self.expected[name]["bytes"] = 1
        with self.assertRaisesRegex(ValueError, "source grew"):
            stage._copy_record(self.root, self.expected, name, f"{stage.CACHE}/include/zlib.h")
        self.assertFalse(list(self.cache.rglob(".xnav-stage-*")))


if __name__ == "__main__":
    unittest.main()

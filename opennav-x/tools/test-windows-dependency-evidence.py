#!/usr/bin/env python3
"""Owned inert fixtures for retained Windows dependency build evidence."""

from __future__ import annotations

from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest import mock

import curl_package
import openssl_package
import windows_dependency_evidence as evidence


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def pe(payload: bytes) -> bytes:
    data = bytearray(128 + len(payload))
    data[:2] = b"MZ"
    struct.pack_into("<I", data, 0x3C, 64)
    data[64:68] = b"PE\0\0"
    struct.pack_into("<H", data, 68, 0x14C)
    data[128:] = payload
    return bytes(data)


class WindowsDependencyEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="windows-dependency-evidence-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)

        self.build = self.root / "build"
        self.installed = self.build / "xnav-install"
        self.cache = self.build / "dependency-downloads"
        self.installed.mkdir(parents=True)
        self.cache.mkdir()
        self.prefixes = {
            "openssl": self.build / "windows-openssl-3.5.9" / "install",
            "zlib": self.build / "windows-zlib-1.3.2" / "install",
            "curl": self.build / "windows-curl-8.22.0" / "install",
        }
        for prefix in self.prefixes.values():
            prefix.mkdir(parents=True)

        self.openssl_archive = b"inert OpenSSL archive fixture\n"
        self.openssl_source = self.cache / "openssl-3.5.9.tar.gz"
        self.openssl_source.write_bytes(self.openssl_archive)
        self.zlib_archive = b"inert zlib archive fixture\n"
        self.curl_archive = b"inert curl archive fixture\n"

        # Patch only upstream source identities so the real package validators
        # can exercise their complete schema/output/provenance checks without
        # downloading or claiming qualification from production archives.
        self.openssl_pin = {
            "version": "3.5.9",
            "configuration": "VC-WIN32 shared",
            "archive": self.openssl_source.name,
            "url": "https://example.invalid/openssl-3.5.9.tar.gz",
            "sha256": digest(self.openssl_archive),
            "bytes": len(self.openssl_archive),
            "signingPrimaryFingerprint": "OPENSSL-FIXTURE",
            "buildTools": {
                "nasm": {
                    "version": "3.02", "archive": "nasm-fixture.zip",
                    "url": "https://example.invalid/nasm-fixture.zip",
                    "sha256": "1" * 64, "bytes": 10, "provenance": "inert test fixture",
                }
            },
        }
        for attr, value in (
            ("EXPECTED_SOURCE_URL", self.openssl_pin["url"]),
            ("EXPECTED_SOURCE_SHA256", self.openssl_pin["sha256"]),
            ("EXPECTED_SOURCE_BYTES", self.openssl_pin["bytes"]),
            ("EXPECTED_SIGNING_FINGERPRINT", self.openssl_pin["signingPrimaryFingerprint"]),
        ):
            self.stack.enter_context(mock.patch.object(openssl_package, attr, value))

        self.curl_sources = {}
        for name, archive, version, configuration, fingerprint in (
            ("curl", "curl-8.22.0.tar.xz", "8.22.0", "Win32 shared OpenSSL", "CURL-FIXTURE"),
            ("zlib", "zlib-1.3.2.tar.gz", "1.3.2", "Win32 shared", "ZLIB-FIXTURE"),
        ):
            content = self.curl_archive if name == "curl" else self.zlib_archive
            self.curl_sources[name] = {
                "version": version,
                "configuration": configuration,
                "archive": archive,
                "url": f"https://example.invalid/{archive}",
                "sha256": digest(content),
                "bytes": len(content),
                "signingPrimaryFingerprint": fingerprint,
            }
            (self.cache / archive).write_bytes(content)
        self.stack.enter_context(mock.patch.object(curl_package, "SOURCES", self.curl_sources))

        tools_root = self.root / "tools"
        tools_root.mkdir()
        patch_helper = Path(__file__).with_name("patch-curl-test-openssl.py")
        self.patch_helper_bytes = patch_helper.read_bytes()
        (tools_root / patch_helper.name).write_bytes(self.patch_helper_bytes)
        self._write_json(tools_root / "windows-openssl.lock.json", self.openssl_pin)
        for lock_name in ("windows-zlib.lock.json", "windows-curl.lock.json"):
            source_lock = json.loads((Path(__file__).with_name(lock_name)).read_text())
            package_name = "zlib" if "zlib" in lock_name else "curl"
            source_lock.update(self.curl_sources[package_name])
            self._write_json(tools_root / lock_name, source_lock)
        self._write_notices()
        self._write_outputs_and_manifests()
        self._write_logs()

    @staticmethod
    def _write_json(path: Path, value, *, bom=False):
        path.parent.mkdir(parents=True, exist_ok=True)
        data = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
        path.write_bytes((b"\xef\xbb\xbf" if bom else b"") + data)

    @staticmethod
    def _record(data: bytes):
        return {"sha256": digest(data), "bytes": len(data)}

    def _write_notices(self):
        open_notice = self.root / "docs/third-party/OpenSSL-3.5.9"
        open_notice.mkdir(parents=True)
        open_license = b"inert OpenSSL license fixture\n"
        (open_notice / "LICENSE.txt").write_bytes(open_license)
        self._write_json(open_notice / "provenance.json", {
            "library": "OpenSSL 3.5.9",
            "sourceArchiveSha256": self.openssl_pin["sha256"],
            "sourceArchiveBytes": self.openssl_pin["bytes"],
            "licenseSha256": digest(open_license),
            "signatureVerification": {
                "primaryFingerprint": self.openssl_pin["signingPrimaryFingerprint"]
            },
        })
        for library, pin in self.curl_sources.items():
            notice = self.root / "docs/third-party" / f"{library}-{pin['version']}"
            notice.mkdir(parents=True)
            license_bytes = (library + " fixture license\n").encode()
            (notice / "LICENSE.txt").write_bytes(license_bytes)
            self._write_json(notice / "provenance.json", {
                "library": library + " " + pin["version"],
                "sourceArchive": pin["url"],
                "sourceArchiveSha256": pin["sha256"],
                "sourceArchiveBytes": pin["bytes"],
                "signatureVerification": {
                    "primary_fingerprint": pin["signingPrimaryFingerprint"]
                },
                "licenseSha256": digest(license_bytes),
            })

    def _write_outputs_and_manifests(self):
        openssl_prefix = self.prefixes["openssl"]
        openssl_outputs = {}
        openssl_contents = {}
        for relative, content in (
            ("include/openssl/opensslv.h", b"fixture OPENSSL_VERSION 3.5.9\n"),
            ("lib/libssl.lib", b"fixture libssl import library"),
            ("lib/libcrypto.lib", b"fixture libcrypto import library"),
            ("bin/openssl.exe", pe(b"openssl tool")),
            ("bin/libssl-3.dll", pe(b"openssl ssl")),
            ("bin/libcrypto-3.dll", pe(b"openssl crypto")),
        ):
            openssl_contents[relative] = content
            openssl_outputs[relative] = self._record(content)
            output_path = openssl_prefix / relative
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_bytes(content)
        openssl_cache_paths = {
            "include/openssl/opensslv.h": "include/openssl/opensslv.h",
            "lib/libssl.lib": "libssl.lib",
            "lib/libcrypto.lib": "libcrypto.lib",
            "bin/libssl-3.dll": "libssl-3.dll",
            "bin/libcrypto-3.dll": "libcrypto-3.dll",
        }
        openssl_cache = {
            cache_name: {"source": relative, **openssl_outputs[relative]}
            for relative, cache_name in openssl_cache_paths.items()
        }
        self.openssl_manifest = {
            "schemaVersion": 1,
            "library": "OpenSSL",
            "version": self.openssl_pin["version"],
            "configuration": self.openssl_pin["configuration"],
            "architecture": "Win32",
            "abi": "x86",
            "source": {key: self.openssl_pin[key] for key in (
                "url", "archive", "sha256", "bytes", "signingPrimaryFingerprint")},
            "outputs": openssl_outputs,
            "cacheBuildwin": openssl_cache,
            "toolchain": {
                "compiler": "inert fixture", "nasmArchiveSha256": self.openssl_pin["buildTools"]["nasm"]["sha256"]
            },
            "buildSteps": {step: "passed" for step in ("configure", "compile", "test", "install")},
            "versionOutput": "OpenSSL 3.5.9 fixture",
        }

        zlib_prefix = self.prefixes["zlib"]
        zlib_outputs = {}
        for relative, content in (
            ("include/zlib.h", b"fixture ZLIB_VERSION 1.3.2\n"),
            ("include/zconf.h", b"fixture zconf"),
            ("lib/zlib1.lib", b"fixture zlib import library"),
            ("bin/zlib1.dll", pe(b"zlib")),
        ):
            zlib_outputs[relative] = self._record(content)
            output_path = zlib_prefix / relative
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_bytes(content)
        zlib_pin = self.curl_sources["zlib"]
        self.zlib_manifest = {
            "schemaVersion": 1, "library": "zlib", "version": zlib_pin["version"],
            "configuration": zlib_pin["configuration"], "architecture": "Win32", "abi": "x86",
            "runtime": "MultiThreadedDLL (/MD)",
            "source": {key: zlib_pin[key] for key in curl_package.SOURCE_KEYS},
            "buildSteps": {step: "passed" for step in ("configure", "compile", "test", "install")},
            "outputs": zlib_outputs,
        }

        curl_prefix = self.prefixes["curl"]
        curl_outputs = {}
        for relative in sorted(curl_package.CURL_OUTPUTS):
            content = pe(b"libcurl") if relative == "bin/libcurl.dll" else (
                b"curl import library" if relative == "lib/libcurl.lib" else relative.encode())
            curl_outputs[relative] = self._record(content)
            output_path = curl_prefix / relative
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_bytes(content)

        self._write_json(openssl_prefix / "openssl-build.json", self.openssl_manifest)
        self._write_json(zlib_prefix / "zlib-build.json", self.zlib_manifest)
        self._write_json(curl_prefix / "curl-build.json", {})

        self._write_json(self.installed / "openssl-build.json", self.openssl_manifest)
        self._write_json(self.installed / "zlib-build.json", self.zlib_manifest)
        for name, relative in (
            ("libssl-3.dll", "bin/libssl-3.dll"),
            ("libcrypto-3.dll", "bin/libcrypto-3.dll"),
        ):
            (self.installed / name).write_bytes(openssl_contents[relative])
        (self.installed / "zlib1.dll").write_bytes(pe(b"zlib"))
        curl_pin = self.curl_sources["curl"]
        self.curl_manifest = {
            "schemaVersion": 1, "library": "curl", "version": curl_pin["version"],
            "configuration": curl_pin["configuration"], "architecture": "Win32", "abi": "x86",
            "runtime": "MultiThreadedDLL (/MD)",
            "source": {key: curl_pin[key] for key in curl_package.SOURCE_KEYS},
            "buildSteps": {
                "configure": "passed", "compile": "passed", "test": "passed", "install": "passed",
                "testTarget": "tests", "testsReported": 4, "testsPassed": 4,
                "log": "evidence/local/windows-curl-native-output.log",
                "logSha256": "0" * 64,
                "certificatePatch": {
                    **curl_package.CURL_CERTIFICATE_PATCH,
                    "helperSha256": digest(self.patch_helper_bytes),
                },
                "certificateTool": {
                    "path": str(self.prefixes["openssl"] / "bin/openssl.exe"),
                    "sha256": openssl_outputs["bin/openssl.exe"]["sha256"],
                    "bytes": openssl_outputs["bin/openssl.exe"]["bytes"],
                    "versionOutput": self.openssl_manifest["versionOutput"],
                },
                "certificateProbe": "passed",
            },
            "outputs": curl_outputs,
            "dependencies": {
                "openssl": {"version": "3.5.9", "manifestSha256": "", "prefix": str(self.prefixes["openssl"])},
                "zlib": {"version": "1.3.2", "manifestSha256": "", "prefix": str(self.prefixes["zlib"])},
            },
            "options": ["inert test configuration"],
            "versionOutput": "curl 8.22.0 fixture",
            "importOutput": "libssl-3.dll\nlibcrypto-3.dll\nzlib1.dll",
            "cacheBuildwin": {},
        }
        for library, manifest in (("openssl", self.openssl_manifest), ("zlib", self.zlib_manifest)):
            path = self.installed / f"{library}-build.json"
            self.curl_manifest["dependencies"][library]["manifestSha256"] = digest(path.read_bytes())
        self.curl_manifest["cacheBuildwin"] = {
            Path(name).name if name.startswith(("bin/", "lib/")) else name:
            {"source": name, **record} for name, record in curl_outputs.items()
        }
        self._write_json(self.installed / "curl-build.json", self.curl_manifest)
        self._write_json(curl_prefix / "curl-build.json", self.curl_manifest)
        (self.installed / "libcurl.dll").write_bytes(pe(b"libcurl"))
        # Package validation reads all declared output records from manifests;
        # the verifier separately checks each producer-prefix output file.

    def _write_logs(self):
        self.logs = {
            "openssl": "All tests successful. Files=1, Tests=3, 0 wallclock secs\nResult: PASS\n",
            "zlib": "100% tests passed, 0 tests failed out of 3\n",
            "curl": "TESTDONE: 4 tests out of 4 reported OK:\n",
        }
        log_paths = {
            "openssl": self.root / "evidence/local/windows-openssl-native-output.log",
            "zlib": self.root / "evidence/local/windows-zlib-1.3.2/windows-zlib-native-output.log",
            "curl": self.root / "evidence/local/windows-curl-native-output.log",
        }
        for name, text in self.logs.items():
            log_paths[name].parent.mkdir(parents=True, exist_ok=True)
            log_paths[name].write_text(text, encoding="utf-8")
        self.curl_manifest["buildSteps"]["logSha256"] = digest(
            log_paths["curl"].read_bytes())
        self._write_json(self.installed / "curl-build.json", self.curl_manifest)
        self._write_json(self.prefixes["curl"] / "curl-build.json", self.curl_manifest)
        source_record = {
            "schemaVersion": 1, "mode": "build", "status": "verified",
            "archive": self.curl_sources["zlib"]["archive"],
            "expected": {"bytes": self.curl_sources["zlib"]["bytes"],
                         "sha256": self.curl_sources["zlib"]["sha256"]},
            "observed": {"exists": True, "bytes": self.curl_sources["zlib"]["bytes"],
                         "sha256": self.curl_sources["zlib"]["sha256"]},
        }
        self._write_json(
            self.root / "evidence/local/windows-zlib-1.3.2/source-verification.json",
            source_record,
        )

    def verify(self, zlib_source_verification=evidence.DEFAULT_ZLIB_SOURCE_VERIFICATION):
        return evidence.verify_dependency_evidence(self.root, zlib_source_verification)

    def _preserve_zlib_build_source(self):
        original = self.root / evidence.DEFAULT_ZLIB_SOURCE_VERIFICATION
        preserved = self.root / evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION
        preserved.write_bytes(original.read_bytes())
        return original, preserved

    def _write_manifest_copies(self, library, manifest):
        prefix = self.prefixes[library]
        self._write_json(prefix / f"{library}-build.json", manifest)
        self._write_json(self.installed / f"{library}-build.json", manifest)

    def test_complete_inert_fixture_matches_producer_and_installed_evidence(self):
        result = self.verify()
        self.assertEqual(set(result["manifests"]), {"openssl", "zlib", "curl"})
        self.assertEqual(set(result["logs"]), {"openssl", "zlib", "curl", "zlibSource"})
        for library in ("openssl", "zlib", "curl"):
            self.assertEqual(result["manifests"][library],
                             digest((self.installed / f"{library}-build.json").read_bytes()))

    def test_preserved_first_success_record_survives_later_source_only_preflight(self):
        original, preserved = self._preserve_zlib_build_source()
        verified_hash = self.verify(evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION)["logs"]["zlibSource"]
        self.assertEqual(verified_hash, digest(preserved.read_bytes()))
        preflight = json.loads(original.read_text())
        preflight["mode"] = "source-only"
        self._write_json(original, preflight)
        with self.assertRaisesRegex(ValueError, "source verification"):
            self.verify()
        self.assertEqual(
            self.verify(evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION)["logs"]["zlibSource"],
            verified_hash,
        )

    def test_alternate_zlib_record_requires_pinned_build_evidence_and_plain_path(self):
        _, preserved = self._preserve_zlib_build_source()
        original = preserved.read_bytes()
        for field, value in (("mode", "source-only"), ("status", "rejected"),
                             ("archive", "other.tar.gz")):
            with self.subTest(field=field):
                changed = json.loads(original)
                changed[field] = value
                self._write_json(preserved, changed)
                with self.assertRaisesRegex(ValueError, "source verification"):
                    self.verify(evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION)
        preserved.write_bytes(original)
        changed = json.loads(original)
        changed["observed"]["sha256"] = "0" * 64
        self._write_json(preserved, changed)
        with self.assertRaisesRegex(ValueError, "source verification"):
            self.verify(evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION)
        preserved.unlink()
        with self.assertRaises((ValueError, OSError)):
            self.verify(evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION)
        try:
            preserved.symlink_to(self.root / evidence.DEFAULT_ZLIB_SOURCE_VERIFICATION)
        except (OSError, NotImplementedError) as error:
            self.skipTest(f"symlinks unavailable: {error}")
        with self.assertRaises((ValueError, OSError)):
            self.verify(evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION)

    def test_alternate_zlib_record_path_is_fixed_and_cannot_escape(self):
        self._preserve_zlib_build_source()
        for unsafe in ("../first-success-source-verification.json",
                       "evidence/local/windows-zlib-1.3.2/../source-verification.json",
                       "evidence/local/windows-zlib-1.3.2/first-success-source-verification.json:stream"):
            with self.subTest(path=unsafe):
                with self.assertRaisesRegex(ValueError, "unreviewed zlib source-verification path"):
                    self.verify(unsafe)

    def test_changed_prefix_output_is_rejected(self):
        path = self.prefixes["curl"] / "bin/libcurl.dll"
        path.write_bytes(path.read_bytes() + b"changed")
        with self.assertRaisesRegex(ValueError, "changed dependency file"):
            self.verify()

    def test_mismatched_or_missing_producer_manifest_is_rejected(self):
        path = self.prefixes["zlib"] / "zlib-build.json"
        path.write_bytes(path.read_bytes() + b" ")
        with self.assertRaisesRegex(ValueError, "producer manifest differs"):
            self.verify()
        path.unlink()
        with self.assertRaises((ValueError, OSError)):
            self.verify()

    def test_strict_json_rejects_duplicate_keys_and_accepts_utf8_bom(self):
        # A duplicate key must not let the permissive package readers and the
        # producer evidence checker disagree about which value was attested.
        duplicate = b'{"schemaVersion":1,"schemaVersion":1}'
        originals = {}
        for path in (
            self.prefixes["openssl"] / "openssl-build.json",
            self.installed / "openssl-build.json",
        ):
            originals[path] = path.read_bytes()
            path.write_bytes(duplicate)
        with self.assertRaises((ValueError, OSError)):
            self.verify()
        for path, original in originals.items():
            path.write_bytes(original)

        # Windows PowerShell may emit UTF-8 JSON with a BOM. Both exact copies
        # should remain valid when the strict reader handles that encoding.
        self._write_json(
            self.prefixes["curl"] / "curl-build.json", self.curl_manifest, bom=True)
        self._write_json(self.installed / "curl-build.json", self.curl_manifest, bom=True)
        self.assertEqual(set(self.verify()["manifests"]), {"openssl", "zlib", "curl"})

    def test_changed_curl_dependency_manifest_hash_is_rejected(self):
        changed = json.loads((self.installed / "curl-build.json").read_text())
        changed["dependencies"]["openssl"]["manifestSha256"] = "f" * 64
        self._write_manifest_copies("curl", changed)
        with self.assertRaisesRegex(ValueError, "linked dependency manifest"):
            self.verify()

    def test_manifest_source_and_retained_zlib_source_record_must_match(self):
        changed = json.loads((self.installed / "zlib-build.json").read_text())
        changed["source"]["sha256"] = "e" * 64
        self._write_manifest_copies("zlib", changed)
        # Also update curl's linked digest so the test reaches the package
        # source-identity check rather than failing on a stale manifest hash.
        curl = json.loads((self.installed / "curl-build.json").read_text())
        curl["dependencies"]["zlib"]["manifestSha256"] = digest(
            (self.installed / "zlib-build.json").read_bytes())
        self._write_manifest_copies("curl", curl)
        with self.assertRaises(ValueError):
            self.verify()

        # Restore valid manifests, then make only the retained source-check
        # receipt disagree with the pinned zlib archive.
        self._write_manifest_copies("zlib", self.zlib_manifest)
        curl["dependencies"]["zlib"]["manifestSha256"] = digest(
            (self.installed / "zlib-build.json").read_bytes())
        self._write_manifest_copies("curl", curl)
        source_path = self.root / "evidence/local/windows-zlib-1.3.2/source-verification.json"
        source = json.loads(source_path.read_text())
        source["observed"]["sha256"] = "d" * 64
        self._write_json(source_path, source)
        with self.assertRaisesRegex(ValueError, "source verification"):
            self.verify()

    def test_missing_or_failed_native_test_summaries_are_rejected(self):
        log_paths = {
            "openssl": self.root / "evidence/local/windows-openssl-native-output.log",
            "zlib": self.root / "evidence/local/windows-zlib-1.3.2/windows-zlib-native-output.log",
            "curl": self.root / "evidence/local/windows-curl-native-output.log",
        }
        for name, path in log_paths.items():
            with self.subTest(missing_log=name):
                original = path.read_bytes()
                path.unlink()
                with self.assertRaises((ValueError, OSError)):
                    self.verify()
                path.write_bytes(original)

        for name, failed in (
            ("openssl", "All tests successful. Files=1, Tests=0, 0 wallclock secs\nResult: PASS\n"),
            ("openssl", "All tests successful. Files=1, Tests=3, 0 wallclock secs\nResult: FAIL\n"),
            ("zlib", "100% tests passed, 0 tests failed out of 0\n"),
            ("zlib", "67% tests passed, 1 tests failed out of 3\n"),
        ):
            with self.subTest(failed_summary=name, contents=failed):
                path = log_paths[name]
                original = path.read_bytes()
                path.write_text(failed, encoding="utf-8")
                with self.assertRaises(ValueError):
                    self.verify()
                path.write_bytes(original)

    def test_openssl_allows_only_the_source_proven_non_fips_prep_notests(self):
        path = self.root / "evidence/local/windows-openssl-native-output.log"
        original = path.read_bytes()
        source_proven = (
            "00-prep_fipsmodule_cnf.t .. skipped: FIPS module config file only supported in a fips build\n"
            "Files=1, Tests=0, 1 wallclock secs (0.02 usr + 0.00 sys = 0.02 CPU)\n"
            "Result: NOTESTS\n"
            "01-test_abort.t ......................... ok\n"
            "All tests successful.\n"
            "Files=347, Tests=4283, 1455 wallclock secs (5.42 usr + 0.92 sys = 6.34 CPU)\n"
            "Result: PASS\n"
        )
        path.write_text(source_proven, encoding="utf-8")
        self.verify()

        rejected = (
            # Unknown prep test or reason cannot be treated as harmless.
            source_proven.replace("00-prep_fipsmodule_cnf.t", "00-prep_other.t"),
            source_proven.replace("FIPS module config file only supported in a fips build",
                                  "another skipped test"),
            # NOTESTS remains unacceptable for the main suite or when repeated.
            source_proven.replace("Files=347, Tests=4283", "Files=1, Tests=0")
                        .replace("Result: PASS", "Result: NOTESTS"),
            source_proven.replace("Result: PASS\n", "Result: NOTESTS\nResult: PASS\n"),
            # A second harmless-looking NOTESTS record is still unaccounted for.
            source_proven.replace(
                "01-test_abort.t ......................... ok\n",
                "00-prep_fipsmodule_cnf.t .. skipped: FIPS module config file only supported in a fips build\n"
                "Files=1, Tests=0, 1 wallclock secs (0.02 usr + 0.00 sys = 0.02 CPU)\n"
                "Result: NOTESTS\n"
                "01-test_abort.t ......................... ok\n",
            ),
            # The allowed prep must occur before the passing main suite.
            source_proven.replace(
                "00-prep_fipsmodule_cnf.t .. skipped: FIPS module config file only supported in a fips build\n"
                "Files=1, Tests=0, 1 wallclock secs (0.02 usr + 0.00 sys = 0.02 CPU)\n"
                "Result: NOTESTS\n",
                "",
            ) + "00-prep_fipsmodule_cnf.t .. skipped: FIPS module config file only supported in a fips build\n"
                "Files=1, Tests=0, 1 wallclock secs (0.02 usr + 0.00 sys = 0.02 CPU)\n"
                "Result: NOTESTS\n",
        )
        for index, contents in enumerate(rejected):
            with self.subTest(case=index):
                path.write_text(contents, encoding="utf-8")
                with self.assertRaises(ValueError):
                    self.verify()
        path.write_bytes(original)

    def test_contradictory_failure_summaries_are_rejected_in_either_order(self):
        log_paths = {
            "openssl": self.root / "evidence/local/windows-openssl-native-output.log",
            "zlib": self.root / "evidence/local/windows-zlib-1.3.2/windows-zlib-native-output.log",
        }
        contradictory = {
            "openssl": (
                "All tests successful. Files=1, Tests=3, 0 wallclock secs\nResult: PASS\n"
                "Result: FAIL\n",
                "Result: FAIL\nAll tests successful. Files=1, Tests=3, 0 wallclock secs\n"
                "Result: PASS\n",
            ),
            "zlib": (
                "100% tests passed, 0 tests failed out of 3\n"
                "67% tests passed, 1 tests failed out of 3\n",
                "67% tests passed, 1 tests failed out of 3\n"
                "100% tests passed, 0 tests failed out of 3\n",
            ),
        }
        for name, cases in contradictory.items():
            path = log_paths[name]
            original = path.read_bytes()
            for contents in cases:
                with self.subTest(log=name, contents=contents):
                    path.write_text(contents, encoding="utf-8")
                    with self.assertRaises(ValueError):
                        self.verify()
            path.write_bytes(original)

    def test_curl_retained_log_hash_and_test_summary_must_match(self):
        path = self.root / "evidence/local/windows-curl-native-output.log"
        original = path.read_bytes()
        path.write_text("TESTDONE: 4 tests out of 4 reported OK:\nextra\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "test log differs"):
            self.verify()

        # When the manifest attests the changed log bytes, the exact nonzero
        # upstream summary is still independently required.
        invalid = "TESTDONE: 0 tests out of 0 reported OK:\n"
        path.write_text(invalid, encoding="utf-8")
        changed = json.loads((self.installed / "curl-build.json").read_text())
        changed["buildSteps"]["logSha256"] = digest(path.read_bytes())
        self._write_manifest_copies("curl", changed)
        with self.assertRaisesRegex(ValueError, "successful upstream execution|test summary"):
            self.verify()

        path.write_bytes(original)


if __name__ == "__main__":
    unittest.main(verbosity=2)

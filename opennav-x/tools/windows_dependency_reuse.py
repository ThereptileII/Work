#!/usr/bin/env python3
"""Capture or check a same-job dependency receipt; never restage or reuse here.

The CI caller must invoke capture only after its complete fixture-side gates
have succeeded. Verification is a prerequisite, not authorization to skip the
native producer tool reprobe, cache checks, or later product/package gates.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

import windows_dependency_evidence as evidence
import windows_dependency_receipt as receipt

RECEIPT = "evidence/local/windows-dependency-first-success-receipt.json"
PRODUCER_FACTS = {
    "openssl-parent": "evidence/local/windows-openssl-parent-tool-facts.json",
    "openssl-child": "evidence/local/windows-openssl-child-tool-facts.json",
    "zlib-parent": "evidence/local/windows-zlib-parent-tool-facts.json",
    "zlib-child": "evidence/local/windows-zlib-child-tool-facts.json",
    "curl-parent": "evidence/local/windows-curl-parent-tool-facts.json",
}
PATCHES = (
    "opencpn-5.12.4-xnav.patch",
    "opencpn-5.12.4-regression-tests.patch",
    "opencpn-5.12.4-ais-transport.patch",
    "opencpn-5.12.4-chart-presentation.patch",
    "opencpn-5.12.4-maintained-curl.patch",
    "opencpn-5.12.4-download-trust.patch",
    "opencpn-5.12.4-wxcurl-trust.patch",
    "opencpn-5.12.4-peer-response-buffer.patch",
    "opencpn-5.12.4-peer-unavailable.patch",
)
# In the published checkout, this module runs from opennav-x while the workflow
# lives at the repository root. The workflow bytes are already bound by the
# verified GITHUB_SHA/checkout HEAD pair; resolving .github beneath `root`
# would point at a nonexistent file and crossing above `root` would weaken the
# receipt's workspace path boundary.
INPUTS = tuple(sorted((
    "tools/build-pristine-windows.ps1",
    "tools/build-openssl-windows.ps1",
    "tools/build-zlib-windows.ps1",
    "tools/build-curl-windows.ps1",
    "tools/windows-curl-environment.ps1",
    "tools/test-curl-source-preflight.ps1",
    "tools/patch-curl-test-openssl.py",
    "tools/windows-native-tool-facts.ps1",
    "tools/windows-native-tool-facts.cmake",
    "tools/windows_dependency_reuse.py",
    "tools/windows_dependency_stage.py",
    "tools/windows_dependency_receipt.py",
    "tools/windows_dependency_evidence.py",
    "tools/openssl_package.py",
    "tools/curl_package.py",
    "tools/prepare-integration.py",
    "tools/verify-upstream.py",
    "tools/test-zlib-source-verification.ps1",
    "tools/windows-openssl.lock.json",
    "tools/windows-zlib.lock.json",
    "tools/windows-curl.lock.json",
    "tools/windows-wx.lock.json",
    "upstream.lock.json",
    "build/integration-source/buildwin/win_deps.bat",
    "build/xnav-install/openssl-build.json",
    "build/xnav-install/zlib-build.json",
    "build/xnav-install/curl-build.json",
    "build/dependency-downloads/openssl-3.5.9.tar.gz",
    "build/dependency-downloads/zlib-1.3.2.tar.gz",
    "build/dependency-downloads/curl-8.22.0.tar.xz",
    "evidence/local/windows-openssl-native-output.log",
    "evidence/local/windows-zlib-1.3.2/windows-zlib-native-output.log",
    "evidence/local/windows-curl-native-output.log",
    "evidence/local/windows-curl-source-preflight/source-analysis.json",
    "evidence/local/windows-curl-source-preflight/test1119.stdout.txt",
    "evidence/local/windows-curl-source-preflight/test1119.stderr.txt",
    "evidence/local/windows-curl-source-preflight/test1167.stdout.txt",
    "evidence/local/windows-curl-source-preflight/test1167.stderr.txt",
    evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION,
    *(f"patches/{name}" for name in PATCHES),
    *(f"docs/third-party/{name}/{file}" for name in (
        "OpenSSL-3.5.9", "zlib-1.3.2", "curl-8.22.0")
      for file in ("LICENSE.txt", "provenance.json")),
)))
ROOTS = sorted((
    *evidence.PREFIXES.values(),
    evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION,
))


def _job_environment(root: Path) -> dict[str, str]:
    if os.environ.get("GITHUB_ACTIONS") != "true":
        raise ValueError("same-job dependency receipt requires GitHub Actions")
    values = {
        "runId": os.environ.get("GITHUB_RUN_ID", ""),
        "runAttempt": os.environ.get("GITHUB_RUN_ATTEMPT", ""),
        "job": os.environ.get("GITHUB_JOB", ""),
        "githubSha": os.environ.get("GITHUB_SHA", ""),
        "architecture": "Win32",
    }
    if values["job"] != "windows-integration":
        raise ValueError("dependency receipt is restricted to the Windows integration job")
    head = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "HEAD"],
        check=True, capture_output=True, text=True,
    ).stdout.strip()
    if not re.fullmatch(r"[0-9a-f]{40}", head) or head != values["githubSha"]:
        raise ValueError("GitHub commit does not match checkout HEAD")
    return values


def _file_hashes(root: Path, names: tuple[str, ...]) -> dict[str, str]:
    return {name: receipt._digest(receipt._plain_path(root, name)) for name in names}


def _context(root: Path) -> dict:
    values = _job_environment(root)
    facts = {}
    for kind, name in PRODUCER_FACTS.items():
        path = receipt._plain_path(root, name)
        value = evidence._strict_json(path)
        if (not isinstance(value, dict) or type(value.get("schemaVersion")) is not int or
                value["schemaVersion"] != 1 or value.get("kind") != kind):
            raise ValueError(f"producer-time native tool facts missing or mismatched: {kind}")
        facts[name] = receipt._digest(path)
    values["inputs"] = _file_hashes(root, INPUTS)
    values["toolchain"] = facts
    return receipt._context(values)


def _receipt_path(root: Path) -> Path:
    return root / RECEIPT


def capture_first_success(root: Path, *, fixture_success: bool) -> None:
    """Capture only after the CI fixture build and later fixture gates passed."""
    if not fixture_success:
        raise ValueError("explicit successful fixture-gate confirmation required")
    root = receipt._workspace_root(Path(root))
    _job_environment(root)
    destination = _receipt_path(root)
    if destination.exists() or destination.is_symlink():
        raise ValueError("first-success dependency receipt already exists")
    # The producer's original mode=build record must pass before preserving it.
    evidence.verify_dependency_evidence(root)
    original = receipt._plain_path(root, evidence.DEFAULT_ZLIB_SOURCE_VERIFICATION)
    preserved = root / evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION
    if preserved.exists() or preserved.is_symlink():
        raise ValueError("first-success zlib source record already exists")
    try:
        with original.open("rb") as source, preserved.open("xb") as target:
            shutil.copyfileobj(source, target)
        evidence.verify_dependency_evidence(root, evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION)
        context = _context(root)
        receipt.capture(root, destination, context, ROOTS)
    except BaseException:
        preserved.unlink(missing_ok=True)
        raise


def verify_same_job(root: Path) -> None:
    """Verify stored bytes and current source evidence; native reprobe is separate."""
    root = receipt._workspace_root(Path(root))
    context = _context(root)
    receipt.verify(root, _receipt_path(root), context, ROOTS)
    evidence.verify_dependency_evidence(root, evidence.FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("capture", "verify"))
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--fixture-success", action="store_true",
                        help="explicit CI assertion that all fixture gates succeeded")
    args = parser.parse_args(argv)
    try:
        if args.mode == "capture":
            capture_first_success(args.root, fixture_success=args.fixture_success)
        elif args.fixture_success:
            raise ValueError("--fixture-success applies only to capture")
        else:
            verify_same_job(args.root)
    except (OSError, ValueError, TypeError, KeyError, subprocess.CalledProcessError) as error:
        print(f"Windows dependency reuse receipt rejected: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Check existing native producer evidence before any same-job reuse decision.

This is read-only and is not called by the Windows workflow yet. A later
orchestrator must bind these exact log bytes and tool inputs to the receipt.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

from curl_package import verify_curl_package_inputs, verify_file
from openssl_package import _sha256, verify_openssl_package_inputs
from windows_dependency_receipt import _plain_path, _unique_pairs, _workspace_root

PREFIXES = {
    "openssl": "build/windows-openssl-3.5.9/install",
    "zlib": "build/windows-zlib-1.3.2/install",
    "curl": "build/windows-curl-8.22.0/install",
}
MAX_LOG_BYTES = 32 * 1024 * 1024
MAX_JSON_BYTES = 8 * 1024 * 1024
DEFAULT_ZLIB_SOURCE_VERIFICATION = "evidence/local/windows-zlib-1.3.2/source-verification.json"
FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION = (
    "evidence/local/windows-zlib-1.3.2/first-success-source-verification.json"
)
ALLOWED_ZLIB_SOURCE_VERIFICATION = frozenset({
    DEFAULT_ZLIB_SOURCE_VERIFICATION, FIRST_SUCCESS_ZLIB_SOURCE_VERIFICATION,
})


def _strict_json(path: Path, *, with_digest: bool = False):
    if not path.is_file() or path.is_symlink() or path.stat().st_size > MAX_JSON_BYTES:
        raise ValueError(f"missing or oversized producer JSON: {path}")
    with path.open("rb") as stream:
        data = stream.read(MAX_JSON_BYTES + 1)
    if len(data) > MAX_JSON_BYTES:
        raise ValueError(f"oversized producer JSON: {path}")
    try:
        value = json.loads(data.decode("utf-8-sig"), object_pairs_hook=_unique_pairs)
    except (UnicodeError, RecursionError, json.JSONDecodeError) as error:
        raise ValueError(f"malformed producer JSON: {path}") from error
    if not isinstance(value, dict):
        raise ValueError(f"producer JSON is not an object: {path}")
    return (value, hashlib.sha256(data).hexdigest()) if with_digest else value


def _log(root: Path, relative: str) -> tuple[str, str]:
    path = _plain_path(root, relative)
    if not path.is_file() or path.stat().st_size > MAX_LOG_BYTES:
        raise ValueError(f"missing or oversized native test log: {relative}")
    with path.open("rb") as stream:
        data = stream.read(MAX_LOG_BYTES + 1)
    if len(data) > MAX_LOG_BYTES:
        raise ValueError(f"oversized native test log: {relative}")
    encoding = "utf-16" if data.startswith((b"\xff\xfe", b"\xfe\xff")) else "utf-8-sig"
    # Windows PowerShell and Python text files can retain CRLF. Parse line
    # boundaries consistently while binding the receipt to the original bytes.
    return data.decode(encoding).replace("\r\n", "\n"), hashlib.sha256(data).hexdigest()


def _require_upstream_tests(root: Path, manifests: dict,
                            zlib_source_verification: str) -> dict[str, str]:
    openssl_text, openssl_hash = _log(root, "evidence/local/windows-openssl-native-output.log")
    # OpenSSL 3.5.9 runs mandatory 00-prep recipes separately before the main
    # TAP suite (test/run_tests.pl). A non-FIPS build legitimately reports its
    # single FIPS-config prep as NOTESTS/zero tests, then runs the real suite.
    # Accept only that exact source-proven skipped recipe, plus one successful
    # nonzero main-suite result. Do not accept a NOTESTS main run or any other
    # result, even if a later summary says PASS.
    openssl_summaries = list(re.finditer(
        r"All tests successful\.[ \t]*(?:\r?\n[ \t]*)?Files=[ \t]*([0-9]+),[ \t]*Tests=[ \t]*([0-9]+),[^\r\n]*\r?\nResult:[ \t]*PASS[ \t]*$",
        openssl_text, flags=re.IGNORECASE | re.MULTILINE,
    ))
    openssl_fips_preps = list(re.finditer(
        r"^[ \t]*00-prep_fipsmodule_cnf\.t\s+\.\.\s+skipped: FIPS module config file only supported in a fips build"
        r"\s*\r?\nFiles=1,\s*Tests=0,[^\r\n]*\r?\nResult:\s*NOTESTS\s*$",
        openssl_text, flags=re.MULTILINE,
    ))
    openssl_results = re.findall(r"(?im)^\s*Result:\s*([A-Z]+)\s*$", openssl_text)
    valid_main = (len(openssl_summaries) == 1 and
                  all(int(openssl_summaries[0].group(i)) > 0 for i in (1, 2)))
    valid_fips_prep = (len(openssl_fips_preps) == 1 and valid_main and
                       openssl_fips_preps[0].end() < openssl_summaries[0].start())
    result_sequence_ok = (openssl_results == ["PASS"] or
                          (openssl_results == ["NOTESTS", "PASS"] and valid_fips_prep))
    if not valid_main or not result_sequence_ok:
        raise ValueError("OpenSSL retained log lacks one successful nonzero upstream test summary")

    zlib_text, zlib_hash = _log(root, "evidence/local/windows-zlib-1.3.2/windows-zlib-native-output.log")
    zlib_summaries = re.findall(
        r"([0-9]+)% tests passed,\s*([0-9]+) tests failed out of\s*([0-9]+)", zlib_text,
        flags=re.IGNORECASE,
    )
    if (len(zlib_summaries) != 1 or zlib_summaries[0][:2] != ("100", "0") or
            int(zlib_summaries[0][2]) <= 0):
        raise ValueError("zlib retained log lacks one successful nonzero CTest summary")
    source, source_hash = _strict_json(
        _plain_path(root, zlib_source_verification),
        with_digest=True,
    )
    zlib_manifest = manifests["zlib"]
    expected = {key: zlib_manifest["source"][key] for key in ("bytes", "sha256")}
    if (set(source) != {"schemaVersion", "mode", "status", "archive", "expected", "observed"} or
            type(source["schemaVersion"]) is not int or source["schemaVersion"] != 1 or
            source["mode"] != "build" or source["status"] != "verified" or
            source["archive"] != zlib_manifest["source"]["archive"] or
            source["expected"] != expected or
            source["observed"] != {"exists": True, **expected}):
        raise ValueError("zlib retained source verification does not match the reviewed archive")

    curl_text, curl_hash = _log(root, "evidence/local/windows-curl-native-output.log")
    curl_steps = manifests["curl"]["buildSteps"]
    if curl_hash != curl_steps["logSha256"]:
        raise ValueError("curl retained upstream test log differs from its producer manifest")
    curl_summaries = re.findall(
        r"(?m)^\s*(?:[0-9]+>)?\s*TESTDONE:\s*([0-9]+) tests out of\s*([0-9]+) reported OK:",
        curl_text,
    )
    expected_curl = (curl_steps["testsPassed"], curl_steps["testsReported"])
    if len(curl_summaries) != 1 or tuple(map(int, curl_summaries[0])) != expected_curl:
        raise ValueError("curl retained upstream test summary differs from its producer manifest")
    return {"openssl": openssl_hash, "zlib": zlib_hash, "curl": curl_hash,
            "zlibSource": source_hash}


def verify_dependency_evidence(
    root: Path,
    zlib_source_verification: str = DEFAULT_ZLIB_SOURCE_VERIFICATION,
) -> dict:
    """Return verified manifest and log digests; raise on incomplete evidence.

    The flat integrated install is deliberately checked with the existing
    packaging boundary. Producer prefix files are then checked independently
    against the exact same manifest bytes. The caller still needs to verify
    toolchain identity and bind all evidence to the same GitHub job receipt.
    """
    if zlib_source_verification not in ALLOWED_ZLIB_SOURCE_VERIFICATION:
        raise ValueError("unreviewed zlib source-verification path")
    root = _workspace_root(Path(root))
    installed = _plain_path(root, "build/xnav-install")
    source_cache = _plain_path(root, "build/dependency-downloads")
    # Package validators are intentionally reused below. Their JSON reader is
    # permissive about duplicate keys, so preflight exactly the same bounded
    # files with a strict parser before it sees any parsed values.
    for library, prefix_name in PREFIXES.items():
        _strict_json(_plain_path(root, prefix_name + f"/{library}-build.json"))
        _strict_json(_plain_path(root, f"build/xnav-install/{library}-build.json"))
    for name in ("windows-openssl.lock.json", "windows-zlib.lock.json", "windows-curl.lock.json"):
        _strict_json(_plain_path(root, "tools/" + name))
    for directory in ("OpenSSL-3.5.9", "zlib-1.3.2", "curl-8.22.0"):
        _strict_json(_plain_path(root, f"docs/third-party/{directory}/provenance.json"))
        _plain_path(root, f"docs/third-party/{directory}/LICENSE.txt")
    for archive in ("openssl-3.5.9.tar.gz", "zlib-1.3.2.tar.gz", "curl-8.22.0.tar.xz"):
        _plain_path(root, f"build/dependency-downloads/{archive}")
    openssl = verify_openssl_package_inputs(
        installed, root / "tools/windows-openssl.lock.json",
        source_cache / "openssl-3.5.9.tar.gz",
        root / "docs/third-party/OpenSSL-3.5.9",
    )
    curl = verify_curl_package_inputs(installed, source_cache, root / "docs/third-party")
    manifests = {"openssl": openssl["manifest"], **curl["manifests"]}
    manifest_hashes = {}
    for library, prefix_name in PREFIXES.items():
        prefix = _plain_path(root, prefix_name)
        producer_manifest = _plain_path(root, prefix_name + f"/{library}-build.json")
        installed_manifest = _plain_path(root, f"build/xnav-install/{library}-build.json")
        digest = _sha256(installed_manifest)
        if _sha256(producer_manifest) != digest:
            raise ValueError(f"{library} producer manifest differs from integrated install")
        manifest_hashes[library] = digest
        for relative, record in manifests[library]["outputs"].items():
            output = _plain_path(root, prefix_name + "/" + relative)
            verify_file(output, record)
    dependencies = manifests["curl"]["dependencies"]
    for library in ("openssl", "zlib"):
        if dependencies[library]["manifestSha256"] != manifest_hashes[library] or \
                Path(dependencies[library]["prefix"]) != root / PREFIXES[library]:
            raise ValueError(f"curl does not identify the verified {library} producer prefix")
    imports = manifests["curl"]["importOutput"]
    if not isinstance(imports, str) or any(
        not re.search(rf"(?im)^\s*{re.escape(name)}\s*$", imports)
        for name in ("libssl-3.dll", "libcrypto-3.dll", "zlib1.dll")
    ) or re.search(r"(?i)ssleay32\.dll|libeay32\.dll", imports):
        raise ValueError("curl retained import closure is incomplete or legacy")
    logs = _require_upstream_tests(root, manifests, zlib_source_verification)
    return {"manifests": manifest_hashes, "logs": logs}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument(
        "--zlib-source-verification", default=DEFAULT_ZLIB_SOURCE_VERIFICATION,
        choices=sorted(ALLOWED_ZLIB_SOURCE_VERIFICATION),
        help="reviewed relative location of the zlib build-source record",
    )
    args = parser.parse_args(argv)
    try:
        verify_dependency_evidence(args.root, args.zlib_source_verification)
    except (OSError, ValueError, TypeError, KeyError) as error:
        print(f"Windows dependency evidence rejected: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

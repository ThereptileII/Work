#!/usr/bin/env python3
"""Restore the checked dependency cache from a same-job receipt's prefixes.

This only stages the fixed Win32 consumer map. It does not grant reuse: the
caller must also reprobe live producer tools and run all downstream gates.
"""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import shutil
import stat
import sys
import tempfile

import windows_dependency_receipt as receipt
import windows_dependency_reuse as reuse

PREFIX = {
    "openssl": "build/windows-openssl-3.5.9/install",
    "zlib": "build/windows-zlib-1.3.2/install",
    "curl": "build/windows-curl-8.22.0/install",
}
CACHE = "build/integration-source/cache/buildwin"
HEADER_TREES = (("openssl", "include/openssl"), ("curl", "include/curl"))
FILES = (
    ("openssl", "lib/libssl.lib", "libssl.lib"),
    ("openssl", "lib/libcrypto.lib", "libcrypto.lib"),
    ("openssl", "bin/libssl-3.dll", "libssl-3.dll"),
    ("openssl", "bin/libcrypto-3.dll", "libcrypto-3.dll"),
    ("zlib", "include/zlib.h", "include/zlib.h"),
    ("zlib", "include/zconf.h", "include/zconf.h"),
    ("zlib", "lib/zlib1.lib", "zlib1.lib"),
    ("zlib", "bin/zlib1.dll", "zlib1.dll"),
    ("curl", "lib/libcurl.lib", "libcurl.lib"),
    ("curl", "bin/libcurl.dll", "libcurl.dll"),
    *((kind, f"{kind}-build.json", f"{kind}-build.json") for kind in PREFIX),
)


def _safe_destination(root: Path, relative: str) -> Path:
    path = root
    for part in receipt._relative(relative).parts:
        path /= part
        try:
            info = path.lstat()
        except FileNotFoundError:
            continue
        if (stat.S_ISLNK(info.st_mode) or
                getattr(info, "st_file_attributes", 0) & 0x400 or
                not (stat.S_ISDIR(info.st_mode) or stat.S_ISREG(info.st_mode))):
            raise ValueError(f"unsafe cache destination: {path}")
    return path


def _copy_record(root: Path, expected: dict, source_name: str, target_name: str) -> None:
    record = expected.get(source_name)
    if not isinstance(record, dict) or set(record) != {"bytes", "sha256"}:
        raise ValueError(f"source omitted from receipt: {source_name}")
    source = receipt._plain_path(root, source_name)
    if not source.is_file():
        raise ValueError(f"source is not a file: {source_name}")
    destination = _safe_destination(root, target_name)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=destination.parent, prefix=".xnav-stage-", delete=False) as stream:
        temporary = Path(stream.name)
    digest = hashlib.sha256()
    size = 0
    try:
        with temporary.open("wb") as output_file:
            with source.open("rb") as input_file:
                for chunk in iter(lambda: input_file.read(1024 * 1024), b""):
                    size += len(chunk)
                    if size > record["bytes"]:
                        raise ValueError(f"source grew after receipt verification: {source_name}")
                    digest.update(chunk)
                    output_file.write(chunk)
        if size != record["bytes"] or digest.hexdigest() != record["sha256"]:
            raise ValueError(f"source changed after receipt verification: {source_name}")
        _safe_destination(root, target_name)
        temporary.replace(destination)
        if destination.stat().st_size != size or receipt._digest(destination) != digest.hexdigest():
            raise ValueError(f"staged cache file differs: {target_name}")
    finally:
        temporary.unlink(missing_ok=True)


def _assert_plain_tree(path: Path) -> None:
    seen = 0
    def fail(error):
        raise error
    for directory, dirs, files in os.walk(path, followlinks=False, onerror=fail):
        for child in [directory, *(str(Path(directory) / name) for name in dirs + files)]:
            info = Path(child).lstat()
            if (stat.S_ISLNK(info.st_mode) or
                    getattr(info, "st_file_attributes", 0) & 0x400 or
                    not (stat.S_ISDIR(info.st_mode) or stat.S_ISREG(info.st_mode))):
                raise ValueError(f"unsafe existing cache entry: {child}")
            seen += 1
            if seen > receipt.MAX_ENTRIES:
                raise ValueError("existing header cache exceeds entry bound")


def stage_verified_cache(root: Path) -> None:
    root = receipt._workspace_root(Path(root))
    reuse.verify_same_job(root)
    document = receipt._read_json(root / reuse.RECEIPT)
    expected = document["files"]
    # The integrated source and stock win_deps must have completed already.
    receipt._plain_path(root, CACHE)
    # Refuse any missing or changed source before mutating even one cache path.
    header_records = {}
    for kind, subdir in HEADER_TREES:
        source_root = f"{PREFIX[kind]}/{subdir}"
        prefix = source_root + "/"
        source_records = {name: record for name, record in expected.items() if name.startswith(prefix)}
        if not source_records or receipt._inventory(root, [source_root]) != source_records:
            raise ValueError(f"header source inventory differs: {source_root}")
        header_records[(kind, subdir)] = source_records
    for kind, source_suffix, _ in FILES:
        source_name = f"{PREFIX[kind]}/{source_suffix}"
        record = expected.get(source_name)
        if (not isinstance(record, dict) or set(record) != {"bytes", "sha256"} or
                receipt._plain_path(root, source_name).stat().st_size != record["bytes"] or
                receipt._digest(root / source_name) != record["sha256"]):
            raise ValueError(f"fixed source inventory differs: {source_name}")
    for kind, subdir in HEADER_TREES:
        source_root = f"{PREFIX[kind]}/{subdir}"
        target_root = f"{CACHE}/{subdir}"
        prefix = source_root + "/"
        source_records = header_records[(kind, subdir)]
        target = _safe_destination(root, target_root)
        if target.exists():
            if not target.is_dir():
                raise ValueError(f"header cache destination is not a directory: {target_root}")
            _assert_plain_tree(target)
            shutil.rmtree(target)
        target.mkdir(parents=True)
        for source_name in source_records:
            target_name = f"{target_root}/{source_name[len(prefix):]}"
            _copy_record(root, expected, source_name, target_name)
        target_records = {
            f"{target_root}/{name[len(prefix):]}": record
            for name, record in source_records.items()
        }
        if receipt._inventory(root, [target_root]) != target_records:
            raise ValueError(f"staged header inventory differs: {target_root}")
    for kind, source_suffix, target_suffix in FILES:
        _copy_record(root, expected, f"{PREFIX[kind]}/{source_suffix}", f"{CACHE}/{target_suffix}")
    # Reject stale stock TLS files that could otherwise be selected by a
    # downstream consumer despite the maintained manifest's explicit paths.
    for name in ("libeay32.dll", "ssleay32.dll", "libeay32.lib", "ssleay32.lib"):
        path = _safe_destination(root, f"{CACHE}/{name}")
        if path.exists():
            if not path.is_file():
                raise ValueError(f"legacy TLS cache entry is not a file: {name}")
            path.unlink()
    print("Restaged verified Win32 dependency cache from same-job producer prefixes")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        stage_verified_cache(args.root)
    except (OSError, ValueError, TypeError, KeyError) as error:
        print(f"Windows dependency cache staging refused: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

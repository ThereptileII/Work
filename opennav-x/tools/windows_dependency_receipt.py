#!/usr/bin/env python3
"""Exact, same-job file receipt for the native Windows dependency closure.

This module only records and verifies bytes. The caller must separately prove
producer manifests, upstream tests, toolchain identity and import closure before
allowing reuse. In particular, a valid receipt is not a standalone cache key.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import sys
import tempfile

SCHEMA = 1
MAX_RECEIPT_BYTES = 8 * 1024 * 1024
MAX_FILES = 20000
MAX_ENTRIES = 40000
MAX_ROOTS = 12
SHA256 = re.compile(r"[0-9a-f]{64}\Z")
CONTEXT_KEYS = frozenset({
    "runId", "runAttempt", "job", "githubSha", "architecture",
    "inputs", "toolchain",
})
RECEIPT_KEYS = frozenset({"schemaVersion", "workspaceRoot", "context", "roots", "files"})
WINDOWS_DEVICES = re.compile(r"(?:CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])(?:\..*)?\Z", re.I)


class ReceiptError(ValueError):
    pass


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ReceiptError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _read_json(path: Path):
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode) or getattr(info, "st_file_attributes", 0) & 0x400 or info.st_size > MAX_RECEIPT_BYTES:
        raise ReceiptError(f"missing or oversized JSON: {path}")
    with path.open("rb") as stream:
        data = stream.read(MAX_RECEIPT_BYTES + 1)
    if len(data) > MAX_RECEIPT_BYTES:
        raise ReceiptError(f"oversized JSON: {path}")
    return json.loads(data.decode("utf-8"), object_pairs_hook=_unique_pairs)


def _digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _relative(value: str) -> PurePosixPath:
    if not isinstance(value, str) or not value or "\\" in value or "\x00" in value:
        raise ReceiptError("invalid relative path")
    path = PurePosixPath(value)
    if path.is_absolute() or str(path) != value or any(part in ("", ".", "..") for part in value.split("/")):
        raise ReceiptError(f"unsafe relative path: {value}")
    if len(value) > 4096 or any(
        ":" in part or part.endswith((".", " ")) or WINDOWS_DEVICES.fullmatch(part)
        for part in path.parts
    ):
        raise ReceiptError(f"unsafe relative path: {value}")
    return path


def _workspace_root(root: Path) -> Path:
    root = root.absolute()
    # Validate every component. A plain final directory may still sit below a
    # symlink or Windows junction, which would redirect the verified inventory.
    for path in (*reversed(root.parents), root):
        info = path.lstat()
        if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & 0x400:
            raise ReceiptError(f"workspace path contains link/reparse point: {path}")
        if not stat.S_ISDIR(info.st_mode):
            raise ReceiptError(f"workspace path is not a directory: {path}")
    return root


def _plain_path(root: Path, relative: str) -> Path:
    parts = _relative(relative).parts
    path = root
    for part in parts:
        path = path / part
        info = path.lstat()
        if stat.S_ISLNK(info.st_mode) or not (stat.S_ISREG(info.st_mode) or stat.S_ISDIR(info.st_mode)):
            raise ReceiptError(f"link or special path refused: {path}")
        # Windows directory junctions and other reparse points are not links on
        # every Python version. Refuse all reparse attributes explicitly.
        if getattr(info, "st_file_attributes", 0) & 0x400:
            raise ReceiptError(f"reparse path refused: {path}")
    return path


def _context(value):
    if not isinstance(value, dict) or set(value) != CONTEXT_KEYS:
        raise ReceiptError("context has missing or unexpected fields")
    for field in ("runId", "runAttempt"):
        if not isinstance(value[field], str) or not re.fullmatch(r"[1-9][0-9]{0,19}", value[field]):
            raise ReceiptError(f"invalid context {field}")
    if not isinstance(value["job"], str) or not re.fullmatch(r"[A-Za-z0-9_.-]{1,128}", value["job"]):
        raise ReceiptError("invalid context job")
    if not isinstance(value["githubSha"], str) or not re.fullmatch(r"[0-9a-f]{40}", value["githubSha"]):
        raise ReceiptError("invalid context githubSha")
    if value["architecture"] != "Win32":
        raise ReceiptError("only the reviewed Win32 architecture is supported")
    for field in ("inputs", "toolchain"):
        records = value[field]
        if not isinstance(records, dict) or not 1 <= len(records) <= 128:
            raise ReceiptError(f"invalid context {field} inventory")
        for name, digest in records.items():
            _relative(name)
            if not isinstance(digest, str) or not SHA256.fullmatch(digest):
                raise ReceiptError(f"invalid context digest: {name}")
    return value


def _roots(values):
    if not isinstance(values, list) or not 1 <= len(values) <= MAX_ROOTS:
        raise ReceiptError("invalid receipt roots")
    if any(not isinstance(value, str) for value in values):
        raise ReceiptError("invalid receipt root")
    for value in values:
        _relative(value)
    if values != sorted(set(values)) or len({value.casefold() for value in values}) != len(values):
        raise ReceiptError("receipt roots must be unique and sorted")
    for first in values:
        if any(other != first and other.casefold().startswith(first.casefold() + "/") for other in values):
            raise ReceiptError("overlapping receipt roots")
    return values


def _inventory(root: Path, roots: list[str]) -> dict[str, dict[str, int | str]]:
    files: dict[str, dict[str, int | str]] = {}
    seen_casefold: set[str] = set()
    entry_count = 0
    for name in roots:
        anchor = _plain_path(root, name)
        if anchor.is_file():
            candidates = (anchor,)
        else:
            def fail(error):
                raise error
            def walk():
                nonlocal entry_count
                for directory, dirs, names in os.walk(anchor, followlinks=False, onerror=fail):
                    dirs.sort()
                    names.sort()
                    entry_count += 1 + len(dirs) + len(names)
                    if entry_count > MAX_ENTRIES:
                        raise ReceiptError("receipt directory inventory exceeds bound")
                    if len(files) + len(names) > MAX_FILES:
                        raise ReceiptError("receipt file inventory exceeds bound")
                    for child in dirs + names:
                        _plain_path(root, (Path(directory) / child).relative_to(root).as_posix())
                    for child in names:
                        yield Path(directory) / child
            candidates = walk()
        for path in candidates:
            relative = path.relative_to(root).as_posix()
            folded = relative.casefold()
            if folded in seen_casefold:
                raise ReceiptError(f"case-insensitive file alias: {relative}")
            seen_casefold.add(folded)
            info = path.stat()
            if not stat.S_ISREG(info.st_mode) or info.st_size < 0:
                raise ReceiptError(f"nonregular inventory entry: {relative}")
            files[relative] = {"bytes": info.st_size, "sha256": _digest(path)}
            if len(files) > MAX_FILES:
                raise ReceiptError("receipt file inventory exceeds bound")
    if not files:
        raise ReceiptError("empty receipt file inventory")
    return dict(sorted(files.items()))


def capture(root: Path, receipt: Path, context: dict, roots: list[str]) -> None:
    context = _context(context)
    roots = _roots(roots)
    root = _workspace_root(root)
    files = _inventory(root, roots)
    document = {"schemaVersion": SCHEMA, "workspaceRoot": str(root), "context": context, "roots": roots, "files": files}
    encoded = (json.dumps(document, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
    if len(encoded) > MAX_RECEIPT_BYTES:
        raise ReceiptError("receipt exceeds size bound")
    receipt.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=receipt.parent, prefix=".dependency-receipt-", delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(encoded)
    try:
        temporary.replace(receipt)
    finally:
        temporary.unlink(missing_ok=True)


def verify(root: Path, receipt: Path, context: dict, roots: list[str]) -> None:
    context = _context(context)
    roots = _roots(roots)
    document = _read_json(receipt)
    if not isinstance(document, dict) or set(document) != RECEIPT_KEYS or type(document["schemaVersion"]) is not int or document["schemaVersion"] != SCHEMA:
        raise ReceiptError("unsupported receipt schema")
    root = _workspace_root(root)
    if document["workspaceRoot"] != str(root):
        raise ReceiptError("receipt workspace differs")
    if _context(document["context"]) != context or _roots(document["roots"]) != roots:
        raise ReceiptError("receipt job identity or roots differ")
    expected = document["files"]
    if not isinstance(expected, dict) or not 1 <= len(expected) <= MAX_FILES:
        raise ReceiptError("invalid receipt file inventory")
    for name, record in expected.items():
        _relative(name)
        if not any(name == anchor or name.startswith(anchor + "/") for anchor in roots):
            raise ReceiptError(f"file outside receipt roots: {name}")
        if not isinstance(record, dict) or set(record) != {"bytes", "sha256"} or \
                type(record["bytes"]) is not int or record["bytes"] < 0 or \
                not isinstance(record["sha256"], str) or not SHA256.fullmatch(record["sha256"]):
            raise ReceiptError(f"malformed file record: {name}")
    if _inventory(root, roots) != expected:
        raise ReceiptError("receipt inventory differs: missing, extra or changed file")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("capture", "verify"))
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--context", type=Path, required=True, help="caller-derived same-job and input hashes JSON")
    parser.add_argument("--path", action="append", required=True, help="relative installed prefix to inventory")
    args = parser.parse_args(argv)
    try:
        context = _read_json(args.context)
        if args.mode == "capture":
            capture(args.root, args.receipt, context, args.path)
        else:
            verify(args.root, args.receipt, context, args.path)
    except (OSError, ValueError, TypeError, KeyError, RecursionError) as error:
        print(f"Windows dependency receipt rejected: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Audit a hash-pinned, closed-session COPY of OpenCPN's SQLite navigation data.

No names, positions, paths or raw database values are emitted. Comparison uses
SQLite storage types and raw text bytes: Windows locale text need not be UTF-8.
This is preservation evidence, not permission to modify or repair a database.
"""
import argparse
import base64
from contextlib import closing
import hashlib
import json
from pathlib import Path
import re
import sqlite3


OWNER = "OpenNavX.NavigationCopyAudit.1"
# model/src/navobj_db.cpp::CreateTables at pinned OpenCPN 37fd0cdd.
REQUIRED = {b"routes", b"routepoints", b"routepoints_link", b"tracks", b"trk_points",
            b"route_html_links", b"routepoint_html_links", b"track_html_links"}
MAX_BYTES = 256 * 1024 * 1024
MAX_ROWS = 1_000_000


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def encoded(value):
    if isinstance(value, bytes):
        return {"bytes": base64.b64encode(value).decode("ascii")}
    if isinstance(value, float):
        return {"float": value.hex()}
    return value


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")


def identifier(raw):
    # Schema identifiers, unlike stored user text, must be valid UTF-8.
    value = raw.decode("utf-8", errors="strict")
    if "\x00" in value:
        raise ValueError("Invalid schema identifier")
    return '"' + value.replace('"', '""') + '"'


def audit(path, expected_sha256):
    path = Path(path)
    if not re.fullmatch(r"[a-f0-9]{64}", expected_sha256):
        raise ValueError("Exact expected copy SHA-256 required")
    if path.is_symlink() or not path.is_file() or not 0 < path.stat().st_size <= MAX_BYTES:
        raise ValueError("Bounded regular database copy required")
    if any(Path(str(path) + suffix).exists() for suffix in ("-wal", "-shm", "-journal")):
        raise ValueError("A closed-session database copy without journals is required")
    if digest(path) != expected_sha256:
        raise ValueError("Database copy differs from the reviewed source hash")
    tables = []
    uri = path.resolve().as_uri() + "?mode=ro&immutable=1"
    with closing(sqlite3.connect(uri, uri=True)) as database:
        database.text_factory = bytes
        database.enable_load_extension(False)
        database.execute("PRAGMA trusted_schema=OFF")
        database.execute("PRAGMA query_only=ON")
        if database.execute("PRAGMA integrity_check").fetchall() != [(b"ok",)]:
            raise ValueError("Navigation database integrity check failed")
        schema = database.execute("SELECT type,name,tbl_name,sql FROM sqlite_master ORDER BY type,name").fetchall()
        definitions = [(row[1], row[3]) for row in schema if row[0] == b"table" and not row[1].startswith(b"sqlite_")]
        if not REQUIRED.issubset({name for name, _ in definitions}) or len(definitions) > 64:
            raise ValueError("Expected bounded OpenCPN navigation schema required")
        for name, statement in definitions:
            if not statement or re.search(rb"\bCREATE\s+VIRTUAL\s+TABLE\b", statement, re.IGNORECASE):
                raise ValueError("Only ordinary stored navigation tables are supported")
            table = identifier(name)
            columns = database.execute("PRAGMA table_info(" + table + ")").fetchall()
            if not 0 < len(columns) <= 256:
                raise ValueError("Unexpected table width")
            selected = ",".join(identifier(column[1]) + ",typeof(" + identifier(column[1]) + ")" for column in columns)
            row_hashes = []
            for row in database.execute("SELECT " + selected + " FROM " + table):
                if len(row_hashes) >= MAX_ROWS:
                    raise ValueError("Navigation table exceeds audit bound")
                row_hashes.append(hashlib.sha256(canonical([encoded(value) for value in row])).digest())
            content = hashlib.sha256()
            for value in sorted(row_hashes):
                content.update(value)
            tables.append({"name": name.decode("utf-8"), "rows": len(row_hashes), "contentSha256": content.hexdigest()})
        schema_hash = hashlib.sha256(canonical([[encoded(value) for value in row] for row in schema])).hexdigest()
    if digest(path) != expected_sha256:
        raise ValueError("Database copy changed during audit")
    return {"schema": 1, "owner": OWNER, "fileSha256": expected_sha256, "integrity": "ok",
            "schemaSha256": schema_hash, "tables": sorted(tables, key=lambda item: item["name"]),
            "comparison": "Storage types and raw stored bytes; row order ignored, duplicates retained"}


def compare(before, after):
    for report in (before, after):
        if report.get("owner") != OWNER or report.get("schema") != 1 or report.get("integrity") != "ok":
            raise ValueError("Expected navigation-copy audit report")
    return before["schemaSha256"] == after["schemaSha256"] and before["tables"] == after["tables"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("copy", type=Path)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--baseline-report", type=Path)
    args = parser.parse_args()
    report = audit(args.copy, args.expected_sha256)
    if args.baseline_report:
        report["navigationContentUnchanged"] = compare(json.loads(args.baseline_report.read_text(encoding="utf-8")), report)
    print(json.dumps(report, indent=2))
    return 2 if report.get("navigationContentUnchanged") is False else 0


if __name__ == "__main__":
    raise SystemExit(main())

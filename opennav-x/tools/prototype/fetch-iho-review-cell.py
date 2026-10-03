#!/usr/bin/env python3
"""Fetch one official IHO presentation-test cell for isolated review only.

Not operational chart data. Do not include the archive/cell/SENC in product,
source or review artifacts. Source/rights inspection is retained in
docs/design/reviews/scrum256-official-cardinal-test-source.md.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import urllib.request
import zipfile

URL = "https://drive.google.com/uc?export=download&id=1iJl0SyymUxfzFMqFZNK0t-qUwNvw61m4"
ARCHIVE_SHA256 = "01fe16330e1a704f1e65de43ca97724623fd8e619d31c6e49ea7f960ecac6057"
ARCHIVE_BYTES = 2676974
MEMBER = "S-64_ENC_Unencrypted_TDS/2.1.1 Power Up/ENC_ROOT/GB4X0000.000"
CELL_SHA256 = "c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3"
CELL_BYTES = 945697


def checked_cell(raw):
    if len(raw) != ARCHIVE_BYTES or hashlib.sha256(raw).hexdigest() != ARCHIVE_SHA256:
        raise ValueError("Official IHO archive differs; inspect source before changing the pin")
    with zipfile.ZipFile(io.BytesIO(raw)) as archive:
        members = [member for member in archive.infolist() if member.filename == MEMBER]
        if len(members) != 1 or members[0].file_size != CELL_BYTES:
            raise ValueError("Expected exact unique IHO presentation-test member")
        # Read this one exact member only. ZipFile verifies its CRC; no archive
        # path is used as a destination and no other cell is extracted.
        cell = archive.read(members[0])
    if len(cell) != CELL_BYTES or hashlib.sha256(cell).hexdigest() != CELL_SHA256:
        raise ValueError("Official IHO presentation-test cell differs")
    return cell


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, help="Reuse exact locally cached official archive")
    parser.add_argument("--output", type=Path, required=True, help="New private fixture directory")
    args = parser.parse_args()
    if args.output.exists() or args.output.is_symlink():
        raise SystemExit("Fresh fixture directory required; existing content is never overwritten")
    if args.archive:
        if args.archive.stat().st_size != ARCHIVE_BYTES:
            raise SystemExit("Cached official archive size differs")
        raw = args.archive.read_bytes()
    else:
        request = urllib.request.Request(URL, headers={"User-Agent": "SKAGER-chart-review/1"})
        with urllib.request.urlopen(request, timeout=60) as response:
            raw = response.read(ARCHIVE_BYTES + 1)
    cell = checked_cell(raw)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "GB4X0000.000").write_bytes(cell)
    receipt = {
        "purpose": "Official IHO presentation-test data; not an operational nautical chart",
        "source_url": URL,
        "archive_sha256": ARCHIVE_SHA256,
        "archive_bytes": ARCHIVE_BYTES,
        "member": MEMBER,
        "cell_sha256": CELL_SHA256,
        "cell_bytes": CELL_BYTES,
        "redistribution_permission_claimed": False,
        "artifact_policy": "Retain receipt only; exclude original cell, archive and generated SENC",
    }
    (args.output / "source-receipt.json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(receipt))


if __name__ == "__main__":
    main()

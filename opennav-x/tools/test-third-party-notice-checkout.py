#!/usr/bin/env python3
"""Verify third-party notices survive a Git autocrlf=true checkout byte-for-byte."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
LIBRARIES = ("OpenSSL-3.5.9", "curl-8.22.0", "zlib-1.3.2")


def run(cwd: Path, *args: str) -> str:
    return subprocess.check_output(("git", "-C", str(cwd), *args), text=True).strip()


def make_repository(destination: Path, with_attributes: bool) -> dict[str, bytes]:
    source = destination / "source"
    source.mkdir(parents=True)
    notices = source / "docs" / "third-party"
    shutil.copytree(ROOT / "docs" / "third-party", notices)
    if with_attributes:
        shutil.copy2(ROOT / ".gitattributes", source / ".gitattributes")
    run(source, "init", "-q")
    run(source, "config", "user.name", "third-party notice checkout test")
    run(source, "config", "user.email", "fixture@example.invalid")
    run(source, "config", "core.autocrlf", "false")
    run(source, "add", ".")
    run(source, "commit", "-qm", "retain third-party notices")

    originals = {
        f"{library}/LICENSE.txt":
        (notices / library / "LICENSE.txt").read_bytes()
        for library in LIBRARIES
    }
    originals.update({
        f"{library}/provenance.json":
        (notices / library / "provenance.json").read_bytes()
        for library in LIBRARIES
    })
    return originals


def checkout(source: Path, destination: Path) -> None:
    subprocess.run(
        ("git", "-c", "core.autocrlf=true", "clone", "--quiet",
         str(source), str(destination)),
        check=True,
    )


def main() -> None:
    attributes = (ROOT / ".gitattributes").read_text()
    assert "docs/design/prototype/** -text" in attributes, (
        "the existing prototype immutable-bytes rule was removed"
    )
    assert "docs/third-party/** -text" in attributes, (
        "third-party notices are not marked immutable"
    )

    with tempfile.TemporaryDirectory(prefix="opennav-third-party-checkout-") as name:
        temporary = Path(name)
        source = temporary / "with-attributes"
        originals = make_repository(source, with_attributes=True)
        checkout(source / "source", temporary / "checkout-with-attributes")
        checked_out = temporary / "checkout-with-attributes" / "docs" / "third-party"

        for relative, expected in originals.items():
            actual = (checked_out / relative).read_bytes()
            assert actual == expected, f"attribute-protected bytes changed: {relative}"

        for library in LIBRARIES:
            license_bytes = (checked_out / library / "LICENSE.txt").read_bytes()
            provenance = json.loads(
                (checked_out / library / "provenance.json").read_bytes()
            )
            assert provenance["licenseSha256"] == hashlib.sha256(
                license_bytes
            ).hexdigest(), f"provenance hash mismatch: {library}"

        without_attributes = temporary / "without-attributes"
        make_repository(without_attributes, with_attributes=False)
        checkout(without_attributes / "source", temporary / "checkout-without-attributes")
        unprotected = (
            temporary / "checkout-without-attributes" / "docs" / "third-party"
            / "OpenSSL-3.5.9" / "LICENSE.txt"
        ).read_bytes()
        assert unprotected != originals["OpenSSL-3.5.9/LICENSE.txt"], (
            "negative control did not demonstrate autocrlf transformation"
        )
        assert b"\r\n" in unprotected, (
            "negative control did not produce CRLF bytes"
        )

    print(
        "PASS: OpenSSL 3.5.9, curl 8.22.0 and zlib 1.3.2 notices remain "
        "byte-identical with core.autocrlf=true; negative control transforms "
        "unprotected LF notices to CRLF"
    )


if __name__ == "__main__":
    main()

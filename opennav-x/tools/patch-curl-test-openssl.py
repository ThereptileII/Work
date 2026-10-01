#!/usr/bin/env python3
"""Apply reviewed Windows executable and subprocess fixes to locked curl test source."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import tempfile
from pathlib import Path


ORIGINAL_SHA256 = "d737cbe77e23e275b4fcfcec36e62d49d1d59d9d9fd0013a428b7143ee75c982"
PATCHED_SHA256 = "4c176ec6a1556f519d6c0c02d17c40caadb9a542894fedc9f2055b7a48ce9ab3"
OLD_SELECTION = b"my $OPENSSL = 'openssl';"
NEW_SELECTION = b"my $OPENSSL = $^O eq 'MSWin32' ? 'openssl.exe' : 'openssl';"
OLD_REDIR = b"""sub redir {
    my $outfn = shift if($_[0] =~ /^>/);
    my $hideerr = shift if($_[0] =~ /^2>/);
    open(my $outfd, $outfn) or die if($outfn);
    my $pid = open3(my $in, my $out, my $err = gensym, @_);
    if(!$hideerr) { while(<$err>) { print STDERR $_; }; }
    if($outfn) { while(<$out>) { print $outfd $_; }; close($outfd); }
    else { while(<$out>) { print $_; }; }
    waitpid($pid, 0);
}
"""
NEW_REDIR = rb"""sub redir {
    my $outfn = shift if($_[0] =~ /^>/);
    my $hideerr = shift if($_[0] =~ /^2>/);
    open(my $infd, '<', File::Spec->devnull()) or die;
    my $outfd;
    if($outfn) { open($outfd, '>', substr($outfn, 1)) or die; }
    else { $outfd = \*STDOUT; }
    my $errfd;
    if($hideerr) { open($errfd, '>', File::Spec->devnull()) or die; }
    else { $errfd = \*STDERR; }
    my $pid = open3('<&' . fileno($infd), '>&' . fileno($outfd), '>&' . fileno($errfd), @_);
    waitpid($pid, 0);
    close($outfd) if($outfn);
    close($errfd) if($hideerr);
}
"""
SOURCE_RELATIVE_PATH = "tests/certs/genserv.pl"
MAX_SOURCE_BYTES = 1_000_000


def _sha256(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def patch_source(source: Path) -> dict[str, object]:
    """Patch exact locked upstream bytes once, or recognize the exact result."""
    source = Path(source)
    if source.is_symlink() or not source.is_file():
        raise ValueError("curl test source must be a regular, non-symlink file")
    if source.stat().st_size > MAX_SOURCE_BYTES:
        raise ValueError("curl test source exceeds the bounded input size")
    original = source.read_bytes()
    before = _sha256(original)

    if before == PATCHED_SHA256:
        state = "already-patched"
        patched = original
    elif before == ORIGINAL_SHA256:
        if original.count(OLD_SELECTION) != 1 or original.count(OLD_REDIR) != 1:
            raise ValueError("locked curl source does not contain the expected selection and redirection")
        patched = original.replace(OLD_SELECTION, NEW_SELECTION, 1).replace(OLD_REDIR, NEW_REDIR, 1)
        if _sha256(patched) != PATCHED_SHA256:
            raise ValueError("generated patch differs from the reviewed curl source result")
        mode = stat.S_IMODE(source.stat().st_mode)
        _atomic_replace(source, patched, mode)
        state = "patched"
    else:
        raise ValueError("curl test source hash is neither the locked original nor reviewed patch")

    after = _sha256(patched)
    return {
        "source": SOURCE_RELATIVE_PATH,
        "state": state,
        "beforeSha256": before,
        "afterSha256": after,
        "lockedOriginalSha256": ORIGINAL_SHA256,
        "reviewedPatchedSha256": PATCHED_SHA256,
    }


def _atomic_replace(path: Path, content: bytes, mode: int) -> None:
    fd, temporary = tempfile.mkstemp(prefix=".genserv-patch-", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as output:
            output.write(content)
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path,
                        help="tests/certs/genserv.pl from the verified curl 8.22.0 extraction")
    parser.add_argument("--evidence", type=Path,
                        help="optional path for the sanitized before/after hash receipt")
    args = parser.parse_args()
    try:
        receipt = patch_source(args.source)
        rendered = json.dumps(receipt, sort_keys=True, separators=(",", ":"))
        if args.evidence:
            evidence = args.evidence
            if evidence.exists() and evidence.is_symlink():
                raise ValueError("evidence target must not be a symlink")
            evidence.parent.mkdir(parents=True, exist_ok=True)
            evidence.write_text(rendered + "\n", encoding="utf-8")
        print(rendered)
        return 0
    except (OSError, ValueError) as error:
        parser.exit(2, f"patch-curl-test-openssl: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())

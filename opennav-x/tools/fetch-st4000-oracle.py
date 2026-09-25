#!/usr/bin/env python3
"""Fetch hash-pinned boat parser sources for host-only interoperability tests.

No firmware build, serial port, CAN connection or hardware access occurs here.
"""
import argparse
import hashlib
from pathlib import Path
import urllib.request

REVISION = "9baf01bca09522794a9678dfb3f3c0720d5c9943"
FILES = {
    "src/BridgeCore.cpp": "874a92dbdd35b97a50ee0452913eb8ce9ff3eded6f7ac0d535c92a543e1183da",
    "include/BridgeCore.h": "431c432904039a049179a2e90ce9dc4f10d43bf5e9a89505d9f9ffdc664d68f5",
    "include/Config.h": "1f1041140fdd7aa85937a5cb81210ad5d7c4e0cee82158c542f4609db1a5d30a",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--source", type=Path, help="Optional existing checkout; same hashes required")
    args = parser.parse_args()
    verified = {}
    for path, digest in FILES.items():
        if args.source:
            content = (args.source / path).read_bytes()
        else:
            url = f"https://raw.githubusercontent.com/ThereptileII/Work/{REVISION}/autopilot-controller/{path}"
            request = urllib.request.Request(url, headers={"User-Agent": "OpenNavX-protocol-test"})
            with urllib.request.urlopen(request, timeout=30) as stream:
                content = stream.read(512 * 1024 + 1)
        if len(content) > 512 * 1024 or hashlib.sha256(content).hexdigest() != digest:
            raise RuntimeError(f"Untrusted translator test source: {path}")
        verified[path] = content
    for path, content in verified.items():
        target = args.destination / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(content)
    print(f"Verified {len(verified)} translator test sources at {REVISION}")


if __name__ == "__main__":
    main()

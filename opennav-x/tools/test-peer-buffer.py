#!/usr/bin/env python3
"""Apply the peer buffer patch and run its portable exact-source tests."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / "patches/opencpn-5.12.4-peer-response-buffer.patch"


def run(*args):
    subprocess.run(args, check=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream", type=Path,
                        default=ROOT / "upstream/OpenCPN")
    args = parser.parse_args()
    source = args.upstream.resolve() / "model/src/peer_client.cpp"
    if not source.is_file():
        raise SystemExit(f"pinned peer_client.cpp missing: {source}")
    with tempfile.TemporaryDirectory(prefix="scrum211-peer-buffer-") as raw:
        work = Path(raw)
        patched = work / "model/src/peer_client.cpp"
        patched.parent.mkdir(parents=True)
        shutil.copy2(source, patched)
        subprocess.run(["git", "apply", "-"], cwd=work, check=True,
                       input=PATCH.read_bytes().replace(b"\r\n", b"\n"))
        build = work / "build"
        platform = ["-G", "Visual Studio 17 2022", "-A", "Win32"] if os.name == "nt" else []
        run("cmake", "-S", str(ROOT / "tests/peer_buffer"), "-B", str(build),
            f"-DOPENNAV_ROOT={ROOT}", f"-DPEER_CLIENT_SOURCE={patched}", *platform)
        run("cmake", "--build", str(build), "--config", "Release", "--parallel", "2")
        run("ctest", "--test-dir", str(build), "--output-on-failure",
            "--no-tests=error", "-C", "Release")


if __name__ == "__main__":
    main()

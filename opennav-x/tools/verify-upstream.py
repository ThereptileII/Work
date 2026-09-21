#!/usr/bin/env python3
"""Fail closed before building when the pinned source changed."""
import json
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
lock = json.loads((root / "upstream.lock.json").read_text())
source = root / "upstream/OpenCPN"

def git(*args):
    return subprocess.check_output(["git", "-C", str(source), *args], text=True).strip()

if git("rev-parse", "HEAD") != lock["commit"]:
    raise SystemExit("Refusing build: OpenCPN commit differs from upstream.lock.json")
if git("status", "--porcelain", "--untracked-files=no"):
    raise SystemExit("Refusing pristine build: tracked OpenCPN source is modified")
print(json.dumps(lock, indent=2))

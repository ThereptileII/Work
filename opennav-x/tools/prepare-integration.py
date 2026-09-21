#!/usr/bin/env python3
"""Apply reviewed hooks only to a disposable worktree of the pinned revision."""
import json
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
source = root / 'upstream/OpenCPN'
target = root / 'build/integration-source'
lock = json.loads((root / 'upstream.lock.json').read_text())

def run(*args):
    subprocess.run(args, check=True)

run(sys.executable, str(root / 'tools/verify-upstream.py'))
if not target.exists():
    target.parent.mkdir(exist_ok=True)
    run('git', '-C', str(source), 'worktree', 'add', '--detach', str(target), lock['commit'])
head = subprocess.check_output(['git', '-C', str(target), 'rev-parse', 'HEAD'], text=True).strip()
if head != lock['commit']:
    raise SystemExit('Refusing to patch a worktree with a different OpenCPN revision')
patch = root / 'patches/opencpn-5.12.4-xnav.patch'
# Refuse to overwrite edits. A second invocation accepts only this exact patch.
expected = subprocess.check_output(['git', 'apply', '--numstat', str(patch)], text=True)
if subprocess.check_output(['git', '-C', str(target), 'status', '--porcelain', '--untracked-files=no'], text=True).strip():
    reverse = subprocess.run(['git', '-C', str(target), 'apply', '--reverse', '--check', str(patch)])
    actual = subprocess.check_output(['git', '-C', str(target), 'diff', '--numstat'], text=True)
    if reverse.returncode or sorted(actual.splitlines()) != sorted(expected.splitlines()):
        raise SystemExit('Integration worktree contains unexpected changes; refusing to overwrite')
else:
    run('git', '-C', str(target), 'apply', '--check', str(patch))
    run('git', '-C', str(target), 'apply', str(patch))
print(target)

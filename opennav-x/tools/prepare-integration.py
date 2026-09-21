#!/usr/bin/env python3
"""Apply reviewed hooks only to a disposable worktree of the pinned revision."""
import json
from pathlib import Path
import subprocess
import sys
import os
import tempfile

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
patches = [root / 'patches/opencpn-5.12.4-xnav.patch',
           root / 'patches/opencpn-5.12.4-regression-tests.patch']
# A temporary index describes the exact reviewed result without touching the
# worktree's real index. This catches extra edits, even with identical line counts.
if not subprocess.check_output(['git', '-C', str(target), 'status', '--porcelain', '--untracked-files=no'], text=True).strip():
    for patch in patches:
        run('git', '-C', str(target), 'apply', '--check', str(patch))
        run('git', '-C', str(target), 'apply', str(patch))
with tempfile.TemporaryDirectory(prefix='opennav-index-') as directory:
    env = dict(os.environ, GIT_INDEX_FILE=str(Path(directory) / 'index'))
    subprocess.run(['git', '-C', str(target), 'read-tree', lock['commit']], env=env, check=True)
    for patch in patches:
        subprocess.run(['git', '-C', str(target), 'apply', '--cached'],
                       input=patch.read_bytes().replace(b'\r\n', b'\n'), env=env, check=True)
    comparison = subprocess.run(['git', '-C', str(target), 'diff', '--quiet'], env=env)
    if comparison.returncode:
        raise SystemExit('Integration worktree differs from reviewed patches; refusing to overwrite')
print(target)

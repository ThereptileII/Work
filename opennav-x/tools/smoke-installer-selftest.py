#!/usr/bin/env python3
"""Actual app loader check exits without opening or changing a user profile."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument('--app', type=Path, required=True)
a = p.parse_args()
with tempfile.TemporaryDirectory(prefix='opennav loader ') as temporary:
    root = Path(temporary)
    profile = root / 'profile'
    profile.mkdir()
    (profile / 'sentinel').write_text('Must not modify any profile in self-test')
    before = {str(f.relative_to(profile)): hashlib.sha256(f.read_bytes()).hexdigest()
              for f in profile.rglob('*') if f.is_file()}
    report = root / 'loader.json'
    command = [str(a.app.resolve()), '--opennav-self-test', str(report), '--configdir', str(profile)]
    if os.name != 'nt': command = ['xvfb-run', '-a'] + command
    r = subprocess.run(command, timeout=30, capture_output=True)
    assert r.returncode == 0, (r.returncode, r.stderr)
    d = json.loads(report.read_text())
    assert d['passed'] and not d['profile_initialized'] and not d['plugins_loaded'], d
    assert d['upstream'] == '37fd0cddb7334fe489e9f18aa163977a9c5c84f7'
    original = report.read_bytes()
    # Existing report cannot be overwritten, even by another self-test.
    r = subprocess.run(command, timeout=30, capture_output=True)
    assert r.returncode != 0 and report.read_bytes() == original
    after = {str(f.relative_to(profile)): hashlib.sha256(f.read_bytes()).hexdigest()
             for f in profile.rglob('*') if f.is_file()}
    assert before == after, (before, after)
    out = ROOT / 'evidence/local/installer-loader-selftest.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({'status': 'passed', 'checks': 5, 'loader': d,
                              'app_sha256': hashlib.sha256(a.app.read_bytes()).hexdigest()}, indent=2)+'\n')
print('Actual loader/resource check passed; profile unchanged; overwrite rejected')

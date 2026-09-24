#!/usr/bin/env python3
"""Compile a distinct prior test version for disposable native update tests.

The fixture is never published. Restore source and the exact already-tested
candidate executable even when compilation/packaging fails.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
    raise SystemExit('Prior-version fixture requires disposable native CI')
root = Path(__file__).resolve().parents[1]
output = root / 'build/prior-alpha-fixture'
output.mkdir()
preview = root / 'build/developer-preview/OpenNavX-Alpha1-Portable'
for name in ('app', 'docs'):
    shutil.copytree(preview / name, output / name)
header = root / 'src/application/Version.h'
source = header.read_bytes()
assert b'0.2.0-alpha1' in source
exe = root / 'build/xnav-windows/Release/opencpn.exe'
backup = output / 'candidate-executable.backup'
shutil.copy2(exe, backup)
try:
    header.write_bytes(source.replace(b'0.2.0-alpha1', b'0.2.0-alpha0-ci'))
    subprocess.run(['cmake', '--build', str(root / 'build/xnav-windows'),
                    '--config', 'Release', '--target', 'opencpn',
                    '--parallel', '2'], check=True)
    assert exe.read_bytes() != backup.read_bytes(), 'Prior fixture must be a distinct executable'
    shutil.copy2(exe, output / 'app/opencpn.exe')
    candidate = [] if json.loads((root / 'installer/windows/compatibility.json').read_text())['supportedOpenCpn'] else ['--candidate']
    subprocess.run([sys.executable, str(root / 'tools/package-alpha-installer.py'),
                    '--preview', str(output), '--output', str(output / 'setup'),
                    *candidate], check=True)
finally:
    header.write_bytes(source)
    shutil.copy2(backup, exe)
    assert hashlib.sha256(exe.read_bytes()).digest() == hashlib.sha256(backup.read_bytes()).digest()
    assert header.read_bytes() == source
print('Distinct alpha0-ci installer fixture built; candidate source/executable restored')

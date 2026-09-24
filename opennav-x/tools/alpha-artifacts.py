#!/usr/bin/env python3
"""Collect exact tested native Alpha payloads and a checksummed download set."""
import hashlib
import json
from pathlib import Path
import shutil
ROOT=Path(__file__).resolve().parents[1]
output=ROOT/'build/alpha-artifacts'
output.mkdir(parents=True,exist_ok=False)
files=[ROOT/'build/developer-preview/OpenNavX-Alpha1-Portable-win64.zip',
       ROOT/'build/developer-preview/OpenNavX-Alpha1-source.zip',
       ROOT/'build/alpha-installer/OpenNavX-Alpha1-Setup.exe',
       ROOT/'docs/alpha/OpenNavX-Alpha1-Test-Guide.md']
checks=[]
for source in files:
    assert source.is_file(),source
    target=output/source.name;shutil.copy2(source,target)
    checks.append(hashlib.sha256(target.read_bytes()).hexdigest()+'  '+target.name)
(output/'SHA256SUMS.txt').write_text('\n'.join(checks)+'\n')
qualified=bool(json.loads((ROOT/'installer/windows/compatibility.json').read_text())['supportedOpenCpn'])
(output/'QUALIFICATION.txt').write_text('Qualified compatibility manifest; final same-commit CI gates apply.\n' if qualified else 'CANDIDATE ONLY: compatibility manifest is not yet accepted. Do not distribute as accepted Alpha.\n')
print(output)

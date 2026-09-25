#!/usr/bin/env python3
"""Fail before compilation if the published distribution inputs are missing."""
import json
import re
from pathlib import Path

root = Path(__file__).resolve().parents[1]
required = (
    'LICENSE', 'AGENTS.md', 'OpenNavX_Codex_Project_Specification.md',
    'docs/design/OpenNavX_Design_Reference.png',
    'installer/windows/compatibility.json', 'installer/windows/AlphaSetup.nsi',
    'installer/windows/Lifecycle.ps1', 'docs/beta/TEST_ME_FIRST.md',
    'docs/beta/OpenNavX-Beta1-Boat-Commissioning.md', 'tools/accepted-alpha.lock.json',
    'docs/beta/KNOWN_LIMITATIONS.md', 'docs/beta/OpenNavX-Beta1-Test-Guide.md',
)
missing = [name for name in required if not (root / name).is_file()]
if missing:
    raise SystemExit('Missing distribution inputs: ' + ', '.join(missing))
manifest = json.loads((root / 'installer/windows/compatibility.json').read_text())
assert isinstance(manifest['supportedOpenCpn'], list)
version = re.search(r'Version\[\] = "([^"]+)"', (root/'src/application/Version.h').read_text()).group(1)
assert version == manifest['openNavVersion'] == '0.3.0-beta1'
assert all(entry['integrationPackage'] == 'OpenNavX-Beta1-Setup.exe' for entry in manifest['supportedOpenCpn'])
assert (root/'docs/boat-commissioning.md').read_bytes() == (root/'docs/beta/OpenNavX-Beta1-Boat-Commissioning.md').read_bytes()

qualification = json.loads((root/'release/qualification.json').read_text())
if qualification['publishNamedRelease'] and qualification['stage'].startswith('beta'):
    assert qualification['enduranceSeconds'] >= 10800, 'Named Beta requires actual three-hour endurance on each platform'
print(f'{len(required)} distribution inputs present; '
      f'{len(manifest["supportedOpenCpn"])} accepted stock configurations')

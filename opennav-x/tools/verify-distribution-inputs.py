#!/usr/bin/env python3
"""Fail before compilation if the published distribution inputs are missing."""
import json
import re
from pathlib import Path
from product_version import read_product_version

root = Path(__file__).resolve().parents[1]
required = (
    'LICENSE', 'AGENTS.md', 'OpenNavX_Codex_Project_Specification.md',
    'docs/design/OpenNavX_Design_Reference.png',
    'installer/windows/compatibility.json', 'installer/windows/AlphaSetup.nsi',
    'installer/windows/Lifecycle.ps1', 'docs/beta2/TEST_ME_FIRST.md',
    'docs/boat-commissioning.md', 'tools/accepted-beta1.lock.json',
    'tools/early-beta2-layout.lock.json',
    'docs/beta2/KNOWN_LIMITATIONS.md', 'docs/beta2/SKAGER-Beta2-Test-Guide.md',
    'docs/beta2/SKAGER-Beta2-Install-Guide.md', 'docs/beta2/SKAGER-Beta2-Release-Notes.md',
    'docs/production-build-contract.md',
)
missing = [name for name in required if not (root / name).is_file()]
if missing:
    raise SystemExit('Missing distribution inputs: ' + ', '.join(missing))
manifest = json.loads((root / 'installer/windows/compatibility.json').read_text())
assert isinstance(manifest['supportedOpenCpn'], list)
version = read_product_version(root/'src/application/Version.h')
assert version == manifest['openNavVersion']
assert all(entry['integrationPackage'] == 'SKAGER-Beta2-Setup.exe' for entry in manifest['supportedOpenCpn'])
assert (root/'src/integration/BuildFeatures.h').is_file()
assert 'option(XNAV_ENABLE_TEST_FIXTURES' in (root/'CMakeLists.txt').read_text()
assert 'option(XNAV_ENABLE_PILOT_LOOPBACK_TESTS' in (root/'CMakeLists.txt').read_text()

qualification = json.loads((root/'release/qualification.json').read_text())
if qualification['publishNamedRelease'] and qualification['stage'].startswith('beta'):
    assert qualification['enduranceSeconds'] >= 10800, 'Named Beta requires actual three-hour endurance on each platform'
print(f'{len(required)} distribution inputs present; '
      f'{len(manifest["supportedOpenCpn"])} accepted stock configurations')

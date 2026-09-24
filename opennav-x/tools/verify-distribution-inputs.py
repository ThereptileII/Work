#!/usr/bin/env python3
"""Fail before compilation if the published distribution inputs are missing."""
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
required = (
    'LICENSE', 'AGENTS.md', 'OpenNavX_Codex_Project_Specification.md',
    'docs/design/OpenNavX_Design_Reference.png',
    'installer/windows/compatibility.json', 'installer/windows/AlphaSetup.nsi',
    'installer/windows/Lifecycle.ps1', 'docs/alpha/TEST_ME_FIRST.md',
    'docs/alpha/KNOWN_LIMITATIONS.md', 'docs/alpha/OpenNavX-Alpha1-Test-Guide.md',
)
missing = [name for name in required if not (root / name).is_file()]
if missing:
    raise SystemExit('Missing distribution inputs: ' + ', '.join(missing))
manifest = json.loads((root / 'installer/windows/compatibility.json').read_text())
assert isinstance(manifest['supportedOpenCpn'], list)
print(f'{len(required)} distribution inputs present; '
      f'{len(manifest["supportedOpenCpn"])} accepted stock configurations')

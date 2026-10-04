#!/usr/bin/env python3
"""Collect verified native Beta 2 product payloads and a checksummed download set.

Staging requires its bounded native install/recovery checks. Production consumes
these exact bytes in the separate explicit promotion workflow.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
from hardware_output_policy import require_status_only

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--boat-review', action='store_true',
                    help='Separate development review bundle; endurance and boat acceptance remain pending')
parser.add_argument('--channel', choices=('staging',), default='staging',
                    help='New packages always enter staging; promotion never rebuilds them')
args = parser.parse_args()
policy = json.loads((ROOT / 'release/qualification.json').read_text())
endurance_state = ('skipped by user direction' if policy.get('enduranceEnabled') is False
                   else 'pending')
gates = {}
for name in ('production-recovery-results.json', 'installer-staging.json'):
    record = json.loads((ROOT / 'evidence/local' / name).read_text())
    gates[name] = record
    if record.get('status') != 'passed':
        raise SystemExit('Product artifact assembly requires passed native gate: ' + name)
product = json.loads((ROOT / 'build/developer-preview/SKAGER-Beta2-Portable-Recovery/docs/PRODUCT_BUILD.json').read_text())
require_status_only(product)
if product.get('test_fixtures') is not False or product.get('build_purpose') != 'INSTALLED PRODUCT' or product['commit'] != os.environ['GITHUB_SHA']:
    raise SystemExit('Artifact set must contain the exact fixture-free product commit')
output = ROOT / ('build/beta-boat-review' if args.boat_review else 'build/beta-artifacts')
output.mkdir(parents=True, exist_ok=False)
files = [ROOT / 'build/developer-preview/SKAGER-Beta2-Portable-Recovery.zip',
         ROOT / 'build/developer-preview/SKAGER-Beta2-source.zip',
         ROOT / 'build/beta-installer/SKAGER-Beta2-Setup.exe',
         ROOT / 'docs/beta2/SKAGER-Beta2-Install-Guide.md',
         ROOT / 'docs/beta2/SKAGER-Beta2-Test-Guide.md',
         ROOT / 'docs/beta2/SKAGER-Beta2-Release-Notes.md']
checks = []
expected = {
    'SKAGER-Beta2-Portable-Recovery.zip': gates['production-recovery-results.json']['package_sha256'],
    'SKAGER-Beta2-Setup.exe': gates['installer-staging.json']['setup_sha256'],
}
for source in files:
    if not source.is_file():
        raise SystemExit('Release file missing: ' + str(source))
    if source.name in expected and hashlib.sha256(source.read_bytes()).hexdigest() != expected[source.name]:
        raise SystemExit('Release payload differs from the native-tested bytes: ' + source.name)
    target = output / source.name
    shutil.copy2(source, target)
    checks.append(hashlib.sha256(target.read_bytes()).hexdigest() + '  ' + target.name)
(output / 'SHA256SUMS.txt').write_text('\n'.join(checks) + '\n')
(output / 'QUALIFICATION.txt').write_text(
    f'STAGING: endurance {endurance_state}; Production promotion requires explicit instruction.\n' +
    'Native package and bounded staging installer gates passed for these exact payload hashes.\n'
    'The full installer lifecycle and Production readiness are qualified separately without rebuilding.\n'
    'Design review is not requested by default. This is not public-launch approval.\n'
    'Never treat an unsupported boat OpenCPN installation as qualified.\n')
print(output)

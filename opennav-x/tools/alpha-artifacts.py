#!/usr/bin/env python3
"""Collect verified native Beta 2 product payloads and a checksummed download set.

The workflow still gates publication on the complete same-commit Linux/Windows
suite. These checks prevent assembling a product set from failed packaging tests.
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
args = parser.parse_args()
gates = {}
for name in ('production-recovery-results.json', 'installer-lifecycle.json'):
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
    'SKAGER-Beta2-Setup.exe': gates['installer-lifecycle.json']['setup_sha256'],
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
    ('DEVELOPMENT BOAT REVIEW ONLY: endurance and release qualification are pending.\n'
     if args.boat_review else 'Beta 2 candidate product: native package and installer gates passed.\n') +
    'Native package and installer lifecycle gates passed for these exact payload hashes.\n'
    'Release acceptance additionally requires same-commit complete CI and boat-PC evidence.\n'
    'Never treat an unsupported boat OpenCPN installation as qualified.\n')
print(output)

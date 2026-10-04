#!/usr/bin/env python3
"""Same-run tool-only gate receipts and exact reviewed-byte archive. No boat IO."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
LOCK = ROOT / 'tools/boat-review-composition.json'
GATES = {
    'policy': ['restart-commissioning.json', 'restart-window-review.json',
               'review-window.json', 'broker-fixture-contracts.json', 'review-staging.json'],
    'window': ['native-window-results.json', 'chart-palette/native-palette-results.json',
               'native-display-results.json'],
    'broker': ['broker/broker-result.json', 'prepare-arm/prepare-arm-result.json'],
}
DISPLAY_SOURCES = {
    'nativeHelperSha256': 'tools/boat/ReviewWindowNative.cs',
    'fixtureSha256': 'tests/display-review/window-fixture.ps1',
}
DISPLAY_REQUIRED_CASES = {
    ('Capture', 'prototype-normal'), ('Navigation', 'prototype-normal'),
    ('PanRight', 'normal'), ('Resize1280x800', 'normal'),
    ('CyclePalette', 'prototype-normal'),
    ('ZoomIn', 'prototype-normal'), ('ZoomOut', 'prototype-normal'),
    *(('ZoomIn', name) for name in (
        'prototype-zoom-wrong-owner', 'prototype-zoom-wrong-pid', 'prototype-duplicate',
        'prototype-zoom-hidden', 'prototype-zoom-occluded', 'prototype-zoom-wrong-surface',
        'prototype-zoom-replace-on-down')),
    *(('System', name) for name in (
        'prototype-system-normal', 'prototype-system-duplicate',
        'prototype-system-hidden', 'prototype-system-wrong-owner')),
    *(('RevealInterfaceRecovery', name) for name in (
        'prototype-recovery-normal', 'prototype-recovery-hidden',
        'prototype-recovery-duplicate', 'prototype-recovery-no-progress',
        'prototype-recovery-changed-body', 'prototype-recovery-wrong-owner')),
    *(('InterfaceRecovery', name) for name in (
        'prototype-recovery-reveal', 'prototype-recovery-clipped',
        'prototype-recovery-wrong-surface')),
}

def verify_display_report(report):
    if (report.get('status') != 'passed' or report.get('error') is not None or
            report.get('cleanupErrors') != [] or report.get('productLaunched') is not False or
            report.get('physicalOutput') is not False):
        raise ValueError('Actual display fixture must pass without cleanup or scope errors')
    for field, name in DISPLAY_SOURCES.items():
        actual = (ROOT / name).read_bytes()
        prefix = git('rev-parse', '--show-prefix').decode().strip()
        source = git('show', 'HEAD:' + prefix + name)
        if actual.replace(b'\r\n', b'\n') != source.replace(b'\r\n', b'\n'):
            raise ValueError('Uncommitted display qualification source: ' + name)
        if report.get(field) != digest(actual):
            raise ValueError('Display fixture/source provenance differs: ' + field)
    cases = report.get('cases')
    if not isinstance(cases, list) or not cases:
        raise ValueError('Actual display case matrix missing')
    identities = [(case.get('action'), case.get('case')) for case in cases]
    if (len(set(identities)) != len(identities) or
            any(type(case.get('fixtureExitCode')) is not int or case['fixtureExitCode'] != 0
                for case in cases)):
        raise ValueError('All distinct display fixtures must exit normally')
    if not DISPLAY_REQUIRED_CASES.issubset(identities):
        raise ValueError('Required prototype display success/refusal cases missing')

def digest(data):
    return hashlib.sha256(data).hexdigest()

def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))

def write(path, data):
    with path.open('x', encoding='utf-8', newline='\n') as stream:
        stream.write(json.dumps(data, indent=2) + '\n')

def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args])

def identity():
    if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
        raise ValueError('Only disposable native GitHub Windows qualification may publish receipts')
    commit = os.environ.get('GITHUB_SHA', '')
    run = os.environ.get('GITHUB_RUN_ID', '')
    attempt = os.environ.get('GITHUB_RUN_ATTEMPT', '')
    if not re.fullmatch('[a-f0-9]{40}', commit) or not re.fullmatch('[1-9][0-9]*', run) or not re.fullmatch('[1-9][0-9]*', attempt):
        raise ValueError('Exact commit/run/attempt required')
    if git('rev-parse', 'HEAD').decode().strip() != commit:
        raise ValueError('Checkout does not match workflow commit')
    return dict(commit=commit, runId=run, runAttempt=attempt)

def composition():
    lock = read(LOCK)
    names = lock['files']
    if lock['schema'] != 1 or len(names) != 117 or len(set(x.casefold() for x in names)) != 117:
        raise ValueError('Exact retained 117-file composition required')
    if names != sorted(names) or any(not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]*\.(ps1|cs|py|json)', x) for x in names):
        raise ValueError('Invalid flat tool inventory')
    if 'inspect-fonts.ps1' in names:
        raise ValueError('Unrelated added font operator is outside this composition')
    return names

def inventory():
    prefix = git('rev-parse', '--show-prefix').decode().strip()
    items = []
    for name in composition():
        path = ROOT / 'tools/boat' / name
        if path.is_symlink() or not path.is_file():
            raise ValueError('Tool must be a regular tracked file: ' + name)
        source = git('show', 'HEAD:' + prefix + 'tools/boat/' + name)
        actual = path.read_bytes()
        # Hash exact tested Windows bytes; only Git's CRLF conversion may differ
        # from the committed blob. No transformed/mixed composition is accepted.
        if actual != source and actual.replace(b'\r\n', b'\n') != source.replace(b'\r\n', b'\n'):
            raise ValueError('Uncommitted tool edit: ' + name)
        if not 0 < len(actual) <= 4194304:
            raise ValueError('Tool outside existing staging bounds: ' + name)
        items.append(dict(name=name, sha256=digest(actual), sourceSha256=digest(source),
                          size=len(actual), lineEndingsDiffer=actual != source))
    return items

def verify_reports(gate, directory):
    reports = []
    for name in GATES[gate]:
        path = directory / name
        report = read(path)
        if report.get('status') != 'passed':
            raise ValueError('Required actual report did not pass: ' + name)
        for field in ('boatTouched', 'boatAccess', 'physicalOutput', 'hardwareCommands', 'realApplication', 'productLaunched'):
            if field in report and report[field] is not False:
                raise ValueError('Unexpected non-fixture scope: ' + name)
        reports.append(dict(path=name, sha256=digest(path.read_bytes())))
    if gate == 'window':
        cases = read(directory / GATES[gate][1])['cases']
        if len(cases) != 16 or any(x.get('fixtureExitCode') != 0 for x in cases):
            raise ValueError('All 16 actual palette windows must exit normally')
        verify_display_report(read(directory / 'native-display-results.json'))
    if gate == 'broker':
        report = read(directory / GATES[gate][0])
        expected = {'success', 'output-connection', 'plugin-bytes', 'expired', 'consumed',
                    'receipt-recording-failure', 'palette-standard', 'palette-xnav',
                    'palette-opposite', 'palette-unarmed', 'palette-missing-intent',
                    'palette-mutated-ready', 'palette-replay-tamper'}
        if report.get('actualBroker') is not True or {x['name'] for x in report['cases']} != expected:
            raise ValueError('Actual broker palette/refusal matrix incomplete')
        prepared = read(directory / GATES[gate][1])
        if not any(x['name'] == 'palette-standard' and x.get('actualArm') is True and x.get('actualCollect') is True and x.get('scheduledTaskExit') == 0 and x.get('status') == 'passed' for x in prepared['cases']):
            raise ValueError('Actual scheduled palette Arm/Collect proof missing')
    return reports

def gate_receipt(gate, directory):
    record = dict(schema=1, gate=gate, status='passed', **identity(),
                  reports=verify_reports(gate, directory), files=inventory(),
                  compositionSha256=digest(LOCK.read_bytes()),
                  scope='Tooling only; marker/desktop fixtures; no application or boat acceptance',
                  actualBoat=False, applicationBuild=False)
    write(directory / 'gate-receipt.json', record)

def validate_gate_set(records, expected, files, composition_sha):
    if len(records) != len(GATES) or {r.get('gate') for r in records} != set(GATES):
        raise ValueError('Exactly the three independent tooling gates required')
    for record in records:
        if record.get('schema') != 1 or record.get('status') != 'passed' or any(record.get(k) != v for k, v in expected.items()):
            raise ValueError('Gate commit/run/attempt/status mismatch')
        if record.get('files') != files or record.get('compositionSha256') != composition_sha:
            raise ValueError('Tested checkout bytes differ between gates and bundle')
        if record.get('actualBoat') is not False or record.get('applicationBuild') is not False:
            raise ValueError('Unexpected gate scope')

def package(evidence, output):
    expected = identity()
    files = inventory()
    found = sorted(evidence.glob('*/gate-receipt.json'))
    records = [read(path) for path in found]
    validate_gate_set(records, expected, files, digest(LOCK.read_bytes()))
    # Retain full downloaded report identities, then independently recheck the
    # reports instead of trusting a status boolean detached from its artifact.
    for path, record in zip(found, records):
        if verify_reports(record['gate'], path.parent) != record['reports']:
            raise ValueError('Downloaded native report bytes changed')
    output.mkdir(parents=True, exist_ok=False)
    archive = output / 'review-tools.zip'
    with zipfile.ZipFile(archive, 'x', compression=zipfile.ZIP_DEFLATED) as target:
        for item in files:
            data = (ROOT / 'tools/boat' / item['name']).read_bytes()
            if digest(data) != item['sha256']:
                raise ValueError('Tool changed during bundle creation')
            entry = zipfile.ZipInfo(item['name'], (1980, 1, 1, 0, 0, 0))
            entry.compress_type = zipfile.ZIP_DEFLATED
            entry.external_attr = 0o100644 << 16
            target.writestr(entry, data)
    manifest = output / 'manifest.json'
    write(manifest, dict(schema=1, commit=expected['commit'], files=files))
    write(output / 'qualification.json', dict(schema=1, status='native-tooling-gates-passed', **expected,
          archiveSha256=digest(archive.read_bytes()), manifestSha256=digest(manifest.read_bytes()),
          gates=[dict(gate=r['gate'], receiptSha256=digest(p.read_bytes())) for p, r in zip(found, records)],
          qualifiedBase=read(LOCK), files=files, staged=False, actualBoat=False, applicationAcceptance=False,
          limitation='Only named delta gates rerun; retained base qualification and application qualification remain separate'))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--gate', choices=GATES)
    group.add_argument('--package', action='store_true')
    parser.add_argument('--evidence', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.package:
        if not args.output:
            parser.error('--package requires --output')
        package(args.evidence, args.output)
    else:
        if args.output:
            parser.error('--output is package-only')
        gate_receipt(args.gate, args.evidence)

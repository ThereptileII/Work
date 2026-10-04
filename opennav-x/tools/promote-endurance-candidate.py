#!/usr/bin/env python3
"""Promote unchanged reviewed payloads after exact native endurance evidence.

The workflow must require successful producer and endurance jobs and download
their artifacts by the exact upload-action IDs. This helper performs no network,
build, installer execution, release publication or boat operation.
"""
import argparse
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import tempfile
import zipfile

from beta2_handoff import FILES, RELEASE_NOTES, REPOSITORY, UPSTREAM, VERSION
from hardware_output_policy import require_status_only

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location('native_endurance_handoff', ROOT/'tools/native-endurance-handoff.py')
handoff = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(handoff)
MAX_BYTES = 1024 * 1024 * 1024
TAIL = ('Native package and installer lifecycle gates passed for these exact payload hashes.\n'
        'Release acceptance additionally requires same-commit complete CI and boat-PC evidence.\n'
        'Never treat an unsupported boat OpenCPN installation as qualified.\n')
PENDING = ('DEVELOPMENT BOAT REVIEW ONLY: endurance and release qualification are pending.\n' + TAIL).encode()
CANDIDATE = ('Beta 2 candidate product: native package and installer gates passed.\n' + TAIL).encode()


def candidate_note(data):
    if data == PENDING:
        return CANDIDATE
    if data == PENDING.replace(b'\n', b'\r\n'):
        return CANDIDATE.replace(b'\n', b'\r\n')
    raise ValueError('Expected original pending-endurance qualification')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def fact(data):
    return {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}


def unique(pairs):
    result = {}
    for name, value in pairs:
        require(name not in result, 'Duplicate JSON key: ' + name)
        result[name] = value
    return result


def document(data):
    return json.loads(data, object_pairs_hook=unique,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON number')))


def read(path):
    require(path.is_file() and not path.is_symlink() and path.stat().st_size <= MAX_BYTES,
            'Missing, linked or excessive input: ' + str(path))
    return path.read_bytes()


def archive(data):
    result = zipfile.ZipFile(io.BytesIO(data))
    entries = result.infolist()
    require(len(entries) == len({entry.filename for entry in entries}), 'Duplicate ZIP entry')
    require(sum(entry.file_size for entry in entries) <= MAX_BYTES, 'Excessive ZIP expansion')
    for entry in entries:
        name = entry.filename.rstrip('/')
        require(name and '\\' not in name and ':' not in name and
                not PurePosixPath(name).is_absolute() and
                all(part not in ('', '.', '..') for part in name.split('/')) and
                ((entry.external_attr >> 16) & 0o170000) != 0o120000,
                'Unsafe ZIP entry')
    require(result.testzip() is None, 'ZIP integrity failure')
    return result


def verify_payloads(review, commit):
    names = FILES | {'SHA256SUMS.txt', 'QUALIFICATION.txt'}
    require(review.is_dir() and not review.is_symlink() and
            {path.name for path in review.iterdir()} == names, 'Expected exact eight-file review inventory')
    payload = {name: read(review/name) for name in names}
    require(sum(map(len, payload.values())) <= MAX_BYTES, 'Review bundle exceeds bound')
    candidate_note(payload['QUALIFICATION.txt'])
    sums = {}
    for line in payload['SHA256SUMS.txt'].decode('utf-8').splitlines():
        match = re.fullmatch(r'([a-f0-9]{64})  ([A-Za-z0-9_.-]+)', line)
        require(match is not None and match[2] not in sums, 'Invalid or duplicate checksum row')
        sums[match[2]] = match[1]
    require(set(sums) == FILES, 'Checksums must cover exactly six payloads')
    require(all(fact(payload[name])['sha256'] == value for name, value in sums.items()), 'Payload hash changed')
    prefix = 'SKAGER-Beta2-Portable-Recovery/'
    with archive(payload['SKAGER-Beta2-Portable-Recovery.zip']) as portable:
        build = document(portable.read(prefix+'docs/PRODUCT_BUILD.json'))
        require_status_only(build)
        require(build.get('commit') == commit and build.get('version') == VERSION and
                build.get('test_fixtures') is False and build.get('build_purpose') == 'INSTALLED PRODUCT',
                'Recovery product is not exact fixture-free candidate')
        require(fact(portable.read(prefix+'app/opencpn.exe'))['sha256'] == build.get('executable_sha256'),
                'Recovery executable identity differs')
        notes_path = 'docs/'+RELEASE_NOTES
        require(portable.read(prefix+notes_path) == payload[RELEASE_NOTES], 'Recovery notes differ')
        inventory = document(portable.read(prefix+'FILE_SHA256.json'))
        require(inventory.get(notes_path) == fact(payload[RELEASE_NOTES])['sha256'], 'Recovery notes hash differs')
    with archive(payload['SKAGER-Beta2-source.zip']) as source:
        reference = document(source.read('SOURCE_REFERENCE.json'))
        require(reference.get('productCommit') == commit and reference.get('upstreamCommit') == UPSTREAM and
                reference.get('openCpnVersion') == '5.12.4', 'Corresponding source identity differs')
        notes_path = 'opennav-x/docs/beta2/'+RELEASE_NOTES
        require(source.read(notes_path) == payload[RELEASE_NOTES] and
                reference.get('files', {}).get(notes_path, {}).get('sha256') == fact(payload[RELEASE_NOTES])['sha256'],
                'Corresponding source notes differ')
    return payload


def expected_identity(environment):
    commit = environment.get('GITHUB_SHA', '')
    require(environment.get('GITHUB_REPOSITORY') == REPOSITORY and re.fullmatch('[a-f0-9]{40}', commit),
            'Exact same-repository GitHub candidate required')
    for name in ('GITHUB_RUN_ID', 'GITHUB_RUN_ATTEMPT'):
        require(re.fullmatch('[1-9][0-9]*', environment.get(name, '')) is not None, 'Exact run/attempt required')
    return {'repository': REPOSITORY, 'commit': commit, 'run_id': environment['GITHUB_RUN_ID'],
            'run_attempt': environment['GITHUB_RUN_ATTEMPT'], 'producer_job': 'windows-integration',
            'architecture': 'Win32'}


def validate_soak(directory, identity, root):
    manifest_bytes = read(directory/'runtime-manifest.json')
    report_bytes = read(directory/'results.json')
    manifest = document(manifest_bytes)
    report = document(report_bytes)
    receipt = document(read(directory/'handoff-qualified.json'))
    require(type(manifest.get('schema')) is int and manifest['schema'] == 1 and manifest.get('owner') == 'SKAGER.NativeEnduranceHandoff.1' and
            manifest.get('identity') == identity, 'Endurance handoff identity differs')
    require(type(receipt.get('schema')) is int and receipt['schema'] == 1 and receipt.get('owner') == 'SKAGER.NativeEnduranceResult.1' and
            receipt.get('identity') == identity and receipt.get('consumer_job') == 'windows-endurance',
            'Endurance result identity differs')
    require(receipt.get('stage_manifest') == fact(manifest_bytes) and receipt.get('report') == fact(report_bytes),
            'Endurance manifest/report identity changed')
    required = document(read(root/'release/qualification.json')).get('enduranceSeconds')
    require(type(required) is int and required >= 10800 and
            manifest.get('required_seconds') == required and receipt.get('required_seconds') == required,
            'Full qualification duration required')
    for name in ('tools/soak-runtime.py', 'release/qualification.json'):
        local = read(root/name)
        require(manifest.get('sources', {}).get(name) in (fact(local), fact(local.replace(b'\n', b'\r\n'))),
                'Endurance source differs from checked-out candidate: ' + name)
    executable = manifest.get('files', {}).get('build/xnav-install/opencpn.exe')
    require(isinstance(executable, dict) and set(executable) == {'bytes', 'sha256'} and
            type(executable['bytes']) is int and executable['bytes'] > 0 and
            isinstance(executable['sha256'], str) and re.fullmatch('[a-f0-9]{64}', executable['sha256']),
            'Invalid handoff executable identity')
    # The consumer and promotion use one pure duration/build identity gate.
    expected = handoff.validate_report(report, manifest, identity)
    expected.update(schema=1, owner='SKAGER.NativeEnduranceResult.1', runtime_unchanged=True,
                    stage_manifest=fact(manifest_bytes), report=fact(report_bytes))
    require(receipt == expected, 'Qualified endurance receipt differs from validated raw result')
    return receipt


def promote(review, output, payload):
    require(not output.exists() and not output.is_symlink(), 'Promotion output must be new')
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix='endurance-promotion-', dir=output.parent))
    try:
        for name, data in payload.items():
            # Recheck downloaded source bytes before the atomic promotion.
            require(read(review/name) == data, 'Review changed during promotion')
            (temporary/name).write_bytes(candidate_note(data) if name == 'QUALIFICATION.txt' else data)
        temporary.rename(output)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--review-dir', type=Path, default=ROOT/'build/beta-boat-review')
    parser.add_argument('--soak-dir', type=Path, default=ROOT/'evidence/local/soak')
    parser.add_argument('--output', type=Path, default=ROOT/'build/beta-artifacts')
    args = parser.parse_args()
    identity = expected_identity(os.environ)
    validate_soak(args.soak_dir, identity, ROOT)
    payload = verify_payloads(args.review_dir, identity['commit'])
    promote(args.review_dir, args.output, payload)
    print('Promoted unchanged candidate payloads:', identity['commit'])


if __name__ == '__main__':
    main()

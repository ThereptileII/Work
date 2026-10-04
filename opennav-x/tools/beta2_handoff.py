"""Publish unchanged Beta 2 candidate bytes after CI and recorded boat review.

This is a release-evidence collector, not an installer, updater or boat command.
It cannot qualify an unfinished run or substitute for the recorded human review.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import urllib.error
import urllib.parse
import urllib.request
import zipfile

REPOSITORY = 'ThereptileII/Work'
UPSTREAM = '37fd0cddb7334fe489e9f18aa163977a9c5c84f7'
VERSION = '0.4.0-beta2'
RELEASE_NOTES = 'SKAGER-Beta2-Release-Notes.md'
FILES = {
    'SKAGER-Beta2-Setup.exe', 'SKAGER-Beta2-Portable-Recovery.zip',
    'SKAGER-Beta2-source.zip', 'SKAGER-Beta2-Install-Guide.md',
    'SKAGER-Beta2-Test-Guide.md', RELEASE_NOTES,
}
LEGACY_FILES = {name.replace('SKAGER-Beta2-', 'OpenNavX-Beta2-') for name in FILES}


def bundle_prefix(record):
    names = set(record.get('payloadSha256', {}))
    if names == FILES:
        return 'SKAGER-Beta2-'
    if names == LEGACY_FILES:
        return 'OpenNavX-Beta2-'
    raise ValueError('All six reviewed payload hashes required in one current or historical bundle; mixed names refused')


JOBS = {
    'contracts (windows-2022)', 'contracts (ubuntu-24.04)',
    'Native Win32 commissioning restart process boundary',
    'Linux integrated regression gate', 'Native MSVC upstream Win32 ABI on Windows x64',
    'Boat C6 producer expiry compile (no flashing)', 'linux',
    'Official OpenCPN installation prerequisite', 'Approved OpenCPN ABI on Windows x64',
    'Native boat recovery and commissioning contracts',
    'Actual official OpenCPN portable upgrade caution (sv)',
    'Actual official OpenCPN portable upgrade caution (en_US)',
    'Fixed guarded mode UI actions on disposable native windows',
    'Actual restart broker with disposable marker-only installation',
    'Native private chart loader refusal and fallback gate',
    'Native MSVC XNav / Legacy / Safe slice',
    'Wait for exact same-run native fixture runtime',
    'Concurrent exact native trip and resource stability',
    'Join native functional gates and exact endurance before candidate promotion',
    'Publish Beta after both platform and installer gates',
}
BOAT_GATES = {
    'feedback', 'primaryScreens', 'realChartsAndData', 'modeLifecycle',
    'maintenance', 'remoteRecovery', 'olderVersions',
}
MAX_ARCHIVE = 1024 * 1024 * 1024


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def sha(value, length=64):
    return isinstance(value, str) and re.fullmatch('[a-f0-9]{%d}' % length, value) is not None


def positive(value):
    return type(value) is int and value > 0


def validate_acceptance(record, root):
    require(record.get('schema') == 1 and type(record['schema']) is int and
            record.get('owner') == 'OpenNavX.Beta2.BoatAcceptance.1', 'Unknown acceptance record')
    require(record.get('version') == VERSION and record.get('status') == 'accepted',
            'Beta 2 boat acceptance is not complete')
    require(record.get('physicalCommands') == 0 and type(record['physicalCommands']) is int,
            'Remote physical commands are outside this handoff')
    require(sha(record.get('commit'), 40), 'Exact candidate commit required')
    require(positive(record.get('runId')) and positive(record.get('runAttempt')),
            'Exact completed CI run and attempt required')
    artifact = record.get('artifact', {})
    require(positive(artifact.get('id')) and positive(artifact.get('bytes')) and
            artifact['bytes'] <= MAX_ARCHIVE and sha(artifact.get('sha256')),
            'Exact downloaded candidate artifact identity required')
    hashes = record.get('payloadSha256', {})
    bundle_prefix(record)
    require(all(sha(value) for value in hashes.values()),
            'All six reviewed payload hashes required')
    gates = record.get('boatGates', {})
    require(set(gates) == BOAT_GATES, 'Incomplete boat review coverage')
    root = Path(root).resolve()
    for name, evidence in gates.items():
        require(evidence.get('status') == 'passed' and sha(evidence.get('sha256')),
                'Boat gate is not accepted: ' + name)
        relative = evidence.get('path', '')
        require(isinstance(relative, str) and relative.startswith('docs/') and
                '\\' not in relative and '..' not in relative.split('/'), 'Invalid evidence path')
        path = root / relative
        require(path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(root),
                'Evidence must be a committed local project document')
        require(digest(path.read_bytes()) == evidence['sha256'], 'Boat evidence changed: ' + name)
    require(isinstance(record.get('limitations'), list), 'Explicit remaining limitations required')


def validate_ci(record, run, jobs, artifact):
    require(run.get('id') == record['runId'] and run.get('run_attempt') == record['runAttempt'] and
            run.get('head_sha') == record['commit'], 'CI run identity differs')
    require(run.get('status') == 'completed' and run.get('conclusion') == 'success' and
            run.get('path') == '.github/workflows/opennav-baseline.yml' and
            run.get('repository', {}).get('full_name') == REPOSITORY,
            'Complete same-repository product qualification required')
    entries = jobs.get('jobs', [])
    require(jobs.get('total_count') == len(entries) and len(entries) == len(JOBS) and
            {job.get('name') for job in entries} == JOBS, 'Required CI job coverage differs')
    require(all(job.get('status') == 'completed' and job.get('conclusion') == 'success' and
                job.get('head_sha') == record['commit'] and job.get('run_id') == record['runId'] and
                job.get('run_attempt') == record['runAttempt'] for job in entries),
            'A product gate is failed, skipped, unfinished or from another attempt')
    expected = record['artifact']
    require(artifact.get('id') == expected['id'] and artifact.get('expired') is False and
            artifact.get('name') == 'beta-candidate-' + record['commit'] and
            artifact.get('size_in_bytes') == expected['bytes'] and
            artifact.get('digest') == 'sha256:' + expected['sha256'] and
            artifact.get('workflow_run', {}).get('id') == record['runId'] and
            artifact['workflow_run'].get('head_sha') == record['commit'],
            'Artifact is not the exact complete candidate')


def verify_payload(record, archive):
    name_prefix = bundle_prefix(record)
    release_notes = name_prefix + 'Release-Notes.md'
    require(len(archive) == record['artifact']['bytes'] and
            digest(archive) == record['artifact']['sha256'], 'Downloaded artifact hash/size differs')
    with zipfile.ZipFile(io.BytesIO(archive)) as outer:
        expected = set(record['payloadSha256']) | {'SHA256SUMS.txt', 'QUALIFICATION.txt'}
        require(len(outer.infolist()) == len(expected) and set(outer.namelist()) == expected,
                'Unexpected, duplicate or nested artifact entries')
        require(sum(entry.file_size for entry in outer.infolist()) <= MAX_ARCHIVE,
                'Artifact expanded size is excessive')
        require(all(not entry.is_dir() and ((entry.external_attr >> 16) & 0o170000) != 0o120000
                    for entry in outer.infolist()), 'Artifact links/directories refused')
        require(outer.testzip() is None, 'Artifact ZIP integrity failed')
        payload = {name: outer.read(name) for name in expected}
    sums = {}
    for line in payload['SHA256SUMS.txt'].decode('utf-8').splitlines():
        match = re.fullmatch(r'([a-f0-9]{64})  ([A-Za-z0-9_.-]+)', line)
        require(match is not None and match[2] not in sums, 'Invalid/duplicate payload checksum')
        sums[match[2]] = match[1]
    require(sums == record['payloadSha256'], 'Checksum manifest differs from reviewed payloads')
    require(all(digest(payload[name]) == value for name, value in sums.items()),
            'A payload differs from the reviewed bytes')
    with zipfile.ZipFile(io.BytesIO(payload[name_prefix + 'Portable-Recovery.zip'])) as portable:
        prefix = name_prefix + 'Portable-Recovery/'
        require(len(portable.namelist()) == len(set(portable.namelist())), 'Duplicate portable entry')
        notes_path = 'docs/' + release_notes
        require(prefix + notes_path in portable.namelist(), 'Portable release notes missing')
        require(portable.read(prefix + notes_path) == payload[release_notes], 'Portable release notes differ')
        file_hashes = json.loads(portable.read(prefix + 'FILE_SHA256.json'))
        require(file_hashes.get(notes_path) == digest(payload[release_notes]), 'Portable release notes hash differs')
        build = json.loads(portable.read(prefix + 'docs/PRODUCT_BUILD.json'))
        require(build.get('version') == VERSION and build.get('commit') == record['commit'] and
                build.get('test_fixtures') is False and build.get('build_purpose') == 'INSTALLED PRODUCT',
                'Portable is not the exact fixture-free product')
        require(digest(portable.read(prefix + 'app/opencpn.exe')) == build.get('executable_sha256'),
                'Portable executable identity differs')
    with zipfile.ZipFile(io.BytesIO(payload[name_prefix + 'source.zip'])) as source:
        require(len(source.namelist()) == len(set(source.namelist())), 'Duplicate source entry')
        reference = json.loads(source.read('SOURCE_REFERENCE.json'))
        notes_path = 'opennav-x/docs/beta2/' + release_notes
        require(notes_path in source.namelist(), 'Corresponding-source release notes missing')
        require(source.read(notes_path) == payload[release_notes], 'Corresponding-source release notes differ')
        require(reference.get('files', {}).get(notes_path, {}).get('sha256') == digest(payload[release_notes]),
                'Corresponding-source release notes hash differs')
        require(reference.get('productCommit') == record['commit'] and
                reference.get('upstreamCommit') == UPSTREAM and reference.get('openCpnVersion') == '5.12.4',
                'Corresponding source identity differs')
    return payload


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def api(path, token):
    request = urllib.request.Request('https://api.github.com/repos/' + REPOSITORY + '/' + path,
                                    headers={'Authorization': 'Bearer ' + token,
                                             'Accept': 'application/vnd.github+json'})
    with urllib.request.build_opener(NoRedirect).open(request, timeout=60) as response:
        return json.load(response)


def download(artifact_id, token):
    # GitHub's archive endpoint redirects to its temporary object-store URL.
    # The token is sent only to api.github.com, never forwarded to that URL.
    try:
        api('actions/artifacts/%d/zip' % artifact_id, token)
    except urllib.error.HTTPError as error:
        require(error.code == 302, 'Artifact download did not return its expected redirect')
        location = error.headers.get('Location', '')
        error.close()
    else:
        raise ValueError('Artifact archive redirect missing')
    parsed = urllib.parse.urlsplit(location)
    require(parsed.scheme == 'https' and parsed.hostname and not parsed.username and not parsed.password,
            'Unsafe artifact location')
    with urllib.request.urlopen(location, timeout=120) as response:
        data = response.read(MAX_ARCHIVE + 1)
    require(len(data) <= MAX_ARCHIVE, 'Artifact download exceeds limit')
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--acceptance', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    record = json.loads(args.acceptance.read_text())
    validate_acceptance(record, root)
    require(not args.output.exists(), 'Output already exists')
    token = os.environ['GH_TOKEN']
    run = api('actions/runs/%d' % record['runId'], token)
    jobs = api('actions/runs/%d/attempts/%d/jobs?per_page=100' %
               (record['runId'], record['runAttempt']), token)
    artifact = api('actions/artifacts/%d' % record['artifact']['id'], token)
    validate_ci(record, run, jobs, artifact)
    payload = verify_payload(record, download(record['artifact']['id'], token))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix='beta2-handoff-', dir=args.output.parent))
    try:
        for name, data in payload.items():
            (temporary / name).write_bytes(data)
        (temporary / 'BETA2_ACCEPTANCE.json').write_text(json.dumps(record, indent=2) + '\n')
        (temporary / 'ACCEPTANCE.md').write_text((
            '# SKAGER Beta 2 test package\n\n'
            'Download **SKAGER Windows test package** from the linked Actions run. '
            'Extract that outer download first; it contains the files below.\n\n'
            '1. Close OpenCPN and older SKAGER/OpenNav copies.\n'
            '2. Read SKAGER-Beta2-Release-Notes.md and SKAGER-Beta2-Install-Guide.md.\n'
            '3. Run **SKAGER-Beta2-Setup.exe** for the real OpenCPN installation.\n'
            '4. Follow SKAGER-Beta2-Test-Guide.md.\n\n'
            '**SKAGER-Beta2-Portable-Recovery.zip** is inside this download. '
            'Extract it separately only for isolated recovery/testing; it does '
            'not automatically use your normal charts or connections.\n\n'
            'The payloads are unchanged from complete CI run %d, commit `%s`.\n\n'
            'See BETA2_ACCEPTANCE.json for boat-review evidence and limitations. '
            'QUALIFICATION.txt is the original build-time candidate note; the '
            'later acceptance record accompanies it without rewriting tested files.\n\n'
            'This Beta is not approved for navigation or production use. '
            'No physical actuator command was part of remote validation.\n' %
            (record['runId'], record['commit'])).replace('SKAGER-Beta2-', bundle_prefix(record)))
        temporary.rename(args.output)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    print('Verified unchanged Beta 2 payloads:', record['commit'], record['runId'])


if __name__ == '__main__':
    main()

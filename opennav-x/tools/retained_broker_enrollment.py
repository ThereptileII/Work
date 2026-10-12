"""Authenticate original native component bytes for a separate SCM-only retest.

The original producer remains failed. Passing producer steps authenticate its
retained components, never installed updater or whole-product acceptance.
"""
import argparse
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import tempfile
import zipfile

from github_release_delivery import GitHub, decimal, file_hash, read_json, sha
from release_manifest import require

REPO = 'ThereptileII/Work'
WORKFLOW = '.github/workflows/update-verifier-probe.yml'
JOB = 'verify (windows-2022, 386)'
MAX_BYTES = 2 << 30
PRODUCER_STEPS = (
    'Verify signed trust, custody and launcher contracts',
    'Run go vet ./...',
    'Verify updater source package rules',
    'Compile native launcher with exact dependency sources',
    'Build same-commit read-only trust validator',
    'Verify retained launcher transfer without rebuilding',
    'Compile inactive broker against the exact launcher source closure',
    'Compile inactive provisioner against the retained native prerequisites',
    'Compile inactive baseline handoff against retained native prerequisites',
    'Package retained operator tools without rebuilding native components',
    'Native authenticated startup and interruption records',
    'Actual installer interrupted rollback and historical generations',
    'Old lifecycle refuses protected ownership and uncertain observations',
)
FAILED_STEP = 'Actual fresh broker enrollment on disposable Windows'


def authenticate(gh, selection):
    require(set(selection) == {'schema', 'repository', 'runId', 'runAttempt',
            'headSha', 'jobId', 'artifactId', 'artifactName', 'artifactDigest'} and
            type(selection['schema']) is int and selection['schema'] == 1 and
            selection['repository'] == REPO == gh.repo, 'Exact original component selection required')
    for field in ('runId', 'runAttempt', 'jobId', 'artifactId'):
        decimal(selection[field])
    sha(selection['headSha'])
    require(re.fullmatch(r'sha256:[a-f0-9]{64}', selection['artifactDigest']), 'Exact artifact digest required')
    run = gh.api(f"{gh.base}/actions/runs/{selection['runId']}/attempts/{selection['runAttempt']}")
    require(str(run.get('id')) == selection['runId'] and
            str(run.get('run_attempt')) == selection['runAttempt'] and
            run.get('head_sha') == selection['headSha'] and run.get('path') == WORKFLOW and
            run.get('head_repository', {}).get('full_name') == REPO and
            run.get('event') == 'workflow_dispatch' and run.get('status') == 'completed' and
            run.get('conclusion') == 'failure', 'Original failed producer identity differs')
    job = gh.api(f"{gh.base}/actions/jobs/{selection['jobId']}")
    require(str(job.get('id')) == selection['jobId'] and str(job.get('run_id')) == selection['runId'] and
            str(job.get('run_attempt')) == selection['runAttempt'] and job.get('head_sha') == selection['headSha'] and
            job.get('name') == JOB and job.get('status') == 'completed' and
            job.get('conclusion') == 'failure', 'Original component job differs')
    steps = job.get('steps')
    require(isinstance(steps, list), 'Original producer steps unavailable')
    for name in PRODUCER_STEPS:
        matches = [s for s in steps if s.get('name') == name]
        require(len(matches) == 1 and matches[0].get('status') == 'completed' and
                matches[0].get('conclusion') == 'success', 'Original required component producer step did not pass')
    failed = [s.get('name') for s in steps if s.get('conclusion') == 'failure']
    require(failed == [FAILED_STEP], 'Original failure is outside the selected SCM-only retest')
    artifact = gh.api(f"{gh.base}/actions/artifacts/{selection['artifactId']}")
    require(selection['artifactName'] == 'update-verifier-windows-2022-386-' + selection['headSha'] and
            str(artifact.get('id')) == selection['artifactId'] and artifact.get('name') == selection['artifactName'] and
            artifact.get('digest') == selection['artifactDigest'] and artifact.get('expired') is False and
            str(artifact.get('workflow_run', {}).get('id')) == selection['runId'] and
            artifact.get('workflow_run', {}).get('head_sha') == selection['headSha'] and
            type(artifact.get('size_in_bytes')) is int and 0 < artifact['size_in_bytes'] <= MAX_BYTES,
            'Original artifact provenance, digest, expiry or bound differs')
    return dict(selection, scope='retained native broker SCM retest only', originalConclusion='failure')


def unpack_broker(archive, output):
    require(not output.exists() and not output.is_symlink(), 'Fresh retained component destination required')
    with zipfile.ZipFile(archive) as stream:
        items = stream.infolist()
        require(0 < len(items) <= 1024 and sum(i.file_size for i in items) <= MAX_BYTES,
                'Retained artifact inventory exceeds bound')
        seen, selected = set(), []
        for item in items:
            name = item.orig_filename
            parts = name.split('/')
            require(name == item.filename and not item.is_dir() and not PurePosixPath(name).is_absolute() and
                    '\\' not in name and all(p and p not in {'.', '..'} and p.rstrip(' .') == p and
                    not re.search(r'[\x00-\x1f<>:"|?*]', p) and p.split('.')[0].upper() not in
                    {'CON', 'PRN', 'AUX', 'NUL', *[f'COM{i}' for i in range(1, 10)],
                     *[f'LPT{i}' for i in range(1, 10)]} for p in parts), 'Unsafe retained artifact member')
            require(name.casefold() not in seen and stat.S_IFMT(item.external_attr >> 16) in {0, stat.S_IFREG} and
                    not item.flag_bits & 1, 'Aliased, linked, encrypted or special retained member')
            seen.add(name.casefold())
            if parts[0] == 'broker-install':
                selected.append(item)
        require(all('/'.join(n.split('/')[:i]) not in seen for n in seen for i in range(1, len(n.split('/')))),
                'Retained artifact file/directory collision')
        require(any(i.filename == 'broker-install/skager-update-broker.exe' for i in selected) and
                any(i.filename == 'broker-install/opennav/third-party/update-broker/build.json' for i in selected),
                'Original broker prerequisites absent')
        output.mkdir(parents=True)
        for item in selected:
            target = output.joinpath(*item.filename.split('/')[1:])
            target.parent.mkdir(parents=True, exist_ok=True)
            with stream.open(item) as source, target.open('xb') as destination:
                shutil.copyfileobj(source, destination)


def fetch(gh, selection, output, receipt, helper_commit):
    sha(helper_commit)
    require(not receipt.exists() and not receipt.is_symlink() and
            not receipt.resolve().is_relative_to(output.resolve()), 'Fresh external retained receipt required')
    authority = authenticate(gh, selection)
    with tempfile.TemporaryDirectory(prefix='skager-retained-broker-') as temporary:
        archive = Path(temporary) / 'artifact.zip'
        with archive.open('xb') as destination:
            gh._run(['api', '--allow-escape-sequences', f"{gh.base}/actions/artifacts/{selection['artifactId']}/zip"], output=destination)
        require(archive.stat().st_size <= MAX_BYTES and 'sha256:' + file_hash(archive) == selection['artifactDigest'],
                'Downloaded original artifact digest differs')
        unpack_broker(archive, output)
    # Exact native PE/build/source and launcher prerequisites are checked before
    # any privileged creation. This verifier never compiles or executes a broker.
    from broker_package import verify_broker_package
    verify_broker_package(output, selection['headSha'])
    authority['helperCommit'] = helper_commit
    receipt.parent.mkdir(parents=True, exist_ok=True)
    with receipt.open('x', encoding='utf-8') as destination:
        destination.write(json.dumps(authority, indent=2) + '\n')
    return authority


def validate_receipt(gh, selection, receipt, helper_commit):
    """Reauthenticate the original producer immediately before native effects."""
    sha(helper_commit)
    expected = authenticate(gh, selection)
    expected['helperCommit'] = helper_commit
    require(read_json(receipt) == expected, 'Retained receipt differs from actual original producer/helper')
    return expected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selection', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--receipt', type=Path, required=True)
    parser.add_argument('--validate-only', action='store_true')
    parser.add_argument('--expected-producer-commit')
    args = parser.parse_args()
    require(os.environ.get('GITHUB_REPOSITORY') == REPO, 'Actual CI repository differs')
    selection = read_json(args.selection)
    helper = os.environ.get('GITHUB_SHA', '')
    if args.validate_only:
        require(args.output is None and args.expected_producer_commit == selection['headSha'],
                'Exact original producer required for receipt validation')
        result = validate_receipt(GitHub(REPO), selection, args.receipt, helper)
    else:
        require(args.output is not None and args.expected_producer_commit is None,
                'Fresh retained output required for download')
        result = fetch(GitHub(REPO), selection, args.output, args.receipt, helper)
    print(json.dumps(result, sort_keys=True))


if __name__ == '__main__':
    main()

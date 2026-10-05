#!/usr/bin/env python3
"""Fetch immutable CI inputs using authenticated GitHub metadata, never 'latest'."""
import argparse
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import tempfile
import zipfile

from github_release_delivery import GitHub, decimal, file_hash, read_json, repository, sha
from release_manifest import require

DEPENDENCIES = '.github/workflows/skager-windows-dependencies.yml'
STAGING = '.github/workflows/opennav-baseline.yml'
MAX_BYTES = 8 * 1024**3


def authenticated_artifact(gh, selection, *, kind):
    require(kind in {'dependencies', 'staging'}, 'Unsupported input kind')
    required = {'repository', 'runId', 'runAttempt', 'headSha', 'artifactId', 'artifactName', 'artifactDigest'}
    require(set(selection) == required, 'Exact immutable artifact selection required')
    require(selection['repository'] == gh.repo, 'Repository differs')
    run_id, attempt = decimal(selection['runId']), decimal(selection['runAttempt'])
    sha(selection['headSha'])
    decimal(selection['artifactId'])
    require(re.fullmatch(r'sha256:[a-f0-9]{64}', selection['artifactDigest']), 'Artifact digest required')
    workflow, job = (DEPENDENCIES, 'windows-dependencies') if kind == 'dependencies' else (STAGING, 'windows-integration')
    run = gh.api(f'{gh.base}/actions/runs/{run_id}/attempts/{attempt}')
    require(str(run.get('id')) == run_id and str(run.get('run_attempt')) == attempt and
            run.get('head_sha') == selection['headSha'] and run.get('path') == workflow and
            run.get('event') in {'push', 'workflow_dispatch'} and
            run.get('head_repository', {}).get('full_name') == gh.repo,
            'Producer workflow/attempt/checkout provenance differs')
    if kind == 'dependencies':
        require(run.get('status') == 'completed' and run.get('conclusion') == 'success',
                'Dependency producer run must have completed successfully')
    jobs = gh.pages(f'{gh.base}/actions/runs/{run_id}/attempts/{attempt}/jobs')
    # Dedicated producer jobs intentionally have stable names, not matrix labels.
    matches = [entry for entry in jobs if entry.get('name') == job]
    require(len(matches) == 1 and matches[0].get('status') == 'completed' and
            matches[0].get('conclusion') == 'success' and
            str(matches[0].get('run_id')) == run_id and
            str(matches[0].get('run_attempt')) == attempt and
            matches[0].get('head_sha') == selection['headSha'], 'Producer job did not pass')
    artifact = gh.api(f"{gh.base}/actions/artifacts/{selection['artifactId']}")
    require(str(artifact.get('id')) == selection['artifactId'] and
            artifact.get('name') == selection['artifactName'] and artifact.get('expired') is False and
            artifact.get('digest') == selection['artifactDigest'] and
            str(artifact.get('workflow_run', {}).get('id')) == run_id and
            artifact.get('workflow_run', {}).get('head_sha') == selection['headSha'] and
            type(artifact.get('size_in_bytes')) is int and 0 < artifact['size_in_bytes'] <= MAX_BYTES,
            'Artifact provenance, expiry, size or digest differs')
    prefix = ('windows-dependencies-[a-f0-9]{64}' if kind == 'dependencies'
              else 'staging-build-' + selection['headSha'])
    require(re.fullmatch(prefix + '-run' + run_id + '-attempt' + attempt, selection['artifactName']),
            'Artifact name does not bind the exact producer attempt')
    return dict(schemaVersion=1, repository=gh.repo, workflowPath=workflow, runId=run_id,
                runAttempt=attempt, headSha=selection['headSha'], job=job, conclusion='success',
                artifactId=selection['artifactId'], artifactName=selection['artifactName'],
                artifactDigest=selection['artifactDigest'])


def unpack(archive, destination, kind):
    """Inspect all ZIP members before writing; Windows-safe paths on all hosts."""
    require(kind in {'dependencies', 'staging'}, 'Unsupported input kind')
    require(not destination.exists() and not destination.is_symlink(), 'Fresh artifact directory required')
    with zipfile.ZipFile(archive) as stream:
        entries = stream.infolist()
        require(0 < len(entries) <= 100000 and sum(x.file_size for x in entries) <= MAX_BYTES,
                'Artifact inventory exceeds bounds')
        seen = set()
        for item in entries:
            name = item.orig_filename
            require(name == item.filename and '\\' not in name and not item.is_dir(), 'Noncanonical ZIP member')
            parts = name.split('/')
            require(all(p and p not in {'.', '..'} and p.rstrip(' .') == p and
                        not re.search(r'[\x00-\x1f<>:"|?*]', p) and
                        p.split('.')[0].upper() not in {'CON','PRN','AUX','NUL',*[f'COM{i}' for i in range(1,10)],*[f'LPT{i}' for i in range(1,10)]}
                        for p in parts), 'Unsafe ZIP path')
            require(not PurePosixPath(name).is_absolute() and name.casefold() not in seen,
                    'Aliased ZIP member')
            seen.add(name.casefold())
            require(stat.S_IFMT(item.external_attr >> 16) in {0, stat.S_IFREG} and
                    not item.flag_bits & 1, 'Linked, special or encrypted ZIP member')
            allowed = (name == 'bundle.json' or name.startswith('payload/')) if kind == 'dependencies' else name in {'STAGING_BUILD_INPUTS.zip', 'receipt.json'}
            require(allowed, 'Unexpected outer artifact member')
        require(('bundle.json' in seen) if kind == 'dependencies' else seen == {'staging_build_inputs.zip','receipt.json'},
                'Missing artifact manifest')
        # Also reject file/directory collisions before touching disk.
        require(all('/'.join(name.split('/')[:i]) not in seen for name in seen
                    for i in range(1, len(name.split('/')))), 'ZIP file/directory collision')
        destination.mkdir(parents=True)
        for item in entries:
            target = destination.joinpath(*item.filename.split('/'))
            target.parent.mkdir(parents=True, exist_ok=True)
            with stream.open(item) as source, target.open('xb') as out:
                shutil.copyfileobj(source, out)


def fetch(gh, selection, output, provenance, *, kind):
    require(not provenance.exists() and not provenance.is_symlink() and
            not provenance.resolve().is_relative_to(output.resolve()),
            'Fresh external provenance location required')
    authority = authenticated_artifact(gh, selection, kind=kind)
    with tempfile.TemporaryDirectory(prefix='skager-ci-') as temp:
        archive = Path(temp) / 'artifact.zip'
        with archive.open('xb') as stream:
            gh._run(['api', f"{gh.base}/actions/artifacts/{selection['artifactId']}/zip"], output=stream)
        require(archive.stat().st_size <= MAX_BYTES and
                'sha256:' + file_hash(archive) == selection['artifactDigest'], 'Downloaded artifact digest differs')
        unpack(archive, output, kind)
    if kind == 'dependencies':
        authority['bundleSha256'] = file_hash(output / 'bundle.json')
    else:
        authority['archiveSha256'] = file_hash(output / 'STAGING_BUILD_INPUTS.zip')
    provenance.parent.mkdir(parents=True, exist_ok=True)
    provenance.write_text(json.dumps(authority, indent=2) + '\n', encoding='utf-8')
    return authority


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=('dependencies', 'staging'))
    parser.add_argument('--selection', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--provenance', type=Path, required=True)
    args = parser.parse_args()
    selection = read_json(args.selection)
    require(os.environ.get('GITHUB_REPOSITORY') == selection['repository'], 'Current repository differs')
    result = fetch(GitHub(repository(selection['repository'])), selection, args.output,
                   args.provenance, kind=args.kind)
    print(json.dumps(result, sort_keys=True))


if __name__ == '__main__':
    main()

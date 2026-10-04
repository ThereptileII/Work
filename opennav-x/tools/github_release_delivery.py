#!/usr/bin/env python3
"""Immutable, draft-only GitHub release transport. Credentials stay in GH_TOKEN."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import zipfile

import release_manifest as manifest

STAGING_WORKFLOW = '.github/workflows/opennav-baseline.yml'
PRODUCTION_WORKFLOW = '.github/workflows/skager-production.yml'
RECEIPT = 'PRODUCTION.json'
require = manifest.require


def valid(pattern, value, message):
    require(manifest.matches(pattern, value), message)
    return value


def repository(value):
    valid(r'[A-Za-z0-9_.-]{1,100}/[A-Za-z0-9_.-]{1,100}', value, 'Explicit safe repository required')
    require(all(part not in {'.', '..'} for part in value.split('/')), 'Unsafe repository path')
    return value


def decimal(value):
    return valid(r'[1-9][0-9]{0,19}', value, 'Canonical run/asset identity required')


def sha(value):
    return valid(r'[0-9a-f]{40}', value, 'Exact lowercase commit required')


def read_json(path):
    require(path.is_file() and not path.is_symlink() and path.stat().st_size <= 1024 * 1024,
            'Regular bounded JSON required')
    result = manifest.strict_json(path.read_bytes())
    require(isinstance(result, dict), 'JSON object required')
    return result


def file_hash(path):
    return manifest.file_record(path)['sha256']


class GitHub:
    def __init__(self, repo):
        self.repo = repository(repo)
        self.base = f'repos/{self.repo}'

    def _run(self, arguments, *, data=None, output=None):
        # No shell, no token arguments, and never echo GH output/errors on failure.
        result = subprocess.run(['gh', *arguments], input=data, stdout=output or subprocess.PIPE,
                                stderr=subprocess.PIPE, check=False)
        require(result.returncode == 0, 'GitHub request failed; inspect the authenticated workflow privately')
        return result.stdout

    def api(self, endpoint, payload=None):
        arguments = ['api', endpoint]
        if payload is not None:
            arguments += ['--method', 'POST', '--input', '-']
        raw = self._run(arguments, data=None if payload is None else json.dumps(payload).encode())
        return manifest.strict_json(raw)

    def pages(self, endpoint):
        result = []
        for page in range(1, 10001):
            items = self.api(f'{endpoint}?per_page=100&page={page}')
            # Workflow jobs use an envelope, releases/assets use an array.
            if isinstance(items, dict):
                items = items.get('jobs')
            require(isinstance(items, list), 'Unexpected paginated GitHub response')
            result.extend(items)
            if len(items) < 100:
                return result
        raise ValueError('GitHub pagination limit exceeded')

    def release(self, tag):
        matches = [item for item in self.pages(self.base + '/releases') if item.get('tag_name') == tag]
        require(len(matches) <= 1, 'Ambiguous release tag')
        return matches[0] if matches else None

    def assets(self, release):
        return self.pages(f"{self.base}/releases/{decimal(str(release['id']))}/assets")

    def download(self, asset, path):
        ident = decimal(str(asset['id']))
        with path.open('xb') as stream:
            self._run(['api', f'{self.base}/releases/assets/{ident}',
                       '-H', 'Accept: application/octet-stream'], output=stream)

    def create(self, tag, commit, name, prerelease):
        return self.api(self.base + '/releases', dict(tag_name=tag, target_commitish=commit,
                        name=name, draft=True, prerelease=prerelease, make_latest='false',
                        body='Immutable candidate delivery. Public access remains closed.'))

    def upload(self, release, path):
        manifest.safe_name(path.name)
        ident = decimal(str(release['id']))
        self._run(['api', f'https://uploads.github.com/repos/{self.repo}/releases/{ident}/assets?name={path.name}',
                   '--method', 'POST', '-H', 'Content-Type: application/octet-stream', '--input', str(path)])


def check_release(release, tag, commit, prerelease):
    require(isinstance(release, dict) and release.get('tag_name') == tag and
            release.get('target_commitish') == commit and release.get('draft') is True and
            release.get('prerelease') is prerelease, 'Release identity/channel/draft differs')


def staging_tag(record):
    return 'skager-staging-' + record['candidateId']


def check_tag(tag):
    valid(r'skager-staging-[A-Za-z0-9.-]{1,180}', tag, 'Explicit Staging tag required')


def local_record(directory):
    record = manifest.verify(directory)
    qualification = read_json(directory / 'QUALIFICATION.json')
    require(set(qualification) == {'schemaVersion', 'channel', 'commit', 'runId', 'runAttempt',
            'gates', 'designReview', 'endurance', 'publicAccess'}, 'Unexpected Staging qualification fields')
    require(type(qualification['schemaVersion']) is int and qualification['schemaVersion'] == 1 and
            qualification['channel'] == 'staging' and qualification['commit'] == record['commit'] and
            qualification['runId'] == record['runId'] and qualification['runAttempt'] == record['runAttempt'] and
            qualification['gates'] == dict(linux='passed', windows='passed', installer='passed',
                                            package='passed', restart='passed') and
            qualification['designReview'] in {'not-requested', 'requested'} and
            qualification['endurance'] == 'skipped' and qualification['publicAccess'] is False,
            'Staging qualification gates or identity differ')
    support = read_json(directory / 'RETEST_SUPPORT.json')
    require(set(support) == {'artifactName', 'runId', 'runAttempt', 'commit', 'archiveName', 'sha256', 'size'},
            'Unexpected retest support fields')
    require(support['commit'] == record['commit'] and support['runId'] == record['runId'] and
            support['runAttempt'] == record['runAttempt'] and
            support['artifactName'] == 'staging-retest-' + record['commit'] + '-attempt' + record['runAttempt'] and
            support['archiveName'] == 'SKAGER-Beta2-Retest-Support.zip', 'Retest support identity differs')
    valid(r'[0-9a-f]{64}', support['sha256'], 'Retest support digest required')
    require(type(support['size']) is int and support['size'] > 0, 'Retest support size required')
    return record, support


def producer(gh, record, *, complete):
    run_id = decimal(record['runId'])
    run = gh.api(f'{gh.base}/actions/runs/{run_id}/attempts/{decimal(record["runAttempt"])}')
    require(str(run.get('id')) == run_id and str(run.get('run_attempt')) == record['runAttempt'] and
            run.get('head_sha') == record['commit'] and run.get('path') == STAGING_WORKFLOW and
            run.get('event') in {'push', 'workflow_dispatch'}, 'Staging workflow provenance differs')
    if complete:
        require(run.get('status') == 'completed' and run.get('conclusion') == 'success',
                'Staging producer is not completed successfully')


def inventory_assets(gh, release):
    assets = {}
    folded = set()
    for asset in gh.assets(release):
        name = asset.get('name')
        manifest.safe_name(name)
        require(name.casefold() not in folded, 'Duplicate release asset')
        require(asset.get('state') == 'uploaded', 'Incomplete release asset')
        folded.add(name.casefold())
        assets[name] = asset
    return assets


def retain_release(gh, directory, record, tag, *, production=False, receipt=None):
    expected = {path.name: path for path in directory.iterdir()}
    if receipt is not None:
        expected[RECEIPT] = receipt
    release = gh.release(tag)
    if release is not None:
        check_release(release, tag, record['commit'], not production)
        assets = inventory_assets(gh, release)
        require(set(assets) <= set(expected), 'Existing release contains unexpected assets')
        # Validate every existing byte before adding anything on an interrupted retry.
        with tempfile.TemporaryDirectory() as tmp:
            for name, asset in assets.items():
                require(asset.get('size') == expected[name].stat().st_size, 'Existing release asset size differs')
                path = Path(tmp) / name
                gh.download(asset, path)
                require(file_hash(path) == file_hash(expected[name]), 'Existing release asset hash differs')
    else:
        assets = {}
        channel = 'PRODUCTION' if production else 'STAGING'
        release = gh.create(tag, record['commit'], f"SKAGER {record['version']} {channel} {record['candidateId']}",
                            not production)
        check_release(release, tag, record['commit'], not production)
    for name in sorted(set(expected) - set(assets)):
        gh.upload(release, expected[name])
    return release


def publish_staging(gh, directory):
    record, _ = local_record(directory)
    require(os.environ.get('GITHUB_SHA') == record['commit'] and
            os.environ.get('GITHUB_RUN_ID') == record['runId'] and
            os.environ.get('GITHUB_RUN_ATTEMPT') == record['runAttempt'], 'Current producer identity differs')
    producer(gh, record, complete=False)
    retain_release(gh, directory, record, staging_tag(record))


def fetch_staging(gh, tag, directory):
    check_tag(tag)
    require(not directory.is_symlink() and (not directory.exists() or
            (directory.is_dir() and not any(directory.iterdir()))), 'Empty regular destination required')
    release = gh.release(tag)
    require(release is not None, 'Staging release not found')
    commit = sha(release.get('target_commitish'))
    check_release(release, tag, commit, True)
    assets = inventory_assets(gh, release)
    require('RELEASE.json' in assets, 'Release manifest missing')
    with tempfile.TemporaryDirectory() as tmp:
        stage = Path(tmp)
        gh.download(assets['RELEASE.json'], stage / 'RELEASE.json')
        raw = read_json(stage / 'RELEASE.json')
        records = raw.get('files')
        require(isinstance(records, list), 'Release manifest inventory required')
        names = [item.get('name') for item in records if isinstance(item, dict)]
        require(len(names) == len(records), 'Invalid release inventory')
        for name in names:
            manifest.safe_name(name)
        require(set(names) <= manifest.RELEASE_FILES | manifest.OPTIONAL_FILES and
                len(set(names)) == len(names) and set(assets) == set(names) | {'RELEASE.json'},
                'Release asset inventory differs')
        for name in names:
            gh.download(assets[name], stage / name)
        record, support = local_record(stage)
        require(staging_tag(record) == tag and record['commit'] == commit, 'Staging tag/commit differs')
        producer(gh, record, complete=True)
        directory.mkdir(parents=True, exist_ok=True)
        for path in stage.iterdir():
            with path.open('rb') as src, (directory / path.name).open('xb') as dst:
                shutil.copyfileobj(src, dst)
    output = os.environ.get('GITHUB_OUTPUT')
    if output:
        values = {'commit': sha(record['commit']), 'run_id': decimal(record['runId']),
                  'support_artifact': support['artifactName'], 'candidate_id': record['candidateId']}
        with open(output, 'a', encoding='utf-8') as stream:
            for key, value in values.items():
                require('\n' not in value and '\r' not in value, 'Unsafe workflow output')
                stream.write(f'{key}={value}\n')
    return record


def publish_production(gh, directory, tag, confirmation, instruction, report_path, qualify_run_id):
    check_tag(tag)
    require(confirmation == 'PROMOTE ' + tag, 'Exact promotion confirmation required')
    require(isinstance(instruction, str) and 1 <= len(instruction.strip()) <= 2000 and
            all(ord(c) >= 32 for c in instruction), 'User instruction reference required')
    require(os.environ.get('GITHUB_EVENT_NAME') == 'workflow_dispatch', 'Manual production dispatch required')
    record, _ = local_record(directory)
    require(staging_tag(record) == tag, 'Selected Staging candidate differs')
    report = read_json(report_path)
    require(set(report) == {'schemaVersion', 'status', 'commit', 'manifestSha256', 'setupSha256',
            'qualificationRunId', 'harnessCommit', 'runAttempt', 'gates'}, 'Unexpected qualification fields')
    run_id = decimal(qualify_run_id)
    harness = sha(os.environ.get('GITHUB_SHA'))
    attempt = decimal(os.environ.get('GITHUB_RUN_ATTEMPT'))
    require(type(report['schemaVersion']) is int and report['schemaVersion'] == 1 and
            report['status'] == 'passed' and report['commit'] == record['commit'] and
            report['manifestSha256'] == file_hash(directory / 'RELEASE.json') and
            report['setupSha256'] == file_hash(directory / 'SKAGER-Beta2-Setup.exe') and
            report['qualificationRunId'] == run_id == os.environ.get('GITHUB_RUN_ID') and
            report['harnessCommit'] == harness and report['runAttempt'] == attempt and
            report['gates'] == dict(installer='passed', functional='passed', package='passed', linux='passed'),
            'Exact-package qualification did not pass or identity differs')
    # All approval and local integrity gates precede any Production API request.
    producer(gh, record, complete=True)
    source = gh.release(tag)
    check_release(source, tag, record['commit'], True)
    with tempfile.TemporaryDirectory() as tmp:
        original = Path(tmp) / 'original'
        fetch_staging(gh, tag, original)
        require(all(file_hash(path) == file_hash(original / path.name) for path in directory.iterdir()),
                'Local package differs from selected Staging release')
        run = gh.api(f'{gh.base}/actions/runs/{run_id}/attempts/{attempt}')
        require(str(run.get('id')) == run_id and str(run.get('run_attempt')) == attempt and
                run.get('event') == 'workflow_dispatch' and run.get('path') == PRODUCTION_WORKFLOW and
                run.get('head_sha') == harness, 'Production workflow provenance differs')
        jobs = gh.pages(f'{gh.base}/actions/runs/{run_id}/attempts/{attempt}/jobs')
        matches = [job for job in jobs if job.get('name') == 'Qualify retained Windows package']
        require(len(matches) == 1 and matches[0].get('status') == 'completed' and
                matches[0].get('conclusion') == 'success', 'Native retained-package job did not pass')
        receipt = Path(tmp) / RECEIPT
        receipt.write_text(json.dumps(dict(schemaVersion=1, channel='production',
            candidateId=record['candidateId'], stagingTag=tag, instructionReference=instruction,
            qualificationRunId=run_id, reportSha256=file_hash(report_path), qualification=report),
            indent=2, sort_keys=True) + '\n', encoding='utf-8')
        retain_release(gh, directory, record, 'skager-production-' + record['candidateId'],
                       production=True, receipt=receipt)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    for command in ('publish-staging', 'fetch-staging', 'publish-production'):
        item = sub.add_parser(command)
        item.add_argument('--directory', type=Path, required=True)
        item.add_argument('--repository', default=os.environ.get('GITHUB_REPOSITORY'))
        if command == 'fetch-staging':
            item.add_argument('--tag', required=True)
        if command == 'publish-production':
            for name in ('staging-tag', 'confirmation', 'instruction', 'report', 'qualify-run-id'):
                item.add_argument('--' + name, required=True)
    args = parser.parse_args()
    try:
        gh = GitHub(args.repository)
        if args.command == 'publish-staging':
            publish_staging(gh, args.directory)
        elif args.command == 'fetch-staging':
            fetch_staging(gh, args.tag, args.directory)
        else:
            publish_production(gh, args.directory, args.staging_tag, args.confirmation,
                               args.instruction, Path(args.report), args.qualify_run_id)
    except (ValueError, OSError, KeyError, TypeError, zipfile.BadZipFile) as error:
        # Do not include untrusted API strings or subprocess output in public logs.
        parser.exit(1, f'Release delivery refused ({type(error).__name__}); no public publication attempted.\n')


if __name__ == '__main__':
    main()

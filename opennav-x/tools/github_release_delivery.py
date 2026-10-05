#!/usr/bin/env python3
"""Immutable, draft-only GitHub release transport. Credentials stay in GH_TOKEN."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import tempfile
import zipfile

import release_manifest as manifest

STAGING_WORKFLOW = '.github/workflows/opennav-baseline.yml'
PRODUCTION_WORKFLOW = '.github/workflows/skager-production.yml'
STAGING_QUALIFICATION_JOB = 'Qualify retained native Staging inputs'
RECEIPT = 'PRODUCTION.json'
PUBLICATION_RECEIPT = 'STAGING_PUBLICATION.json'
require = manifest.require


class GitHubTransportError(ValueError):
    """Only locally constructed diagnostic fields; never raw GitHub output."""


class PublicationReceiptConflict(ValueError):
    """Fixed local diagnostic for a retained transport receipt needing review."""


def transport_error(result, phase):
    stderr = (result.stderr or b'').decode('utf-8', errors='replace').lower()
    match = re.search(r'\bhttp ([1-5][0-9]{2})\b', stderr)
    status = match.group(1) if match else 'unavailable'
    category = ('timeout' if 'timeout' in stderr or 'timed out' in stderr else
                'connection' if any(word in stderr for word in ('connection reset', 'unexpected eof', 'broken pipe')) else
                'tls' if 'tls handshake' in stderr or 'certificate' in stderr else
                'http' if match else 'process-terminated' if result.returncode < 0 else 'unclassified')
    return GitHubTransportError(f'GitHub transport failed: {phase}; exit={result.returncode}; http={status}; category={category}. '
                                'Inspect this exact phase; no automatic overwrite or public publication occurred.')


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

    def _run(self, arguments, *, data=None, output=None, phase='API request'):
        # No shell, no token arguments, and never echo GH output/errors on failure.
        result = subprocess.run(['gh', *arguments], input=data, stdout=output or subprocess.PIPE,
                                stderr=subprocess.PIPE, check=False)
        if result.returncode != 0:
            raise transport_error(result, phase)
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
                   '--method', 'POST', '-H', 'Content-Type: application/octet-stream', '--input', str(path)],
                  phase=f'upload asset={path.name}; bytes={path.stat().st_size}; release={ident}')


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
    if qualification.get('schemaVersion') == 2:
        from staging_composition import validate_qualification
        support = read_json(directory / 'RETEST_SUPPORT.json')
        validate_qualification(qualification, record, support)
        return record, support
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
    support_attempt = decimal(support['runAttempt'])
    release_attempt = decimal(record['runAttempt'])
    require(support['commit'] == record['commit'] and support['runId'] == record['runId'] and
            int(support_attempt) <= int(release_attempt) and
            support['artifactName'] == 'staging-retest-' + record['commit'] + '-attempt' + support_attempt and
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
    return run


def support_producer(gh, record, support):
    # A publish-only retry keeps the successful qualification attempt's support
    # identity. The older overall run may have failed at publication; only the
    # exact named qualification job is allowed to supply the retained inputs.
    if support['runAttempt'] == record['runAttempt']:
        return
    earlier = dict(record, runAttempt=support['runAttempt'])
    run = producer(gh, earlier, complete=False)
    require(run.get('status') == 'completed', 'Earlier support attempt is not completed')
    jobs = gh.pages(f'{gh.base}/actions/runs/{decimal(record["runId"])}/'
                    f'attempts/{decimal(support["runAttempt"])}/jobs')
    matches = [job for job in jobs if job.get('name') == STAGING_QUALIFICATION_JOB]
    require(len(matches) == 1 and matches[0].get('status') == 'completed' and
            matches[0].get('conclusion') == 'success',
            'Earlier native Staging qualification job did not pass')


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
    record, support = local_record(directory)
    qualification = read_json(directory / 'QUALIFICATION.json')
    if qualification.get('schemaVersion') == 2:
        from staging_composition import verify_provenance
        execution = qualification['composition']['execution']
        require(execution == dict(commit=os.environ.get('GITHUB_SHA'),
                                  runId=os.environ.get('GITHUB_RUN_ID'),
                                  runAttempt=os.environ.get('GITHUB_RUN_ATTEMPT')),
                'Current composition identity differs')
        verify_provenance(gh, qualification, complete=False)
        retain_release(gh, directory, record, staging_tag(record))
        return
    require(os.environ.get('GITHUB_SHA') == record['commit'] and
            os.environ.get('GITHUB_RUN_ID') == record['runId'] and
            os.environ.get('GITHUB_RUN_ATTEMPT') == record['runAttempt'], 'Current producer identity differs')
    producer(gh, record, complete=False)
    support_producer(gh, record, support)
    retain_release(gh, directory, record, staging_tag(record))


def verify_asset(gh, asset, path):
    require(asset.get('size') == path.stat().st_size, 'Existing release asset size differs')
    with tempfile.TemporaryDirectory(prefix='skager-asset-') as temporary:
        downloaded = Path(temporary) / path.name
        gh.download(asset, downloaded)
        require(file_hash(downloaded) == file_hash(path), 'Existing release asset hash differs')


def upload_or_confirm(gh, release, path):
    """An uncertain response may hide a successful upload; only fresh bytes decide."""
    failure = None
    try:
        gh.upload(release, path)
    except GitHubTransportError as error:
        failure = error
    assets = inventory_assets(gh, release)
    if path.name not in assets:
        if failure is not None:
            raise failure
        raise ValueError('Upload did not publish the expected complete asset')
    verify_asset(gh, assets[path.name], path)
    return assets[path.name]


def resume_retained(gh, directory, publication):
    """Resume only an existing pinned draft; every product byte stays immutable."""
    from staging_composition import verify_publication_provenance
    record, _ = local_record(directory)
    qualification = read_json(directory / 'QUALIFICATION.json')
    release = gh.release(staging_tag(record))
    require(release is not None, 'Pinned draft is missing; resume cannot create a release')
    check_release(release, staging_tag(record), record['commit'], True)
    verify_publication_provenance(gh, qualification, publication,
        manifest_sha256=file_hash(directory/'RELEASE.json'), qualification_sha256=file_hash(directory/'QUALIFICATION.json'),
        release_id=release['id'], complete=False)
    expected = {path.name:path for path in directory.iterdir()}
    with tempfile.TemporaryDirectory(prefix='skager-publication-') as temporary:
        receipt = Path(temporary)/PUBLICATION_RECEIPT
        receipt.write_text(json.dumps(publication, indent=2, sort_keys=True)+'\n', encoding='utf-8')
        assets = inventory_assets(gh, release)
        require(set(assets) <= set(expected) | {PUBLICATION_RECEIPT}, 'Pinned draft contains unexpected assets')
        for name, asset in assets.items():
            if name == PUBLICATION_RECEIPT:
                try:
                    verify_asset(gh, asset, receipt)
                except GitHubTransportError:
                    raise
                except ValueError as error:
                    raise PublicationReceiptConflict(
                        'Existing transport receipt differs from this publication execution; '
                        'a prior failed publication may have retained it. Review is required; no overwrite attempted.') from error
            else:
                verify_asset(gh, asset, expected[name])
        if PUBLICATION_RECEIPT in assets:
            require(set(assets) == set(expected) | {PUBLICATION_RECEIPT}, 'Transport receipt precedes complete payloads')
        for name in sorted(set(expected) - set(assets)):
            assets[name] = upload_or_confirm(gh, release, expected[name])
        # Do not attest a concurrently replaced, added or removed asset.
        final = inventory_assets(gh, release)
        require(set(final) == set(assets) and all(final[n].get('id') == assets[n].get('id') and
                final[n].get('size') == assets[n].get('size') for n in final), 'Draft inventory changed during resume')
        if PUBLICATION_RECEIPT not in assets:
            upload_or_confirm(gh, release, receipt)
    if os.environ.get('GITHUB_OUTPUT'):
        with open(os.environ['GITHUB_OUTPUT'], 'a', encoding='utf-8') as stream:
            stream.write('staging_tag='+staging_tag(record)+'\nrelease_id='+decimal(str(release['id']))+'\n')
    return release


def unpack_candidate(archive, directory):
    """Flat fixed release inventory only; no executable or source processing."""
    require(not directory.exists() and not directory.is_symlink(), 'Fresh retained candidate directory required')
    names = manifest.RELEASE_FILES | manifest.OPTIONAL_FILES | {manifest.MANIFEST}
    with zipfile.ZipFile(archive) as source:
        entries = source.infolist()
        require(len(entries) == len(names) and {e.filename for e in entries} == names,
                'Retained candidate inventory differs')
        require(sum(e.file_size for e in entries) <= 8 * 1024**3, 'Retained candidate exceeds bound')
        for entry in entries:
            require(entry.orig_filename == entry.filename and not entry.is_dir() and
                    stat.S_IFMT(entry.external_attr >> 16) in (0, stat.S_IFREG) and
                    not entry.flag_bits & 1 and 0 < entry.file_size <= 4 * 1024**3,
                    'Retained candidate member unsafe')
        directory.mkdir(parents=True)
        for entry in entries:
            with source.open(entry) as stream, (directory/entry.filename).open('xb') as target:
                shutil.copyfileobj(stream, target, 1024*1024)


def resume_staging(gh, request_path, directory):
    from staging_composition import (PUBLICATION_REQUEST, REPOSITORY, _download,
                                     publication_receipt, validate_publication_request)
    root = Path(__file__).resolve().parents[1]
    require(request_path.resolve() == root/'tools/staging-publication-request.json' and
            not request_path.is_symlink(), 'Only the committed publication request is supported')
    request = read_json(request_path); validate_publication_request(request)
    raw = request_path.read_bytes()
    publisher = dict(commit=sha(os.environ.get('GITHUB_SHA')), runId=decimal(os.environ.get('GITHUB_RUN_ID')),
                     runAttempt=decimal(os.environ.get('GITHUB_RUN_ATTEMPT')))
    require(gh.repo == REPOSITORY and os.environ.get('GITHUB_REPOSITORY') == REPOSITORY and
            os.environ.get('GITHUB_REF') == 'refs/heads/skager-staging-publish', 'Unapproved publication execution')
    repo = subprocess.check_output(['git','-C',str(root),'rev-parse','--show-toplevel'],text=True).strip()
    require(subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip() == publisher['commit'] and
            subprocess.check_output(['git','-C',repo,'show','HEAD:'+PUBLICATION_REQUEST]) == raw,
            'Publication request is not the exact committed selection')
    publication = publication_receipt(request, manifest.digest(raw), publisher)
    with tempfile.TemporaryDirectory(prefix='skager-retained-publication-') as temporary:
        archive = Path(temporary)/'candidate.zip'
        _download(gh, request['candidateArtifact'], request['composition'], archive)
        require(archive.stat().st_size == request['candidateArtifact']['size'], 'Pinned candidate download size differs')
        unpack_candidate(archive, directory)
    return resume_retained(gh, directory, publication)


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
        stage = Path(tmp)/'product'
        stage.mkdir()
        gh.download(assets['RELEASE.json'], stage / 'RELEASE.json')
        raw = read_json(stage / 'RELEASE.json')
        records = raw.get('files')
        require(isinstance(records, list), 'Release manifest inventory required')
        names = [item.get('name') for item in records if isinstance(item, dict)]
        require(len(names) == len(records), 'Invalid release inventory')
        for name in names:
            manifest.safe_name(name)
        transport = PUBLICATION_RECEIPT in assets
        extras = {PUBLICATION_RECEIPT} if transport else set()
        require(set(names) <= manifest.RELEASE_FILES | manifest.OPTIONAL_FILES and
                len(set(names)) == len(names) and set(assets) == set(names) | {'RELEASE.json'} | extras,
                'Release asset inventory differs')
        for name in names:
            gh.download(assets[name], stage / name)
        record, support = local_record(stage)
        require(staging_tag(record) == tag and record['commit'] == commit, 'Staging tag/commit differs')
        qualification = read_json(stage / 'QUALIFICATION.json')
        if transport:
            require(qualification.get('schemaVersion') == 2, 'Transport continuation cannot authorize a v1 product')
            from staging_composition import verify_publication_provenance
            receipt = Path(tmp)/PUBLICATION_RECEIPT
            gh.download(assets[PUBLICATION_RECEIPT], receipt)
            verify_publication_provenance(gh, qualification, read_json(receipt),
                manifest_sha256=file_hash(stage/'RELEASE.json'), qualification_sha256=file_hash(stage/'QUALIFICATION.json'),
                release_id=release['id'], complete=True)
        elif qualification.get('schemaVersion') == 2:
            from staging_composition import verify_provenance
            verify_provenance(gh, qualification, complete=True)
        else:
            producer(gh, record, complete=True)
            support_producer(gh, record, support)
        directory.mkdir(parents=True, exist_ok=True)
        for path in stage.iterdir():
            with path.open('rb') as src, (directory / path.name).open('xb') as dst:
                shutil.copyfileobj(src, dst)
    output = os.environ.get('GITHUB_OUTPUT')
    if output:
        values = {'commit': sha(record['commit']), 'run_id': decimal(record['runId']),
                  'support_run_id': decimal(support['runId']),
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
    qualification = read_json(directory / 'QUALIFICATION.json')
    if qualification.get('schemaVersion') != 2:
        producer(gh, record, complete=True)
    # v2 provenance (including an optional transport-only continuation) is
    # authenticated by the mandatory fetch below before any Production writes.
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
    for command in ('publish-staging', 'fetch-staging', 'publish-production', 'resume-staging'):
        item = sub.add_parser(command)
        item.add_argument('--directory', type=Path, required=True)
        item.add_argument('--repository', default=os.environ.get('GITHUB_REPOSITORY'))
        if command == 'resume-staging':
            item.add_argument('--request', type=Path, required=True)
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
        elif args.command == 'resume-staging':
            resume_staging(gh, args.request, args.directory)
        elif args.command == 'fetch-staging':
            fetch_staging(gh, args.tag, args.directory)
        else:
            publish_production(gh, args.directory, args.staging_tag, args.confirmation,
                               args.instruction, Path(args.report), args.qualify_run_id)
    except (GitHubTransportError, PublicationReceiptConflict) as error:
        parser.exit(1, str(error) + '\n')
    except (ValueError, OSError, KeyError, TypeError, zipfile.BadZipFile, subprocess.CalledProcessError) as error:
        # Do not include untrusted API strings or subprocess output in public logs.
        parser.exit(1, f'Release delivery refused ({type(error).__name__}); no public publication attempted.\n')


if __name__ == '__main__':
    main()

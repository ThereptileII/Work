#!/usr/bin/env python3
"""Retry checks against authenticated compiled bytes; never publish or rebuild."""
import argparse
import json
import os
import re
from pathlib import Path
import subprocess
import sys

from fetch_ci_inputs import fetch
from github_release_delivery import GitHub, decimal, read_json, repository, sha
from release_manifest import require
from staging_build_inputs import restore, producer

ROOT = Path(__file__).resolve().parents[1]


def selection_for_run(gh, run_id, attempt):
    run_id, attempt = decimal(run_id), decimal(attempt)
    run = gh.api(f'{gh.base}/actions/runs/{run_id}/attempts/{attempt}')
    commit = sha(run.get('head_sha', ''))
    name = f'staging-build-{commit}-run{run_id}-attempt{attempt}'
    matches = []
    for page in range(1, 101):
        response = gh.api(f'{gh.base}/actions/runs/{run_id}/artifacts?per_page=100&page={page}')
        items = response.get('artifacts')
        require(isinstance(items, list), 'Invalid artifact inventory')
        matches.extend(item for item in items if item.get('name') == name)
        if len(items) < 100:
            break
    else:
        raise ValueError('Artifact inventory exceeds bounds')
    require(len(matches) == 1, 'Exact retained build artifact missing or ambiguous')
    artifact = matches[0]
    return dict(repository=gh.repo, runId=run_id, runAttempt=attempt, headSha=commit,
                artifactId=decimal(str(artifact['id'])), artifactName=name,
                artifactDigest=artifact.get('digest'))


def pinned_selection(path):
    require(os.environ.get('GITHUB_EVENT_NAME') == 'push' and
            os.environ.get('GITHUB_REF') == 'refs/heads/skager-staging-retest',
            'Pinned retest requires the exact trusted branch push')
    require(path.resolve() == (ROOT / 'tools/staging-installer-retest.json').resolve(),
            'Only the committed installer retest selection is supported')
    request = read_json(path)
    require(set(request) == {'schema', 'scope', 'artifact', 'archiveSha256'} and
            type(request['schema']) is int and request['schema'] == 1 and
            request['scope'] == 'installer-charts' and
            isinstance(request['archiveSha256'], str) and
            re.fullmatch('[a-f0-9]{64}', request['archiveSha256']), 'Invalid pinned installer retest request')
    return request


def prepare(run_id, attempt, selection_path=None):
    gh = GitHub(repository(os.environ['GITHUB_REPOSITORY']))
    request = pinned_selection(selection_path) if selection_path else None
    selection = request['artifact'] if request else selection_for_run(gh, run_id, attempt)
    run_id, attempt = selection['runId'], selection['runAttempt']
    out = ROOT / 'build/retained-staging-download'
    authority_path = ROOT / 'evidence/local/staging-producer.json'
    authority = fetch(gh, selection, out, authority_path, kind='staging')
    if request:
        require(authority['archiveSha256'] == request['archiveSha256'], 'Pinned inner retained archive differs')
    harness = subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip()
    require(harness == os.environ['GITHUB_SHA'], 'Harness checkout differs')
    identity = producer(selection['headSha'], run_id, attempt)
    restore(ROOT, out / 'STAGING_BUILD_INPUTS.zip', authority['archiveSha256'], identity,
            harness, ROOT / 'evidence/local/staging-inputs.json')
    if os.environ.get('GITHUB_OUTPUT'):
        with open(os.environ['GITHUB_OUTPUT'], 'a', encoding='utf-8') as stream:
            stream.write(f'producer_run={decimal(run_id)}\nproducer_attempt={decimal(attempt)}\n')
    return selection['headSha']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=('prepare', 'test'))
    parser.add_argument('--producer-run-id')
    parser.add_argument('--producer-run-attempt')
    parser.add_argument('--selection', type=Path)
    parser.add_argument('--scope', choices=('installer', 'installer-charts', 'all'), default='all')
    args = parser.parse_args()
    require(sys.platform == 'win32' and os.environ.get('GITHUB_ACTIONS') == 'true' and
            os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted', 'Disposable native CI required')
    if os.environ.get('GITHUB_EVENT_NAME') == 'push':
        require(os.environ.get('GITHUB_REF') == 'refs/heads/skager-staging-retest' and
                ((args.phase == 'prepare' and args.selection is not None and
                  not args.producer_run_id and not args.producer_run_attempt) or
                 (args.phase == 'test' and args.scope == 'installer-charts')),
                'Trusted push permits only the pinned installer/chart retest')
    if args.phase == 'prepare':
        print('Restored original product commit: ' + prepare(args.producer_run_id, args.producer_run_attempt, args.selection))
        return
    require(not os.environ.get('GH_TOKEN') and not os.environ.get('GITHUB_TOKEN'),
            'Remove Actions credentials before launching retained applications')
    record = json.loads((ROOT / 'evidence/local/staging-inputs.json').read_text())
    report = dict(status='running', scope=args.scope, productCommit=record['producer']['commit'],
                  producer=record['producer'], harnessCommit=record['harnessCommit'],
                  archiveSha256=record['archiveSha256'], publicAccess=False, releaseQualification=False)
    target = ROOT / 'evidence/local/staging-retest.json'
    def save(): target.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    save()
    run_checks(args.scope, report, save)


def run_checks(scope, report, save):
    installer = [sys.executable, str(ROOT / 'tools/smoke-installer-windows.py'), '--mode', 'staging',
                 '--compiled-input-receipt', str(ROOT / 'evidence/local/staging-inputs.json')]
    if scope in ('installer', 'installer-charts'):
        commands = [('installer', installer)]
        if scope == 'installer-charts':
            # Exactly the existing next qualifier command, using restored
            # build/xnav-install bytes; smoke-charts has no product CLI flags.
            commands.append(('charts', [sys.executable, str(ROOT / 'tools/smoke-charts.py')]))
    else:
        commands = [('all', ['pwsh', '-NoProfile', '-File', str(ROOT / 'tools/qualify-staging-windows.ps1'),
                            '-ProductCommit', report['productCommit'], '-CompiledRetest'])]
    report['checks'] = [dict(name=name, status='not-run') for name, _ in commands]
    save()
    try:
        for check, (_, command) in zip(report['checks'], commands):
            check['status'] = 'running'; save()
            try:
                subprocess.run(command, check=True)
            except BaseException:
                check['status'] = 'failed'
                raise
            check['status'] = 'passed'; save()
        report['status'] = 'passed'
    except BaseException:
        report['status'] = 'failed'
        raise
    finally:
        save()


if __name__ == '__main__':
    main()

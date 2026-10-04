#!/usr/bin/env python3
"""Retry checks against authenticated compiled bytes; never publish or rebuild."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

from fetch_ci_inputs import fetch
from github_release_delivery import GitHub, decimal, repository, sha
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


def prepare(run_id, attempt):
    gh = GitHub(repository(os.environ['GITHUB_REPOSITORY']))
    selection = selection_for_run(gh, run_id, attempt)
    out = ROOT / 'build/retained-staging-download'
    authority_path = ROOT / 'evidence/local/staging-producer.json'
    authority = fetch(gh, selection, out, authority_path, kind='staging')
    harness = subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip()
    require(harness == os.environ['GITHUB_SHA'], 'Harness checkout differs')
    identity = producer(selection['headSha'], run_id, attempt)
    restore(ROOT, out / 'STAGING_BUILD_INPUTS.zip', authority['archiveSha256'], identity,
            harness, ROOT / 'evidence/local/staging-inputs.json')
    return selection['headSha']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=('prepare', 'test'))
    parser.add_argument('--producer-run-id')
    parser.add_argument('--producer-run-attempt')
    parser.add_argument('--scope', choices=('installer', 'all'), default='all')
    args = parser.parse_args()
    require(sys.platform == 'win32' and os.environ.get('GITHUB_ACTIONS') == 'true' and
            os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted', 'Disposable native CI required')
    if args.phase == 'prepare':
        print('Restored original product commit: ' + prepare(args.producer_run_id, args.producer_run_attempt))
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
    try:
        if args.scope == 'installer':
            command = [sys.executable, str(ROOT / 'tools/smoke-installer-windows.py'), '--mode', 'staging',
                       '--compiled-input-receipt', str(ROOT / 'evidence/local/staging-inputs.json')]
        else:
            command = ['pwsh', '-NoProfile', '-File', str(ROOT / 'tools/qualify-staging-windows.ps1'),
                       '-ProductCommit', record['producer']['commit'], '-CompiledRetest']
        subprocess.run(command, check=True)
        report['status'] = 'passed'
    except BaseException:
        report['status'] = 'failed'
        raise
    finally:
        save()


if __name__ == '__main__':
    main()

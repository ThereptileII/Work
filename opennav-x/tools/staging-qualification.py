#!/usr/bin/env python3
"""Bind required staging jobs to a versioned immutable Release set."""
import argparse
import json
import os
from pathlib import Path
import zipfile
from release_manifest import create

REQUIRED_JOBS = ('contracts', 'boat-maintenance', 'native-restart-transport',
                 'native-restart-broker', 'native-restart-window', 'native-stock-welcome',
                 'linux-integration', 'windows-integration', 'windows-qualification', 'application-abi', 'stock-prerequisite')


def qualify(directory, results, env):
    for name in REQUIRED_JOBS:
        if results.get(name, {}).get('result') != 'success':
            raise ValueError('Required staging job did not pass: ' + name)
    receipt = {'schemaVersion': 1, 'channel': 'staging', 'commit': env['GITHUB_SHA'],
               'runId': env['GITHUB_RUN_ID'], 'runAttempt': env['GITHUB_RUN_ATTEMPT'],
               'gates': {k: 'passed' for k in ('linux','windows','installer','package','restart')},
               'designReview': 'requested' if env.get('SKAGER_DESIGN_VALIDATION') == 'true' else 'not-requested',
               'endurance': 'skipped', 'publicAccess': False}
    with zipfile.ZipFile(directory/'SKAGER-Beta2-Portable-Recovery.zip') as archive:
        product = json.loads(archive.read('SKAGER-Beta2-Portable-Recovery/docs/PRODUCT_BUILD.json'))
    (directory/'QUALIFICATION.json').write_text(json.dumps(receipt, indent=2)+'\n')
    return create(directory, env['GITHUB_SHA'], env['GITHUB_RUN_ID'],
                  env['GITHUB_RUN_ATTEMPT'], product['version'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(qualify(args.directory, json.loads(os.environ['STAGING_RESULTS']), os.environ)))

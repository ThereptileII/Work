#!/usr/bin/env python3
"""Record completed exact-package readiness; never rebuild or publish anything."""
import argparse
import hashlib
import json
import os
from pathlib import Path
from release_manifest import verify


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def qualify(directory, evidence, env):
    record = verify(directory)
    installer = json.loads((evidence/'installer-lifecycle.json').read_text())
    portable = json.loads((evidence/'production-recovery-results.json').read_text())
    functional = json.loads((evidence/'production-functional.json').read_text())
    inherited = json.loads((directory/'QUALIFICATION.json').read_text())
    for report in (installer, portable, functional):
        if report.get('status') != 'passed':
            raise ValueError('Production readiness check did not pass')
    if (installer.get('mode') != 'production' or
            installer.get('product_commit') != record['commit'] or
            installer.get('harness_commit') != env['GITHUB_SHA']):
        raise ValueError('Installer readiness scope or source identity differs')
    if (portable.get('build', {}).get('commit') != record['commit'] or
            portable.get('harness_commit') != env['GITHUB_SHA']):
        raise ValueError('Recovery source identity differs')
    if installer.get('setup_sha256') != digest(directory/'SKAGER-Beta2-Setup.exe'):
        raise ValueError('Installer evidence belongs to different bytes')
    if portable.get('package_sha256') != digest(directory/'SKAGER-Beta2-Portable-Recovery.zip'):
        raise ValueError('Recovery evidence belongs to different bytes')
    if functional.get('commit') != record['commit'] or functional.get('harnessCommit') != env['GITHUB_SHA']:
        raise ValueError('Functional source identity differs')
    if inherited.get('commit') != record['commit'] or inherited.get('gates', {}).get('linux') != 'passed':
        raise ValueError('Exact-source Linux qualification missing')
    return {'schemaVersion': 1, 'status': 'passed', 'commit': record['commit'],
            'manifestSha256': digest(directory/'RELEASE.json'),
            'setupSha256': digest(directory/'SKAGER-Beta2-Setup.exe'),
            'qualificationRunId': env['GITHUB_RUN_ID'], 'harnessCommit': env['GITHUB_SHA'],
            'runAttempt': env['GITHUB_RUN_ATTEMPT'],
            'gates': dict(installer='passed',functional='passed',package='passed',linux='passed')}


if __name__ == '__main__':
    p=argparse.ArgumentParser()
    for name in ('directory','evidence','output'):
        p.add_argument('--'+name, type=Path, required=True)
    a=p.parse_args()
    a.output.write_text(json.dumps(qualify(a.directory,a.evidence,os.environ),indent=2)+'\n')

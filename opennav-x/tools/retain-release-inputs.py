#!/usr/bin/env python3
"""Retain CI-only installer/fixture inputs without adding them to the product."""
import hashlib
import json
import os
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def retain(root, commit, run_id, run_attempt):
    import re
    if (not re.fullmatch(r'[0-9a-f]{40}', commit) or not re.fullmatch(r'[1-9][0-9]{0,19}', run_id)
            or not re.fullmatch(r'[1-9][0-9]{0,9}', run_attempt)):
        raise ValueError('Exact source/run identity required')
    inputs = [root/'build/beta-installer/package.json', root/'build/beta-installer/payload.zip',
              root/'build/production-windows/include/config.h', root/'build/xnav-windows/include/config.h']
    fixture = root/'build/xnav-install'
    if not (fixture/'opencpn.exe').is_file():
        raise ValueError('Missing actual fixture application for later regressions')
    inputs.extend(sorted(fixture.rglob('*')))
    files = []
    for path in inputs:
        if path.is_symlink():
            raise ValueError('Linked qualification input refused')
        if path.is_dir():
            continue
        if not path.is_file():
            raise ValueError('Missing qualification input: ' + str(path))
        files.append(path)
    output = root/'build/release-retest'
    output.mkdir(parents=True, exist_ok=False)
    archive = output/'SKAGER-Beta2-Retest-Support.zip'
    with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED, compresslevel=1) as z:
        for path in files:
            z.write(path, path.relative_to(root).as_posix())
    with archive.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    receipt = {'artifactName': 'staging-retest-' + commit + '-attempt' + run_attempt,
               'runId': run_id, 'runAttempt': run_attempt,
               'commit': commit, 'archiveName': archive.name, 'sha256': digest,
               'size': archive.stat().st_size}
    (root/'build/beta-artifacts/RETEST_SUPPORT.json').write_text(json.dumps(receipt, indent=2)+'\n')
    return receipt


if __name__ == '__main__':
    print(json.dumps(retain(ROOT, os.environ['GITHUB_SHA'], os.environ['GITHUB_RUN_ID'],
                            os.environ['GITHUB_RUN_ATTEMPT'])))

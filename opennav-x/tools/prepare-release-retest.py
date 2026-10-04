#!/usr/bin/env python3
"""Combine verified Release bytes with its separately retained CI-only inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
from release_manifest import verify


def prepare(release, support, output):
    manifest = verify(release)
    receipt = json.loads((release/'RETEST_SUPPORT.json').read_text())
    if (receipt.get('archiveName') != 'SKAGER-Beta2-Retest-Support.zip' or
            receipt.get('artifactName') != 'staging-retest-' + manifest['commit'] + '-attempt' + manifest['runAttempt'] or
            receipt.get('commit') != manifest['commit'] or receipt.get('runId') != manifest['runId'] or
            receipt.get('runAttempt') != manifest['runAttempt']):
        raise ValueError('Retest inputs are not bound to this release')
    archive = support/receipt['archiveName']
    if archive.is_symlink() or not archive.is_file() or archive.stat().st_size != receipt.get('size'):
        raise ValueError('Retest archive identity differs')
    with archive.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    if digest != receipt.get('sha256'):
        raise ValueError('Retest archive checksum differs')
    # Work on a disposable copy. Published manifest/checksums remain untouched.
    shutil.copytree(release, output)
    shutil.copy2(archive, output/archive.name)
    with (output/'SHA256SUMS.txt').open('a', encoding='utf-8') as stream:
        stream.write(digest+'  '+archive.name+'\n')
    return manifest


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    for key in ('release','support','output'):
        p.add_argument('--'+key, type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(prepare(a.release, a.support, a.output)))

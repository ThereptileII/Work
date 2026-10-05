#!/usr/bin/env python3
"""Combine verified Release bytes with its separately retained CI-only inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
from github_release_delivery import local_record


def prepare(release, support, output):
    # fetch-staging authenticates any earlier support-producing qualification
    # job. Reuse its central local binding rules here: same run/commit, canonical
    # support attempt no later than release attempt, original artifact identity.
    manifest, receipt = local_record(release)
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

#!/usr/bin/env python3
"""Retain unqualified native compile outputs before any updater/package operation.

This diagnostic archive is deliberately NOT a STAGING_BUILD_INPUTS archive and
has no restore/qualification command. Dirty tracked source is retained honestly;
untracked checkout content is never swept into the source snapshot. All existing
packaging source, SDK, dependency and qualification gates still apply on retry.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import zipfile

import staging_build_inputs as sealed
from source_package import PINNED_UPSTREAM

ARCHIVE = 'COMPILED_RECOVERY.zip'
MANIFEST = 'COMPILED_RECOVERY.json'
KIND = 'skager-unqualified-prepackage-recovery'
MAX_SOURCE = 2 * 1024**3


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args])


def ordinary(path):
    """Reject junctions as well as symlinks, including existing ancestors."""
    for part in (path, *path.parents):
        info = part.lstat()
        sealed.require(not stat.S_ISLNK(info.st_mode) and
                       not getattr(info, 'st_file_attributes', 0) & 0x400,
                       'Redirected recovery input refused')
    sealed.require(path.is_file(), 'Nonregular recovery input refused')
    return path


def source_inputs(root, commit):
    repository = Path(git(root, 'rev-parse', '--show-toplevel').decode().strip())
    upstream = root / 'build/integration-source'
    sealed.require(git(root, 'rev-parse', 'HEAD').decode().strip() == commit,
                   'Recovery checkout differs from producer')
    sealed.require(git(upstream, 'rev-parse', 'HEAD').decode().strip() == PINNED_UPSTREAM,
                   'Recovery integration baseline differs')
    sources, references = {}, []
    for directory, prefix, selected in (
            (root, 'source/product', '.'),
            (upstream, 'source/integrated', '.'),
            (repository, 'source/recipe', '.github/workflows')):
        entries = git(directory, 'ls-tree', '-r', '-z', 'HEAD', '--', selected)
        tracked_dirty = git(directory, 'diff', '--name-status', 'HEAD', '--', selected).decode('utf-8')
        # Ignore untracked files entirely: they are neither source authority nor
        # safe to include automatically (tool caches may contain private data).
        reference = dict(prefix=prefix, commit=git(directory, 'rev-parse', 'HEAD').decode().strip(),
                         trackedDirty=bool(tracked_dirty), trackedChanges=tracked_dirty,
                         entries=[])
        for entry in entries.split(b'\0'):
            if not entry:
                continue
            metadata, raw = entry.split(b'\t', 1)
            mode, kind, blob = metadata.decode().split()
            name = raw.decode('utf-8'); sealed.safe_name(name)
            record = dict(path=name, gitMode=mode, headObject=blob)
            if kind == 'commit':
                record['retained'] = 'gitlink-reference-only'
            else:
                path = directory / name
                if path.is_symlink():
                    # Preserve the link text as provenance, never its target.
                    record.update(retained='link-text-only', linkText=os.readlink(path))
                elif not path.exists():
                    record['retained'] = 'missing-tracked-file'
                else:
                    ordinary(path)
                    archive_name = prefix + '/' + name; sealed.safe_name(archive_name)
                    sources[archive_name] = path
                    record.update(retained='actual-checkout-bytes', archivePath=archive_name)
            reference['entries'].append(record)
        sealed.require(reference['entries'], 'Empty tracked source provenance')
        references.append(reference)
    sealed.require('.github/workflows/opennav-baseline.yml' in
                   {entry['path'] for entry in references[-1]['entries']},
                   'Producer workflow missing from tracked provenance')
    return sources, references


def retain(root, output, expected):
    root, output = Path(root).absolute(), Path(output).absolute()
    sealed.require(not output.exists(), 'Recovery output must be fresh')
    # Reuse the established compiled-input allowlist, excluding package trees.
    names = [name for name in sealed.inventory(root) if
             not name.startswith(('build/developer-preview/', 'build/beta-installer/'))]
    required = {f'build/{v}-install/{binary}' for v in sealed.VARIANTS
                for binary in ('opencpn.exe', 'opennav-restart.exe')}
    required |= {f'build/{v}-windows/include/{header}' for v in sealed.VARIANTS
                 for header in ('config.h', 'OpenNavBuild.h')}
    sealed.require(required <= set(names), 'Both completed native install/header trees required')
    for variant in sealed.VARIANTS:
        header = ordinary(root / f'build/{variant}-windows/include/OpenNavBuild.h').read_text(encoding='utf-8-sig')
        sealed.require('#define OPENNAV_BUILD_COMMIT "' + expected['commit'] + '"' in header,
                       'Compiled recovery header differs from producer')
    sources, references = source_inputs(root, expected['commit'])
    files = {name: ordinary(root / name) for name in names}
    files.update(sources)
    sealed.require(0 < len(files) <= sealed.MAX_FILES and
                   len({name.casefold() for name in files}) == len(files), 'Recovery path count/collision')
    output.mkdir(parents=True)
    records, total, source_total = [], 0, 0
    try:
        archive = output / ARCHIVE
        with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED, compresslevel=1) as target:
            for name, path in sorted(files.items()):
                sealed.safe_name(name); ordinary(path)
                before = path.stat(); total += before.st_size
                if name.startswith('source/'):
                    source_total += before.st_size
                sealed.require(before.st_size <= sealed.MAX_FILE and total <= sealed.MAX_TOTAL and
                               source_total <= MAX_SOURCE, 'Recovery size budget exceeded')
                digest, copied = hashlib.sha256(), 0
                with path.open('rb') as content, target.open(sealed.archive_entry(name), 'w', force_zip64=True) as destination:
                    for block in iter(lambda: content.read(1024 * 1024), b''):
                        copied += len(block)
                        sealed.require(copied <= before.st_size, 'Recovery input grew during retention')
                        digest.update(block); destination.write(block)
                after = ordinary(path).stat()
                sealed.require((before.st_size, before.st_mtime_ns) == (after.st_size, after.st_mtime_ns) and
                               copied == before.st_size, 'Recovery input changed during retention')
                records.append(dict(path=name, size=copied, sha256=digest.hexdigest()))
            _, final_references = source_inputs(root, expected['commit'])
            sealed.require(final_references == references, 'Source identity changed during retention')
            for record in records:
                if record['path'].startswith('source/'):
                    sealed.require(sealed.sha(ordinary(files[record['path']])) == record['sha256'],
                                   'Tracked source bytes changed during retention')
            manifest = dict(schema=1, kind=KIND, producer=expected, qualification='not-run',
                            packaging='not-run', scope='diagnostic recovery only; all original gates required',
                            sources=references, files=records)
            encoded = sealed.canonical(manifest)
            sealed.require(len(encoded) <= sealed.MAX_MANIFEST, 'Recovery manifest exceeds bound')
            target.writestr(sealed.archive_entry(MANIFEST), encoded)
        receipt = dict(schema=1, kind=KIND, status='retained', qualification='not-run', packaging='not-run',
                       producer=expected, archive=ARCHIVE, archiveSha256=sealed.sha(archive),
                       manifestSha256=hashlib.sha256(encoded).hexdigest(), files=len(records), bytes=total,
                       trackedSourceDirty=any(item['trackedDirty'] for item in references))
        (output / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n', encoding='utf-8')
        return receipt
    except BaseException:
        shutil.rmtree(output)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    for name in ('commit', 'run-id', 'run-attempt'):
        parser.add_argument('--producer-' + name, required=True)
    args = parser.parse_args()
    expected = sealed.producer(args.producer_commit, args.producer_run_id, args.producer_run_attempt)
    sealed.require(os.name == 'nt' and os.environ.get('GITHUB_ACTIONS') == 'true' and
                   os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted' and
                   os.environ.get('GITHUB_REPOSITORY') == expected['repository'] and
                   all(os.environ.get(env) == expected[key] for env, key in (
                       ('GITHUB_SHA', 'commit'), ('GITHUB_RUN_ID', 'runId'),
                       ('GITHUB_RUN_ATTEMPT', 'runAttempt'), ('GITHUB_JOB', 'job'))),
                   'Recovery retention requires the exact disposable native producer')
    print(json.dumps(retain(args.root, args.output, expected), sort_keys=True))


if __name__ == '__main__':
    main()

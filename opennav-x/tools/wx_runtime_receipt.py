#!/usr/bin/env python3
"""Match installed wx DLL bytes to the locked, prebuilt ReleaseDLL archive.

This is binary-archive provenance, not source rebuild proof or legal clearance.
The committed lock must separately be bound to the release's corresponding source.
No download, DLL execution or filesystem archive extraction is performed here.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import tempfile

from windows_dependency_receipt import _plain_path, _read_json, _relative, _workspace_root

RECEIPT_NAME = 'wx-runtime-receipt.json'
ARCHIVE_NAME = 'wxMSW-3.2.8_vc14x_ReleaseDLL.7z'
ARCHIVE_URL = 'https://github.com/wxWidgets/wxWidgets/releases/download/v3.2.8/' + ARCHIVE_NAME
VERSION = '3.2.8'
ABI = 'vc14x x86 DLL'
SCOPE = 'prebuilt-runtime-archive-members-only'
DLL_NAME = re.compile(r'wx(?:base|msw)32u(?:_[a-z]+)?_vc14x\.dll\Z')
REQUIRED = {'wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll'}


def _regular(path):
    path = Path(path).absolute()
    _workspace_root(path.parent)
    _plain_path(path.parent, path.name)
    if not path.is_file():
        raise ValueError('Required wx evidence is not a regular file: ' + str(path))
    return path


def _record(path):
    digest = hashlib.sha256()
    size = 0
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
            size += len(block)
    return {'sha256': digest.hexdigest(), 'bytes': size}


def _locked_archive(lock_path, archive_path):
    lock = _read_json(_regular(lock_path))
    if (not isinstance(lock, dict) or lock.get('version') != VERSION or
            lock.get('abi') != ABI or not isinstance(lock.get('archives'), list)):
        raise ValueError('Unsupported wx lock version/ABI or missing archives')
    names = set()
    selected = None
    for item in lock['archives']:
        if not isinstance(item, dict) or set(item) != {'file', 'url', 'sha256', 'bytes'}:
            raise ValueError('Malformed wx archive lock record')
        name = item['file']
        _relative(name)
        if '/' in name or name.casefold() in names:
            raise ValueError('Duplicate or unsafe wx archive lock name')
        names.add(name.casefold())
        if name == ARCHIVE_NAME:
            selected = item
    if (selected is None or selected['url'] != ARCHIVE_URL or
            not isinstance(selected['sha256'], str) or
            re.fullmatch('[0-9a-f]{64}', selected['sha256']) is None or
            type(selected['bytes']) is not int or selected['bytes'] <= 0):
        raise ValueError('Missing or malformed locked wx ReleaseDLL archive')
    archive_path = _regular(archive_path)
    if archive_path.name != ARCHIVE_NAME or _record(archive_path) != {
            key: selected[key] for key in ('sha256', 'bytes')}:
        raise ValueError('wx ReleaseDLL archive differs from lock')
    return selected, archive_path


def _run_archive(seven_zip, args):
    result = subprocess.run([str(seven_zip), *args], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=60)
    if result.returncode:
        raise ValueError('wx archive inspection failed')
    return result.stdout


def _member_listing(data):
    """Parse 7z's bare technical listing, rejecting duplicate/unsafe members."""
    if len(data) > 1024 * 1024:
        raise ValueError('Oversized wx archive member listing')
    records = []
    current = {}
    for line in (data.decode('utf-8') + '\n\n').splitlines():
        if not line:
            if current:
                records.append(current)
                current = {}
            continue
        key, separator, value = line.partition(' = ')
        if not separator or key in current:
            raise ValueError('Malformed or duplicate wx archive member field')
        current[key] = value
    members = {}
    for item in records:
        name = item.get('Path', '').replace('\\', '/')
        _relative(name)
        path = PurePosixPath(name)
        if (path.parent.as_posix() != 'lib/vc14x_dll' or
                DLL_NAME.fullmatch(path.name) is None or
                item.get('Folder', '-') != '-' or
                re.fullmatch('[0-9]+', item.get('Size', '')) is None or
                not 0 < int(item['Size']) <= 64 * 1024 * 1024):
            raise ValueError('Unexpected wx archive member: ' + name)
        folded = path.name.casefold()
        if folded in members:
            raise ValueError('Duplicate wx archive member: ' + name)
        members[folded] = (name, int(item['Size']))
    if not REQUIRED <= members.keys() or len(members) > 64:
        raise ValueError('Missing or unexpected wx archive runtime inventory')
    return members


def _installed_dlls(install):
    install = _workspace_root(Path(install))
    files = {}
    for path in sorted(install.rglob('*')):
        info = path.lstat()
        if (stat.S_ISLNK(info.st_mode) or getattr(info, 'st_file_attributes', 0) & 0x400 or
                not (stat.S_ISREG(info.st_mode) or stat.S_ISDIR(info.st_mode))):
            raise ValueError('Unsafe path in wx runtime install: ' + str(path))
        if not path.name.casefold().startswith('wx') or path.suffix.casefold() != '.dll':
            continue
        if not stat.S_ISREG(info.st_mode) or DLL_NAME.fullmatch(path.name) is None:
            raise ValueError('Unexpected installed wx DLL: ' + str(path))
        name = path.relative_to(install).as_posix()
        _relative(name)
        if path.name.casefold() in files:
            raise ValueError('Duplicate installed wx DLL: ' + name)
        files[path.name.casefold()] = (name, path)
    if not REQUIRED <= files.keys():
        raise ValueError('Required installed wx DLLs missing')
    return files


def create_runtime_receipt(install, lock_path, archive_path, *, seven_zip='7z'):
    """Derive a deterministic receipt from current install and locked archive bytes."""
    archive, archive_path = _locked_archive(lock_path, archive_path)
    members = _member_listing(_run_archive(seven_zip, [
        'l', '-slt', '-ba', '-sccUTF-8', '--', str(archive_path)]))
    files = []
    for basename, (name, path) in _installed_dlls(install).items():
        if basename not in members:
            raise ValueError('Installed wx DLL has no archive member: ' + name)
        member, size = members[basename]
        data = _run_archive(seven_zip, [
            'x', '-so', '-y', '-bd', '-spd', '--', str(archive_path), member])
        record = {'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}
        if len(data) != size or _record(path) != record:
            raise ValueError('Installed wx DLL differs from archive member: ' + name)
        files.append({'path': name, 'member': member, **record})
    # A concurrent replacement must not turn a checked archive into a different
    # source for later members. This is a point-in-time check, not custody.
    if _record(archive_path) != {key: archive[key] for key in ('sha256', 'bytes')}:
        raise ValueError('wx archive changed during inspection')
    return {'schema': 1, 'component': 'wxWidgets', 'version': VERSION, 'abi': ABI,
            'archive': archive, 'files': sorted(files, key=lambda item: item['path']),
            'scope': SCOPE}


def verify_runtime_receipt(install, lock_path, archive_path, *, seven_zip='7z'):
    """Recompute all mappings; disk receipt fields alone are never authority."""
    actual = _read_json(_regular(Path(install) / RECEIPT_NAME))
    expected = create_runtime_receipt(install, lock_path, archive_path, seven_zip=seven_zip)
    # Compare canonical JSON too: Python object equality conflates true with 1.
    if json.dumps(actual, sort_keys=True) != json.dumps(expected, sort_keys=True):
        raise ValueError('wx runtime receipt differs from verified archive/install mapping')
    return expected


def write_runtime_receipt(install, lock_path, archive_path, *, seven_zip='7z'):
    receipt = create_runtime_receipt(install, lock_path, archive_path, seven_zip=seven_zip)
    root = _workspace_root(Path(install))
    output = root / RECEIPT_NAME
    if output.exists() or output.is_symlink():
        _regular(output)
    # A rerun may replace an old receipt only after checking all current bytes.
    fd, temporary = tempfile.mkstemp(prefix='.wx-runtime-', dir=root)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8', newline='\n') as stream:
            stream.write(json.dumps(receipt, indent=2) + '\n')
        os.replace(temporary, output)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('create', 'verify'))
    parser.add_argument('--install', type=Path, required=True)
    parser.add_argument('--lock', type=Path, required=True)
    parser.add_argument('--archive', type=Path, required=True)
    args = parser.parse_args()
    action = write_runtime_receipt if args.action == 'create' else verify_runtime_receipt
    action(args.install, args.lock, args.archive)
    print('wx runtime bytes matched to locked ReleaseDLL archive members')


if __name__ == '__main__':
    main()

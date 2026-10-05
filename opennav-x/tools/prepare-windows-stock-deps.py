#!/usr/bin/env python3
"""Invalidate only the verified maintained-curl sentinel before stock provisioning."""
import argparse
import hashlib
import json
from pathlib import Path

import curl_package
import windows_dependency_bundle as bundle_api
import windows_dependency_receipt as receipt
import windows_dependency_stage as stage

# Exact pinned OpenCPN 37fd0cdd win_deps.bat, normalized only for checkout CRLF.
BATCH_SHA256 = '4e75abdfe34a77e15e5f16e9f7b51de151d9d44924544a43f30127c3a8bf5098'
BATCH = 'build/integration-source/buildwin/win_deps.bat'
STOCK_FILES = ('include/archive.h', 'include/archive_entry.h', 'archive.dll',
               'liblzma.dll', 'lzma.lib', 'iphlpapi.lib', 'glew32.dll', 'glew32.lib',
               'include/glew/glew.h', 'crashrpt/CrashRpt.h', 'crashrpt/CrashRpt1403.lib',
               'crashrpt/CrashRpt1403.dll', 'crashrpt/CrashSender1403.exe',
               'crashrpt/crashrpt_lang.ini', 'crashrpt/dbghelp.dll')


def prefix_inventory(root):
    return receipt._inventory(root, list(stage.PREFIX.values()))


def stock_present(root):
    # Presence avoids needless stock downloads; it is not library qualification.
    # The normal native configure/build/package gates remain authoritative.
    names = list(STOCK_FILES)
    libraries = [name for name in ('archive.lib', 'libarchive.lib')
                 if (root / stage.CACHE / name).exists()]
    if not libraries:
        return False
    for name in names + libraries:
        try:
            path = receipt._plain_path(root, stage.CACHE + '/' + name)
        except FileNotFoundError:
            return False
        if not path.is_file() or path.stat().st_size == 0:
            return False
    return True


def prepare(root, bundle, provenance):
    root = receipt._workspace_root(Path(root))
    authority = bundle_api.verify_restored(root, Path(bundle), Path(provenance))
    batch = receipt._plain_path(root, BATCH).read_bytes().replace(b'\r\n', b'\n')
    if hashlib.sha256(batch).hexdigest() != BATCH_SHA256:
        raise ValueError('Pinned stock dependency batch or curl sentinel logic changed')
    before = prefix_inventory(root)
    manifest = receipt._read_json(receipt._plain_path(root, stage.PREFIX['curl'] + '/curl-build.json'))
    expected = manifest['outputs']['bin/libcurl.dll']
    prefix = receipt._plain_path(root, stage.PREFIX['curl'] + '/bin/libcurl.dll')
    sentinel = receipt._plain_path(root, stage.CACHE + '/libcurl.dll')
    for path in (prefix, sentinel):
        curl_package.verify_file(path, expected)
        curl_package._require_win32_pe(path)
    present = stock_present(root)
    if not present:
        # The upstream batch confuses this maintained TLS file with its entire
        # stock support bundle. Prefixes and every other cache file stay intact.
        sentinel.unlink()
    if prefix_inventory(root) != before:
        raise ValueError('Authenticated producer prefixes changed during stock preparation')
    return {'schema': 1, 'removed': not present, 'stockPresent': present,
            'sentinel': stage.CACHE + '/libcurl.dll', 'verifiedSentinel': expected,
            'batchSha256': BATCH_SHA256, 'producer': authority['producer'],
            'fingerprint': authority['fingerprint'], 'prefixesUnchanged': True,
            'prefixInventorySha256': hashlib.sha256(json.dumps(before, sort_keys=True).encode()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--provenance', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if args.report.exists():
        raise ValueError('Stock preparation report already exists')
    result = prepare(args.root, args.bundle, args.provenance)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open('x', encoding='utf-8') as output:
        json.dump(result, output, indent=2)
        output.write('\n')


if __name__ == '__main__':
    main()

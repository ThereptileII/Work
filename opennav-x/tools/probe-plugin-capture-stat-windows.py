#!/usr/bin/env python3
"""Read-only native metadata proof; never execute dumpbin or inspected bytes."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile

spec = importlib.util.spec_from_file_location('capture_stat_probe', Path(__file__).with_name('retain-plugin-guard-inputs.py'))
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)
FIELDS = ('st_dev', 'st_ino', 'st_mode', 'st_size', 'st_mtime_ns',
          'st_ctime_ns', 'st_nlink', 'st_file_attributes')


def observe(path):
    root = capture.receipt._workspace_root(path.parent)
    before = capture.plain(root, path.name).lstat()
    with path.open('rb') as stream:
        opened = os.fstat(stream.fileno())
        if capture.common_stamp(before) != capture.common_stamp(opened):
            raise ValueError('Native path and descriptor identity differ')
        differences = [name for name in FIELDS if getattr(before, name, 0) != getattr(opened, name, 0)]
        result, identity = capture.record(root, path.name)
        if capture.stamp(os.fstat(stream.fileno())) != capture.stamp(opened):
            raise ValueError('Original native descriptor changed')
    if identity != capture.stamp(before) or capture.stamp(capture.plain(root, path.name).lstat()) != identity:
        raise ValueError('Original native path changed')
    return dict(name=path.name, **result, commonIdentityMatched=True,
                completeBaselinesStable=True, differingStatFields=differences)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dumpbin', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    capture.runner.host_guard()
    tool = args.dumpbin
    if not tool.is_absolute() or tool.name.lower() != 'dumpbin.exe':
        raise ValueError('Explicit actual dumpbin path required')
    output = args.evidence
    if not output.is_absolute() or output.exists() or output.is_symlink():
        raise ValueError('Fresh absolute evidence file required')
    capture.receipt._workspace_root(output.parent)
    records = [observe(tool), observe(Path(sys.executable))]
    with tempfile.TemporaryDirectory(prefix='skager-capture-stat-') as temporary:
        for suffix in ('.exe', '.dll', '.txt'):
            fixture = Path(temporary) / ('inert' + suffix)
            fixture.write_bytes(b'Inert metadata fixture; never executed.\n')
            records.append(observe(fixture))
    report = dict(schema=1, nativeHelperProbe=True, pythonVersion=sys.version,
                  scope='actual native path/descriptor stat and record helper only',
                  records=records, payloadExecuted=False, productAcceptance=False)
    with output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

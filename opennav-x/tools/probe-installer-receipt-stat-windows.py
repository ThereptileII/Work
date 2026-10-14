#!/usr/bin/env python3
"""Read-only receipt stat proof using existing Python and inert exe/txt bytes.

No NSIS/product execution, installer generation, artifact authentication or
product acceptance. Keep the failed producer evidence separately: a matching
stat difference here does not establish that failed run's unlogged field.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import tempfile

import installer_package_receipt as receipt

FIELDS = ('st_dev', 'st_ino', 'st_mode', 'st_size', 'st_mtime_ns',
          'st_ctime_ns', 'st_nlink', 'st_file_attributes')


def observe(path, label):
    path = receipt._local(path)
    before = receipt._plain(path)
    with path.open('rb') as stream:
        opened = os.fstat(stream.fileno())
        differences = [name for name in FIELDS if getattr(before, name, 0) != getattr(opened, name, 0)]
        if receipt._common_signature(before) != receipt._common_signature(opened):
            raise ValueError('Native common identity differs: ' + ','.join(differences))
        digest = receipt._read(path)
        if receipt._signature(os.fstat(stream.fileno())) != receipt._signature(opened):
            raise ValueError('Original native descriptor changed during receipt probe')
    if receipt._signature(receipt._plain(path)) != receipt._signature(before):
        raise ValueError('Original native path changed during receipt probe')
    return dict(source=label, bytes=before.st_size, sha256=digest,
                commonIdentityMatched=True, completeBaselinesStable=True, differingStatFields=differences)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    if sys.platform != 'win32':
        raise ValueError('Native Windows receipt stat probe required')
    output = args.evidence
    if not output.is_absolute() or output.exists() or output.is_symlink():
        raise ValueError('Fresh absolute evidence file required')
    receipt._local(output.parent, directory=True)
    records = [observe(Path(sys.executable), 'actual-python-executable')]
    with tempfile.TemporaryDirectory(prefix='skager-receipt-stat-') as temporary:
        for suffix in ('.exe', '.txt'):
            fixture = Path(temporary) / ('inert-Setup' + suffix)
            fixture.write_bytes(b'Inert receipt stat fixture; never executed.\r\n')
            records.append(observe(fixture, 'inert' + suffix))
    report = dict(schema=1, nativeHelperProbe=True, pythonVersion=sys.version,
                  scope='receipt native path/descriptor baselines and read only', records=records,
                  inspectedPayloadExecuted=False, installerBuilt=False, productAcceptance=False)
    with output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, sort_keys=True, indent=2)
        stream.write('\n')
    print(json.dumps(report, sort_keys=True, indent=2))


if __name__ == '__main__':
    main()

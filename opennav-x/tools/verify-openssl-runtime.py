#!/usr/bin/env python3
"""Compare immutable OpenSSL identity; retain live CPU observations separately."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess

import curl_package
from windows_dependency_receipt import _read_json

CPU = re.compile(r'CPUINFO: OPENSSL_ia32cap=((?:0x[0-9a-f]{16}:){4}0x[0-9a-f]{16})\Z')
FIELDS = ('OpenSSL 3.5.9 ', 'built on: ', 'platform: VC-WIN32', 'options: ',
          'compiler: ', 'OPENSSLDIR: ', 'ENGINESDIR: ', 'MODULESDIR: ',
          'Seeding source: ', 'CPUINFO: ')


def version_parts(value):
    if not isinstance(value, str) or len(value) > 16384:
        raise ValueError('Invalid OpenSSL version output')
    lines = value.splitlines(keepends=True)
    if len(lines) != len(FIELDS) or any(not line.startswith(prefix)
            for line, prefix in zip(lines, FIELDS)):
        raise ValueError('Missing, reordered or unexpected OpenSSL version fields')
    if not re.fullmatch(r'OpenSSL 3\.5\.9 [^\r\n]+ \(Library: OpenSSL 3\.5\.9 [^\r\n]+\)',
                        lines[0].rstrip('\r\n')):
        raise ValueError('OpenSSL executable/library version differs')
    if lines[2].rstrip('\r\n') != 'platform: VC-WIN32':
        raise ValueError('OpenSSL platform differs')
    match = CPU.fullmatch(lines[-1])
    if match is None:
        raise ValueError('Malformed, overridden or unexpected OpenSSL CPU observation')
    return ''.join(lines[:-1]), match.group(1)


def compare(expected, actual, environment):
    if any(name.casefold() == 'openssl_ia32cap' for name in environment):
        raise ValueError('OpenSSL CPU capability overrides are forbidden')
    producer, producer_cpu = version_parts(expected)
    current, current_cpu = version_parts(actual)
    if producer != current:
        raise ValueError('Immutable OpenSSL version/build/configuration identity differs')
    return dict(status='passed', producerVersionOutput=expected,
                consumerVersionOutput=actual, producerCpu=producer_cpu,
                consumerCpu=current_cpu, cpuChanged=producer_cpu != current_cpu,
                scope='Runtime observation only; original producer evidence unchanged')


def verify(manifest_path, executable, environment):
    manifest = _read_json(manifest_path)
    prefix = manifest_path.parent
    if executable.absolute() != (prefix / 'bin/openssl.exe').absolute():
        raise ValueError('OpenSSL executable must be the original producer prefix tool')
    for name in ('openssl.exe', 'libcrypto-3.dll', 'libssl-3.dll'):
        path = prefix / 'bin' / name
        curl_package.verify_file(path, manifest['outputs']['bin/' + name])
    if any(name.casefold() == 'openssl_ia32cap' for name in environment):
        raise ValueError('OpenSSL CPU capability overrides are forbidden')
    result = subprocess.run([str(executable), 'version', '-a'], env=environment,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
    if result.returncode or result.stderr or len(result.stdout) > 16384:
        raise ValueError('OpenSSL version probe failed or emitted unexpected output')
    actual = result.stdout.decode('utf-8').rstrip('\r\n')
    report = compare(manifest['versionOutput'], actual, environment)
    report['executableSha256'] = manifest['outputs']['bin/openssl.exe']['sha256']
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.name != 'nt':
        raise ValueError('Native Windows runtime required')
    report = verify(args.manifest, args.executable, dict(os.environ))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print('OpenSSL immutable identity verified; live CPU observation retained separately')


if __name__ == '__main__':
    main()

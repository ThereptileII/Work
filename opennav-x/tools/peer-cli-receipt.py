#!/usr/bin/env python3
"""Capture or verify same-attempt evidence from the early installed peer CLI test."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ('tools/test-peer-cli.py', 'tools/peer-cli-receipt.py',
           'tools/build-pristine-windows.ps1')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def identity(executable):
    if (os.name != 'nt' or os.environ.get('GITHUB_ACTIONS') != 'true' or
            os.environ.get('RUNNER_ENVIRONMENT') != 'github-hosted' or
            os.environ.get('GITHUB_JOB') != 'windows-integration'):
        raise ValueError('Peer CLI receipt requires the disposable native integration job')
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    if commit != os.environ.get('GITHUB_SHA'):
        raise ValueError('Peer CLI receipt checkout does not match the workflow commit')
    sources = {}
    for name in SOURCES:
        tracked = subprocess.check_output(['git', 'ls-files', '--full-name', '--', name], cwd=ROOT, text=True).strip()
        if not tracked or '\n' in tracked:
            raise ValueError('Peer CLI receipt source is not tracked uniquely')
        committed = subprocess.check_output(['git', 'show', commit + ':' + tracked], cwd=ROOT)
        actual = (ROOT / name).read_bytes()
        sources[name] = hashlib.sha256(actual).hexdigest()
        # Git for Windows may check text out as CRLF. Bind exact executed bytes
        # in the receipt, while allowing only that checkout newline conversion.
        if committed.replace(b'\r\n', b'\n') != actual.replace(b'\r\n', b'\n'):
            raise ValueError('Peer CLI receipt source differs from the candidate commit')
    return {'schema': 1, 'owner': 'OpenNavX.CI.PeerCli.1', 'status': 'success',
            'commit': commit, 'runId': os.environ['GITHUB_RUN_ID'],
            'runAttempt': os.environ['GITHUB_RUN_ATTEMPT'],
            'job': os.environ['GITHUB_JOB'], 'sourcesSha256': sources,
            'executableSha256': sha(executable)}


def verify(receipt, expected):
    if receipt != expected:
        raise ValueError('Missing, stale, failed or changed-source peer CLI receipt')


def capture(executable, destination, expected):
    if destination.exists():
        raise ValueError('Refusing to overwrite an existing peer CLI receipt')
    subprocess.run([sys.executable, str(ROOT / 'tools/test-peer-cli.py'),
                    '--cli', str(executable)], check=True, timeout=90)
    # Revalidate after execution, then publish only a positive immutable result.
    verify(identity(executable), expected)
    with destination.open('x', encoding='utf-8') as stream:
        json.dump(expected, stream, indent=2)
        stream.write('\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('capture', 'verify'))
    parser.add_argument('--cli', required=True, type=Path)
    parser.add_argument('--receipt', required=True, type=Path)
    args = parser.parse_args()
    executable = args.cli.resolve(strict=True)
    expected = identity(executable)
    if args.action == 'capture':
        capture(executable, args.receipt, expected)
    else:
        verify(json.loads(args.receipt.read_text(encoding='utf-8')), expected)
        print('Early installed peer CLI success receipt verified for this exact source, executable and run attempt.')


if __name__ == '__main__':
    main()

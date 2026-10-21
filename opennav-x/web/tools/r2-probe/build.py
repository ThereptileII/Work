#!/usr/bin/env python3
"""Bundle only the admin probe, preserving pinned SDK bytes and complete notice."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

SOURCE = Path(__file__).resolve().parent
WEB = SOURCE.parents[1]
SDK = WEB / 'edge/vendor/aws4fetch-1.0.20'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source-commit', required=True)
    args = parser.parse_args()
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=WEB, text=True).strip()
    if head != args.source_commit or subprocess.check_output(['git', 'status', '--porcelain', '--', '.'], cwd=WEB, text=True).strip():
        parser.error('Use exact committed clean web source')
    output = args.output.resolve()
    if output.exists() or output == WEB or WEB in output.parents:
        parser.error('Use a fresh output directory outside web source')
    dependency = json.loads((SDK / 'manifest.json').read_text())
    for name, expected in dependency['files'].items():
        if hashlib.sha256((SDK / name).read_bytes()).hexdigest() != expected:
            raise ValueError('Pinned SDK provenance mismatch')
    output.mkdir(parents=True)
    bundle = output / 'index.ts'
    subprocess.run(['npm', 'exec', '--yes', '--package=esbuild@0.28.2', '--', 'esbuild',
                    str(SOURCE / 'index.ts'), '--bundle', '--format=esm', '--platform=browser',
                    '--outfile=' + str(bundle)], cwd=WEB, check=True)
    bundle.write_text('/*! aws4fetch ' + dependency['version'] + '\n' +
                      (SDK / 'LICENSE').read_text() + '*/\n' + bundle.read_text())
    info = dict(schema=1, purpose='administrative-r2-head-bucket-only', sourceCommit=head,
                esbuildVersion='0.28.2', serverDependencies={'aws4fetch': dependency},
                sha256=hashlib.sha256(bundle.read_bytes()).hexdigest())
    (output / 'build-info.json').write_text(json.dumps(info, indent=2) + '\n')
    print(json.dumps(info, indent=2))


if __name__ == '__main__':
    main()

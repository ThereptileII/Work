#!/usr/bin/env python3
"""Build immutable public Worker assets and server-only Supabase Edge code.

Output stays outside Git. No environment values or credentials are read.
"""
import argparse
import base64
import hashlib
import json
import mimetypes
from pathlib import Path
import shutil
import subprocess
import sys

WEB = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source-commit', required=True)
    args = parser.parse_args()
    actual = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=WEB, text=True).strip()
    if args.source_commit != actual:
        parser.error('Source commit must be the current exact HEAD')
    if subprocess.check_output(['git', 'status', '--porcelain', '--', '.'], cwd=WEB, text=True).strip():
        parser.error('Commit the web source before packaging')
    output = args.output.resolve()
    if output.exists() or WEB in output.parents:
        parser.error('Use a fresh output directory outside the web source')
    output.mkdir(parents=True)
    subprocess.run([sys.executable, str(WEB/'tools/export_preview.py'),
                    '--output', str(output/'public'), '--source-commit', actual], cwd=WEB, check=True)
    for item in (WEB/'edge/static').iterdir():
        if item.is_file() and item.suffix in {'.html', '.js', '.css'}:
            shutil.copy2(item, output/'public'/item.name)
    assets = {}
    for item in sorted((output/'public').rglob('*')):
        if not item.is_file() or item.name in {'_headers', 'preview-provenance.json'}:
            continue
        mime = mimetypes.guess_type(item.name)[0] or 'application/octet-stream'
        assets['/'+item.relative_to(output/'public').as_posix()] = {
            'type': mime, 'body': base64.b64encode(item.read_bytes()).decode('ascii')}
    (output/'generated-assets.mjs').write_text('export default Object.freeze('+json.dumps(assets)+');\n')
    shutil.copy2(WEB/'edge/cloudflare-worker.mjs', output/'cloudflare-worker.mjs')
    command = ['npm', 'exec', '--yes', '--package=esbuild@0.28.2', '--', 'esbuild']
    subprocess.run(command + [str(output/'cloudflare-worker.mjs'), '--bundle', '--format=esm',
        '--platform=browser', '--minify', '--outfile='+str(output/'cloudflare-upload.js')], cwd=WEB, check=True)
    subprocess.run(command + [str(WEB/'edge/index.ts'), '--bundle', '--format=esm',
        '--platform=browser', '--outfile='+str(output/'supabase-index.ts')], cwd=WEB, check=True)
    files = ['cloudflare-upload.js', 'supabase-index.ts']
    info = {'schema':1, 'sourceCommit':actual, 'environment':'private-beta',
            'hostname':'beta.skager.app', 'livePayments':False,
            'esbuildVersion':'0.28.2', 'files':{name:hashlib.sha256((output/name).read_bytes()).hexdigest() for name in files}}
    (output/'build-info.json').write_text(json.dumps(info, indent=2)+'\n')
    print(json.dumps(info, indent=2))


if __name__ == '__main__':
    main()

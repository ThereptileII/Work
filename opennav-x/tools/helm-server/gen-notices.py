#!/usr/bin/env python3
"""Regenerates THIRD_PARTY_NOTICES.txt from the modules linked into skager-helm
(Windows and Linux builds). Run after changing go.mod: python3 gen-notices.py"""
import glob, os, subprocess
from pathlib import Path

here = Path(__file__).resolve().parent
go = os.environ.get('GO', 'go')
mods = {}
for goos in ('windows', 'linux'):
    out = subprocess.check_output([go, 'list', '-deps', '-f', '{{if .Module}}{{.Module.Path}}\t{{.Module.Version}}\t{{.Module.Dir}}{{end}}',
                                   './cmd/helm-server'], cwd=here, env={**os.environ, 'GOOS': goos, 'CGO_ENABLED': '0'}, text=True)
    for line in out.splitlines():
        if line.strip():
            path, version, directory = line.split('\t')
            if path != 'skager.app/helm':
                mods[path] = (version, directory)
parts = ['SKAGER Helm Server (skager-helm) - third-party software notices', '',
         'skager-helm is built with Go and the modules below, under their own licenses.',
         'The Go standard library is Copyright The Go Authors, BSD-3-Clause (https://go.dev/LICENSE).', '']
for path in sorted(mods):
    version, directory = mods[path]
    found = []
    for f in sorted(glob.glob(os.path.join(directory, '*'))):
        if os.path.basename(f).upper().startswith(('LICENSE', 'LICENCE', 'COPYING', 'NOTICE', 'PATENTS')):
            found += sorted(glob.glob(os.path.join(f, '*'))) if os.path.isdir(f) else [f]
    if not found:
        raise SystemExit('no license file for ' + path)
    parts += ['=' * 78, f'{path} {version}', '=' * 78]
    for f in found:
        parts += ['--- ' + os.path.relpath(f, directory), Path(f).read_text(encoding='utf-8', errors='replace').rstrip()]
    parts.append('')
(here / 'THIRD_PARTY_NOTICES.txt').write_text('\n'.join(parts) + '\n', encoding='utf-8')
print(f'{len(mods)} modules')

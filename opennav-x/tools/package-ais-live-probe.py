#!/usr/bin/env python3
"""Package the explicit read-only AIS probe in disposable native Windows CI.

No credential reads, live connection, product installation or hardware access.
The package contains only its verified import closure and corresponding source.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile
from source_package import create_source_archive, PINNED_UPSTREAM
from openssl_package import verify_openssl_package_inputs, verify_packaged_openssl

ROOT = Path(__file__).resolve().parents[1]


def isolated_environment(environment):
    # os.environ is case-insensitive on Windows; a copied dict is not. Python
    # commonly exposes SYSTEMROOT even if its spelling at creation differed.
    env = {key.upper(): value for key, value in environment.items()}
    env['PATH'] = str(Path(env['SYSTEMROOT']) / 'System32')
    env.pop('AISSTREAM_API_KEY', None)
    return env


def dependency_candidates(install, runtime):
    """Use the licensed current MSVC CRT, as the main product packager does.

    The supported OpenCPN dependency bundle includes older same-name CRTs.
    Only this explicitly selected CRT directory may override those copies.
    An ambiguous non-CRT dependency is still a hard failure.
    """
    if runtime.name != 'Microsoft.VC143.CRT' or not all(
            (runtime / n).is_file() for n in ('msvcp140.dll', 'vcruntime140.dll')):
        raise ValueError('Expected licensed MSVC x86 CRT directory')
    candidates = {}
    for folder in (install, runtime):
        for file in folder.glob('*.dll'):
            name = file.name.lower()
            crt = folder == runtime and name.startswith(('msvcp', 'vcruntime', 'vccorlib', 'concrt', 'vcomp'))
            if name in candidates and file.read_bytes() != candidates[name].read_bytes() and not crt:
                raise ValueError('Ambiguous non-CRT dependency in the validated build: ' + name)
            candidates[name] = file
    return candidates


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--openssl-source-cache', type=Path, required=True,
                   help='Preverified upstream source tar; packaging never downloads it')
    a = p.parse_args()
    if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
        raise SystemExit('Use the qualified disposable native Windows CI build')
    output = a.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    openssl_source = verify_openssl_package_inputs(
        ROOT / 'build/production-install', ROOT / 'tools/windows-openssl.lock.json',
        a.openssl_source_cache, ROOT / 'docs/third-party/OpenSSL-3.5.9')
    package = output / 'SKAGER-AIS-ReadOnly-Probe'
    app = package / 'app'
    app.mkdir(parents=True)
    exe = ROOT / 'build/production-windows/Release/aisstream_live_probe.exe'
    shutil.copy2(exe, app / exe.name)
    spec = importlib.util.spec_from_file_location('pe', ROOT / 'tools/verify-preview-pe.py')
    pe = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pe)
    candidates = dependency_candidates(ROOT / 'build/production-install', a.runtime.resolve())
    pending = [app / exe.name]
    copied = set()
    while pending:
        for name in pe.imports(pending.pop()):
            if name in candidates and name not in copied:
                copied.add(name)
                path = app / candidates[name].name
                shutil.copy2(candidates[name], path)
                pending.append(path)
    verify_packaged_openssl(app, openssl_source['manifest'])
    subprocess.run([sys.executable, str(ROOT / 'tools/verify-preview-pe.py'), str(app),
                    '--report', str(package / 'dependency-audit.json')], check=True)
    env = isolated_environment(os.environ)
    result = subprocess.run([str(app / exe.name), '--describe'], cwd=app, env=env,
                            capture_output=True, text=True, timeout=15, check=True)
    description = json.loads(result.stdout)
    assert description['profile_access'] is False and description['marine_equipment'] is False
    (package / 'capabilities.json').write_text(json.dumps(description, indent=2) + '\n')
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    source = output / 'SKAGER-AIS-ReadOnly-Probe-source.zip'
    create_source_archive(ROOT, commit, source, [openssl_source['sourceBundle']])
    licenses = package / 'licenses'
    licenses.mkdir()
    shutil.copy2(ROOT / 'LICENSE', licenses / 'SKAGER-COPYING.txt')
    shutil.copytree(ROOT / 'docs/third-party/OpenSSL-3.5.9', licenses / 'OpenSSL-3.5.9')
    upstream = ROOT / 'build/integration-source'
    for file in upstream.rglob('*'):
        if file.is_file() and file.name.lower().startswith(('copying', 'license', 'copyright')):
            target = licenses / 'OpenCPN' / file.relative_to(upstream)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(file, target)
    (package / 'READ_ME.md').write_text('''# Read-only AIS commissioning probe

This is a developer diagnostic, not the SKAGER application or a navigation product.
It cannot load OpenCPN profiles, charts or hardware plugins. It does not change
network settings. No fixture data is included. Run only by deliberate choice:

`app\\aisstream_live_probe.exe --describe` (offline capabilities only)

`app\\aisstream_live_probe.exe --read-only-live-ais stockholm`

The second command connects to the fixed AISStream service for up to 45 seconds
using the current user's `OpenNavX/AISStream/v1` Windows Credential Manager entry.
Do not pass a key on the command line. `oresund` is the other supported public area.
It prints only connection states and aggregate report/target counts. A nonzero
exit is negative commissioning evidence. This does not qualify the chart or UI.

The sibling source ZIP includes exact application, integration and bundled
dependency source and all SKAGER CI recipes. See licenses/ for notices.
MSVC DLLs are licensed x86 redistributables from the native CI toolchain.
''', encoding='utf-8')
    (package / 'BUILD_INFO.json').write_text(json.dumps({
        'commit': commit, 'upstreamCommit': PINNED_UPSTREAM,
        'architecture': 'Win32 x86 on Windows x64',
        'ciRun': 'https://github.com/' + os.environ['GITHUB_REPOSITORY'] + '/actions/runs/' + os.environ['GITHUB_RUN_ID'],
        'sourceArchive': source.name, 'sourceSha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'binaries': {str(f.relative_to(package)): hashlib.sha256(f.read_bytes()).hexdigest()
                     for f in sorted(app.iterdir())},
        'liveServiceAccepted': False, 'chartOrUiAccepted': False,
    }, indent=2) + '\n')
    archive = output / (package.name + '.zip')
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
        for file in sorted(package.rglob('*')):
            if file.is_file():
                z.write(file, file.relative_to(output))
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
    (output / 'SHA256SUMS.txt').write_text(''.join(
        hashlib.sha256(f.read_bytes()).hexdigest() + '  ' + f.name + '\n'
        for f in [archive, source]))
    print('Read-only AIS probe closure, clean-PATH launch, source and archive verified; no live connection')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Assemble a fresh portable preview from the native CMake install, then ZIP it.

Packaging never reads an installed OpenCPN or a normal user profile.
Windows runtime preparation and native gates are performed by the caller.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--install', type=Path, required=True)
parser.add_argument('--build', type=Path, required=True)
parser.add_argument('--runtime', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
destination = args.output / 'OpenNavX-Alpha1-Portable'
if destination.exists():
    raise SystemExit('Refusing to overwrite an existing preview directory')
destination.mkdir(parents=True)
shutil.copytree(args.install, destination / 'app')
app = destination / 'app'
if not (app / 'opencpn.exe').is_file() or not (app / 'opennav-restart.exe').is_file():
    raise SystemExit('Native executable/restart helper missing from CMake install')
for dll in args.runtime.glob('*.dll'):
    shutil.copy2(dll, app / dll.name)
for required in ['msvcp140.dll', 'vcruntime140.dll']:
    if not (app / required).is_file():
        raise SystemExit('App-local MSVC runtime missing: ' + required)
(app / 'OPENNAV_PORTABLE_PREVIEW').write_text('OpenNav X portable Alpha 1\n')
for directory in ['profile', 'logs', 'demo', 'docs/licenses']:
    (destination / directory).mkdir(parents=True)
# PluginPaths::InitWindowsPaths and GetPluginDataPath use PrivateDataDir/plugins
# in portable mode, independently of the platform's app/plugins directory.
# Retain the installed resources and provide the same bundled plugins at that
# upstream portable location; never discover/copy a normal installed plugin.
shutil.copytree(app / 'plugins', destination / 'profile/plugins')
for required in ['dashboard_pi.dll', 'chartdldr_pi.dll', 'grib_pi.dll', 'wmm_pi.dll']:
    if not (destination / 'profile/plugins' / required).is_file():
        raise SystemExit('Bundled portable plugin missing: ' + required)
config = (args.build / 'include/config.h').read_text()
version = re.search(r'#define VERSION_FULL "([^"]+)"', config).group(1)
date = re.search(r'#define VERSION_DATE "([^"]+)"', config).group(1)
(destination / 'profile/opencpn.conf').write_text(
    '[Settings]\n' + f'ConfigVersionString=Version {version} Build {date}\n' +
    'NavMessageShown=1\nShowStatusBar=1\nShowMenuBar=1\n'
    '[Settings/GlobalState]\nFrameWinX=1280\nFrameWinY=800\nFrameWinPosX=0\nFrameWinPosY=0\nFrameMax=0\n'
    'VPLatLon=59.0800,18.5000\nVPScale=0.003\n'
    '[OpenNav]\nInterfaceMode=xnav\n', encoding='utf-8')
(destination / 'profile/README.txt').write_text('Isolated preview profile. Do not copy a production OpenCPN profile here.\n')
(destination / 'logs/README.txt').write_text('OpenNav diagnostics and launcher output live here. Current OpenCPN log: ../profile/opencpn.log\n')
launchers = {'Run-XNav': '--xnav', 'Run-XNav-Demo': '--xnav --xnav-demo',
             'Run-Legacy': '--legacy', 'Run-Safe': '--safe-mode'}
for name, mode in launchers.items():
    text = f'''@echo off
setlocal
cd /d "%~dp0"
if not exist "%~dp0app\\opencpn.exe" (
  echo Extract the entire OpenNav Alpha ZIP before running this launcher.
  pause
  exit /b 1
)
if not exist "%~dp0profile" mkdir "%~dp0profile"
if not exist "%~dp0logs" mkdir "%~dp0logs"
"%~dp0app\\opencpn.exe" --portable --configdir "%~dp0profile" --no_opengl {mode} %* >> "%~dp0logs\\{name}.log" 2>&1
set "preview_exit=%errorlevel%"
if exist "%~dp0profile\\opencpn.log" copy /y "%~dp0profile\\opencpn.log" "%~dp0logs\\opencpn.log" >nul
if not "%preview_exit%"=="0" (
  echo OpenNav exited with code %preview_exit%. See logs\\{name}.log and profile\\opencpn.log.
  pause
)
exit /b %preview_exit%
'''
    (destination / (name + '.cmd')).write_bytes(text.replace('\n', '\r\n').encode('utf-8'))
for file in (ROOT / 'docs/alpha').glob('*.md'):
    shutil.copy2(file, destination / 'docs' / file.name)
shutil.copy2(ROOT / 'docs/physical-validation.md', destination / 'docs/physical-validation.md')
version_header = (ROOT / 'src/application/Version.h').read_text()
product_version = re.search(r'Version\[\] = "([^"]+)"', version_header).group(1)
commit = os.environ.get('GITHUB_SHA') or subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
run = 'https://github.com/' + os.environ.get('GITHUB_REPOSITORY', 'ThereptileII/Work') + '/actions/runs/' + os.environ.get('GITHUB_RUN_ID', 'local')
build_header = (args.build / 'include/OpenNavBuild.h').read_text()
def build_value(key):
    return re.search(r'#define ' + key + r' "([^"]+)"', build_header).group(1)
if build_value('OPENNAV_BUILD_COMMIT') != commit:
    raise SystemExit('Executable build commit does not match package commit')
info = f'''# Build information

- OpenNav X: Alpha 1 / {product_version}
- Git commit: `{commit}`
- OpenCPN: 5.12.4
- Pinned upstream: `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`
- Compiler: {build_value('OPENNAV_BUILD_COMPILER')}
- Architecture: Win32/x86 application and plugin ABI; Windows 10/11 x64 host
- Build date (UTC): {build_value('OPENNAV_BUILD_DATE')}
- CI run: {run}
- Modes: XNav, explicit Demo, Legacy, Safe; package-local profile only
- UI gates: native 1280×800 / 96, 120, 144 DPI; software and available OpenGL/fallback
- Stock OpenCPN executable SHA-256 for installer qualification: `7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`
- Setup uses the normal profile; this ZIP uses its own profile only.
- See the same-commit CI/evidence record for actual acceptance and limitations.
- ZIP SHA-256: supplied alongside the ZIP. It cannot be embedded in itself.
- File hashes: `FILE_SHA256.json` in the package root.

See TEST_ME_FIRST.md and KNOWN_LIMITATIONS.md before running.
'''
(destination / 'docs/BUILD_INFO.md').write_text(info, encoding='utf-8')
(destination / 'demo/scenarios.json').write_text(json.dumps({
    'fixture_version': '0.1', 'source': 'DEMO built-in deterministic generator',
    'note': 'Reference manifest; editing this file does not change the compiled fixture.',
    'usable_capacity_kwh': 48, 'reserve_soc_percent': 15, 'hotel_load_kw': 0.4,
    'trip_speedup': 60, 'initial_distance_nm': 18.2,
    'scenarios': ['Cruising', 'Sensors stale', 'Sensors unavailable', 'Route inactive',
                  'Route ending', 'Low battery', 'High power', 'Energy shortfall']
}, indent=2) + '\n')
source = ROOT / 'upstream/OpenCPN'
for license_file in source.rglob('*'):
    if license_file.is_file() and license_file.name.lower().startswith(('copying', 'license', 'copyright')) and 'cache' not in license_file.relative_to(source).parts:
        relative = license_file.relative_to(source)
        target = destination / 'docs/licenses/OpenCPN' / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(license_file, target)
shutil.copytree(ROOT / 'docs/third-party/wxWidgets-3.2.8', destination / 'docs/licenses/wxWidgets-3.2.8')
shutil.copy2(ROOT / 'LICENSE', destination / 'docs/licenses/OpenNavX-COPYING.txt')
(destination / 'docs/SOURCE_AND_LICENSES.md').write_text(f'''# Source and third-party notices

OpenCPN and this integration are distributed under their applicable GPL terms.
Full project source: https://github.com/ThereptileII/Work/tree/{commit}/opennav-x
Pinned OpenCPN source: https://github.com/OpenCPN/OpenCPN/tree/37fd0cddb7334fe489e9f18aa163977a9c5c84f7
Build scripts, dependency locks and exact integration patches are in the project.
The CI artifact also supplies a corresponding-source archive. See `licenses/`
for bundled OpenCPN/library notices and the installed application's license files.

MSVC runtime DLLs are the x86 redistributable files from the licensed CI toolchain.
Application-local deployment is described by Microsoft:
https://learn.microsoft.com/en-us/cpp/windows/redistributing-visual-cpp-files
wxWidgets uses the wxWindows Library Licence; dependency provenance is retained
in the Windows evidence and `tools/windows-wx.lock.json` in the source archive.
''', encoding='utf-8')
manifest = {str(f.relative_to(destination)).replace('\\', '/'): hashlib.sha256(f.read_bytes()).hexdigest()
            for f in sorted(destination.rglob('*')) if f.is_file()}
(destination / 'FILE_SHA256.json').write_text(json.dumps(manifest, indent=2) + '\n')
archive = args.output / 'OpenNavX-Alpha1-Portable-win64.zip'
with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
    for file in sorted(destination.rglob('*')):
        if file.is_file(): z.write(file, file.relative_to(args.output))
(archive.with_suffix('.zip.sha256')).write_text(hashlib.sha256(archive.read_bytes()).hexdigest() + '  ' + archive.name + '\n')
# Complete tracked integration source, not an expiring offer to fetch it later.
with zipfile.ZipFile(args.output / 'OpenNavX-Alpha1-source.zip', 'w', zipfile.ZIP_DEFLATED) as z:
    for directory, prefix in [(ROOT, 'opennav-x'), (ROOT / 'build/integration-source', 'OpenCPN-5.12.4-integrated')]:
        files = subprocess.check_output(['git', 'ls-files', '-z'], cwd=directory).decode().split('\0')
        for name in files:
            f = directory / name
            if name and f.is_file() and not name.startswith('upstream/'):
                z.write(f, prefix + '/' + name)
print(archive)

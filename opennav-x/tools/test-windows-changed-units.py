#!/usr/bin/env python3
"""Compile complete patched Windows units without rebuilding dependencies.

Compile-only by default. Optional Settings component execution provides isolated
native interaction evidence, never product, package or dependency qualification.
The optional negative control restores exactly the seven legacy max calls in
the two real source files; it must fail before the untouched fixed files pass.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
UNITS = ('model/src/downloader.cpp', 'model/src/peer_client.cpp',
         'libs/wxcurl/src/base.cpp', 'libs/wxcurl/src/http.cpp')
UI_CHANGED = tuple('src/ui/' + name + '.cpp' for name in (
    'AisDrawer', 'AnchorDrawer', 'ChoiceField', 'ContextCard', 'Controls',
    'Drawer', 'HealthDrawer', 'Horizon', 'PassageDrawer', 'ProductPanel',
    'SettingsDrawer', 'Sheet', 'Shell'))
SETTINGS = 'src/integration/SettingsStore.cpp'


def record(path):
    data = path.read_bytes()
    return {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}


def run(args, log, *, cwd=ROOT, timeout=300, expected_failure=None):
    args = [str(a) for a in args]
    log.with_suffix('.command.json').write_text(json.dumps(args, indent=2))
    with log.open('wb') as output:
        child = subprocess.Popen(args, cwd=cwd, stdout=output, stderr=subprocess.STDOUT)
        try:
            code = child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            subprocess.run(['taskkill', '/PID', str(child.pid), '/T', '/F'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
            child.wait(timeout=30)
            raise
    if expected_failure:
        text = log.read_text(errors='replace')
        filename = re.escape(expected_failure)
        if (code == 0 or
                not re.search(filename + r'\(\d+,\d+\): warning C4003:.*\bmax\b', text) or
                not re.search(filename + r'\(\d+,\d+\): error C2589:', text)):
            raise RuntimeError(f'Legacy Windows max collision was not reproduced: {log}')
    elif code:
        raise RuntimeError(f'Command failed ({code}); retained log: {log}')
    return code


def fetch(item, path):
    expected = {k: item[k] for k in ('bytes', 'sha256')}
    if path.is_file() and record(path) == expected:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    request = urllib.request.Request(item['url'], headers={'User-Agent': 'Mozilla/5.0'})
    with urllib.request.urlopen(request, timeout=60) as source, path.open('wb') as output:
        total = 0
        while block := source.read(65536):
            total += len(block)
            if total > expected['bytes']:
                raise ValueError('SDK download exceeds locked length')
            output.write(block)
    if record(path) != expected:
        raise ValueError(f'SDK download differs from lock: {path.name}')


def stage_native_runtime(client, wx, wx_dlls):
    """Stage the locked wx DLLs and installed Win32 compiler runtime app-local."""
    runtime = []
    for name in wx_dlls:
        source = wx / 'lib/vc14x_dll' / name
        shutil.copy2(source, client.parent / name)
        runtime.append(source)
    vswhere = Path(os.environ['ProgramFiles(x86)']) / 'Microsoft Visual Studio/Installer/vswhere.exe'
    install = subprocess.check_output([str(vswhere), '-latest', '-products', '*',
        '-requires', 'Microsoft.VisualStudio.Component.VC.Tools.x86.x64',
        '-property', 'installationPath'], text=True).strip()
    if not install:
        raise ValueError('Native Visual C++ toolchain installation missing')
    # A Win32 executable needs x86 CRT DLLs, irrespective of runner bitness.
    versions = sorted((Path(install) / 'VC/Redist/MSVC').glob('*/x86/Microsoft.VC143.CRT'),
                      key=lambda p: tuple(int(n) for n in p.parts[-3].split('.')))
    if not versions:
        raise ValueError('Native x86 VC143 runtime missing')
    crt = versions[-1]
    if not all((crt / name).is_file() for name in ('msvcp140.dll', 'vcruntime140.dll')):
        raise ValueError('Native x86 VC143 runtime incomplete')
    for source in sorted(crt.glob('*.dll')):
        shutil.copy2(source, client.parent / source.name)
        runtime.append(source)
    return {source.name: dict(source=str(source), **record(client.parent / source.name))
            for source in runtime}


def settings_component(build, wx, evidence):
    """Use the existing offline component and capture gate with app-local DLLs."""
    client = build / 'Release/settings_drawer_test.exe'
    manifest = stage_native_runtime(client, wx,
        ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll', 'wxmsw32u_aui_vc14x.dll'))
    (evidence / 'settings-runtime.json').write_text(json.dumps(manifest, indent=2) + '\n')
    output = evidence / 'settings-component'
    run([sys.executable, ROOT / 'tools/prototype/capture-ais-component.py',
         '--component', 'settings', '--client', client, '--output', output],
        evidence / 'settings-component.log', timeout=90)
    capture = json.loads((output / 'capture.json').read_text())
    if capture['platform'] != 'win32' or capture['executable_sha256'] != record(client)['sha256']:
        raise ValueError('Settings component capture does not identify the native tested executable')
    return {'executable': record(client), 'runtime': manifest, 'capture': capture}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', type=Path, default=ROOT / 'upstream/OpenCPN')
    parser.add_argument('--evidence', type=Path, default=ROOT / 'evidence/local/windows-changed-units')
    parser.add_argument('--legacy-control', action='store_true')
    parser.add_argument('--ui', action='store_true', help='also compile the real production UI static library and SettingsStore')
    parser.add_argument('--settings-component', action='store_true',
                        help='with --ui, link and run the existing offline native Settings component')
    args = parser.parse_args()
    if args.settings_component and not args.ui:
        parser.error('--settings-component requires --ui')
    if sys.platform != 'win32':
        raise SystemExit('Native Windows required; Linux compilation does not qualify this gate')
    for name in ('CL', '_CL_', 'CXXFLAGS', 'CFLAGS'):
        if os.environ.get(name):
            raise SystemExit(f'Refusing inherited compiler override: {name}')
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    report = {'status': 'failed', 'scope': 'native Win32 complete translation-unit compilation only'}
    if args.settings_component:
        report['scope'] = ('native Win32 translation-unit compilation and offline Settings '
                           'component interaction/capture; not product qualification')
    try:
        report['candidate'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
        report['gateInputs'] = {p: record(ROOT / p) for p in
                               ('tools/test-windows-changed-units.py', 'tools/prepare-integration.py',
                                'tests/windows_changed_units/CMakeLists.txt', 'upstream.lock.json')}
        if args.ui:
            report['uiBuildPolicy'] = {'testFixtures': False, 'pilotLoopback': False,
                                       'runtimeTests': args.settings_component}
            report['uiInputs'] = {p: record(ROOT / p) for p in
                                  ('CMakeLists.txt', 'cmake/AisJson.cmake', *UI_CHANGED, SETTINGS)}
            # Bind the complete local header/source tree consumed by the real UI
            # target, including unchanged units and transitive build-policy code.
            report['uiSourceTree'] = {str(p.relative_to(ROOT)): record(p)
                                      for p in sorted((ROOT / 'src').rglob('*')) if p.is_file()}
        if args.settings_component:
            report['settingsInputs'] = {p: record(ROOT / p) for p in
                ('tests/settings_drawer_test.cpp', 'tools/prototype/capture-ais-component.py',
                 'tools/windows-ui.py')}
        lock = json.loads((ROOT / 'upstream.lock.json').read_text())
        upstream = args.upstream.resolve()
        default = ROOT / 'upstream/OpenCPN'
        if upstream != default.resolve():
            if default.exists():
                raise ValueError('Default upstream exists; refusing to replace it')
            default.parent.mkdir(parents=True, exist_ok=True)
            run(['git', 'clone', '--no-hardlinks', upstream, default], evidence / 'clone.log')
            run(['git', '-C', default, 'checkout', '--detach', lock['commit']], evidence / 'checkout.log')
        run([sys.executable, ROOT / 'tools/prepare-integration.py'], evidence / 'prepare.log')
        source = ROOT / 'build/integration-source'
        report['upstream'] = lock['commit']
        report['patches'] = {p.name: record(p) for p in sorted((ROOT / 'patches').glob('*.patch'))}
        report['sources'] = {p: record(source / p) for p in UNITS}
        report['configTemplate'] = record(source / 'cmake/in-files/config.h.in')
        for relative in UNITS:
            dest = evidence / 'source' / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source / relative, dest)
        sdk = ROOT / 'build/windows-changed-unit-sdk'
        sdk.mkdir(parents=True, exist_ok=True)
        wx = sdk / 'wx'
        wxlock = json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())
        for item in wxlock['archives']:
            archive = sdk / item['file']
            fetch(item, archive)
            run(['7z', 'x', '-y', '-o' + str(wx), archive], evidence / (item['file'] + '.log'))
        curl = json.loads((ROOT / 'tools/windows-curl.lock.json').read_text())
        archive = sdk / curl['archive']
        fetch(curl, archive)
        headers = sdk / 'curl-headers'
        with tarfile.open(archive) as tar:
            for member in tar:
                if not re.fullmatch(r'curl-8\.22\.0/include/curl/[a-z0-9_-]+\.h', member.name):
                    continue
                if not member.isfile() or member.size > 1024 * 1024:
                    raise ValueError('Unexpected curl header member')
                dest = headers / 'curl' / Path(member.name).name
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(tar.extractfile(member).read())
        report['sdkLocks'] = {p: record(ROOT / 'tools' / p) for p in ('windows-wx.lock.json', 'windows-curl.lock.json')}
        build = evidence / 'build'
        run(['cmake', '-S', ROOT / 'tests/windows_changed_units', '-B', build,
             '-G', 'Visual Studio 17 2022', '-A', 'Win32',
             '-DOPENNAV_SOURCE_DIR:PATH=' + source.as_posix(), '-DCURL_INCLUDE:PATH=' + headers.as_posix(),
             '-DOPENNAV_CHECK_UI=' + ('ON' if args.ui else 'OFF'),
             '-DOPENNAV_CHECK_SETTINGS_COMPONENT=' + ('ON' if args.settings_component else 'OFF'),
             '-DwxWidgets_ROOT_DIR:PATH=' + wx.as_posix(), '-DwxWidgets_LIB_DIR:PATH=' + (wx / 'lib/vc14x_dll').as_posix(),
             '-DwxWidgets_CONFIGURATION=mswu'], evidence / 'configure.log')
        report['generatedConfig'] = record(build / 'include/config.h')
        for project in build.glob('check_*.vcxproj'):
            if re.search(r'NOMINMAX|OPENNAV_\w+_TEST|ForcedIncludeFiles|PrecompiledHeader>Use', project.read_text()):
                raise ValueError('Generated project masks production headers/macros')
        if args.legacy_control:
            report['legacyControls'] = {}
            for relative, count in zip(UNITS[:2], (4, 3)):
                path = source / relative
                original = path.read_bytes()
                legacy, changed = re.subn(rb'\(std::numeric_limits<([^>]+)>::max\)\(\)',
                                          rb'std::numeric_limits<\1>::max()', original)
                if changed != count:
                    raise ValueError(f'Unexpected fixed max-call count: {relative}: {changed}')
                try:
                    path.write_bytes(legacy)
                    code = run(['cmake', '--build', build, '--config', 'Release', '--target', 'check_' + path.stem,
                                '--parallel', '2'], evidence / ('legacy-' + path.stem + '.log'), expected_failure=path.name)
                    report['legacyControls'][relative] = {'exitCode': code, 'source': record(path)}
                finally:
                    path.write_bytes(original)
        targets = ['check_' + Path(p).stem for p in UNITS]
        if args.ui:
            targets += ['opennav_ui', 'check_SettingsStore']
        if args.settings_component:
            targets += ['settings_drawer_test']
        run(['cmake', '--build', build, '--config', 'Release', '--target', *targets,
             '--parallel', '2', '--', '/verbosity:normal'],
            evidence / 'compile.log', timeout=420)
        if any(record(source / p) != report['sources'][p] for p in UNITS):
            raise ValueError('Production source changed during compile')
        objects = sorted(build.glob('check_*.dir/Release/*.obj'))
        if len(objects) != len(UNITS) + int(args.ui) or any(p.stat().st_size == 0 for p in objects):
            raise ValueError('Missing native translation-unit objects')
        report['objects'] = {str(p.relative_to(evidence)): record(p) for p in objects}
        if args.ui:
            ui_objects = sorted((build / 'opennav/opennav_ui.dir/Release').glob('*.obj'))
            names = {p.stem for p in ui_objects if p.stat().st_size > 0}
            if not {Path(p).stem for p in UI_CHANGED}.issubset(names):
                raise ValueError('Actual UI target did not compile every changed UI translation unit')
            report['uiObjects'] = {str(p.relative_to(evidence)): record(p) for p in ui_objects}
            if any(record(ROOT / p) != rec for p, rec in report['uiSourceTree'].items()):
                raise ValueError('UI source tree changed during compilation')
        if args.settings_component:
            report['settingsComponent'] = settings_component(build, wx, evidence)
            if (report['settingsComponent']['capture']['source_commit'] != report['candidate'] or
                    any(record(ROOT / p) != rec for p, rec in report['settingsInputs'].items())):
                raise ValueError('Settings component inputs changed during native proof')
        report['status'] = 'passed'
    finally:
        (evidence / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Compile complete patched Windows units without rebuilding dependencies.

Compile-only by default. Optional Settings and prototype components provide
isolated native interaction evidence, never product, package or dependency qualification.
The opt-in prototype proof compiles the real navigation bridge and runs existing
Settings/Search components, with optional Energy, without rebuilding old macro-control units.
The optional negative control restores exactly the seven legacy max calls in
the two real source files; it must fail before the untouched fixed files pass.
"""
import argparse
import hashlib
import json
import importlib.util
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
NAVIGATION_UNITS = ('src/integration/NavigationActions.cpp', 'src/integration/NavigationObjects.cpp')
SEARCH_CAPTURES = {
    'saved-objects-day': (1280, 800), 'empty-night': (1280, 800),
    'shell-1280': (1280, 800), 'shell-853': (853, 600),
    'shell-853-search': (853, 600),
}
ENERGY_CAPTURES = {
    'energy-day', 'energy-dusk', 'energy-night', 'energy-stale-day',
    'energy-gps-unavailable-day', 'energy-shortfall-day', 'energy-inactive-day',
}


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
        # Keep the actionable compiler/configuration error in the job log;
        # the complete log remains in the uploaded exact-run evidence.
        tail = '\n'.join(log.read_text(encoding='utf-8', errors='replace').splitlines()[-60:])
        encoding = sys.stdout.encoding or 'utf-8'
        print(tail.encode(encoding, errors='backslashreplace').decode(encoding), flush=True)
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


def offline_component(build, wx, evidence, component):
    """Use the existing offline component and capture gate with app-local DLLs."""
    clients = {'settings': 'settings_drawer_test.exe', 'energy': 'energy_panel_test.exe'}
    client = build / 'Release' / clients[component]
    identity = record(client)
    manifest = stage_native_runtime(client, wx,
        ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll', 'wxmsw32u_aui_vc14x.dll'))
    (evidence / (component + '-runtime.json')).write_text(json.dumps(manifest, indent=2) + '\n')
    output = evidence / (component + '-component')
    run([sys.executable, ROOT / 'tools/prototype/capture-ais-component.py',
         '--component', component, '--client', client, '--output', output],
        evidence / (component + '-component.log'), timeout=90)
    capture = json.loads((output / 'capture.json').read_text())
    if capture['platform'] != 'win32' or capture['executable_sha256'] != identity['sha256']:
        raise ValueError(component + ' component capture does not identify the native tested executable')
    if component == 'energy' and set(capture['captures']) != ENERGY_CAPTURES:
        raise ValueError('Missing/unexpected canonical Energy captures')
    if record(client) != identity:
        raise ValueError('Offline component executable changed during capture')
    for name, item in manifest.items():
        if record(client.parent / name)['sha256'] != item['sha256']:
            raise ValueError('Offline component runtime changed during capture: ' + name)
    return {'executable': identity, 'runtime': manifest, 'capture': capture}


def settings_component(build, wx, evidence):
    return offline_component(build, wx, evidence, 'settings')


def prototype_component(build, wx, evidence, component='search'):
    if component not in ('search', 'chart-presentation'):
        raise ValueError('Unknown prototype component')
    if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
        raise ValueError('Prototype capture requires disposable native Windows CI, never the boat')
    chart = component == 'chart-presentation'
    client = build / ('Release/chart_presentation_drawer_test.exe' if chart else 'Release/search_drawer_test.exe')
    expected_captures = ({name: (1280, 800) for name in (
        'chart-day-top', 'chart-day-bottom', 'chart-dusk-raster', 'chart-night-unavailable')}
        if chart else SEARCH_CAPTURES)
    identity = record(client)
    manifest = stage_native_runtime(client, wx,
        ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll', 'wxmsw32u_aui_vc14x.dll'))
    (evidence / (component + '-runtime.json')).write_text(json.dumps(manifest, indent=2) + '\n')
    output = evidence / (component + '-component')
    if output.exists():
        raise ValueError('Prototype capture requires a new evidence directory')
    spec = importlib.util.spec_from_file_location('windows_ui', ROOT / 'tools/windows-ui.py')
    ui = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ui)
    ui.ensure_desktop(1440, 900)
    run([client, output], evidence / (component + '-component.log'), timeout=45)
    if chart:
        result = json.loads((output / 'result.json').read_text())
        if (not result['passed'] or not result['fixture_only'] or result['checks'] < 46 or
                len(result['captures']) != 4 or set(result['captures']) != set(expected_captures)):
            raise ValueError('Chart component did not finish its actual focused interaction checks')
        checks = result['checks']
    else:
        text = (evidence / 'search-component.log').read_text(errors='replace')
        matches = re.findall(r'^(\d+) focused search checks passed; native Windows and boat remain pending$', text, re.M)
        if len(matches) != 1 or int(matches[0]) < 85:
            raise ValueError('Search component did not finish its actual focused interaction checks')
        checks = int(matches[0])
    from PIL import Image
    if {p.stem for p in output.glob('*.png')} != set(expected_captures):
        raise ValueError('Missing/unexpected prototype captures')
    screenshots = {}
    for name, dimensions in expected_captures.items():
        path = output / (name + '.png')
        with Image.open(path) as image:
            if image.size != dimensions or len(image.convert('RGB').getcolors(2000000) or ()) < 25:
                raise ValueError('Missing/blank prototype surface or wrong viewport: ' + name)
            if name in ('saved-objects-day', 'empty-night'):
                expected = (12, 17, 21) if name == 'empty-night' else (21, 35, 38)
                if image.convert('RGB').getpixel((690, 400)) != expected:
                    raise ValueError('Search drawer theme/placement differs: ' + name)
        screenshots[name] = dict(size=list(dimensions), **record(path))
    if len({v['sha256'] for v in screenshots.values()}) != len(screenshots):
        raise ValueError('Prototype captures reused stale pixels')
    if record(client) != identity:
        raise ValueError('Prototype executable changed during capture')
    for name, item in manifest.items():
        if record(client.parent / name)['sha256'] != item['sha256']:
            raise ValueError('Prototype runtime changed during capture: ' + name)
    capture = {'passed': True, 'checks': checks, 'platform': sys.platform,
               'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
               'executable_sha256': identity['sha256'], 'screenshots': screenshots,
               'conformance': 'PENDING native visual review and boat acceptance; offline component only'}
    (output / 'capture.json').write_text(json.dumps(capture, indent=2) + '\n')
    return {'executable': identity, 'runtime': manifest, 'capture': capture}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', type=Path, default=ROOT / 'upstream/OpenCPN')
    parser.add_argument('--evidence', type=Path, default=ROOT / 'evidence/local/windows-changed-units')
    parser.add_argument('--legacy-control', action='store_true')
    parser.add_argument('--ui', action='store_true', help='also compile the real production UI static library and SettingsStore')
    parser.add_argument('--settings-component', action='store_true',
                        help='with --ui, link and run the existing offline native Settings component')
    parser.add_argument('--prototype-proof', action='store_true',
                        help='opt in to complete navigation bridge objects plus existing Settings and Search components; skips old macro-control units')
    parser.add_argument('--chart-presentation-component', action='store_true',
                        help='with --prototype-proof, also run the tracked offline chart presentation harness')
    parser.add_argument('--energy-component', action='store_true',
                        help='with --prototype-proof, also run the existing offline Energy component and seven captures')
    args = parser.parse_args()
    if args.chart_presentation_component and not args.prototype_proof:
        parser.error('--chart-presentation-component requires --prototype-proof')
    if args.energy_component and not args.prototype_proof:
        parser.error('--energy-component requires --prototype-proof')
    if args.prototype_proof:
        if args.legacy_control:
            parser.error('--prototype-proof does not repeat --legacy-control')
        args.ui = args.settings_component = True
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
    if args.prototype_proof:
        components = 'Settings/Search' + ('/Energy' if args.energy_component else '')
        report['scope'] = ('native Win32 navigation bridge translation units and offline ' + components +
                           ' components; not full OpenCPN, dependency, package or boat qualification')
    active_units = () if args.prototype_proof else UNITS
    try:
        report['candidate'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
        report['gateInputs'] = {p: record(ROOT / p) for p in
                               ('tools/test-windows-changed-units.py', 'tools/prepare-integration.py',
                                'tests/windows_changed_units/CMakeLists.txt', 'upstream.lock.json')}
        if args.prototype_proof:
            report['prototypeInputs'] = {p: record(ROOT / p) for p in
                (*NAVIGATION_UNITS, 'tests/search_drawer_test.cpp',
                 'tools/windows-prototype-headers.lock.json', 'tools/windows-ui.py')}
        if args.chart_presentation_component:
            report['prototypeInputs']['tests/chart_presentation_drawer_test.cpp'] = record(ROOT / 'tests/chart_presentation_drawer_test.cpp')
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
        if args.energy_component:
            report['energyInputs'] = {p: record(ROOT / p) for p in
                ('tests/energy_panel_test.cpp', 'tools/prototype/capture-ais-component.py',
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
        report['sources'] = {p: record(source / p) for p in active_units}
        report['configTemplate'] = record(source / 'cmake/in-files/config.h.in')
        for relative in active_units:
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
        glew = sdk / 'prototype-headers'
        if args.prototype_proof:
            header_lock = json.loads((ROOT / 'tools/windows-prototype-headers.lock.json').read_text())
            for item in header_lock['files']:
                fetch(item, glew / item['file'])
            report['prototypeHeaders'] = {item['file']: record(glew / item['file']) for item in header_lock['files']}
            # Bind the actual pinned header/build-contract input, not just our
            # object filenames. Source tree remains owned by prepare-integration.
            report['navigationHeaderTree'] = {str(p.relative_to(source)): record(p)
                for p in sorted(source.rglob('*')) if p.is_file() and p.suffix in ('.h', '.hpp', '.in')}
            report['navigationBuildContracts'] = {str(p.relative_to(source)): record(p)
                for p in sorted(source.rglob('CMakeLists.txt'))}

        build = evidence / 'build'
        run(['cmake', '-S', ROOT / 'tests/windows_changed_units', '-B', build,
             '-G', 'Visual Studio 17 2022', '-A', 'Win32',
             '-DOPENNAV_SOURCE_DIR:PATH=' + source.as_posix(), '-DCURL_INCLUDE:PATH=' + headers.as_posix(),
             '-DOPENNAV_CHECK_UI=' + ('ON' if args.ui else 'OFF'),
             '-DOPENNAV_CHECK_SETTINGS_COMPONENT=' + ('ON' if args.settings_component else 'OFF'),
             '-DOPENNAV_CHECK_PROTOTYPE=' + ('ON' if args.prototype_proof else 'OFF'),
             '-DOPENNAV_CHECK_CHART_COMPONENT=' + ('ON' if args.chart_presentation_component else 'OFF'),
             '-DOPENNAV_CHECK_ENERGY_COMPONENT=' + ('ON' if args.energy_component else 'OFF'),
             '-DOPENNAV_GLEW_INCLUDE:PATH=' + glew.as_posix(),
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
        targets = ['check_' + Path(p).stem for p in active_units]
        if args.prototype_proof:
            targets += ['check_' + Path(p).stem for p in NAVIGATION_UNITS] + ['search_drawer_test']
            report['navigationGeneratedConfig'] = record(build / 'navigation-include/config.h')
        if args.chart_presentation_component:
            targets += ['chart_presentation_drawer_test']
        if args.energy_component:
            targets += ['energy_panel_test']
        if args.ui:
            targets += ['opennav_ui', 'check_SettingsStore']
        if args.settings_component:
            targets += ['settings_drawer_test']
        run(['cmake', '--build', build, '--config', 'Release', '--target', *targets,
             '--parallel', '2', '--', '/verbosity:normal'],
            evidence / 'compile.log', timeout=420)
        if any(record(source / p) != report['sources'][p] for p in active_units):
            raise ValueError('Production source changed during compile')
        objects = sorted(build.glob('check_*.dir/Release/*.obj'))
        if len(objects) != len(active_units) + int(args.ui) + len(NAVIGATION_UNITS) * int(args.prototype_proof) or any(p.stat().st_size == 0 for p in objects):
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
        if args.prototype_proof:
            report['searchComponent'] = prototype_component(build, wx, evidence)
            if args.chart_presentation_component:
                report['chartPresentationComponent'] = prototype_component(build, wx, evidence, 'chart-presentation')
            if (report['searchComponent']['capture']['source_commit'] != report['candidate'] or
                    any(record(ROOT / p) != rec for p, rec in report['prototypeInputs'].items())):
                raise ValueError('Prototype component inputs changed during native proof')
        if args.energy_component:
            report['energyComponent'] = offline_component(build, wx, evidence, 'energy')
            if (report['energyComponent']['capture']['source_commit'] != report['candidate'] or
                    any(record(ROOT / p) != rec for p, rec in report['energyInputs'].items())):
                raise ValueError('Energy component inputs changed during native proof')
        report['status'] = 'passed'
    finally:
        (evidence / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Native private production objects and path regression, without a DLL link."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

from ocharts_cmake_path_probe import path_regression

ROOT = Path(__file__).resolve().parents[1]
# Independently counted from the locked per-library SRC lists and production
# adapter recipe. A source-list change requires explicit review of this gate.
TARGET_COUNTS = {'CPL': 7, 'DSA': 3, 'WXJSON': 3, 'ISO8211': 6, 'TINYXML': 3,
                 'GEOPRIM': 6, 'OCPN_PUGIXML': 1, 'S52PLIB': 9,
                 'skager_wxcurl': 12, 'skager-ocharts-adapter': 20}
TARGETS = set(TARGET_COUNTS)
NS = {'m': 'http://schemas.microsoft.com/developer/msbuild/2003'}


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result



def project_inventory(build, prepared):
    """Cross-check actual CMake codemodel sources against generated VC projects."""
    reply = build / '.cmake/api/v1/reply'
    index = json.loads(next(reply.glob('index-*.json')).read_text())
    model = json.loads((reply / index['reply']['codemodel-v2']['jsonFile']).read_text())
    configuration = next(c for c in model['configurations'] if c['name'] == 'Release')
    targets = {}
    for item in configuration['targets']:
        if item['name'] not in TARGETS:
            continue
        target = json.loads((reply / item['jsonFile']).read_text())
        sources = []
        for entry in target['sources']:
            if 'compileGroupIndex' not in entry:
                continue
            path = Path(entry['path'])
            if not path.is_absolute():
                path = Path(model['paths']['source']) / path
            path = path.resolve()
            path.relative_to(prepared.resolve())
            if not path.is_file() or path.suffix.lower() not in ('.c', '.cpp', '.cxx', '.cc'):
                raise ValueError('Unexpected production translation unit: ' + str(path))
            sources.append(path)
        if len(sources) != TARGET_COUNTS[item['name']] or len(sources) != len(set(sources)):
            raise ValueError('Missing, extra or duplicate production source inventory')
        project = build / target['paths']['build'] / (item['name'] + '.vcxproj')
        tree = ET.parse(project)
        actual = {Path(n.attrib['Include']).resolve() for n in tree.findall('.//m:ClCompile[@Include]', NS)}
        if actual != set(sources):
            raise ValueError('VC project differs from actual CMake source inventory')
        content = project.read_text()
        if re.search(r'NOMINMAX|ForcedIncludeFiles|PrecompiledHeader>Use', content):
            raise ValueError('Probe must not mask actual native headers/macros')
        targets[item['name']] = {'project': project, 'sources': sources}
    if set(targets) != TARGETS:
        raise ValueError('Missing actual private production target(s)')
    # Independently pin the requested highest-risk units, not just any projects.
    required = {'source/src/eSENCChart.cpp', 'source/src/o-charts_pi.cpp',
                'source/libs/s52plib/src/s52plib.cpp', 'source/libs/s52plib/src/s52cnsy.cpp',
                'source/libs/s52plib/src/chartsymbols.cpp', 'source/libs/wxcurl/src/base.cpp',
                'source/libs/wxcurl/src/http.cpp',
                'local/src/plugin-adapters/ocharts/ChartPresentationAdapter.cpp',
                'local/src/integration/ChartNameAlphaWindows.cpp'}
    actual = {p.relative_to(prepared).as_posix() for t in targets.values() for p in t['sources']}
    if not required <= actual:
        raise ValueError('Required actual private units omitted')
    return targets


def object_inventory(build, targets, api):
    result = {}
    for name, target in targets.items():
        directory = target['project'].parent / (name + '.dir/Release')
        objects = list(directory.rglob('*.obj'))
        expected = {p.stem.casefold() for p in target['sources']}
        if len(expected) != len(target['sources']):
            raise ValueError('Source stem collision needs explicit object mapping')
        if len(objects) != len(expected) or {p.stem.casefold() for p in objects} != expected:
            raise ValueError('Missing/extra production objects for ' + name)
        for path in objects:
            data = path.read_bytes()
            machine = struct.unpack_from('<H', data, 0)[0] if len(data) >= 20 else 0
            if data[:4] == b'\0\0\xff\xff' and len(data) >= 56:
                machine = struct.unpack_from('<H', data, 6)[0]
            if machine != 0x14c:
                raise ValueError('Object is not actual x86 COFF: ' + str(path))
            result[path.relative_to(build).as_posix()] = api.record(path)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--cmake', default='cmake')
    args = parser.parse_args()
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    api = module('native_api', ROOT / 'tools/test-windows-changed-units.py')
    report = {'scope': 'actual private production objects only; no maintained producer builds, DLL link, package or runtime acceptance',
              'nativeProductAcceptance': False, 'passed': False,
              'remoteCommit': os.environ.get('GITHUB_SHA'),
              'runId': os.environ.get('GITHUB_RUN_ID')}
    names = {'tools/test-ocharts-compile-windows.py', 'tools/ocharts_compile_probe_inputs.py',
             'tools/ocharts_cmake_path_probe.py', 'tools/test-windows-changed-units.py',
             'tools/generate-xnav-chart-style.py', 'tests/windows_ocharts_compile/CMakeLists.txt',
             '.github/workflows/skager-ocharts-compile.yml', 'docs/design/prototype-tokens.json'}
    names.update(p.relative_to(ROOT).as_posix() for p in (ROOT / 'tools').glob('chart_*.py'))
    names.update(p.relative_to(ROOT).as_posix() for p in (ROOT / 'cmake/ocharts-adapter').iterdir() if p.is_file())
    for directory in ('resources/chart-style', 'docs/design/prototype'):
        names.update(p.relative_to(ROOT).as_posix() for p in (ROOT / directory).rglob('*') if p.is_file())
    input_paths = {name: ROOT / name for name in names}
    workflow = '.github/workflows/skager-ocharts-compile.yml'
    if not input_paths[workflow].is_file():
        input_paths[workflow] = ROOT.parent / workflow
    report['probeInputs'] = {name: api.record(path) for name, path in sorted(input_paths.items())}
    def save():
        (evidence / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    try:
        report['paths'] = path_regression(evidence, api, args.cmake)
        save()
        if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
            raise ValueError('Actual object probe requires disposable native Windows CI, never the boat')
        from ocharts_compile_probe_inputs import stage
        prepared = (ROOT / 'build/ocharts object probe/inputs').resolve()
        report['stage'] = stage(ROOT, prepared, ROOT / 'build/ocharts-probe-cache', evidence, api)
        # No production receipt may be manufactured for a headers-only SDK.
        if (prepared / 'preparation.json').exists() or any((prepared / 'sdk').glob('*-build.json')):
            raise ValueError('Compile probe must not impersonate qualified preparation')
        build = evidence / 'build'
        query = build / '.cmake/api/v1/query'
        query.mkdir(parents=True)
        (query / 'codemodel-v2').touch()
        api.run([args.cmake, '-S', ROOT / 'tests/windows_ocharts_compile', '-B', build,
                 '-G', 'Visual Studio 17 2022', '-A', 'Win32',
                 '-DCMAKE_POLICY_VERSION_MINIMUM=3.5',
                 '-DSKAGER_PREPARED=' + str(prepared)], evidence / 'configure.log', timeout=180)
        targets = project_inventory(build, prepared)
        for target in targets.values():
            for source in target['sources']:
                copy = evidence / 'source' / source.relative_to(prepared)
                copy.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, copy)
        report['targets'] = {name: {'project': api.record(t['project']),
            'sources': {p.relative_to(prepared).as_posix(): api.record(p) for p in t['sources']}}
            for name, t in targets.items()}
        save()
        vswhere = Path(os.environ['ProgramFiles(x86)']) / 'Microsoft Visual Studio/Installer/vswhere.exe'
        msbuild = subprocess.check_output([str(vswhere), '-latest', '-products', '*',
            '-requires', 'Microsoft.Component.MSBuild', '-find', r'MSBuild\**\Bin\MSBuild.exe'], text=True).strip().splitlines()[0]
        report['msbuild'] = dict(path=msbuild, **api.record(Path(msbuild)))
        for name, target in targets.items():
            api.run([msbuild, target['project'], '/t:ClCompile', '/p:Configuration=Release',
                     '/p:Platform=Win32', '/p:BuildProjectReferences=false', '/m:2', '/verbosity:normal'],
                    evidence / (name + '-compile.log'), timeout=600)
        report['objects'] = object_inventory(build, targets, api)
        # Capture objects without uploading downloaded SDK/import libraries.
        if list(build.rglob('*.dll')) or list(build.rglob('*.lib')):
            raise ValueError('Object-only probe unexpectedly linked a library')
        for target in report['targets'].values():
            if any(api.record(prepared / p) != record for p, record in target['sources'].items()):
                raise ValueError('Production translation unit changed during compilation')
        for directory, records in report['stage']['files'].items():
            if any(api.record(prepared / directory / p) != record for p, record in records.items()):
                raise ValueError('Actual staged input changed during compilation: ' + directory)
        for name, record in {**report['probeInputs'], **report['stage']['production_inputs']}.items():
            if api.record(input_paths.get(name, ROOT / name)) != record:
                raise ValueError('Product/probe input changed during compilation: ' + name)
        report['passed'] = True
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        save()


if __name__ == '__main__':
    main()

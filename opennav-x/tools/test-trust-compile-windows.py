#!/usr/bin/env python3
"""Compile real core/private trust probes only. Never link, TLS or package evidence."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import xml.etree.ElementTree as ET

from importlib.util import module_from_spec, spec_from_file_location

ROOT = Path(__file__).resolve().parents[1]
NS = {'m': 'http://schemas.microsoft.com/developer/msbuild/2003'}


def module(name, path):
    spec = spec_from_file_location(name, path)
    result = module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def inventory(directory, api):
    return {p.relative_to(directory).as_posix(): api.record(p)
            for p in sorted(directory.rglob('*')) if p.is_file() and '.git' not in p.parts}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
        raise ValueError('Requires disposable native Windows CI, never the boat')
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    api = module('trust_native_api', ROOT / 'tools/test-windows-changed-units.py')
    objects_api = module('trust_objects', ROOT / 'tools/test-ocharts-compile-windows.py')
    prep = module('trust_private_source', ROOT / 'tools/prepare-ocharts-adapter.py')
    report = {'scope': 'Only real core/private trust probe configure and x86 compilation',
              'runtimeAcceptance': False, 'packageAcceptance': False,
              'dependencyProducerAcceptance': False, 'tlsAcceptance': False,
              'passed': False, 'remoteCommit': os.environ.get('GITHUB_SHA'),
              'runId': os.environ.get('GITHUB_RUN_ID')}
    def save():
        (evidence / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    try:
        names = set(prep.INPUTS) | {
            'tools/test-trust-compile-windows.py', 'tools/test-downloader-trust-paths.py',
            'tools/test-windows-changed-units.py', 'tools/test-ocharts-compile-windows.py',
            'tools/ocharts_cmake_path_probe.py', 'tools/prepare-integration.py',
            'tools/verify-upstream.py', 'upstream.lock.json', '.gitattributes',
            'tests/windows_trust_compile/CMakeLists.txt',
            '.github/workflows/skager-trust-compile.yml'}
        names.update(p.relative_to(ROOT).as_posix() for p in (ROOT / 'patches').glob('opencpn-*.patch'))
        local = {name: ROOT / name for name in names}
        workflow = '.github/workflows/skager-trust-compile.yml'
        if not local[workflow].is_file():
            local[workflow] = ROOT.parent / workflow
        report['localInputs'] = {name: api.record(path) for name, path in sorted(local.items())}
        save()
        api.run([sys.executable, ROOT / 'tools/test-downloader-trust-paths.py'], evidence / 'paths.log')
        api.run([sys.executable, ROOT / 'tools/prepare-integration.py'], evidence / 'prepare-core.log')
        core = ROOT / 'build/integration-source'
        inputs = ROOT / 'build/trust compile inputs'
        inputs.mkdir(parents=True, exist_ok=False)
        private = inputs / 'private-source'
        prep.fetch_sources(private, inputs / 'source-cache')
        report['privateOriginal'] = inventory(private, api)
        prep.apply_patches(private, ROOT)
        report['privatePatched'] = inventory(private, api)
        sdk = inputs / 'sdk'
        for item in json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())['archives']:
            if 'ReleaseDLL' in item['file']:
                continue  # No runtime is staged or exercised.
            archive = inputs / item['file']
            api.fetch(item, archive)
            api.run(['7z', 'x', '-y', '-o' + str(sdk / 'wx'), archive],
                    evidence / (item['file'] + '.log'))
        curl = json.loads((ROOT / 'tools/windows-curl.lock.json').read_text())
        archive = inputs / curl['archive']
        api.fetch(curl, archive)
        with tarfile.open(archive) as stream:
            for member in stream:
                if not re.fullmatch(r'curl-8\.22\.0/include/curl/[a-z0-9_-]+\.h', member.name):
                    continue
                if not member.isfile() or member.size > 1024 * 1024:
                    raise ValueError('Unexpected curl header archive member')
                path = sdk / 'include/curl' / Path(member.name).name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(stream.extractfile(member).read())
        report['sdkInputs'] = inventory(sdk, api)
        report['coreInputs'] = {p.relative_to(core).as_posix(): api.record(p)
            for directory in ('model/include', 'libs/wxcurl')
            for p in (core / directory).rglob('*') if p.is_file()}
        report['coreInputs']['model/src/downloader.cpp'] = api.record(core / 'model/src/downloader.cpp')
        save()
        # Raw, untyped native paths reproduce the caller that failed in run 37126951293.
        common = ['-G', 'Visual Studio 17 2022', '-A', 'Win32',
            '-DOPENNAV_SOURCE_DIR=' + str(core), '-DOPENNAV_TOOLS_DIR=' + str(ROOT / 'tools'),
            '-DCURL_ROOT=' + str(sdk), '-DwxWidgets_ROOT_DIR=' + str(sdk / 'wx'),
            '-DwxWidgets_LIB_DIR=' + str(sdk / 'wx/lib/vc14x_dll'),
            '-DwxWidgets_CONFIGURATION=mswu']
        wrapper = ROOT / 'tests/windows_trust_compile/CMakeLists.txt'
        negative = evidence / 'original-native'
        negative.mkdir()
        text = wrapper.read_text()
        old = 'get_filename_component(TRUST_RECIPE "${CMAKE_CURRENT_LIST_DIR}/../downloader_trust" ABSOLUTE)'
        if text.count(old) != 1 or text.count('include("${TRUST_RECIPE}/InputPaths.cmake")') != 1:
            raise ValueError('Actual wrapper normalization boundary changed')
        text = text.replace(old, 'set(TRUST_RECIPE "' + (ROOT / 'tests/downloader_trust').as_posix() + '")')
        text = text.replace('include("${TRUST_RECIPE}/InputPaths.cmake")', '')
        (negative / 'CMakeLists.txt').write_text(text)
        command = ['cmake', '-S', str(negative), '-B', str(negative / 'build')] + common
        (negative / 'command.json').write_text(json.dumps(command, indent=2))
        with (negative / 'configure.log').open('wb') as log:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=180)
        log = (negative / 'configure.log').read_text(errors='replace')
        if result.returncode == 0 or 'Invalid character escape' not in log or 'downloader.cpp' not in log:
            raise ValueError('Original native trust path failure was not reproduced')
        report['negativeControl'] = {'returncode': result.returncode, 'log': api.record(negative / 'configure.log')}
        vswhere = Path(os.environ['ProgramFiles(x86)']) / 'Microsoft Visual Studio/Installer/vswhere.exe'
        msbuild = subprocess.check_output([str(vswhere), '-latest', '-products', '*',
            '-requires', 'Microsoft.Component.MSBuild', '-find', r'MSBuild\**\Bin\MSBuild.exe'],
            text=True).strip().splitlines()[0]
        report['msbuild'] = dict(path=msbuild, **api.record(Path(msbuild)))
        report['variants'] = {}
        for variant in ('core', 'private'):
            build = evidence / variant
            wxsource = core / 'libs/wxcurl/src' if variant == 'core' else private / 'libs/wxcurl/src'
            expected = {'downloader-trust-probe': [core / 'model/src/downloader.cpp', ROOT / 'tools/downloader-trust-probe.cpp'],
                        'wxcurl-trust-probe': [wxsource / 'base.cpp', wxsource / 'http.cpp', ROOT / 'tools/wxcurl-trust-probe.cpp']}
            command = ['cmake', '-S', wrapper.parent, '-B', build] + common
            if variant == 'private':
                command += ['-DTRUST_PRIVATE_SOURCE=' + str(private)]
            api.run(command, evidence / (variant + '-configure.log'), timeout=180)
            targets = {}
            for name, sources in expected.items():
                project = build / (name + '.vcxproj')
                actual = {Path(n.attrib['Include']).resolve() for n in
                          ET.parse(project).findall('.//m:ClCompile[@Include]', NS)}
                if actual != {p.resolve() for p in sources}:
                    raise ValueError('Actual trust project source inventory differs')
                content = project.read_text()
                if re.search(r'NOMINMAX|OPENNAV_\w+_TEST|ForcedIncludeFiles|PrecompiledHeader>Use', content):
                    raise ValueError('Trust compile project masks production headers/macros')
                if '<RuntimeLibrary>MultiThreadedDLL</RuntimeLibrary>' not in content:
                    raise ValueError('Trust target is not the production MD runtime')
                targets[name] = {'project': project, 'sources': sources}
                for source in sources:
                    copy = evidence / 'sources' / variant / source.relative_to(ROOT)
                    copy.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(source, copy)
                api.run([msbuild, project, '/t:ClCompile', '/p:Configuration=Release',
                         '/p:Platform=Win32', '/p:BuildProjectReferences=false', '/m:2', '/verbosity:normal'],
                        evidence / (variant + '-' + name + '-compile.log'), timeout=300)
            report['variants'][variant] = {
                'sources': {str(p.relative_to(ROOT)): api.record(p) for t in targets.values() for p in t['sources']},
                'projects': {name: api.record(t['project']) for name, t in targets.items()},
                'objects': objects_api.object_inventory(build, targets, api)}
            for t in targets.values():
                directory = t['project'].parent / (t['project'].stem + '.dir')
                if any(directory.rglob('*.exe')) or any(directory.rglob('*.dll')) or any(directory.rglob('*.lib')):
                    raise ValueError('Trust compile preflight unexpectedly linked')
            if list(build.glob('Release/*.exe')):
                raise ValueError('Trust executable was unexpectedly linked')
            save()
        for name, record in report['localInputs'].items():
            if api.record(local[name]) != record:
                raise ValueError('Local input changed during compilation: ' + name)
        for prefix, key in ((core, 'coreInputs'), (private, 'privatePatched'), (sdk, 'sdkInputs')):
            for name, record in report[key].items():
                if api.record(prefix / name) != record:
                    raise ValueError('Actual compile input changed: ' + name)
        report['passed'] = True
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        save()


if __name__ == '__main__':
    main()

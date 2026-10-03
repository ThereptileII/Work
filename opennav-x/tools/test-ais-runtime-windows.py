#!/usr/bin/env python3
"""Same-job MSVC AIS loopback gate; never dependency, product, WFP or boat qualification."""
import argparse
import importlib.util
import json
import os
from pathlib import Path, PureWindowsPath
import re
import shutil
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

import windows_dependency_receipt as receipt
import windows_dependency_reuse as reuse
import windows_dependency_stage as stage

ROOT = Path(__file__).resolve().parents[1]
TARGETS = ('ais_transport_test_client', 'ais_provider_test_client', 'ais_session_native')
NS = {'m': 'http://schemas.microsoft.com/developer/msbuild/2003'}
SYSTEM_DLLS = {'kernel32.dll', 'user32.dll', 'advapi32.dll', 'crypt32.dll', 'ws2_32.dll',
              'wsock32.dll', 'shlwapi.dll', 'bcrypt.dll', 'ncrypt.dll', 'ntdll.dll',
              'secur32.dll', 'shell32.dll', 'ole32.dll', 'oleaut32.dll', 'gdi32.dll',
              'normaliz.dll', 'iphlpapi.dll', 'ucrtbase.dll'}


def api_module():
    spec = importlib.util.spec_from_file_location('ais_native_api', ROOT / 'tools/test-windows-changed-units.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run(command, log, *, cwd=ROOT, env=None, timeout=180):
    command = [str(arg) for arg in command]
    log.with_suffix('.command.json').write_text(json.dumps(command, indent=2) + '\n')
    with log.open('wb') as stream:
        child = subprocess.Popen(command, cwd=cwd, env=env, stdout=stream, stderr=subprocess.STDOUT)
        try:
            code = child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            subprocess.run(['taskkill', '/PID', str(child.pid), '/T', '/F'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
            child.wait(timeout=30)
            raise
        if code:
            raise RuntimeError(f'Command failed ({code}); retained log: {log}')


def win32(path):
    data = path.read_bytes()
    if len(data) < 64 or data[:2] != b'MZ':
        raise ValueError('Missing PE header: ' + str(path))
    offset = struct.unpack_from('<I', data, 60)[0]
    if (offset > len(data) - 6 or data[offset:offset + 4] != b'PE\0\0' or
            struct.unpack_from('<H', data, offset + 4)[0] != 0x14c):
        raise ValueError('Not native Win32: ' + str(path))


def private_links(project, cache):
    """Check generated consumers: static IX propagates its imports to these."""
    nodes = ET.parse(project).findall('.//m:Link/m:AdditionalDependencies', NS)
    dependencies = {item.replace('\\', '/').casefold()
                    for node in nodes for item in (node.text or '').split(';')}
    for name in ('libssl.lib', 'libcrypto.lib', 'zlib1.lib'):
        wanted = (cache / name).as_posix().casefold()
        matches = {item for item in dependencies if item.rsplit('/', 1)[-1] == name}
        if matches != {wanted}:
            raise ValueError('Import library is not exclusively private verified input: ' + name)


def project_closure(projects):
    """Follow real MSBuild item edges, excluding per-configuration metadata."""
    closure, pending = {}, list(TARGETS)
    while pending:
        name = pending.pop()
        if name in closure or name == 'ZERO_CHECK': continue
        project = projects[name]
        closure[name] = project
        for node in ET.parse(project).findall('.//m:ItemGroup/m:ProjectReference', NS):
            pending.append(PureWindowsPath(node.attrib['Include']).stem)
    return closure


def child_environment(private, runtime):
    # No SDK, Git, MSYS, or user PATH entry participates in runtime resolution.
    windows = Path(os.environ['SystemRoot'])
    return dict(os.environ, PATH=os.pathsep.join(map(str, (
        private / 'openssl/bin', runtime, windows / 'System32', windows))),
        OPENSSL_CONF=str(private / 'openssl/ssl/openssl.cnf'),
        OPENSSL_MODULES=str(private / 'openssl/lib/ossl-modules'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
        raise ValueError('AIS runtime gate requires disposable native Windows CI')
    api = api_module()
    evidence = ROOT / 'evidence/local/ais-native-runtime'
    evidence.mkdir(parents=True, exist_ok=False)
    report = {'passed': False, 'scope': 'Same-job actual AIS MSVC Win32 loopback only',
              'runId': os.environ.get('GITHUB_RUN_ID'), 'runAttempt': os.environ.get('GITHUB_RUN_ATTEMPT'),
              'commit': os.environ.get('GITHUB_SHA'), 'productAcceptance': False,
              'wfpAcceptance': False, 'boatAcceptance': False, 'dependencyBuilds': False,
              'requestedChecks': {'providerTlsLifecycles': 6, 'transportCases': 18, 'sessionChecks': 178}}
    def save():
        (evidence / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    def inventory(directory):
        return {p.relative_to(directory).as_posix(): api.record(p)
                for p in sorted(directory.rglob('*')) if p.is_file() and '.git' not in p.parts}
    try:
        reuse.verify_same_job(ROOT)  # unchanged authority; never recreate/relabel a receipt
        document = receipt._read_json(ROOT / reuse.RECEIPT)
        expected = document['files']
        report['dependencyReceipt'] = api.record(ROOT / reuse.RECEIPT)
        shutil.copyfile(ROOT / reuse.RECEIPT, evidence / 'same-job-receipt.json')
        tracked = subprocess.check_output(['git', 'ls-files'], cwd=ROOT, text=True).splitlines()
        selected = {name for name in tracked if name.startswith(('src/', 'cmake/', 'patches/', 'tests/ais_transport/'))}
        selected.update({'CMakeLists.txt', 'upstream.lock.json', 'tests/ais_session_tests.cpp',
            'tests/windows_ais_runtime/CMakeLists.txt', 'tools/test-ais-runtime-windows.py',
            'tools/test-windows-changed-units.py', 'tools/prepare-integration.py', 'tools/verify-upstream.py',
            'tools/windows_dependency_receipt.py', 'tools/windows_dependency_reuse.py',
            'tools/windows_dependency_stage.py', 'tools/windows_dependency_evidence.py',
            'tools/windows-parent-environment.ps1',
            'tools/test-ais-runtime-gate.py'})
        local = {name: ROOT / name for name in selected}
        workflow = '.github/workflows/opennav-baseline.yml'
        local[workflow] = ROOT / workflow if (ROOT / workflow).is_file() else ROOT.parent / workflow
        report['localInputs'] = {name: api.record(path) for name, path in sorted(local.items())}
        save()
        # Existing prepare command verifies exact pin + complete ordered patches;
        # this job already has the prepared tree, so its source bytes are retained.
        upstream = ROOT / 'build/integration-source'
        if not (upstream / 'libs/IXWebSocket/CMakeLists.txt').is_file():
            raise ValueError('Same-job prepared source is missing')
        run([sys.executable, ROOT / 'tools/prepare-integration.py'], evidence / 'prepared-source.log')
        report['ixInputs'] = inventory(upstream / 'libs/IXWebSocket')
        private = ROOT / 'build/ais-runtime-inputs'
        private.mkdir(parents=True, exist_ok=False)
        copies = {}
        def copy(source, target):
            target_name = target.relative_to(ROOT).as_posix()
            stage._copy_record(ROOT, expected, source, target_name)
            copies[target_name] = expected[source]
        # Keep complete immutable private prefixes, including generated headers,
        # tool/config/module files. The maintained verifier validates originals.
        for kind in ('openssl', 'zlib'):
            prefix = stage.PREFIX[kind] + '/'
            for source in expected:
                if source.startswith(prefix):
                    copy(source, private / kind / source[len(prefix):])
        wrapper = evidence / 'source'
        wrapper.mkdir()
        shutil.copyfile(ROOT / 'tests/windows_ais_runtime/CMakeLists.txt', wrapper / 'CMakeLists.txt')
        cache = wrapper / 'cache/buildwin'
        for source in expected:
            prefix = stage.PREFIX['openssl'] + '/include/openssl/'
            if source.startswith(prefix):
                copy(source, cache / 'include/openssl' / source[len(prefix):])
        for kind, source, target in stage.FILES:
            if kind in ('openssl', 'zlib'):
                copy(stage.PREFIX[kind] + '/' + source, cache / target)
        report['stagedInputs'] = copies
        save()
        build = evidence / 'build'
        run(['cmake', '-S', wrapper, '-B', build, '-G', 'Visual Studio 17 2022', '-A', 'Win32',
             '-DCMAKE_POLICY_VERSION_MINIMUM=3.5', '-DOPENNAV_ROOT:PATH=' + ROOT.as_posix(),
             '-DOPENNAV_PINNED_SOURCE:PATH=' + upstream.as_posix()], evidence / 'configure.log')
        projects = {p.stem: p for p in build.rglob('*.vcxproj')}
        if not set(TARGETS) <= projects.keys() or 'ixwebsocket' not in projects:
            raise ValueError('Actual AIS projects missing')
        compiler_file = next((build / 'CMakeFiles').glob('*/CMakeCXXCompiler.cmake'))
        matches = re.findall(r'set\(CMAKE_CXX_COMPILER "([^"]+)"\)', compiler_file.read_text())
        if len(matches) != 1 or not re.search(r'/Host[^/]+/x86/cl\.exe$', matches[0], re.I):
            raise ValueError('CMake did not select native x86 MSVC')
        compiler = Path(matches[0]); dumpbin = compiler.with_name('dumpbin.exe')
        report['tools'] = {str(path): api.record(path) for path in
                           (compiler, dumpbin, Path(shutil.which('cmake')), Path(sys.executable))}
        ix = projects['ixwebsocket'].read_text()
        for definition in ('IXWEBSOCKET_USE_OPEN_SSL', 'IXWEBSOCKET_USE_TLS', 'IXWEBSOCKET_USE_ZLIB'):
            if definition not in ix: raise ValueError('Missing production IX backend: ' + definition)
        if re.search(r'IXWEBSOCKET_USE_MBED_TLS|IXWEBSOCKET_USE_SECURE_TRANSPORT|NOMINMAX', ix):
            raise ValueError('Alternate TLS backend or macro masking')
        for name in TARGETS[:2]:
            private_links(projects[name], cache)
        # Follow only the three selected targets' actual generated dependencies.
        closure = project_closure(projects)
        report['projects'] = {name: api.record(path) for name, path in closure.items()}
        sources = set()
        for name, project in closure.items():
            xml = ET.parse(project)
            for group in xml.findall('.//m:ItemDefinitionGroup', NS):
                if 'Release|Win32' not in group.attrib.get('Condition', ''): continue
                if group.findtext('m:ClCompile/m:RuntimeLibrary', namespaces=NS) != 'MultiThreadedDLL':
                    raise ValueError('Selected target does not use Release /MD: ' + name)
            for node in xml.findall('.//m:ClCompile[@Include]', NS):
                source = Path(node.attrib['Include']).resolve()
                if not source.is_file() or not (source.is_relative_to(upstream / 'libs/IXWebSocket') or
                        source.is_relative_to(ROOT / 'src/ais') or source.is_relative_to(ROOT / 'src/vessel') or
                        source.is_relative_to(ROOT / 'tests/ais_transport') or
                        source == ROOT / 'tests/ais_session_tests.cpp' or
                        source == ROOT / 'src/platform/windows/AisCredentials.cpp'):
                    raise ValueError('Unexpected source in bounded AIS build: ' + str(source))
                sources.add(source)
        required = {ROOT / path for path in ('tests/ais_transport/client.cpp',
            'tests/ais_transport/provider_client.cpp', 'src/ais/AisStreamProvider.cpp',
            'src/ais/AisStreamSession.cpp', 'tests/ais_session_tests.cpp')}
        if not required <= sources: raise ValueError('Actual provider/session/transport sources missing')
        report['sources'] = {p.relative_to(ROOT).as_posix(): api.record(p) for p in sorted(sources)}
        retained = sources | {upstream / 'libs/IXWebSocket' / name for name in report['ixInputs']}
        retained |= {path for name, path in local.items() if path.suffix in ('.h', '.hpp')}
        for source in retained:
            target = evidence / 'compiled-source' / source.relative_to(ROOT)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
        report['jsonHeaders'] = inventory(build / '_deps/opennav_rapidjson-src/include')
        if not report['jsonHeaders']: raise ValueError('Locked standalone JSON headers missing')
        save()
        api.run(['cmake', '--build', build, '--config', 'Release', '--target', *TARGETS, '--parallel', '2', '--verbose'],
                evidence / 'compile.log', timeout=600)
        runtime = build / 'bin/Release'
        if {p.stem for p in runtime.glob('*.exe')} != set(TARGETS):
            raise ValueError('Bounded runtime must contain exactly the three selected clients')
        for kind, name in (('openssl', 'libssl-3.dll'), ('openssl', 'libcrypto-3.dll'), ('zlib', 'zlib1.dll')):
            copy(stage.PREFIX[kind] + '/bin/' + name, runtime / name)
        report['crt'] = api.stage_native_runtime(runtime / (TARGETS[0] + '.exe'), runtime, ())
        report['runtime'] = inventory(runtime)
        # The selected producer tool uses its copied DLL/config/module closure;
        # tests receive this environment only, with no host PATH/trust mutation.
        child = child_environment(private, runtime)
        if not Path(child['OPENSSL_CONF']).is_file(): raise ValueError('Verified OpenSSL config missing')
        report['childEnvironment'] = {name: child[name] for name in ('PATH', 'OPENSSL_CONF', 'OPENSSL_MODULES')}
        if Path(shutil.which('openssl', path=child['PATH'])).resolve() != (private / 'openssl/bin/openssl.exe').resolve():
            raise ValueError('Certificate tool resolved outside private producer copy')
        audit_files = list(runtime.glob('*.exe')) + list(runtime.glob('*.dll')) + [private / 'openssl/bin/openssl.exe']
        report['imports'] = {}
        available = {p.name.lower() for p in runtime.glob('*.dll')}
        for path in audit_files:
            win32(path)
            log = evidence / ('imports-' + path.name + '.log')
            run([dumpbin, '/DEPENDENTS', path], log)
            imports = set(re.findall(r'^\s*([\w.-]+\.dll)\s*$', log.read_text(), re.M | re.I))
            lower = {name.lower() for name in imports}
            if not lower or any(n not in SYSTEM_DLLS | available and not n.startswith('api-ms-win-') for n in lower):
                raise ValueError('Unresolved/unapproved runtime import: ' + str(path))
            if path.stem in TARGETS[:2] and not {'libssl-3.dll', 'libcrypto-3.dll', 'zlib1.dll'} <= lower:
                raise ValueError('Actual AIS executable lacks maintained TLS/zlib imports')
            report['imports'][str(path.relative_to(ROOT))] = sorted(imports)
        save()
        run([private / 'openssl/bin/openssl.exe', 'version', '-a'], evidence / 'openssl-version.log', env=child)
        run([runtime / 'ais_session_native.exe'], evidence / 'session.log', env=child)
        run([sys.executable, ROOT / 'tests/ais_transport/network_tests.py', '--client', runtime / 'ais_transport_test_client.exe'],
            evidence / 'transport.log', env=child, timeout=180)
        run([sys.executable, ROOT / 'tests/ais_transport/provider_tests.py', '--client', runtime / 'ais_provider_test_client.exe'],
            evidence / 'provider.log', env=child, timeout=180)
        reuse.verify_same_job(ROOT)
        for name, record in {**report['localInputs'], **copies, **report['sources']}.items():
            if api.record(local.get(name, ROOT / name)) != record: raise ValueError('Input changed: ' + name)
        for directory, key in ((runtime, 'runtime'), (upstream / 'libs/IXWebSocket', 'ixInputs'),
                               (build / '_deps/opennav_rapidjson-src/include', 'jsonHeaders')):
            if inventory(directory) != report[key]: raise ValueError('Input/output tree changed: ' + key)
        if api.record(wrapper / 'CMakeLists.txt') != report['localInputs']['tests/windows_ais_runtime/CMakeLists.txt']:
            raise ValueError('Retained CMake wrapper changed')
        report['passed'] = True
    except Exception as error:
        report['error'] = type(error).__name__ + ': ' + str(error)
        raise
    finally:
        save()


if __name__ == '__main__':
    main()

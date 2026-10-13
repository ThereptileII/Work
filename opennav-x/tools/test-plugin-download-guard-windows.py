#!/usr/bin/env python3
"""Disposable native PluginHandler/Downloader gate; no product build or CI wiring.

Retained-input inventory hash is an external review pin, not authority supplied
by the inventory itself. Maintained TLS additionally requires the existing SDK
bundle authentication and native reprobes. See plugin-download-guard.md.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import windows_dependency_receipt as receipt
import windows_dependency_stage as stage

ROOT = Path(__file__).resolve().parents[1]
INPUTS = {'curlInclude', 'curlImport', 'archiveInclude', 'archiveImport', 'wxRoot', 'wxLib'}
DLLS = {'libcurl.dll', 'archive.dll', 'libssl-3.dll', 'libcrypto-3.dll', 'zlib1.dll',
        'wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll', 'msvcp140.dll', 'vcruntime140.dll'}
PLUGIN_SYSTEM_DLLS = {'comdlg32.dll', 'comctl32.dll', 'winspool.drv', 'uxtheme.dll',
                      'version.dll', 'winmm.dll', 'imm32.dll', 'oleacc.dll', 'dwmapi.dll',
                      'msimg32.dll', 'rpcrt4.dll', 'usp10.dll', 'wldap32.dll'}
NS = {'m': 'http://schemas.microsoft.com/developer/msbuild/2003'}


def module(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / filename)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def host_guard():
    if (sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true' or
            os.environ.get('RUNNER_ENVIRONMENT') != 'github-hosted' or
            os.environ.get('RUNNER_OS') != 'Windows' or
            os.environ.get('GITHUB_REPOSITORY') != 'ThereptileII/Work'):
        raise ValueError('Requires disposable GitHub-hosted Windows in ThereptileII/Work; never the boat')
    for key in ('CL', '_CL_', 'CXXFLAGS', 'CFLAGS', 'CMAKE_TOOLCHAIN_FILE'):
        if os.environ.get(key):
            raise ValueError('Inherited compiler override refused: ' + key)


def retained_inputs(root, manifest, pin, candidate):
    if not re.fullmatch('[0-9a-f]{64}', pin) or receipt._digest(manifest) != pin:
        raise ValueError('Retained inventory differs from external reviewed hash')
    data = receipt._read_json(manifest)
    if (set(data) != {'schema', 'candidateCommit', 'inputs', 'runtime', 'files'} or
            type(data['schema']) is not int or data['schema'] != 1 or
            not re.fullmatch('[0-9a-f]{40}', candidate) or data['candidateCommit'] != candidate or
            set(data['inputs']) != INPUTS or not DLLS <= set(data['runtime'])):
        raise ValueError('Retained candidate/input schema differs')
    root = receipt._workspace_root(root)
    names = list(data['files'])
    if len({n.casefold() for n in names}) != len(names):
        raise ValueError('Case-aliased retained files')
    for name in names:
        receipt._relative(name)
        item = data['files'][name]
        if (not isinstance(item, dict) or set(item) != {'bytes', 'sha256'} or
                type(item['bytes']) is not int or item['bytes'] < 0 or
                not isinstance(item['sha256'], str) or not re.fullmatch('[0-9a-f]{64}', item['sha256'])):
            raise ValueError('Invalid retained file record')
    for name, relative in data['runtime'].items():
        if not re.fullmatch(r'[a-z0-9_.-]+\.dll', name) or Path(relative).name.lower() != name:
            raise ValueError('Invalid runtime DLL mapping')
    if 'wldap32.dll' in data['runtime']:
        raise ValueError('System LDAP DLL cannot be supplied by retained runtime')
    roots = list(data['inputs'].values()) + list(data['runtime'].values())
    expected = {}
    for relative in roots:
        path = receipt._plain_path(root, relative)
        if path.is_dir():
            expected.update(receipt._inventory(root, [relative]))
        else:
            expected[relative] = {'bytes': path.stat().st_size, 'sha256': receipt._digest(path)}
    if not expected or expected != data['files']:
        raise ValueError('Retained SDK/runtime has missing, changed, extra or aliased files')
    for key in ('curlImport', 'archiveImport'):
        if not receipt._plain_path(root, data['inputs'][key]).is_file():
            raise ValueError('Import library must be a file')
    for key in INPUTS - {'curlImport', 'archiveImport'}:
        if not receipt._plain_path(root, data['inputs'][key]).is_dir():
            raise ValueError('Include/wx input must be a directory')
    return data


def bind_tls(data, expected):
    # This external candidate inventory may not replace producer authority.
    bindings = [(data['inputs']['curlImport'], stage.PREFIX['curl'] + '/lib/libcurl.lib')]
    for kind, dll in (('curl', 'libcurl.dll'), ('openssl', 'libssl-3.dll'),
                      ('openssl', 'libcrypto-3.dll'), ('zlib', 'zlib1.dll')):
        bindings.append((data['runtime'][dll], stage.PREFIX[kind] + '/bin/' + dll))
    prefix = stage.PREFIX['curl'] + '/include/curl/'
    headers = {data['inputs']['curlInclude'] + '/curl/' + name[len(prefix):]: value
               for name, value in expected.items() if name.startswith(prefix)}
    if not headers:
        raise ValueError('Authenticated curl headers missing')
    observed_headers = {name: value for name, value in data['files'].items()
                        if name.startswith(data['inputs']['curlInclude'] + '/curl/')}
    if observed_headers != headers:
        raise ValueError('Candidate curl headers differ from authenticated producer')
    for local, producer in bindings:
        if local not in data['files'] or producer not in expected or data['files'][local] != expected[producer]:
            raise ValueError('Candidate TLS input differs from authenticated producer: ' + local)


def verify_project(project, source, tools, sdk, inputs):
    tree = ET.parse(project)
    compiled = {Path(n.attrib['Include']).resolve() for n in tree.findall('.//m:ClCompile[@Include]', NS)}
    if compiled != {(source / 'model/src/downloader.cpp').resolve(),
                    (tools / 'plugin-download-guard-probe.cpp').resolve()}:
        raise ValueError('Unexpected native probe source inventory')
    content = project.read_text()
    if re.search(r'OPENNAV_DOWNLOADER_TLS_TEST|downloader-trust-shim|ForcedIncludeFiles|PrecompiledHeader>Use', content):
        raise ValueError('Native project substitutes production trust/headers')
    release = [g for g in tree.findall('.//m:ItemDefinitionGroup', NS)
               if 'Release|Win32' in g.attrib.get('Condition', '')]
    if len(release) != 1 or release[0].findtext('m:ClCompile/m:RuntimeLibrary', namespaces=NS) != 'MultiThreadedDLL':
        raise ValueError('Native probe requires Release Win32 /MD')
    includes = (release[0].findtext('m:ClCompile/m:AdditionalIncludeDirectories', namespaces=NS) or '').split(';')
    allowed = tuple(p.as_posix().casefold().rstrip('/') + '/' for p in (source, tools, sdk))
    explicit = [p.replace('\\', '/').casefold().rstrip('/') + '/' for p in includes
                if p and p != '%(AdditionalIncludeDirectories)']
    if not explicit or any(not any(p.startswith(root) for root in allowed) for p in explicit):
        raise ValueError('Native header escaped verified source/SDK')
    links = (release[0].findtext('m:Link/m:AdditionalDependencies', namespaces=NS) or '').split(';')
    normalized = {p.replace('\\', '/').casefold() for p in links}
    wx_links = [p for p in normalized if p.rsplit('/', 1)[-1].startswith('wx') and p.endswith('.lib')]
    if not wx_links or any(not p.startswith((sdk / inputs['wxLib']).as_posix().casefold() + '/') for p in wx_links):
        raise ValueError('wx import escaped verified SDK')
    for key in ('curlImport', 'archiveImport'):
        wanted = (sdk / inputs[key]).as_posix().casefold()
        matches = {p for p in normalized if p.rsplit('/', 1)[-1] == Path(inputs[key]).name.casefold()}
        if matches != {wanted}:
            raise ValueError('Native probe import escaped verified SDK: ' + key)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dependency-bundle', required=True, type=Path)
    parser.add_argument('--dependency-bundle-provenance', required=True, type=Path)
    parser.add_argument('--retained-root', required=True, type=Path)
    parser.add_argument('--retained-inputs', required=True, type=Path)
    parser.add_argument('--retained-inputs-sha256', required=True)
    parser.add_argument('--candidate-commit', required=True)
    parser.add_argument('--upstream', type=Path, default=ROOT / 'upstream/OpenCPN')
    parser.add_argument('--patch-program', default='patch')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    host_guard()
    output = args.output.absolute()
    output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'failed', 'scope': 'Native source-slice PluginHandler/Downloader probe only',
              'candidateCommit': args.candidate_commit, 'helperCommit': subprocess.check_output(
                  ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'runId': os.environ.get('GITHUB_RUN_ID'), 'runAttempt': os.environ.get('GITHUB_RUN_ATTEMPT'),
              'productAcceptance': False, 'boatAcceptance': False, 'cleanup': 'pending'}
    native = module('plugin_guard_native', 'test-ais-runtime-windows.py')
    guard = module('plugin_guard_cases', 'test-plugin-download-guard.py')
    tls = module('plugin_guard_certificates', 'test-downloader-trust.py')
    trust_script = ROOT / 'tools/plugin-guard-owned-trust.ps1'
    old_environment = dict(os.environ)
    work = Path(tempfile.mkdtemp(prefix='skager-native-plugin-', dir=os.environ['RUNNER_TEMP']))
    ca = ownership = None
    cleanup_errors = []
    trust_calls = 0
    def save():
        (output / 'results.json').write_text(json.dumps(report, indent=2) + '\n')
    def trust(action):
        nonlocal trust_calls
        trust_calls += 1
        native.run(['pwsh', '-NoProfile', '-NonInteractive', '-File', trust_script,
                    '-Action', action, '-Certificate', ca, '-Ownership', ownership],
                   output / (f'trust-{trust_calls}-' + action.lower() + '.log'), timeout=30)
    try:
        local = {name: receipt._digest(ROOT / name) for name in (
            'tools/test-plugin-download-guard-windows.py', 'tools/plugin-guard-owned-trust.ps1',
            'tools/test-plugin-download-guard.py', 'tools/test-downloader-trust.py',
            'tools/test-ais-runtime-windows.py', 'tools/windows_dependency_receipt.py',
            'tools/windows_dependency_stage.py', 'tools/windows_dependency_bundle.py',
            'tools/windows_dependency_reuse.py',
            'tools/plugin-download-guard-server.py', 'tools/plugin-download-guard-probe.cpp',
            'tools/TrustProbeConsole.h', 'patches/opencpn-5.12.4-download-trust.patch',
            'tests/plugin_download_guard/CMakeLists.txt', 'tests/plugin_download_guard/InputPaths.cmake')}
        report['localInputs'] = local
        expected, authority = native.dependency_inputs(ROOT, args.dependency_bundle, args.dependency_bundle_provenance)
        report['dependencyAuthority'] = authority
        native.reprobe_bundle_inputs(args.dependency_bundle, args.dependency_bundle_provenance,
                                     output / 'dependency-reprobe.log', expected)
        if native.dependency_inputs(ROOT, args.dependency_bundle, args.dependency_bundle_provenance) != (expected, authority):
            raise ValueError('Dependency authority changed during reprobe')
        data = retained_inputs(args.retained_root, args.retained_inputs,
                               args.retained_inputs_sha256, args.candidate_commit)
        bind_tls(data, expected)
        report['retainedInputsSha256'] = args.retained_inputs_sha256
        report['retainedInputs'] = data
        sdk = work / 'sdk'
        for name in data['files']:
            target = sdk / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(receipt._plain_path(args.retained_root, name), target)
        retained_inputs(sdk, args.retained_inputs, args.retained_inputs_sha256, args.candidate_commit)
        source = work / 'source'
        # Use the shared exact pin/hash/slice preparation, with caller-selected
        # native patch tool. No prepared executable or source is blindly reused.
        for relative in guard.EXPECTED:
            target = source / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(subprocess.check_output(['git', '-C', str(args.upstream), 'show', f'{guard.PIN}:{relative}']))
        report['source'] = guard.patch_and_slice(source, args.patch_program)
        mutant = work / 'mutant-source'
        shutil.copytree(source, mutant)
        slices = mutant / 'plugin-handler-slices.inc'
        code = slices.read_text()
        needle = 'if (!downloader.download(path)) {'
        if code.count(needle) != 1:
            raise ValueError('Guard-bypass mutation boundary changed')
        slices.write_text(code.replace(needle, 'if ((downloader.download(path), false)) {'), newline='\n')
        shutil.copytree(source, output / 'verified-source')
        binaries = []
        for name, selected in (('normal', source), ('mutant', mutant)):
            build = work / (name + '-build')
            inputs = data['inputs']
            cmake = {'SOURCE_DIR': selected, 'TOOLS_DIR': ROOT / 'tools',
                     'CURL_INCLUDE': sdk / inputs['curlInclude'], 'CURL_IMPORT': sdk / inputs['curlImport'],
                     'ARCHIVE_INCLUDE': sdk / inputs['archiveInclude'], 'ARCHIVE_IMPORT': sdk / inputs['archiveImport'],
                     'wxWidgets_ROOT_DIR': sdk / inputs['wxRoot'], 'wxWidgets_LIB_DIR': sdk / inputs['wxLib']}
            native.run(['cmake', '-S', ROOT / 'tests/plugin_download_guard', '-B', build,
                        '-G', 'Visual Studio 17 2022', '-A', 'Win32', '-DwxWidgets_CONFIGURATION=mswu',
                        *[f'-D{k}:PATH={v.as_posix()}' for k, v in cmake.items()]], output / (name + '-configure.log'))
            project = build / 'plugin-download-guard-probe.vcxproj'
            verify_project(project, selected, ROOT / 'tools', sdk, inputs)
            shutil.copyfile(project, output / (name + '.vcxproj'))
            compiler_file = next((build / 'CMakeFiles').glob('*/CMakeCXXCompiler.cmake'))
            matches = re.findall(r'set\(CMAKE_CXX_COMPILER "([^"]+)"\)', compiler_file.read_text())
            if len(matches) != 1 or not re.search(r'/Host[^/]+/x86/cl\.exe$', matches[0], re.I):
                raise ValueError('CMake did not select native x86 MSVC')
            compiler = Path(matches[0]); dumpbin = compiler.with_name('dumpbin.exe')
            report.setdefault('tools', {}).update({str(path): receipt._digest(path) for path in
                (compiler, dumpbin, Path(shutil.which('cmake')), Path(sys.executable), Path(shutil.which('pwsh')))})
            native.run(['cmake', '--build', build, '--config', 'Release', '--target',
                        'plugin-download-guard-probe', '--parallel', '2'], output / (name + '-compile.log'), timeout=300)
            runtime = work / (name + '-runtime')
            runtime.mkdir()
            binary = runtime / 'plugin-download-guard-probe.exe'
            shutil.copyfile(build / 'Release/plugin-download-guard-probe.exe', binary)
            native.win32(binary)
            for dll, relative in data['runtime'].items():
                shutil.copyfile(sdk / relative, runtime / dll)
                native.win32(runtime / dll)
                if receipt._digest(runtime / dll) != data['files'][relative]['sha256'] or (runtime / dll).stat().st_size != data['files'][relative]['bytes']:
                    raise ValueError('Staged runtime differs from retained input: ' + dll)
            report.setdefault('binaries', {})[name] = {'sha256': receipt._digest(binary),
                'runtimeSha256': {dll: receipt._digest(runtime / dll) for dll in data['runtime']}}
            available = set(data['runtime'])
            system = native.SYSTEM_DLLS | PLUGIN_SYSTEM_DLLS
            imports_report = {}
            for path in (binary, *sorted(runtime.glob('*.dll'))):
                log = output / (name + '-imports-' + path.name + '.log')
                native.run([dumpbin, '/DEPENDENTS', path], log)
                imports = {n.lower() for n in re.findall(r'^\s*([\w.-]+\.(?:dll|drv))\s*$', log.read_text(), re.M | re.I)}
                if not imports or any(n not in system | available and not n.startswith('api-ms-win-') for n in imports):
                    raise ValueError('Unresolved/unapproved native import: ' + str(path))
                imports_report[path.name] = sorted(imports)
            if not {'libcurl.dll', 'archive.dll'} <= set(imports_report[binary.name]):
                raise ValueError('Native probe does not import retained transport/archive runtime')
            report['binaries'][name]['imports'] = imports_report
            binaries.append(binary)
        save()
        # Certificate generation uses the already-verified maintained producer
        # executable. Probe children further reduce PATH to runtime + System32.
        openssl = receipt._plain_path(ROOT, stage.PREFIX['openssl'] + '/bin/openssl.exe')
        os.environ['PATH'] = str(openssl.parent) + os.pathsep + old_environment.get('PATH', '')
        os.environ['OPENSSL_CONF'] = str(ROOT / stage.PREFIX['openssl'] / 'ssl/openssl.cnf')
        os.environ['OPENSSL_MODULES'] = str(ROOT / stage.PREFIX['openssl'] / 'lib/ossl-modules')
        for key in ('SSL_CERT_FILE', 'SSL_CERT_DIR', 'CURL_CA_BUNDLE', 'OPENNAV_DOWNLOADER_TEST_CA_FILE'):
            os.environ.pop(key, None)
        if Path(shutil.which('openssl')).resolve() != openssl.resolve():
            raise ValueError('Certificate tool escaped authenticated producer')
        certs = work / 'certificates'; certs.mkdir()
        key, ca = tls.make_ca(certs, 'plugin-owned-' + work.name)
        other_key, other_ca = tls.make_ca(certs, 'plugin-untrusted-' + work.name)
        certificates = {'valid': tls.make_leaf(certs, 'valid', key, ca, 'localhost'),
            'untrusted': tls.make_leaf(certs, 'untrusted', other_key, other_ca, 'localhost'),
            'wrong_host': tls.make_leaf(certs, 'wrong', key, ca, 'wrong.invalid'),
            'expired': tls.make_leaf(certs, 'expired', key, ca, 'localhost', expired=True)}
        ownership = work / 'owned-root.json'
        shutil.copyfile(ca, output / 'owned-root.pem')
        trust('Import')
        cases = output / 'cases'; cases.mkdir()
        guard.run_cases(binaries[0], cases, None, certificates, report, binaries[1])
        trust('Remove')
        server, port = guard.start_server(cases, 'trust_removed', certificates['valid'][1], certificates['valid'][0], cases / 'inert.tar')
        try:
            report['cases'].append(guard.probe(binaries[0], cases, 'trust_removed',
                f'https://localhost:{port}/valid', None, False))
            if (cases / 'trust_removed.requests').exists():
                raise ValueError('HTTP request crossed removed native trust')
        finally:
            server.terminate(); server.wait(timeout=5)
        if len(report['cases']) != 13:
            raise ValueError('Native case inventory differs')
        retained_inputs(args.retained_root, args.retained_inputs, args.retained_inputs_sha256, args.candidate_commit)
        retained_inputs(sdk, args.retained_inputs, args.retained_inputs_sha256, args.candidate_commit)
        if any(receipt._digest(ROOT / name) != digest for name, digest in local.items()):
            raise ValueError('Local source changed during native qualification')
        for name, binary in zip(('normal', 'mutant'), binaries):
            if receipt._digest(binary) != report['binaries'][name]['sha256'] or {
                dll: receipt._digest(binary.parent / dll) for dll in data['runtime']
            } != report['binaries'][name]['runtimeSha256']:
                raise ValueError('Executed native runtime changed')
        if native.dependency_inputs(ROOT, args.dependency_bundle, args.dependency_bundle_provenance) != (expected, authority):
            raise ValueError('Dependency inputs changed during qualification')
        report['status'] = 'passed'
    except Exception as error:
        report['failure'] = str(error)
        raise
    finally:
        if ownership is not None and ownership.exists():
            try:
                trust('Remove')
            except Exception as error:
                cleanup_errors.append(str(error))
        if ownership is not None and ownership.exists():
            try:
                shutil.copyfile(ownership, output / 'owned-root.json')
            except Exception as error:
                cleanup_errors.append(str(error))
        os.environ.clear(); os.environ.update(old_environment)
        try:
            shutil.rmtree(work)
        except Exception as error:
            cleanup_errors.append(str(error))
        report['cleanup'] = 'failed' if cleanup_errors else 'verified'
        report['cleanupErrors'] = cleanup_errors
        if cleanup_errors:
            report['status'] = 'failed'
        save()
        if cleanup_errors:
            raise RuntimeError('Owned native trust/work cleanup failed: ' + '; '.join(cleanup_errors))


if __name__ == '__main__':
    main()

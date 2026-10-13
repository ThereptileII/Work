#!/usr/bin/env python3
"""Native Win32 loader guard proof with actual production mechanics and harmless DLLs."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tarfile

ROOT = Path(__file__).resolve().parents[1]
INPUTS = (
    'src/integration/OChartsModuleLoader.cpp',
    'src/integration/OChartsModuleLoader.h',
    'src/integration/PluginPresentationLoader.cpp',
    'src/integration/PluginPresentationLoader.h',
    'src/integration/PluginPresentationFallback.h',
    'patches/opencpn-5.12.4-chart-presentation.patch',
    'src/integration/OriginalPluginTlsGuard.h',
    'src/integration/Win32PluginTlsImports.h',
    'src/integration/PluginTlsBytePolicy.h',
    'src/integration/Win32PeImports.h',
    'src/plugin-adapters/ChartPresentationBindingV1.h',
    'src/plugin-adapters/ocharts/BindingState.h',
    'tests/windows_ocharts_loader/CMakeLists.txt',
    'tests/windows_ocharts_loader/fixture.cpp',
    'tests/windows_ocharts_loader/indirect_dependency.cpp',
    'tests/windows_ocharts_loader/crossroot_dependency.cpp',
    'tests/windows_ocharts_loader/unmapped_dependency.cpp',
    'tests/windows_ocharts_loader/legacy_dependency.cpp',
    'tests/windows_ocharts_loader/forward_dependency.cpp',
    'tests/windows_ocharts_loader/forwarder_fixtures.py',
    'tests/win32_pe_forwarders_test.cpp',
    'tests/plugin_tls_byte_policy_test.cpp',
    'tests/windows_ocharts_loader/loader_test.cpp',
    'tests/windows_ocharts_loader/inputs.lock.json',
    'tools/test-ocharts-loader-windows.py',
    'tools/test-windows-changed-units.py',
    'tools/windows-wx.lock.json',
)
VARIANTS = ('good', 'no_bind', 'no_status', 'no_create', 'no_destroy',
            'legacy_normal', 'legacy_delay', 'indirect_legacy', 'indirect_clean',
            'indirect_unmapped', 'indirect_external', 'indirect_cross',
            'indirect_declared_legacy', 'indirect_declared_clean', 'forward_legacy_name', 'forward_legacy_ordinal',
            'forward_ssleay_ordinal', 'forward_indirect', 'forward_clean_name',
            'forward_clean_ordinal', 'forward_explicit_dll', 'forward_multi_dot')


def identity(path):
    raw = path.read_bytes()
    return {'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest()}


def extract_read_only_vendor(archive, destination, lock):
    """Select one exact regular member. Never extract a helper or invoke a DLL."""
    if identity(archive) != {k: lock[k] for k in ('bytes', 'sha256')}:
        raise ValueError('Vendor archive differs from accepted lock')
    with tarfile.open(archive, 'r:gz') as package:
        found = [m for m in package.getmembers() if m.name == lock['member']]
        if len(found) != 1 or not found[0].isfile() or found[0].size != lock['dllBytes']:
            raise ValueError('Missing, ambiguous or changed vendor DLL member')
        with package.extractfile(found[0]) as stream:
            raw = stream.read(lock['dllBytes'] + 1)
    if len(raw) != lock['dllBytes'] or hashlib.sha256(raw).hexdigest() != lock['dllSha256']:
        raise ValueError('Vendor DLL differs from accepted hash')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(raw)
    return identity(destination)


def require_win32_dll(path):
    raw = path.read_bytes()
    if len(raw) < 64 or raw[:2] != b'MZ':
        raise ValueError('Missing DOS header: ' + str(path))
    offset = struct.unpack_from('<I', raw, 0x3c)[0]
    if offset > len(raw) - 26 or raw[offset:offset + 4] != b'PE\0\0':
        raise ValueError('Missing bounded PE header: ' + str(path))
    machine = struct.unpack_from('<H', raw, offset + 4)[0]
    characteristics = struct.unpack_from('<H', raw, offset + 22)[0]
    optional = struct.unpack_from('<H', raw, offset + 24)[0]
    if machine != 0x14c or optional != 0x10b or not characteristics & 0x2000:
        raise ValueError('Expected native x86 PE32 DLL: ' + str(path))
    return {'machine': 'I386', 'optionalHeader': 'PE32', 'dll': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
        raise SystemExit('Disposable native Windows CI required; never the boat')
    for key in ('CL', '_CL_', 'CXXFLAGS', 'CFLAGS'):
        if os.environ.get(key):
            raise SystemExit('Refusing inherited compiler override: ' + key)
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    spec = importlib.util.spec_from_file_location('changed_units', ROOT / 'tools/test-windows-changed-units.py')
    api = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(api)
    repository = Path(subprocess.check_output(['git', 'rev-parse', '--show-toplevel'], cwd=ROOT, text=True).strip()).resolve()
    if repository != ROOT and ROOT != repository / 'opennav-x':
        raise ValueError('Unexpected repository layout')
    workflow_name = os.environ.get('GITHUB_WORKFLOW_REF', '').partition('/.github/workflows/')[2].split('@', 1)[0]
    if not workflow_name or Path(workflow_name).name != workflow_name or not workflow_name.endswith(('.yml', '.yaml')):
        raise ValueError('Missing or invalid actual workflow reference')
    workflow = repository / '.github/workflows' / workflow_name
    sources = {name: identity(ROOT / name) for name in INPUTS}
    sources[os.path.relpath(workflow, ROOT).replace('\\', '/')] = identity(workflow)
    report = {'status': 'failed', 'scope': 'actual native loader/lock/hash/bind/status/fallback mechanics',
              'candidate': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'sources': sources, 'nativeProductAcceptance': False,
              'vendorExecuted': False, 'pluginFactoryExecuted': False,
              'gaps': ['real adapter OpenCPN import ABI and renderer integration',
                       'full app Standard/Safe selection and plugin lifecycle',
                       'genuine loaded-module unload refusal; fault test uses a reserved non-module address',
                       'forwarder grammar beyond the supported single-separator subset; native probes do not relax policy',
                       'declared plugin/application/upstream-Libdirs graph only; system/other-PATH/redirected or preloaded modules, arbitrary dynamic/helper loads and general module identity',
                       'encrypted-chart rendering, licensing and physical hardware']}
    lock = json.loads((ROOT / 'tests/windows_ocharts_loader/inputs.lock.json').read_text())
    scratch = ROOT / 'build/windows-ocharts-loader-guards'
    try:
        # Fresh scratch: it contains accepted read-only vendor input and private
        # copies, so it is not part of the uploaded evidence artifact.
        scratch.mkdir(parents=True, exist_ok=False)
        sdk, build = scratch / 'sdk', scratch / 'build'
        wx = sdk / 'wx'
        wxlock = json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())
        report['wxLock'] = wxlock
        for item in wxlock['archives']:
            archive = sdk / item['file']
            api.fetch(item, archive)
            api.run(['7z', 'x', '-y', '-o' + str(wx), archive], evidence / (item['file'] + '.log'))
        pico = sdk / 'picosha2/picosha2.h'
        api.fetch(lock['picosha2'], pico)
        archive = sdk / lock['vendorReadOnly']['file']
        api.fetch(lock['vendorReadOnly'], archive)
        vendor = scratch / 'read-only-vendor/o-charts_pi.dll'
        report['vendorInput'] = extract_read_only_vendor(archive, vendor, lock['vendorReadOnly'])
        report['vendorPe'] = require_win32_dll(vendor)
        report['picosha2'] = identity(pico)
        api.run(['cmake', '-S', ROOT / 'tests/windows_ocharts_loader', '-B', build,
                 '-G', 'Visual Studio 17 2022', '-A', 'Win32',
                 '-DPICOSHA2_DIR:PATH=' + pico.parent.as_posix(),
                 '-DwxWidgets_ROOT_DIR:PATH=' + wx.as_posix(),
                 '-DwxWidgets_LIB_DIR:PATH=' + (wx / 'lib/vc14x_dll').as_posix(),
                 '-DwxWidgets_CONFIGURATION=mswu'], evidence / 'configure.log')
        api.run(['cmake', '--build', build, '--config', 'Release', '--parallel', '2',
                 '--', '/verbosity:normal'], evidence / 'compile.log', timeout=180)
        forward_spec = importlib.util.spec_from_file_location('forwarder_fixtures',
            ROOT / 'tests/windows_ocharts_loader/forwarder_fixtures.py')
        forward_api = importlib.util.module_from_spec(forward_spec)
        forward_spec.loader.exec_module(forward_api)
        report['fixtureGeneration'] = forward_api.patch_fixtures(
            build / 'Release/forwarders', evidence / 'fixture-generation.json')
        client = build / 'Release/ocharts_loader_test.exe'
        report['executable'] = identity(client)
        report['runtime'] = api.stage_native_runtime(client, wx,
            ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll'))
        fixtures = client.parent
        api.run([fixtures / 'win32_pe_forwarders_test.exe'], evidence / 'native-pe-forwarders.log', timeout=60)
        report['peForwarderExecutable'] = identity(fixtures / 'win32_pe_forwarders_test.exe')
        byte_policy = fixtures / 'plugin_tls_byte_policy_test.exe'
        report['pluginTlsBytePolicyExecutable'] = identity(byte_policy)
        api.run([byte_policy], evidence / 'native-plugin-tls-byte-policy.log', timeout=60)
        report['fixtures'] = {}
        for name in VARIANTS:
            relative = ('forwarders/' if name.startswith('forward_') else '') + 'fixture_' + name + '.dll'
            fixture = fixtures / relative
            report['fixtures'][relative] = dict(identity(fixture), pe=require_win32_dll(fixture))
        stub = fixtures / 'libeay32.dll'
        report['fixtures'][stub.name] = dict(identity(stub), pe=require_win32_dll(stub))
        for name in ('helper_legacy.dll', 'helper_clean.dll', 'helper_external.dll',
                     'helper_cross_host.dll', 'crossroot/helper_cross.dll', 'unmapped/helper_unmapped.dll',
                     'declared/helper_declared_legacy.dll', 'declared/helper_declared_clean.dll',
                     'forwarders/ssleay32.dll', 'forwarders/forward_clean.dll',
                     'forwarders/forward.dll', 'forwarders/forward.clean.dll',
                     'forwarders/forward_helper.dll'):
            helper = fixtures / name
            report['fixtures'][name] = dict(identity(helper), pe=require_win32_dll(helper))
        junction_original = scratch / 'original-junction'
        shared_adapter = scratch / 'shared-adapter/skager-ocharts-adapter.dll'
        shared_adapter.parent.mkdir()
        shutil.copy2(fixtures / 'fixture_good.dll', shared_adapter)
        junction_adapter = scratch / 'adapter-junction'
        for name, target in ((junction_original, vendor.parent), (junction_adapter, shared_adapter.parent)):
            # Junction creation on local NTFS requires no symlink privilege.
            api.run(['cmd.exe', '/d', '/c', 'mklink', '/J', str(name), str(target)],
                    evidence / (name.name + '.log'))
        work = scratch / 'cases'
        api.run([client, vendor, fixtures, work, junction_original, junction_adapter,
                 evidence / 'native-tests.json'], evidence / 'native-tests.log', timeout=120)
        native = json.loads((evidence / 'native-tests.json').read_text())
        if native != {'status': 'passed', 'checks': 58, 'vendorExecuted': False,
                      'pluginFactoryExecuted': False, 'invalidHandleFaultInjected': True,
                      'unmappedSiblingGuardControl': True, 'exportForwarderControls': True,
                      'forwarderGrammarProbes': True,
                      'applicationDirectoryClosure': {
                          'legacyRefusedBeforeInitialization': True,
                          'cleanSiblingCannotMaskHost': True, 'crossRootChainRefused': True,
                          'benignFirstMappingIdentity': True, 'custodyThroughUnload': True},
                      'declaredPluginRoots': {
                          'unsafeRefusedBeforeInitialization': True,
                          'cleanSiblingCannotMaskRoot': True, 'benignFirstMappingIdentity': True,
                          'custodyThroughUnload': True, 'absentRootPreserved': True},
                      'productAcceptance': False}:
            raise ValueError('Native guard receipt is incomplete or changed')
        report['native'] = native
        if identity(client) != report['executable'] or identity(vendor) != report['vendorInput']:
            raise ValueError('Executable or read-only vendor input changed during tests')
        if identity(fixtures / 'win32_pe_forwarders_test.exe') != report['peForwarderExecutable']:
            raise ValueError('Native pure forwarder test executable changed during tests')
        if identity(byte_policy) != report['pluginTlsBytePolicyExecutable']:
            raise ValueError('Native TLS byte-policy test executable changed during tests')
        for name, item in report['fixtures'].items():
            if identity(fixtures / name) != {k: item[k] for k in ('bytes', 'sha256')}:
                raise ValueError('Fixture changed: ' + name)
        for name, item in report['runtime'].items():
            if identity(client.parent / name) != {k: item[k] for k in ('bytes', 'sha256')}:
                raise ValueError('Runtime changed: ' + name)
        report['status'] = 'passed'
    finally:
        for path in sorted((scratch / 'cases').glob('*/events.log')):
            target = evidence / 'dll-events' / (path.parent.name + '.log')
            target.parent.mkdir(exist_ok=True)
            shutil.copy2(path, target)
        drift = [name for name, value in sources.items()
                 if not (ROOT / name).is_file() or identity(ROOT / name) != value]
        if drift:
            report['status'] = 'failed'
            report['sourceDrift'] = drift
        (evidence / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    if report['status'] != 'passed':
        raise RuntimeError('Native loader guard proof did not pass')


if __name__ == '__main__':
    main()

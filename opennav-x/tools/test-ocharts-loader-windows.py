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
    'src/plugin-adapters/ChartPresentationBindingV1.h',
    'src/plugin-adapters/ocharts/BindingState.h',
    'tests/windows_ocharts_loader/CMakeLists.txt',
    'tests/windows_ocharts_loader/fixture.cpp',
    'tests/windows_ocharts_loader/loader_test.cpp',
    'tests/windows_ocharts_loader/inputs.lock.json',
    'tools/test-ocharts-loader-windows.py',
    'tools/test-windows-changed-units.py',
    'tools/windows-wx.lock.json',
)
VARIANTS = ('good', 'no_bind', 'no_status', 'no_create', 'no_destroy')


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
    workflow = repository / '.github/workflows/skager-ocharts-loader.yml'
    if repository != ROOT and ROOT != repository / 'opennav-x':
        raise ValueError('Unexpected repository layout')
    sources = {name: identity(ROOT / name) for name in INPUTS}
    sources[os.path.relpath(workflow, ROOT).replace('\\', '/')] = identity(workflow)
    report = {'status': 'failed', 'scope': 'actual native loader/lock/hash/bind/status/fallback mechanics',
              'candidate': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'sources': sources, 'nativeProductAcceptance': False,
              'vendorExecuted': False, 'pluginFactoryExecuted': False,
              'gaps': ['real adapter OpenCPN import ABI and renderer integration',
                       'full app Standard/Safe selection and plugin lifecycle',
                       'genuine loaded-module unload refusal; fault test uses a reserved non-module address',
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
        client = build / 'Release/ocharts_loader_test.exe'
        report['executable'] = identity(client)
        report['runtime'] = api.stage_native_runtime(client, wx,
            ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll'))
        fixtures = client.parent
        report['fixtures'] = {}
        for name in VARIANTS:
            fixture = fixtures / ('fixture_' + name + '.dll')
            report['fixtures'][fixture.name] = dict(identity(fixture), pe=require_win32_dll(fixture))
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
        if native != {'status': 'passed', 'checks': 38, 'vendorExecuted': False,
                      'pluginFactoryExecuted': False, 'invalidHandleFaultInjected': True, 'productAcceptance': False}:
            raise ValueError('Native guard receipt is incomplete or changed')
        report['native'] = native
        if identity(client) != report['executable'] or identity(vendor) != report['vendorInput']:
            raise ValueError('Executable or read-only vendor input changed during tests')
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

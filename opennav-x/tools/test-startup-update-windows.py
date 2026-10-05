#!/usr/bin/env python3
"""Build/run the startup update model and popup on disposable native Windows."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    if (sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true'
            or os.environ.get('RUNNER_ENVIRONMENT') != 'github-hosted'):
        raise SystemExit('Disposable native Windows CI required; never the boat')
    for key in ('CL', '_CL_', 'CXXFLAGS', 'CFLAGS'):
        if os.environ.get(key):
            raise SystemExit('Refusing inherited compiler override: ' + key)
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    spec = importlib.util.spec_from_file_location(
        'changed_units', ROOT / 'tools/test-windows-changed-units.py')
    api = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(api)
    paths = [ROOT / name for name in (
        'tools/test-startup-update-windows.py', 'tools/test-windows-changed-units.py',
        'tools/windows-wx.lock.json', 'tests/startup_update/CMakeLists.txt',
        'tests/startup_update_tests.cpp', 'tests/startup_update_dialog_test.cpp',
        'src/application/StartupUpdate.cpp', 'src/ui/StartupUpdateDialog.cpp',
        'src/ui/Controls.cpp', 'src/vessel/VesselState.cpp')]
    paths += sorted((ROOT / 'src').rglob('*.h'))
    sources = {str(p.relative_to(ROOT)): api.record(p) for p in paths}
    report = {
        'status': 'failed',
        'scope': 'isolated startup model and native popup; no installer, product, '
                 'network, boat or visual-design acceptance',
        'commit': subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'sources': sources, 'nativeProductAcceptance': False,
    }
    try:
        sdk = ROOT / 'build/startup-update-sdk'
        wx = sdk / 'wx'
        report['wxLock'] = json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())
        for item in report['wxLock']['archives']:
            archive = sdk / item['file']
            api.fetch(item, archive)
            api.run(['7z', 'x', '-y', '-o' + str(wx), archive],
                    evidence / (item['file'] + '.log'))
        build = evidence / 'build'
        api.run(['cmake', '-S', ROOT / 'tests/startup_update', '-B', build,
                 '-G', 'Visual Studio 17 2022', '-A', 'Win32',
                 '-DwxWidgets_ROOT_DIR:PATH=' + wx.as_posix(),
                 '-DwxWidgets_LIB_DIR:PATH=' + (wx / 'lib/vc14x_dll').as_posix(),
                 '-DwxWidgets_CONFIGURATION=mswu'], evidence / 'configure.log')
        api.run(['cmake', '--build', build, '--config', 'Release', '--target',
                 'startup_update_tests', 'startup_update_dialog_test', '--parallel', '2'],
                evidence / 'compile.log', timeout=180)
        clients = {name: build / ('Release/' + name + '.exe') for name in
                   ('startup_update_tests', 'startup_update_dialog_test')}
        report['executables'] = {name: api.record(path) for name, path in clients.items()}
        report['runtime'] = api.stage_native_runtime(clients['startup_update_dialog_test'], wx,
            ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll'))
        for name, client in clients.items():
            api.run([client], evidence / (name + '.log'), timeout=60)
            if api.record(client) != report['executables'][name]:
                raise ValueError('Fixture executable changed during replay: ' + name)
        for name, identity in report['runtime'].items():
            if api.record(build / 'Release' / name) != {
                    k: identity[k] for k in ('bytes', 'sha256')}:
                raise ValueError('Runtime changed during replay: ' + name)
        report['status'] = 'passed'
    finally:
        if any(api.record(ROOT / name) != identity for name, identity in sources.items()):
            report['status'] = 'failed'
            report['sourceDrift'] = True
        (evidence / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    if report['status'] != 'passed':
        raise RuntimeError('Native startup update fixture did not pass')


if __name__ == '__main__':
    main()

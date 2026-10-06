#!/usr/bin/env python3
"""Focused native setup/backup checks; no installed app, charts or hardware."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
TARGETS = ('settings_backup_tests', 'settings_backup_store_tests',
           'boat_setup_tests', 'boat_setup_store_tests', 'boat_setup_dialog_test')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    if (sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true'
            or os.environ.get('RUNNER_ENVIRONMENT') != 'github-hosted'):
        raise SystemExit('Disposable native Windows CI required; never the boat')
    if any(os.environ.get(key) for key in ('CL', '_CL_', 'CXXFLAGS', 'CFLAGS')):
        raise SystemExit('Inherited compiler overrides refused')
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    spec = importlib.util.spec_from_file_location('changed_units', ROOT / 'tools/test-windows-changed-units.py')
    api = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(api)
    report = {'status': 'failed', 'scope': 'native setup dialog and settings persistence; no boat or visual acceptance',
              'commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()}
    paths = [ROOT / 'CMakeLists.txt', Path(__file__), ROOT / 'tools/windows-wx.lock.json']
    paths += [p for folder in ('src', 'tests') for p in (ROOT / folder).rglob('*') if p.is_file()]
    sources = {p.relative_to(ROOT).as_posix(): api.record(p) for p in paths}
    report['sources'] = sources
    try:
        # Reuse the same locked SDK directory as the startup interaction step.
        sdk = ROOT / 'build/startup-update-sdk'
        wx = sdk / 'wx'
        report['wxLock'] = json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())
        for item in report['wxLock']['archives']:
            archive = sdk / item['file']
            api.fetch(item, archive)
            api.run(['7z', 'x', '-y', '-o' + str(wx), archive], evidence / (item['file'] + '.log'))
        build = evidence / 'build'
        api.run(['cmake', '-S', ROOT, '-B', build, '-G', 'Visual Studio 17 2022', '-A', 'Win32',
                 '-DOPENNAV_BUILD_UI_COMPONENTS=ON', '-DOPENNAV_BUILD_TESTS=ON',
                 '-DwxWidgets_ROOT_DIR:PATH=' + wx.as_posix(),
                 '-DwxWidgets_LIB_DIR:PATH=' + (wx / 'lib/vc14x_dll').as_posix(),
                 '-DwxWidgets_CONFIGURATION=mswu'], evidence / 'configure.log')
        api.run(['cmake', '--build', build, '--config', 'Release', '--target', *TARGETS,
                 '--parallel', '3'], evidence / 'compile.log', timeout=600)
        clients = {name: build / 'Release' / (name + '.exe') for name in TARGETS}
        report['executables'] = {name: api.record(path) for name, path in clients.items()}
        report['runtime'] = api.stage_native_runtime(clients['boat_setup_dialog_test'], wx,
            ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll'))
        for name, client in clients.items():
            api.run([client], evidence / (name + '.log'), timeout=30)
            if api.record(client) != report['executables'][name]:
                raise ValueError('Native test executable changed')
        report['status'] = 'passed'
    finally:
        if any(api.record(ROOT / name) != identity for name, identity in sources.items()):
            report['status'] = 'failed'
            report['sourceDrift'] = True
        (evidence / 'summary.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    if report['status'] != 'passed':
        raise RuntimeError('Native settings checks failed')


if __name__ == '__main__':
    main()

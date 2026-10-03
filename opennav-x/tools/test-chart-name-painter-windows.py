#!/usr/bin/env python3
"""Replay one production name painter on disposable native Windows."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
INPUTS = (
    'tests/chart_name_text_test.cpp',
    'src/integration/ChartNameText.h',
    'src/integration/ChartNameAlphaWindows.cpp',
    'src/integration/ChartNameSpacing.h',
    'tests/windows_chart_name_painter/CMakeLists.txt',
    'tools/test-chart-name-painter-windows.py',
    'tools/test-windows-changed-units.py',
    'tools/windows-wx.lock.json',
)


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
    workflow = Path('.github/workflows/skager-chart-name-painter.yml')
    repository = Path(subprocess.check_output(['git', 'rev-parse', '--show-toplevel'], cwd=ROOT, text=True).strip()).resolve()
    if repository == ROOT:
        workflow_path = ROOT / workflow
    elif ROOT == repository / 'opennav-x' and not (ROOT / workflow).exists():
        workflow_path = repository / workflow
    else:
        raise ValueError('Unexpected or ambiguous repository layout')
    sources = {str(p): api.record(ROOT / p) for p in INPUTS}
    sources[os.path.relpath(workflow_path, ROOT).replace('\\', '/')] = api.record(workflow_path)
    report = {'status': 'failed', 'scope': 'one offline geographic name painter; Windows GDI+ translucent coverage correction',
              'candidate': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'sources': sources, 'nativeProductAcceptance': False}
    try:
        sdk = ROOT / 'build/windows-chart-name-sdk'
        wx = sdk / 'wx'
        wxlock = json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())
        report['wxLock'] = wxlock
        for item in wxlock['archives']:
            archive = sdk / item['file']
            api.fetch(item, archive)
            api.run(['7z', 'x', '-y', '-o' + str(wx), archive], evidence / (item['file'] + '.log'))
        build = evidence / 'build'
        api.run(['cmake', '-S', ROOT / 'tests/windows_chart_name_painter', '-B', build,
                 '-G', 'Visual Studio 17 2022', '-A', 'Win32',
                 '-DwxWidgets_ROOT_DIR:PATH=' + wx.as_posix(),
                 '-DwxWidgets_LIB_DIR:PATH=' + (wx / 'lib/vc14x_dll').as_posix(),
                 '-DwxWidgets_CONFIGURATION=mswu'], evidence / 'configure.log')
        api.run(['cmake', '--build', build, '--config', 'Release', '--target',
                 'chart_name_text_test', '--parallel', '1', '--', '/verbosity:normal'],
                evidence / 'compile.log', timeout=120)
        client = build / 'Release/chart_name_text_test.exe'
        report['executable'] = api.record(client)
        report['runtime'] = api.stage_native_runtime(client, wx,
            ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll'))
        api.run([client, evidence / 'chart-names.png'], evidence / 'painter.log', timeout=60)
        if api.record(client) != report['executable']:
            raise ValueError('Painter executable changed during replay')
        for name, identity in report['runtime'].items():
            if api.record(client.parent / name) != {k: identity[k] for k in ('bytes', 'sha256')}:
                raise ValueError('Runtime changed during replay: ' + name)
        report['status'] = 'passed'
    finally:
        if any(api.record(ROOT / name) != identity for name, identity in sources.items()):
            report['status'] = 'failed'
            report['sourceDrift'] = True
        image = evidence / 'chart-names.png'
        if image.is_file():
            raw = image.read_bytes()
            report['image'] = api.record(image)
            if raw[:8] == b'\x89PNG\r\n\x1a\n' and len(raw) >= 24:
                report['image']['dimensions'] = list(struct.unpack('>II', raw[16:24]))
        (evidence / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    if report['status'] != 'passed':
        raise RuntimeError('Native painter replay did not pass')


if __name__ == '__main__':
    main()

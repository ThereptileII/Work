#!/usr/bin/env python3
"""Build only the offline pilot component fixtures on disposable native Windows."""
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
    os.environ.pop('AISSTREAM_API_KEY', None)
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    spec = importlib.util.spec_from_file_location('changed_units', ROOT / 'tools/test-windows-changed-units.py')
    api = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(api)
    paths = [ROOT / 'tools/test-pilot-component-windows.py', ROOT / 'tools/test-windows-changed-units.py',
             ROOT / 'tools/windows-wx.lock.json', *sorted((ROOT / 'tests/pilot_component').glob('*'))]
    paths += sorted((ROOT / 'src').rglob('*.h'))
    paths += [ROOT / 'tests/pilot_drawer_test.cpp', ROOT / 'tests/pilot_presentation_tests.cpp']
    for directory in ('ui', 'vessel', 'adapters', 'application'):
        paths += sorted((ROOT / 'src' / directory).glob('*.cpp'))
    sources = {str(p.relative_to(ROOT)): api.record(p) for p in paths if p.is_file()}
    report = {'status': 'failed', 'scope': 'offline pilot drawer and presentation fixtures only; no product, chart, network or hardware acceptance',
              'commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'sources': sources, 'nativeProductAcceptance': False}
    try:
        sdk = ROOT / 'build/pilot-component-sdk'
        wx = sdk / 'wx'
        report['wxLock'] = json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())
        for item in report['wxLock']['archives']:
            archive = sdk / item['file']
            api.fetch(item, archive)
            api.run(['7z', 'x', '-y', '-o' + str(wx), archive], evidence / (item['file'] + '.log'))
        build = evidence / 'build'
        api.run(['cmake', '-S', ROOT / 'tests/pilot_component', '-B', build,
                 '-G', 'Visual Studio 17 2022', '-A', 'Win32',
                 '-DwxWidgets_ROOT_DIR:PATH=' + wx.as_posix(),
                 '-DwxWidgets_LIB_DIR:PATH=' + (wx / 'lib/vc14x_dll').as_posix(),
                 '-DwxWidgets_CONFIGURATION=mswu'], evidence / 'configure.log')
        api.run(['cmake', '--build', build, '--config', 'Release', '--target',
                 'pilot_drawer_test', 'pilot_presentation_tests', '--parallel', '2'], evidence / 'compile.log', timeout=180)
        client = build / 'Release/pilot_drawer_test.exe'
        report['executable'] = api.record(client)
        report['runtime'] = api.stage_native_runtime(client, wx,
            ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll'))
        presentation = build / 'Release/pilot_presentation_tests.exe'
        report['presentationExecutable'] = api.record(presentation)
        api.run([presentation, '--gtest_output=json:' + str(evidence / 'presentation.json')],
                evidence / 'presentation.log', timeout=30)
        presentation_result = json.loads((evidence / 'presentation.json').read_text())
        cases = [case for suite in presentation_result['testsuites']
                 for case in suite['testsuite']]
        if (presentation_result['tests'] < 6 or presentation_result['failures'] != 0
                or presentation_result.get('disabled', 0) != 0
                or presentation_result.get('errors', 0) != 0
                or len(cases) != presentation_result['tests']
                or any(case.get('status') != 'RUN' or case.get('result') != 'COMPLETED'
                       for case in cases)):
            raise ValueError('Presentation tests missing, disabled or failed')
        if api.record(presentation) != report['presentationExecutable']:
            raise ValueError('Presentation executable changed during execution')
        captures = evidence / 'captures'
        api.run([client, captures], evidence / 'interaction.log', timeout=60)
        interaction = json.loads((captures / 'result.json').read_text())
        expected = {'autopilot-day', 'autopilot-dusk', 'autopilot-night',
                    'autopilot-pending-day', 'autopilot-auto-day', 'autopilot-stale-day',
                    'autopilot-replay-day', 'autopilot-status-only-day', 'autopilot-status-only-night'}
        if (interaction.get('passed') is not True or interaction.get('checks', 0) < 100
                or set(interaction.get('captures', [])) != expected):
            raise ValueError('Pilot interaction evidence missing or failed')
        report['captures'] = {name: api.record(captures / (name + '.png')) for name in sorted(expected)}
        report['interaction'] = interaction
        report['presentation'] = presentation_result
        report['googleTestSources'] = {
            p.relative_to(build / '_deps/googletest-src').as_posix(): api.record(p)
            for p in sorted((build / '_deps/googletest-src').rglob('*')) if p.is_file()}
        report['googleTestRevision'] = '58d77fa8070e8cec2dc1ed015d66b454c8d78850'
        if api.record(client) != report['executable']:
            raise ValueError('Fixture executable changed during replay')
        for name, identity in report['runtime'].items():
            if api.record(client.parent / name) != {k: identity[k] for k in ('bytes', 'sha256')}:
                raise ValueError('Runtime changed during replay: ' + name)
        report['status'] = 'passed'
    finally:
        if any(api.record(ROOT / name) != identity for name, identity in sources.items()):
            report['status'] = 'failed'
            report['sourceDrift'] = True
        (evidence / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    if report['status'] != 'passed':
        raise RuntimeError('Native pilot component fixtures did not pass')


if __name__ == '__main__':
    main()

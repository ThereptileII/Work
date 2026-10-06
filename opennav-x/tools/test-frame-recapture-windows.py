#!/usr/bin/env python3
"""Native Win32 component red/green for modeless setup recapture; no application."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
GUARD = '  if (boat_setup_ && boat_setup_->IsShownOnScreen()) return true;\n'
WITNESSES = {
    'original': 'PASS original reproduces focus-loss cancellation',
    'guarded': 'PASS guarded activates exactly once',
}


def method(text, start):
    if text.count(start) != 1:
        raise ValueError('Expected one production method: ' + start)
    begin = text.index(start)
    end = text.index('{', begin) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[begin:end] + '\n'


def extract(root):
    patch = (root / 'patches/opencpn-5.12.4-xnav.patch').read_text()
    added = '\n'.join(line[1:] for line in patch.splitlines()
                      if line.startswith('+') and not line.startswith('+++'))
    frame = method(added, 'void MyFrame::OnRecaptureTimer(')
    if 'if (opennav::HasXNavTransientSurface()) return;' not in frame:
        raise ValueError('Patched production recapture guard missing')
    integration = method((root / 'src/integration/OpenCPNIntegration.cpp').read_text(),
                         'bool HasXNavTransientSurface()')
    transient = method((root / 'src/ui/Shell.cpp').read_text(),
                       'bool Shell::HasTransientSurface() const')
    if transient.count(GUARD) != 1:
        raise ValueError('Exact reviewed setup guard missing or ambiguous')
    common = {'frame-recapture.inc': frame, 'integration-transient.inc': integration}
    return {variant: dict(common, **{'shell-transient.inc':
            transient if variant == 'guarded' else transient.replace(GUARD, '')})
            for variant in WITNESSES}


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
    paths = [Path(__file__), ROOT / 'tools/test-windows-changed-units.py',
             ROOT / 'tools/windows-wx.lock.json', ROOT / 'patches/opencpn-5.12.4-xnav.patch',
             ROOT / 'src/ui/Shell.cpp', ROOT / 'src/integration/OpenCPNIntegration.cpp',
             ROOT / 'src/ui/Controls.cpp', ROOT / 'src/vessel/VesselState.cpp',
             *sorted((ROOT / 'tests/frame_recapture_windows').glob('*')),
             *sorted((ROOT / 'src').rglob('*.h'))]
    sources = {p.relative_to(ROOT).as_posix(): api.record(p) for p in paths if p.is_file()}
    report = {'status': 'failed', 'scope': 'isolated Win32 modeless focus red/green; no product, package or boat qualification',
              'commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'sources': sources, 'nativeProductAcceptance': False}
    try:
        extracted = evidence / 'extracted'
        report['extracted'] = {}
        for variant, files in extract(ROOT).items():
            folder = extracted / variant
            folder.mkdir(parents=True)
            report['extracted'][variant] = {}
            for name, text in files.items():
                path = folder / name
                path.write_text(text, encoding='utf-8', newline='\n')
                report['extracted'][variant][name] = api.record(path)
        sdk = ROOT / 'build/startup-update-sdk'
        wx = sdk / 'wx'
        report['wxLock'] = json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())
        for item in report['wxLock']['archives']:
            archive = sdk / item['file']
            api.fetch(item, archive)
            api.run(['7z', 'x', '-y', '-o' + str(wx), archive], evidence / (item['file'] + '.log'))
        build = evidence / 'build'
        api.run(['cmake', '-S', ROOT / 'tests/frame_recapture_windows', '-B', build,
                 '-G', 'Visual Studio 17 2022', '-A', 'Win32',
                 '-DRECAPTURE_EXTRACTED:PATH=' + extracted.as_posix(),
                 '-DwxWidgets_ROOT_DIR:PATH=' + wx.as_posix(),
                 '-DwxWidgets_LIB_DIR:PATH=' + (wx / 'lib/vc14x_dll').as_posix(),
                 '-DwxWidgets_CONFIGURATION=mswu'], evidence / 'configure.log')
        api.run(['cmake', '--build', build, '--config', 'Release', '--target',
                 'recapture_original', 'recapture_guarded', '--parallel', '2'],
                evidence / 'compile.log', timeout=180)
        clients = {v: build / 'Release' / ('recapture_' + v + '.exe') for v in WITNESSES}
        report['executables'] = {v: api.record(p) for v, p in clients.items()}
        report['runtime'] = api.stage_native_runtime(clients['guarded'], wx,
            ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll'))
        report['cases'] = []
        for variant, client in clients.items():
            log = evidence / (variant + '.log')
            api.run([client], log, timeout=20)
            if log.read_text(encoding='utf-8', errors='replace').count(WITNESSES[variant]) != 1:
                raise ValueError('Native focus witness missing: ' + variant)
            report['cases'].append({'variant': variant, 'status': 'passed', 'witness': WITNESSES[variant],
                                    'log': api.record(log)})
        for variant, client in clients.items():
            if api.record(client) != report['executables'][variant]:
                raise ValueError('Native component changed during test')
        for name, identity in report['runtime'].items():
            if api.record(build / 'Release' / name) != {k: identity[k] for k in ('bytes', 'sha256')}:
                raise ValueError('Native runtime changed during test')
        report['status'] = 'passed'
    finally:
        if any(api.record(ROOT / name) != identity for name, identity in sources.items()):
            report['status'] = 'failed'
            report['sourceDrift'] = True
        (evidence / 'summary.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    if report['status'] != 'passed':
        raise RuntimeError('Native recapture red/green failed')


if __name__ == '__main__':
    main()

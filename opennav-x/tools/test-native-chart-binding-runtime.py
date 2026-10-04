#!/usr/bin/env python3
"""Bounded actual BindingState mutex A/B; no application, plugin load or boat acceptance."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CRT_PREFIX = ('https://raw.githubusercontent.com/OpenCPN/OCPNWindowsCoreBuildSupport/'
              'e90cc5842b02d5502a549f0f90424e3e1614bf67/buildwin/vc/')
CRT = (
    {'file': 'msvcp140.dll', 'bytes': 457000,
     'sha256': 'fa21058e50d0d6860da87d784f573670bf5d3efd65158145954ef96d0cd403cf'},
    {'file': 'vcruntime140.dll', 'bytes': 86840,
     'sha256': '1b372f064eacb455a0351863706e6326ca31b08e779a70de5de986b5be8069a1'},
)
INPUTS = ('tools/test-native-chart-binding-runtime.py', 'tools/test-windows-changed-units.py',
          'tests/native_chart_binding_runtime/CMakeLists.txt',
          'tests/native_chart_binding_runtime/probe.cpp',
          'src/plugin-adapters/ocharts/BindingState.h',
          'src/plugin-adapters/ChartPresentationBindingV1.h')
STAGES = ('before-bind', 'after-bind', 'before-status', 'after-status',
          'before-initialization', 'after-initialization', 'before-complete',
          'after-complete', 'before-selected-status', 'after-selected-status',
          'before-repeat-bind', 'after-repeat-bind', 'complete')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    if (sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true' or
            os.environ.get('RUNNER_ENVIRONMENT') != 'github-hosted' or
            os.environ.get('GITHUB_REPOSITORY') != 'ThereptileII/Work'):
        raise SystemExit('Exact disposable hosted Windows repository required')
    for name in ('CL', '_CL_', 'CXXFLAGS', 'CFLAGS'):
        if os.environ.get(name):
            raise SystemExit('Inherited compiler override refused: ' + name)
    spec = importlib.util.spec_from_file_location('changed_units', ROOT / 'tools/test-windows-changed-units.py')
    api = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(api)
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    repository = Path(subprocess.check_output(['git', 'rev-parse', '--show-toplevel'], cwd=ROOT, text=True).strip())
    workflow = repository / '.github/workflows/skager-chart-binding-runtime.yml'
    sources = {name: ROOT / name for name in INPUTS}
    sources[os.path.relpath(workflow, ROOT).replace('\\', '/')] = workflow
    report = {'status': 'failed', 'scope': __doc__, 'applicationBuild': False,
              'actualAdapterLoaded': False, 'boatAccess': False,
              'candidate': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'runId': os.environ.get('GITHUB_RUN_ID'), 'runAttempt': os.environ.get('GITHUB_RUN_ATTEMPT'),
              'sources': {name: api.record(path) for name, path in sources.items()},
              'runtimeProvenance': {'repository': 'OpenCPN/OCPNWindowsCoreBuildSupport',
                  'commit': 'e90cc5842b02d5502a549f0f90424e3e1614bf67',
                  'failedRun': 37191400051, 'failedArtifact': 11301641242,
                  'identityReceipt': 'evidence/local/ocharts-private-wxcurl-trust-windows/prerequisites.json'},
              'cases': []}
    try:
        for item in CRT:
            api.fetch(dict(item, url=CRT_PREFIX + item['file']), evidence / 'runtime' / item['file'])
        build = evidence / 'build'
        api.run(['cmake', '-S', ROOT / 'tests/native_chart_binding_runtime', '-B', build,
                 '-G', 'Visual Studio 17 2022', '-A', 'Win32'], evidence / 'configure.log', timeout=90)
        api.run(['cmake', '--build', build, '--config', 'Release', '--parallel', '2',
                 '--', '/verbosity:normal'], evidence / 'compile.log', timeout=120)
        observations = {}
        for mode in ('original', 'guarded'):
            run_dir = evidence / mode
            run_dir.mkdir()
            executable = run_dir / ('binding-' + mode + '.exe')
            shutil.copy2(build / 'Release' / executable.name, executable)
            for item in CRT:
                shutil.copy2(evidence / 'runtime' / item['file'], run_dir / item['file'])
            identity = api.record(executable)
            command = [str(executable)]
            (evidence / (mode + '.command.json')).write_text(json.dumps(command, indent=2) + '\n')
            child = subprocess.Popen(command, cwd=run_dir, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            timeout = False
            try:
                stdout, stderr = child.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                timeout = True
                child.kill()
                stdout, stderr = child.communicate(timeout=5)
            finally:
                if child.poll() is None:
                    child.kill()
                    child.wait(timeout=5)
            (evidence / (mode + '.stdout.txt')).write_bytes(stdout)
            (evidence / (mode + '.stderr.txt')).write_bytes(stderr)
            case = {'mode': mode, 'exitCode': child.returncode,
                    'exitCodeUnsigned': child.returncode & 0xffffffff,
                    'timedOut': timeout, 'timeoutSeconds': 5, 'executable': identity}
            report['cases'].append(case)
            (evidence / (mode + '.process.json')).write_text(json.dumps(case, indent=2) + '\n')
            stdout = stdout.decode('utf-8', errors='strict')
            stderr = stderr.decode('utf-8', errors='strict')
            observations[mode] = (stdout, stderr)
            loaded = {}
            for line in stdout.splitlines():
                if line.startswith('runtime|'):
                    _, name, path, version = line.split('|')
                    if name in loaded or name not in {item['file'] for item in CRT}:
                        raise ValueError('Unexpected/duplicate loaded runtime identity')
                    actual = Path(path)
                    if actual.resolve() != (run_dir / name).resolve():
                        raise ValueError('Child loaded a different runtime path')
                    loaded[name] = dict(api.record(actual), version=version)
            if set(loaded) != {item['file'] for item in CRT}:
                raise ValueError('Child omitted actual loaded CRT paths/version')
            for item in CRT:
                if {k: loaded[item['file']][k] for k in ('bytes', 'sha256')} != {k: item[k] for k in ('bytes', 'sha256')}:
                    raise ValueError('Loaded runtime differs from failed-job lock')
            case['loadedRuntime'] = loaded
            if api.record(executable) != identity:
                raise ValueError('Child executable changed')
        original, guarded = report['cases']
        old_out, old_err = observations['original']
        new_out, new_err = observations['guarded']
        if (original['timedOut'] or original['exitCodeUnsigned'] != 0xc0000005 or
                'guard=disabled' not in old_out or
                [x for x in old_err.splitlines() if x.startswith('stage=')] != ['stage=before-bind'] or
                'exception=C0000005' not in old_err):
            raise ValueError('Original actual binding first-lock access violation not reproduced')
        if (guarded['timedOut'] or guarded['exitCode'] != 0 or 'guard=enabled' not in new_out or
                [x for x in new_err.splitlines() if x.startswith('stage=')] != ['stage=' + x for x in STAGES] or
                'exception=' in new_err):
            raise ValueError('Guarded actual binding sequence did not complete cleanly')
        if original['loadedRuntime'] != guarded['loadedRuntime']:
            raise ValueError('A/B runtime differs')
        report['status'] = 'passed'
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        if any(api.record(path) != report['sources'][name] for name, path in sources.items()):
            report['status'] = 'failed'
            report['sourceDrift'] = True
        (evidence / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    if report['status'] != 'passed':
        raise RuntimeError('Native binding runtime proof failed')


if __name__ == '__main__':
    main()

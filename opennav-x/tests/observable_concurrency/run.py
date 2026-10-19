"""Build the actual pinned Observable source in an isolated, offline fixture."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

PIN = '37fd0cddb7334fe489e9f18aa163977a9c5c84f7'
HERE = Path(__file__).resolve().parent
FILES = ('libs/observable/src/observable.cpp',
         'libs/observable/include/observable.h',
         'libs/observable/include/observable_evt.h')
parser = argparse.ArgumentParser()
parser.add_argument('--upstream', type=Path, required=True)
parser.add_argument('--build-dir', type=Path, required=True)
parser.add_argument('--cmake', default='cmake')
parser.add_argument('--ctest', default='ctest')
parser.add_argument('--cmake-arg', action='append', default=[])
parser.add_argument('--sanitize', action='store_true')
parser.add_argument('--negative-control', action='store_true',
                    help='Require the unpatched source to fail unregister and move scenarios')
args = parser.parse_args()
build = args.build_dir.resolve()
source = build / 'source'
source.mkdir(parents=True, exist_ok=True)

def run(*command, cwd=None):
    subprocess.run(list(map(str, command)), cwd=cwd, check=True, timeout=120)

for name in FILES:
    target = source / name
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(subprocess.check_output(
        ['git', '-C', str(args.upstream.resolve()), 'show', f'{PIN}:{name}'],
        timeout=30))

# Keep patch application inside this disposable fixture, never the supplied
# upstream checkout or a parent's integration source.
run('git', 'init', '-q', source)
patch = HERE.parents[1] / 'patches/opencpn-5.12.4-observable-locking.patch'
if not args.negative_control:
    run('git', 'apply', '--check', patch, cwd=source)
    run('git', 'apply', patch, cwd=source)
    run('git', 'apply', '--reverse', '--check', patch, cwd=source)
run(args.cmake, '-S', HERE, '-B', build / 'build',
    f'-DOPENCPN_SOURCE_DIR:PATH={source.as_posix()}',
    f'-DOBSERVABLE_SANITIZE={"ON" if args.sanitize else "OFF"}',
    '-DCMAKE_BUILD_TYPE=Debug', *args.cmake_arg)
run(args.cmake, '--build', build / 'build', '--config', 'Debug', '--parallel', '2')
if args.negative_control:
    executable = build / 'build' / 'observable_concurrency'
    if not executable.exists():
        executable = build / 'build' / 'Debug' / 'observable_concurrency.exe'
    for scenario, expected in (
        ('unregister', 'unregister waits until queue delivery returns'),
        ('move', 'move assignment removes the previous registration')):
        result = subprocess.run([str(executable), scenario], text=True,
                                capture_output=True, timeout=15)
        print(result.stdout + result.stderr, end='')
        if result.returncode != 1 or expected not in result.stderr:
            raise RuntimeError(f'Unpatched {scenario} did not fail at the expected boundary')
else:
    run(args.ctest, '--test-dir', build / 'build', '-C', 'Debug', '--output-on-failure')
(build / 'evidence.json').write_text(json.dumps({
    'pin': PIN,
    'result': 'unpatched defects reproduced' if args.negative_control else 'four scenarios passed',
    'patch_sha256': hashlib.sha256(patch.read_bytes()).hexdigest(),
    'source_sha256': {name: hashlib.sha256((source / name).read_bytes()).hexdigest()
                      for name in FILES},
    'fixture_sha256': {name: hashlib.sha256((HERE / name).read_bytes()).hexdigest()
                       for name in ('main.cpp', 'CMakeLists.txt', 'run.py')},
    'sanitizers': args.sanitize,
    'scope': 'Actual pinned Observable implementation and wx event queue; controlled queue barrier and bounded concurrent churn; no device or full application.'
}, indent=2) + '\n')

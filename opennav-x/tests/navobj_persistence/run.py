#!/usr/bin/env python3
"""Configure, build and run only the isolated native persistence fixture."""
import argparse
from pathlib import Path
import subprocess
import sys

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--source', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--generator', help='CMake generator, e.g. Visual Studio 17 2022')
p.add_argument('--architecture', help='Generator platform, e.g. Win32 (requires --generator)')
p.add_argument('--config', default='Release')
p.add_argument('--parallel', type=int, default=2)
p.add_argument('--cmake', default='cmake')
p.add_argument('--ctest', default='ctest')
a = p.parse_args()
if a.architecture and not a.generator:
    p.error('--architecture requires an explicit --generator')
if a.parallel < 1:
    p.error('--parallel must be positive')
here = Path(__file__).resolve().parent
build = a.output.resolve()
configure = [a.cmake, '-S', str(here), '-B', str(build),
             '-DOPENCPN_SOURCE_DIR:PATH=' + a.source.resolve().as_posix(),
             '-DPython3_EXECUTABLE:FILEPATH=' + Path(sys.executable).as_posix(),
             '-DCMAKE_BUILD_TYPE=' + a.config]
if a.generator:
    configure += ['-G', a.generator]
if a.architecture:
    configure += ['-A', a.architecture]
subprocess.run(configure, check=True)
subprocess.run([a.cmake, '--build', str(build), '--config', a.config,
                '--parallel', str(a.parallel)], check=True)
subprocess.run([a.ctest, '--test-dir', str(build), '-C', a.config,
                '--output-on-failure', '--no-tests=error'], check=True)

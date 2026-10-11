#!/usr/bin/env python3
"""Compile/run only the byte-reader fixtures. Never load a PE control as code."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cxx', default=os.environ.get('CXX', 'cl' if sys.platform == 'win32' else 'c++'))
    parser.add_argument('--sanitize', action='store_true', help='ASan/UBSan with GCC/Clang')
    parser.add_argument('--pe', type=Path, action='append', default=[], help='Optional existing Win32 DLL/EXE read-only positive control')
    args = parser.parse_args()
    controls = [str(path.resolve(strict=True)) for path in args.pe]
    with tempfile.TemporaryDirectory(prefix='skager-pe-imports-') as directory:
        output = Path(directory)
        exe = output / ('reader.exe' if sys.platform == 'win32' else 'reader')
        source = ROOT / 'tests/win32_pe_imports_test.cpp'
        if Path(args.cxx).name.lower() in ('cl', 'cl.exe'):
            if args.sanitize:
                parser.error('--sanitize requires GCC/Clang')
            command = [args.cxx, '/nologo', '/std:c++17', '/EHsc', '/W4', '/WX',
                       '/I' + str(ROOT / 'src'), str(source), '/Fe:' + str(exe)]
        else:
            command = [args.cxx, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic',
                       '-I' + str(ROOT / 'src'), str(source), '-o', str(exe)]
            if args.sanitize:
                command += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
        subprocess.run(command, cwd=output, check=True, timeout=60)
        subprocess.run([str(exe), *controls], cwd=output, check=True, timeout=30)


if __name__ == '__main__':
    main()

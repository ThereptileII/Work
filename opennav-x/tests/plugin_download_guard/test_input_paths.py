#!/usr/bin/env python3
"""Exercise only the native probe's CMake input boundary; no configure/build."""
from pathlib import Path
import subprocess
import tempfile

HELPER = Path(__file__).with_name('InputPaths.cmake').resolve()


def main():
    with tempfile.TemporaryDirectory(prefix='plugin probe paths ') as raw:
        work = Path(raw)
        paths = {}
        for name in ('SOURCE_DIR', 'TOOLS_DIR', 'CURL_INCLUDE', 'ARCHIVE_INCLUDE'):
            path = work / (name + ' with spaces')
            path.mkdir()
            paths[name] = path.as_posix()
        for name in ('CURL_IMPORT', 'ARCHIVE_IMPORT'):
            path = work / (name + ' with spaces.lib')
            path.write_bytes(b'path fixture only')
            paths[name] = path.as_posix()
        # These optional SDK hints need no real Windows SDK for path conversion.
        paths['wxWidgets_ROOT_DIR'] = 'C:/Program Files/verified wx'
        paths['wxWidgets_LIB_DIR'] = '//server/share/verified wx/lib'
        script = work / 'boundary.cmake'
        script.write_text(f'include([[{HELPER.as_posix()}]])\n' + '\n'.join(
            f'if(NOT "${{{name}}}" STREQUAL [[{value}]])\n'
            f'  message(FATAL_ERROR "Normalization changed {name}")\nendif()'
            for name, value in paths.items()) + '\n')

        def execute(values):
            arguments = ['cmake'] + ['-D'+name+'='+value.replace('/', '\\')
                                      for name, value in values.items()]
            return subprocess.run(arguments + ['-P', str(script)], capture_output=True, text=True)

        positive = execute(paths)
        if positive.returncode:
            raise AssertionError(positive.stdout + positive.stderr)
        print('PASS: spaces/backslashes, drive and UNC hints preserve normalized paths')
        negatives = [
            ('missing required input', {k: v for k, v in paths.items() if k != 'SOURCE_DIR'}, 'Missing SOURCE_DIR'),
            ('invalid directory input', dict(paths, SOURCE_DIR=paths['CURL_IMPORT']), 'Required native probe directory is missing'),
            ('missing import file', dict(paths, CURL_IMPORT=(work/'absent.lib').as_posix()), 'Required native probe file is missing'),
            ('directory as import file', dict(paths, CURL_IMPORT=paths['SOURCE_DIR']), 'Required native probe file is missing'),
        ]
        for name, values, expected in negatives:
            result = execute(values)
            if result.returncode == 0 or expected not in result.stdout + result.stderr:
                raise AssertionError(name + ': ' + result.stdout + result.stderr)
            print('PASS: refused ' + name)


if __name__ == '__main__':
    main()

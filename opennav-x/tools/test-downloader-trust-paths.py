#!/usr/bin/env python3
"""Focused normalization and unchanged native trust-boundary regression."""
import argparse
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PATHS = ('OPENNAV_SOURCE_DIR', 'OPENNAV_TOOLS_DIR', 'CURL_ROOT',
         'wxWidgets_ROOT_DIR', 'wxWidgets_LIB_DIR', 'SKAGER_OCHARTS_PREPARED')


def check(cmake):
    helper = ROOT / 'tests/downloader_trust/InputPaths.cmake'
    cases = [(r'D:\a\Work with spaces\opennav-x', 'D:/a/Work with spaces/opennav-x'),
             (r'\\server\share with spaces\sdk', '//server/share with spaces/sdk'),
             ('/already/normalized path', '/already/normalized path')]
    with tempfile.TemporaryDirectory(prefix='trust-paths-') as raw:
        script = Path(raw) / 'check.cmake'
        for raw_value, expected in cases:
            text = ''.join(f'set({name} [==[{raw_value}]==])\n' for name in PATHS)
            text += f'include("{helper.as_posix()}")\n'
            for name in PATHS:
                text += (f'if(NOT "${{{name}}}" STREQUAL [==[{expected}]==])\n'
                         f'  message(FATAL_ERROR "Path normalization changed {name}")\nendif()\n')
            script.write_text(text)
            subprocess.run([cmake, '-P', str(script)], check=True)
        script.write_text(f'include("{helper.as_posix()}")\n' + ''.join(
            f'if(DEFINED {name})\nmessage(FATAL_ERROR "Invented absent {name}")\nendif()\n'
            for name in PATHS))
        subprocess.run([cmake, '-P', str(script)], check=True)
    recipe = (ROOT / 'tests/downloader_trust/CMakeLists.txt').read_text()
    assert recipe.index('InputPaths.cmake') < recipe.index('--verify-prepared')
    assert recipe.index('InputPaths.cmake') < recipe.index('find_package(wxWidgets')
    assert recipe.index('--verify-prepared') < recipe.index('Targets.cmake')
    assert '"${CURL_ROOT}/libcurl.lib"' in recipe
    assert 'COMPILE_ONLY' not in recipe
    print('18 drive/UNC/POSIX path checks; 6 absent-input checks; production boundaries passed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cmake', default='cmake')
    check(parser.parse_args().cmake)

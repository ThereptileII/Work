#!/usr/bin/env python3
"""Native regression of the exact private-adapter path/glob target fragment."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

class Identity:
    @staticmethod
    def record(path):
        data = path.read_bytes()
        return {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}


def path_regression(evidence, api, cmake='cmake'):
    """Run the actual target fragment with an inert configuration-only source unit."""
    recipe = (ROOT / 'cmake/ocharts-adapter/Targets.cmake').read_text()
    root_line = re.findall(r'^set\(P "\$\{SKAGER_PREPARED\}/source"\)$', recipe, re.M)
    glob_lines = re.findall(r'^file\(GLOB WXCURL CONFIGURE_DEPENDS [^\n]+\)\n'
                           r'add_library\(skager_wxcurl STATIC \$\{WXCURL\}\)', recipe, re.M)
    if len(root_line) != 1 or len(glob_lines) != 1:
        raise ValueError('Actual production path/glob fragment changed')
    fragment = root_line[0] + '\n' + glob_lines[0] + '\n'
    prepared = (evidence / 'a/Work with spaces/ocharts-prepared').resolve()
    source = prepared / 'source/libs/wxcurl/src/base.cpp'
    source.parent.mkdir(parents=True, exist_ok=False)
    source.write_text('int path_probe_translation_unit;\n')
    results = {}
    cases = ('original-native', 'normalized-native', 'normalized-typed') if sys.platform == 'win32' else ('normalized-posix',)
    for case in cases:
        project = evidence / case
        project.mkdir()
        normalize = '' if case.startswith('original') else 'include("' + (ROOT / 'cmake/ocharts-adapter/PreparedPath.cmake').as_posix() + '")\n'
        (project / 'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.20)\nproject(path_regression LANGUAGES CXX)\n' + normalize + fragment)
        value = str(prepared) if sys.platform == 'win32' else prepared.as_posix()
        arg = '-DSKAGER_PREPARED' + (':PATH' if case.endswith('typed') else '') + '=' + value
        command = [cmake, '-S', str(project), '-B', str(project / 'build'), arg]
        if sys.platform == 'win32':
            command += ['-G', 'Visual Studio 17 2022', '-A', 'Win32']
        (project / 'command.json').write_text(json.dumps(command, indent=2) + '\n')
        with (project / 'configure.log').open('wb') as stream:
            result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, timeout=120)
        text = (project / 'configure.log').read_text(errors='replace')
        results[case] = {'returncode': result.returncode, 'argument': arg,
                         'log': api.record(project / 'configure.log')}
        (evidence / 'path-results.json').write_text(json.dumps(results, indent=2) + '\n')
        if case.startswith('original'):
            if result.returncode == 0 or 'Invalid character escape' not in text or 'base.cpp' not in text:
                raise ValueError('Original native CMake path failure was not reproduced')
        elif result.returncode:
            raise ValueError('Normalized actual path fragment failed: ' + case)
    return results



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--cmake', default='cmake')
    args = parser.parse_args()
    args.evidence.mkdir(parents=True, exist_ok=False)
    inputs = ('cmake/ocharts-adapter/CMakeLists.txt',
              'cmake/ocharts-adapter/PreparedPath.cmake',
              'cmake/ocharts-adapter/Targets.cmake',
              'tools/ocharts_cmake_path_probe.py',
              '.github/workflows/skager-ocharts-compile.yml')
    records = {}
    for name in inputs:
        path = ROOT / name
        if name.startswith('.github/workflows/') and not path.is_file():
            # Published monorepo places workflows beside opennav-x/, while
            # the local standalone product worktree contains its own .github/.
            path = ROOT.parent / name
        records[name] = Identity.record(path)
    (args.evidence / 'inputs.json').write_text(json.dumps(records, indent=2) + '\n')
    print(json.dumps(path_regression(args.evidence.resolve(), Identity, args.cmake), indent=2))

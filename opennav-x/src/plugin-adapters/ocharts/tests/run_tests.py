#!/usr/bin/env python3
"""Focused copied-binding and actual resource-loader tests; no plugin/chart launch."""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--plugin-source', type=Path, required=True)
p.add_argument('--resources', type=Path, required=True)
p.add_argument('--wx-config', default='wx-config')
p.add_argument('--wx-prefix', type=Path)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
repo = Path(__file__).resolve().parents[4]
source, resources, output = a.plugin_source.resolve(), a.resources.resolve(), a.output.resolve()
output.mkdir(parents=True, exist_ok=True)
wx = [a.wx_config] + (['--prefix=' + str(a.wx_prefix)] if a.wx_prefix else [])
cflags = shlex.split(subprocess.check_output(wx + ['--cxxflags'], text=True))
libs = shlex.split(subprocess.check_output(wx + ['--libs'], text=True))
common = ['-std=c++17', '-I' + str(repo / 'src')]
commands = []
def run(command):
    commands.append(command)
    result = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    with (output / 'test.log').open('a') as log:
        log.write(shlex.join(command) + '\n' + result.stdout)
    print(result.stdout, end='')
    result.check_returncode()
try:
    run([os.environ.get('CXX', 'c++'), *common, '-Wall', '-Wextra', '-Werror',
         str(Path(__file__).with_name('binding_test.cpp')), '-o', str(output / 'binding_test')])
    run([str(output / 'binding_test')])
    run([os.environ.get('CC', 'cc'), '-c', str(source / 'src/sha256.c'),
         '-o', str(output / 'sha256.o')])
    run([os.environ.get('CXX', 'c++'), *common, *cflags,
         '-I' + str(resources), '-I' + str(source / 'src'),
         '-I' + str(source / 'libs/pugixml'),
         str(Path(__file__).with_name('resource_test.cpp')),
         str(source / 'libs/pugixml/pugixml.cpp'), str(output / 'sha256.o'),
         *libs, '-o', str(output / 'resource_test')])
    with tempfile.TemporaryDirectory(prefix='skager-ocharts-resource-') as scratch:
        run([str(output / 'resource_test'), str(resources), str(Path(scratch) / 'negative-control')])
finally:
    (output / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')

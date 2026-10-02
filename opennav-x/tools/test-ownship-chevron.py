#!/usr/bin/env python3
"""Focused production-painter test with real wx raster output; not native/GL acceptance."""
import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--wx-config', required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
a.output = a.output.resolve()
source = (ROOT / 'src/integration/ChartPresentation.cpp').read_text()
start = source.index('bool DrawChartOwnship(')
end = source.index('\n}', start) + 2
(a.output / 'production-ownship.h').write_text(source[start:end])
# The immutable SVG supplies the independent geometry oracle, in original order.
prototype = (ROOT / 'docs/design/prototype/index.html').read_text()
path = re.search(r'id="ownShipHeading"[^>]*>\s*<path d="([^"]+)"', prototype)[1]
assert re.fullmatch(r'M[-\d .]+Z', path), path
numbers = list(map(float, re.findall(r'-?\d+(?:\.\d+)?', path)))
assert len(numbers) == 8
(a.output / 'prototype-ownship.h').write_text(
    'const std::array<wxPoint2DDouble, 4> reference{{' +
    ','.join('{%s,%s}' % tuple(numbers[i:i+2]) for i in range(0, 8, 2)) + '}};\n')
flags = shlex.split(subprocess.check_output([a.wx_config, '--cxxflags'], text=True))
libs = shlex.split(subprocess.check_output([a.wx_config, '--libs', 'std'], text=True))
exe = a.output / 'ownship-chevron-test'
subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O0', '-g', *flags,
                '-I' + str(ROOT / 'src'), '-I' + str(a.output),
                str(ROOT / 'tests/ownship_chevron_test.cpp'), *libs, '-o', str(exe)], check=True)
subprocess.run([str(exe), str(a.output / 'ownship-day-dusk-night.png')], check=True)

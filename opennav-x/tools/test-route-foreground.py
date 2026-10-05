#!/usr/bin/env python3
"""Focused production route painter/geometry checks; no native GL acceptance."""
import argparse
import os
import re
from pathlib import Path
import shlex
import subprocess
ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--wx-config', required=True)
p.add_argument('--output', required=True, type=Path)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
a.output = a.output.resolve()
source = (ROOT / 'src/integration/ChartPresentation.cpp').read_text()
start = source.index('bool DefaultChartRouteStyle(')
end = source.index('bool UseChartCogPredictorStyle(', start)
(a.output / 'production-route.h').write_text(source[start:end])
prototype = (ROOT / 'docs/design/prototype/index.html').read_text()
style = {}
for rule in re.findall(r'\.chart-route\s*\{([^}]+)\}', prototype):
    style.update(item.split(':', 1) for item in rule.split(';') if ':' in item)
assert style['stroke-linejoin'] == 'round'
(a.output / 'reference-route.h').write_text(
    'constexpr double reference_route_width = ' + style['stroke-width'] + ';\n')
flags = shlex.split(subprocess.check_output([a.wx_config, '--cxxflags'], text=True))
libs = shlex.split(subprocess.check_output([a.wx_config, '--libs', 'std'], text=True))
exe = a.output / 'route-foreground-test'
subprocess.run([os.environ.get('CXX','c++'), '-std=c++17', '-O2', *flags,
               '-I'+str(ROOT/'src'), '-I'+str(a.output),
               str(ROOT/'tests/route_foreground_test.cpp'), *libs, '-o', str(exe)], check=True)
subprocess.run([str(exe), str(a.output/'route-foreground-day-dusk-night.png')], check=True)

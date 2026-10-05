#!/usr/bin/env python3
"""Actual COG paint/ownership code with real wx raster; recording GL is not a driver."""
import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--wx-config',required=True)
p.add_argument('--output',required=True,type=Path)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);a.output=a.output.resolve()
source=(ROOT/'src/integration/ChartPresentation.cpp').read_text()
start=source.index('void CaptureChartCogPredictorStyle(')
end=source.index('\n}',start)+2
capture=source[start:end]
start=source.index('bool UseChartCogPredictorStyle(')
end=source.index('bool DrawChartOwnship(',start)
(a.output/'production-cog.h').write_text(capture+'\n'+source[start:end])
html=(ROOT/'docs/design/prototype/index.html').read_text()
attrs=re.search(r'<path d="M0-21V-100" ([^>]+)',html)[1]
width=re.search(r'stroke-width="([^"]+)"',attrs)[1]
dash=re.search(r'stroke-dasharray="([^"]+)"',attrs)[1].split()
alpha=re.search(r'opacity="([^"]+)"',attrs)[1]
(a.output/'reference-cog.h').write_text(f'constexpr double ref_width={width},ref_dash={dash[0]},ref_gap={dash[1]},ref_alpha={alpha};\n')
flags=shlex.split(subprocess.check_output([a.wx_config,'--cxxflags'],text=True))
libs=shlex.split(subprocess.check_output([a.wx_config,'--libs','std'],text=True))
exe=a.output/'cog-predictor-test'
subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-O2',*flags,'-I'+str(ROOT/'src'),'-I'+str(a.output),str(ROOT/'tests/cog_predictor_test.cpp'),*libs,'-o',str(exe)],check=True)
subprocess.run([str(exe),str(a.output/'cog-predictor-day-dusk-night.png')],check=True)

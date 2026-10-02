#!/usr/bin/env python3
"""Exercise actual COG guard/endpoint and ocpnDC software polygon bodies."""
import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--wx-config',required=True)
p.add_argument('--opencpn-source',required=True,type=Path)
p.add_argument('--output',required=True,type=Path)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);a.output=a.output.resolve()
def block(source,needle):
    start=source.index(needle);end=source.index('{',start);depth=1
    while depth:
        end+=1;depth+=(source[end]=='{')-(source[end]=='}')
    return source[start:end+1]
source=(a.opencpn_source/'gui/src/chcanv.cpp').read_text()
stock=subprocess.check_output(['git','-C',str(a.opencpn_source),'show','37fd0cddb7334fe489e9f18aa163977a9c5c84f7:gui/src/chcanv.cpp'],text=True)
marker='if (g_cog_predictor_endmarker) {'
(a.output/'production-endpoint.h').write_text(block(source,marker))
(a.output/'stock-endpoint.h').write_text(block(stock,marker))
start=source.index('bool xnav_cog_painted = false;')
(a.output/'production-cog-guard.h').write_text(source[start:source.index('if (!xnav_cog_painted)',start)])
(a.output/'production-endpoint-shape.h').write_text(re.search(r'static int s_png_pred_icon\[\][^;]+;',source)[0])
source=(a.opencpn_source/'gui/src/ocpndc.cpp').read_text()
(a.output/'production-stroke-polygon.h').write_text(block(source,'void ocpnDC::StrokePolygon('))
html=(ROOT/'docs/design/prototype/index.html').read_text()
def variables(selector):
    return dict(re.findall(r'--([\w-]+):([^;}]+)',re.search(re.escape(selector)+r'\{([^}]+)\}',html)[1]))
variables_day=variables(':root')
inks=[variables_day['route'],variables('#app[data-theme=dusk]')['route'],variables('#app[data-theme=night]')['route']]
(a.output/'reference-endpoint.h').write_text('const unsigned reference_route_ink[]{'+','.join('0x'+v[1:] for v in inks)+'};\n')
flags=shlex.split(subprocess.check_output([a.wx_config,'--cxxflags'],text=True))
libs=shlex.split(subprocess.check_output([a.wx_config,'--libs','std'],text=True))
exe=a.output/'cog-endpoint-test'
subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-O2',*flags,'-I'+str(a.output),str(ROOT/'tests/cog_endpoint_test.cpp'),*libs,'-o',str(exe)],check=True)
subprocess.run([str(exe),str(a.output/'cog-endpoint-day-dusk-night.png')],check=True)

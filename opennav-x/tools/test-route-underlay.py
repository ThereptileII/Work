#!/usr/bin/env python3
"""Real pinned tess2 union and production wx painter; recording GL is not driver acceptance."""
import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--raw-tess2-defect',action='store_true',help='intentionally fail the retained pinned-tess2 counterexample')
p.add_argument('--wx-config',required=True)
p.add_argument('--opencpn-source',required=True,type=Path)
p.add_argument('--output',required=True,type=Path)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);a.output=a.output.resolve()
source=(ROOT/'src/integration/ChartRouteUnderlay.cpp').read_text()
start=source.index('bool ChartRouteUnderlay::Draw(');end=source.index('\n} // namespace',start)
(a.output/'production-underlay.h').write_text(source[start:end])
source=(ROOT/'src/integration/ChartPresentation.cpp').read_text()
start=source.index('bool DrawChartRouteSegment(');end=source.index('\n}',start)+2
(a.output/'production-foreground.h').write_text(source[start:end])
html=(ROOT/'docs/design/prototype/index.html').read_text()
rule=re.search(r'\.chart-route-under\{([^}]+)\}',html)[1]
style=dict(item.split(':',1) for item in rule.split(';') if ':' in item)
assert 'stroke-linejoin' not in style and 'stroke-linecap' not in style
(a.output/'reference-underlay.h').write_text(f'constexpr double ref_width={style["stroke-width"]},ref_alpha={style["opacity"]};\n')
tess=a.opencpn_source/'libs/libtess2';objects=[]
for source in sorted((tess/'Source').glob('*.c')):
    obj=a.output/(source.stem+'.o');objects.append(str(obj))
    subprocess.run([os.environ.get('CC','cc'),'-O2','-I'+str(tess/'Include'),'-c',str(source),'-o',str(obj)],check=True)
flags=shlex.split(subprocess.check_output([a.wx_config,'--cxxflags'],text=True))
libs=shlex.split(subprocess.check_output([a.wx_config,'--libs','std'],text=True))
exe=a.output/'route-underlay-test'
subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-O2',*flags,'-I'+str(ROOT/'src'),'-I'+str(a.output),'-I'+str(tess/'Include'),str(ROOT/'tests/route_underlay_test.cpp'),str(ROOT/'src/integration/ChartRouteUnderlayGeometry.cpp'),*objects,*libs,'-o',str(exe)],check=True)
subprocess.run([str(exe),str(a.output/'route-underlay-turns-crossings.png'),*(['--raw-tess2-defect'] if a.raw_tess2_defect else [])],check=True)

# Exercise the collector from the actually patched production source, including
# the unchanged wrap decision code and pinned integer clipping implementation.
route=(a.opencpn_source/'gui/src/route_gui.cpp').read_text()
longitude=route[route.index('static void TestLongitude('):route.index('void RouteGui::Draw(')]
start=route.index('void RouteGui::DrawGLLines(')
gl=route[start:route.index('void RouteGui::CalculateDCRect(',start)]
start=route.index('void RouteGui::RenderSegment(')
clip=route[start:route.index('  //    If hilite is desired',start)]+'}\n'
assert 'underlay->Add(' in gl and 'underlay->Add(' in clip
(a.output/'production-collection.h').write_text(longitude+gl+clip)
exe=a.output/'route-underlay-collection-test'
geoprim=a.opencpn_source/'libs/geoprim/src'
subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-O2',*flags,'-I'+str(a.output),'-I'+str(geoprim),str(ROOT/'tests/route_underlay_collection_test.cpp'),str(geoprim/'line_clip.cpp'),*libs,'-o',str(exe)],check=True)
subprocess.run([str(exe)],check=True)

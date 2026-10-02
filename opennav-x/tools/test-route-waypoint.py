#!/usr/bin/env python3
"""Focused actual marker painter, ordinal and icon replacement regression.
GL records submissions/state; driver and native Windows acceptance remain separate.
"""
import argparse, os, re, shlex, subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--wx-config',required=True)
p.add_argument('--upstream',required=True,type=Path,help='Pinned upstream with current reviewed patches applied')
p.add_argument('--output',required=True,type=Path)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);a.output=a.output.resolve();a.upstream=a.upstream.resolve()
s=(ROOT/'src/integration/ChartRouteWaypoint.cpp').read_text()
(a.output/'production-marker.h').write_text(s[s.index('namespace opennav::integration {'):])
s=(ROOT/'src/ui/Controls.cpp').read_text();start=s.index('wxFont UiFontWeight(');end=s.index('\ndouble UiTextWidth',start)
(a.output/'production-marker-font.h').write_text(s[start:end])
s=(a.upstream/'gui/src/waypointman_gui.cpp').read_text(); start=s.index('MarkIcon *WayPointmanGui::ProcessIcon(');end=s.index('\nvoid WayPointmanGui::ProcessIcons(',start)
replace=s[start:end];start=s.index('bool WayPointmanGui::IsPinnedRouteDiamond(');end=s.index('\nwxRect WayPointmanGui::CropImageOnAlpha',start)
(a.output/'production-marker-provenance.h').write_text(replace+'\n'+s[start:end])
html=(ROOT/'docs/design/prototype/index.html').read_text()
assert '.map-waypoint circle{fill:var(--floating);stroke:var(--route);stroke-width:2}' in html
assert 'r="10"/><text x="${x}" y="${y+.5}"' in html
assert '.map-waypoint text{fill:var(--route);font-size:8px;font-weight:650;' in html
flags=shlex.split(subprocess.check_output([a.wx_config,'--cxxflags'],text=True));libs=shlex.split(subprocess.check_output([a.wx_config,'--libs','std'],text=True))
exe=a.output/'route-waypoint-test'
subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-O2',*flags,'-I'+str(ROOT/'src'),'-I'+str(a.output),'-I'+str(a.upstream/'model/include'),'-I'+str(a.upstream/'libs/picosha2'),str(ROOT/'tests/route_waypoint_test.cpp'),*libs,'-o',str(exe)],check=True)
subprocess.run([str(exe),str(a.output),str(a.upstream/'data/svg/markicons/1st-Diamond.svg')],check=True)

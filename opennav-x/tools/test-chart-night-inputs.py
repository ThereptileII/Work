#!/usr/bin/env python3
"""Exercise actual production route and Online body Night input gates, offline."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess

ROOT=Path(__file__).resolve().parents[1]
def function(text,signature):
    start=text.index(signature);end=text.index('{',start)+1;depth=1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start:end]

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--wx-config',required=True)
p.add_argument('--wx-prefix')
p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
route=function((ROOT/'src/integration/ChartPresentation.cpp').read_text(),'bool ChartActiveRouteInk(')
colour=function((ROOT/'src/ui/Controls.cpp').read_text(),'wxColour Colour(')
online=(ROOT/'src/integration/OnlineAisOverlay.cpp').read_text()
body=online[online.index('  auto colors=ui::OnlineChartTheme(mode);'):online.index('  const auto pen=dc.GetPen();')]
patch=(ROOT/'patches/opencpn-5.12.4-chart-presentation.patch').read_text()
light=next(line[1:] for line in patch.splitlines() if line.startswith('+      const S52color *ink = getColor('))
source=r'''
#include "integration/ChartCanvasInk.h"
#include <wx/wx.h>
#include "color_types.h"
#include <wx/colour.h>
#include <wx/thread.h>
#include <stdexcept>
#include <iostream>
namespace opennav::ui { COLOUR }
struct ChartCanvas { ColorScheme scheme; ColorScheme GetColorScheme(){return scheme;} };
namespace opennav::integration {
bool xnav_mode=false,active=false;
bool ChartBackground(ColorScheme,wxColour&,wxColour&){return xnav_mode&&active;}
ROUTE
ui::OnlineChartPalette OnlineBody(ChartCanvas& canvas,ui::LightMode mode) {
BODY
return colors;
}
}
const S52color* getColor(const char* name) {
  static S52color safety{},geographic{},water{};
  safety.R=1;geographic.R=2;water.R=3;
  return std::string(name)=="CHBLK"?&safety:std::string(name)=="XNGEO"?&geographic:&water;
}
int LightRole(wxString m_ColorScheme) {
LIGHT_STATEMENT
if(water->R!=3)throw std::runtime_error("LIGHTS halo lost water role");
return ink->R;
}
int main() {
  using namespace opennav;using namespace integration;
  unsigned checks=0;
  auto check=[&](bool value){++checks;if(!value)throw std::runtime_error("Actual Night paint gate failed");};
  const ui::LightMode modes[]={ui::LightMode::Day,ui::LightMode::Dusk,ui::LightMode::Night};
  const ColorScheme schemes[]={GLOBAL_COLOR_SCHEME_DAY,GLOBAL_COLOR_SCHEME_DUSK,GLOBAL_COLOR_SCHEME_NIGHT};
  for(unsigned theme=0;theme<3;++theme)for(bool xnav:{false,true})for(bool verified:{false,true}) {
    xnav_mode=xnav;active=verified;ChartCanvas canvas{schemes[theme]};wxColour ink(1,2,3);
    const bool accepted=ChartActiveRouteInk(canvas,ink);
    check(accepted==(xnav&&verified));
    check(ink==(accepted?ui::Colour(theme==2?0x71937e:ui::ActiveRouteInk(modes[theme])):wxColour(1,2,3)));
    const auto raw=ui::OnlineChartTheme(modes[theme]),actual=OnlineBody(canvas,modes[theme]);
    const bool dim=theme==2&&xnav&&verified;
    check(actual.stroke==(dim?0x714e5d:raw.stroke));
    check(actual.fill==(dim?0x101a20:raw.fill));
    check(actual.selected==(dim?0x9e7a8a:raw.selected));
    check(actual.stale==raw.stale);
    check(actual.label==raw.label); // Already-effective SCRUM-252 role must not dim twice.
  }
  check(LightRole("NIGHT")==1);
  check(LightRole("DAY_BRIGHT")==2);
  check(LightRole("DUSK")==2);
  std::cout<<checks<<" actual route/Online Night gate checks passed\n";
}
'''.replace('COLOUR',colour).replace('ROUTE',route).replace('BODY',body).replace('LIGHT_STATEMENT',light)
cpp=a.output/'night-inputs.cpp';cpp.write_text(source)
wx=[a.wx_config]+(['--prefix='+a.wx_prefix]if a.wx_prefix else [])
flags=shlex.split(subprocess.check_output(wx+['--cxxflags'],text=True))
libs=shlex.split(subprocess.check_output(wx+['--libs','core,base'],text=True))
cmd=['c++','-std=c++17','-Wall','-Wextra','-Werror',*flags,'-I'+str(ROOT/'src'),
     '-I'+str(ROOT/'upstream/OpenCPN/libs/s52plib/src'),str(cpp),*libs,'-o',str(a.output/'night-inputs')]
(a.output/'command.json').write_text(json.dumps(cmd,indent=2)+'\n')
subprocess.run(cmd,check=True)
result=subprocess.run([a.output/'night-inputs'],check=True,capture_output=True,text=True)
(a.output/'checks.txt').write_text(result.stdout);print(result.stdout,end='')

#!/usr/bin/env python3
"""Exercise the real vector-ink resolver and pinned Piano brush method, offline."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def body(text, start):
    begin = text.index(start)
    opening = text.index('{', begin)
    level = 1
    end = opening + 1
    while level:
        level += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[begin:end]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--wx-config', required=True)
    ap.add_argument('--piano-source', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    own = (ROOT / 'src/integration/ChartPresentation.cpp').read_text()
    resolver = body(own, 'bool ChartVectorSelectorInk(')
    color = body(own, 'wxColour Color(')
    piano = body(args.piano_source.read_text(), 'void Piano::SetColorScheme(')
    sync = body(args.piano_source.read_text(), 'void Piano::SyncChartPresentation(')
    # Only inspect the entry preambles here. The full upstream paint methods
    # contain inactive preprocessor branches with unbalanced raw-text braces.
    raw_piano = args.piano_source.read_text()
    paint = raw_piano.split('void Piano::Paint(int y, ocpnDC &dc,', 1)[1][:500]
    gl = raw_piano.split('void Piano::DrawGLSL(', 1)[1][:700]
    assert paint.index('SyncChartPresentation();') < paint.index('if (shapeDC)')
    assert gl.index('SyncChartPresentation();') < gl.index('BuildGLTexture();')
    # Current implementation must preserve every stock color assignment and
    # invalidate the shared GL atlas after the optional local override.
    pinned = (ROOT / 'upstream/OpenCPN/gui/src/piano.cpp').read_text()
    stock = body(pinned, 'void Piano::SetColorScheme(')
    first, last = piano.index('#ifdef OPENNAV_X'), piano.index('#endif') + len('#endif')
    assert piano[:first] + piano[last:].lstrip('\n') == stock
    tokens = json.loads((ROOT / 'docs/design/prototype-tokens.json').read_text())['themes']
    expected = []
    for mode in ('day', 'dusk', 'night'):
        expected.append('{' + ','.join('0x' + tokens[mode][k].lstrip('#')
                        for k in ('--route', '--float-muted')) + '}')
    source = r'''
#include <wx/wx.h>
#include "color_types.h"
#include "ui/Theme.h"
#include <array>
#include <iostream>
#include <stdexcept>
namespace opennav::integration {
bool xnav_mode=false, active=false;
COLOR_FUNCTION
RESOLVER_FUNCTION
}
wxColour GetGlobalColor(const wxString& name) {
  const std::array<const char*,10> keys={"UIBDR","BLUE2","BLUE1","GREEN2","GREEN1","VIO01","VIO02","YELO2","YELO1","UINFD"};
  for(std::size_t i=0;i<keys.size();++i)
    if(name==keys[i]) return wxColour(20+i*19,30+i*17,40+i*13);
  throw std::runtime_error("Unknown global role");
}
struct Canvas { ColorScheme scheme=GLOBAL_COLOR_SCHEME_DAY; ColorScheme GetColorScheme() { return scheme; } };
struct Piano {
  Canvas canvas; Canvas *m_parentCanvas=&canvas;
  wxBrush m_backBrush,m_rBrush,m_srBrush,m_vBrush,m_svBrush,m_utileBrush,m_tileBrush,m_cBrush,m_scBrush,m_unavailableBrush;
  int m_tex_piano_height=99;
  void SetColorScheme(ColorScheme);
  void SyncChartPresentation();
  std::array<wxColour,10> Colors() const {
    return {m_backBrush.GetColour(),m_rBrush.GetColour(),m_srBrush.GetColour(),m_vBrush.GetColour(),m_svBrush.GetColour(),m_utileBrush.GetColour(),m_tileBrush.GetColour(),m_cBrush.GetColour(),m_scBrush.GetColour(),m_unavailableBrush.GetColour()};
  }
};
PIANO_FUNCTION
SYNC_FUNCTION
int main() {
  using namespace opennav::integration;
  const unsigned long expected[3][2]={EXPECTED_COLORS};
  const ColorScheme schemes[]={GLOBAL_COLOR_SCHEME_DAY,GLOBAL_COLOR_SCHEME_DUSK,GLOBAL_COLOR_SCHEME_NIGHT};
  int checks=0;
  auto require=[&](bool value){++checks;if(!value)throw std::runtime_error("Selector palette check failed "+std::to_string(checks));};
  for(int theme=0;theme<3;++theme) {
    Piano stock; xnav_mode=active=false;stock.SetColorScheme(schemes[theme]);
    for(bool mode:{false,true})for(bool verified:{false,true}) {
      xnav_mode=mode;active=verified; Piano actual;
      actual.SetColorScheme(schemes[theme]);
      require(actual.m_tex_piano_height==0);
      auto a=actual.Colors(),b=stock.Colors();
      for(std::size_t i=0;i<a.size();++i) {
        wxColour wanted=b[i];
        if(mode&&verified&&(i==3||i==4))wanted=Color(expected[theme][i==4?0:1]);
        require(a[i]==wanted);
      }
      if(mode&&verified)require(a[3]!=a[4]);
    }
    // Actual deferred library failure: no theme callback occurs between paint
    // frames. Both brushes and the GL atlas must return to stock before drawing.
    Piano retained; retained.canvas.scheme=schemes[theme];
    xnav_mode=active=true; retained.SetColorScheme(schemes[theme]);
    retained.m_tex_piano_height=99;
    retained.SyncChartPresentation();
    require(retained.m_tex_piano_height==99); // unchanged paint avoids reupload
    active=false; retained.SyncChartPresentation();
    require(retained.m_tex_piano_height==0);
    require(retained.Colors()==stock.Colors());
    retained.m_tex_piano_height=99; retained.SyncChartPresentation();
    require(retained.m_tex_piano_height==99);
    active=true; retained.SyncChartPresentation();
    require(retained.m_tex_piano_height==0);
    require(retained.m_svBrush.GetColour()==Color(expected[theme][0]));
    require(retained.m_vBrush.GetColour()==Color(expected[theme][1]));
    xnav_mode=false; retained.SyncChartPresentation();
    require(retained.Colors()==stock.Colors());
  }
  std::cout<<checks<<" actual chart-selector brush checks passed\n";
}
'''.replace('COLOR_FUNCTION', color).replace('RESOLVER_FUNCTION', resolver)
    source = source.replace('PIANO_FUNCTION', piano).replace('SYNC_FUNCTION', sync).replace('EXPECTED_COLORS', ','.join(expected))
    cpp = args.output / 'selector.cpp'
    cpp.write_text(source)
    # wx flags select the same local runtime; no installed navigation profile.
    flags = shlex.split(subprocess.check_output([args.wx_config, '--cxxflags'], text=True))
    libs = shlex.split(subprocess.check_output([args.wx_config, '--libs', 'core,base'], text=True))
    command = ['c++', '-std=c++17', '-DOPENNAV_X=1', *flags,
               '-I' + str(ROOT / 'src'), '-I' + str(ROOT / 'upstream/OpenCPN/libs/s52plib/src'),
               str(cpp), *libs, '-o', str(args.output / 'selector')]
    (args.output / 'command.json').write_text(json.dumps(command, indent=2)+'\n')
    subprocess.run(command, check=True)
    result = subprocess.run([args.output / 'selector'], check=True, capture_output=True, text=True)
    (args.output / 'checks.txt').write_text(result.stdout)
    print(result.stdout, end='')


if __name__ == '__main__':
    main()

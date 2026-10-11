#!/usr/bin/env python3
"""Execute the production route-card draw function with recording DC/GL seams.

No UI, raster fixture, driver, Windows or boat performance acceptance.
"""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'src/integration/ChartRouteLabel.cpp').read_text()
draw = source[source.index('bool DrawChartRouteLabel('):source.rindex('} // namespace')]
harness = r'''
#include "application/ChartDeclutter.h"
#include <algorithm>
#include <memory>
#include <vector>
#include <cstdlib>
#include <limits>
#include <iostream>
#define ocpnUSE_GL
using GLuint=unsigned; using GLint=int;
enum {GL_MAX_TEXTURE_SIZE,GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_TEXTURE_MAG_FILTER,
      GL_NEAREST,GL_RGBA,GL_UNSIGNED_BYTE,GL_TEXTURE_WIDTH,GL_TEXTURE_HEIGHT,
      GL_BLEND,GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA};
int deletes=0,uploads=0,gl_draws=0;
void glDeleteTextures(int,const unsigned*) {++deletes;}
void glGetIntegerv(int,int* p) {*p=4096;}
void glGenTextures(int,unsigned* p) {*p=71;}
void glBindTexture(int,unsigned) {}
void glTexParameteri(int,int,int) {}
void glTexImage2D(int,int,int,int,int,int,int,int,const void*) {++uploads;}
void glGetTexLevelParameteriv(int,int,int,int* p) {*p=1;}
void glEnable(int) {} void glDisable(int) {} void glBlendFunc(int,int) {}
struct Image { int GetWidth(){return 1;} int GetHeight(){return 1;}
  const unsigned char* GetData(){static unsigned char v[3]={};return v;}
  const unsigned char* GetAlpha(){static unsigned char v[1]={255};return v;} };
struct XY {int x=0,y=0; int GetRight(){return 1;} int GetBottom(){return 1;}};
struct Cache {Image image; int bitmap=1; XY origin,bounds;
  bool texture_current=true,texture_owned=true,texture_failed=false;
  template<class Delete> bool FailTexture(unsigned& texture,Delete destroy) {
    if(texture_owned && texture){destroy(texture);texture=0;}
    texture_owned=texture_current=false;texture_failed=true;return false;
  }
};
struct RoutePoint {std::shared_ptr<Cache> m_skagerLabel=std::make_shared<Cache>();
  unsigned m_iTextTexture=31; int m_iTextTextureWidth=1,m_iTextTextureHeight=1;};
struct ViewPort {double chart_scale=10000;};
struct ChartCanvas {ViewPort vp; bool missing=false;
  ViewPort* GetpVP(){return missing?nullptr:&vp;}};
struct ocpnDC {bool software=true; int bitmaps=0,bounds=0;
  void* GetDC(){return software?this:nullptr;}
  void DrawBitmap(int,int,int,bool){++bitmaps;}
  void CalcBoundingBox(int,int){++bounds;}};
struct glChartCanvas {static void RenderSingleTexture(ocpnDC&,float*,float*,ViewPort*,int,int,int){++gl_draws;}};
namespace opennav::integration {
PRODUCTION_DRAW
}
int main() {
  using opennav::integration::DrawChartRouteLabel;
  int checks=0;
  const auto check=[&](bool ok,const char* why){++checks;if(!ok){std::cerr<<why<<'\n';std::exit(1);}};
  for(bool software:{true,false}) {
    ChartCanvas canvas; ocpnDC dc; dc.software=software; RoutePoint point;
    const auto cache=point.m_skagerLabel; const auto texture=point.m_iTextTexture;
    for(double scale:{149999.,150000.,599999.,600000.,1000000.,149999.}) {
      canvas.vp.chart_scale=scale;
      const int before=dc.bitmaps+gl_draws,old_bounds=dc.bounds;
      check(DrawChartRouteLabel(dc,canvas,point,10,20,true),"Owned name must not fall back to stock");
      const bool visible=scale<150000;
      check(dc.bitmaps+gl_draws-before==int(visible),"Route card must reduce and restore at scale boundary");
      check(dc.bounds-old_bounds==(visible?2:0),"Suppressed card must submit no draw bounds");
      check(point.m_skagerLabel==cache && point.m_iTextTexture==texture,
            "Zoom must retain card and texture ownership");
    }
    canvas.missing=true;
    int before=dc.bitmaps+gl_draws;
    check(DrawChartRouteLabel(dc,canvas,point,0,0,true) && dc.bitmaps+gl_draws==before+1,
          "Unavailable viewport must preserve full detail");
    canvas.missing=false;canvas.vp.chart_scale=std::numeric_limits<double>::quiet_NaN();
    before=dc.bitmaps+gl_draws;
    check(DrawChartRouteLabel(dc,canvas,point,0,0,true) && dc.bitmaps+gl_draws==before+1,
          "Unknown scale must preserve full detail");
    canvas.vp.chart_scale=1000000;
    const int old_deletes=deletes;
    check(!DrawChartRouteLabel(dc,canvas,point,0,0,false),"Ineligible custom name must retain stock fallback");
    if(!software)check(!point.m_skagerLabel && point.m_iTextTexture==0 && deletes==old_deletes+1,
                      "Wide-scale fallback must clean up old owned texture");
    point.m_skagerLabel.reset();
    check(!DrawChartRouteLabel(dc,canvas,point,0,0,true),"Missing cache must not suppress stock fallback");
  }
  check(uploads==0,"Wide-scale suppression and restoration must not upload textures");
  std::cout<<checks<<" production route-label declutter checks passed\n";
}
'''.replace('PRODUCTION_DRAW', draw)
with tempfile.TemporaryDirectory(prefix='skager-route-label-lod-') as temporary:
    directory = Path(temporary)
    test = directory / 'test.cpp'
    test.write_text(harness)
    exe = directory / 'test'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O2',
                    '-I' + str(root / 'src'), str(test), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

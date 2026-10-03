// Runs production painter code with real wx software rendering and a recording
// GL interface. The latter checks submission/state only, never driver output.
#include "integration/ChartRouteGeometry.h"
#include "reference-route.h"
#include <wx/wx.h>
#include <wx/graphics.h>
#include <memory>
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
class App : public wxApp { public: bool OnInit() override { return true; } };
wxIMPLEMENT_APP_NO_MAIN(App);
struct RoutePoint { wxString icon; wxString GetIconName() { return icon; } };
struct Node {
  RoutePoint point; Node *next = nullptr;
  Node *GetNext() { return next; } RoutePoint *GetData() { return &point; }
};
struct PointList { Node first; Node *GetFirst() { return &first; } };
constexpr int WIDTH_UNDEFINED = -1;
int g_route_line_width = 2;
struct Route {
  bool m_bVisible = true, m_bRtIsActive = true, m_bRtIsSelected = false,
       m_bIsBeingEdited = false;
  int m_hiliteWidth = 0, m_width = WIDTH_UNDEFINED;
  wxPenStyle m_style = wxPENSTYLE_INVALID;
  bool IsVisible() { return m_bVisible; }
  wxString m_Colour; PointList list; PointList *pRoutePointList = &list;
};
struct VP { float vp_matrix_transform[16]{}; };
struct ChartCanvas {
  int dpi = 100, theme = 0; VP vp;
  int FromDIP(int v) { return v*dpi/100; } VP *GetpVP() { return &vp; }
};
struct ocpnDC {
  wxDC *native = nullptr; int m_canvasIndex = 0;
  void GetSize(int *w,int *h) { *w=640; *h=360; }
  wxDC *GetDC() { return native; }
  void CalcBoundingBox(int x,int y) { if(native) native->CalcBoundingBox(x,y); }
};
#define ocpnUSE_GL
using GLint = int; using GLfloat = float;
constexpr int GL_CURRENT_PROGRAM=1, GL_BLEND=2, GL_TRIANGLES=3;
int program=17, draws=0, vertices=0; bool blend=true;
void glGetIntegerv(int,int *v) { *v=program; }
bool glIsEnabled(int) { return blend; }
void glEnable(int) { blend=true; } void glDisable(int) { blend=false; }
void glUseProgram(int v) { program=v; }
void glDrawArrays(int mode,int,int count) {
  if(mode!=GL_TRIANGLES || count%3) throw std::runtime_error("GL triangle topology");
  ++draws; vertices=count;
}
struct Shader {
  float color[4]{}; const float *positions=nullptr;
  void Bind() { program=29; } void UnBind() { program=0; }
  void SetUniformMatrix4fv(const char*,float*) {}
  void SetUniform4fv(const char*,float *v) { std::copy(v,v+4,color); }
  void SetAttributePointerf(const char*,float *v) { positions=v; }
};
Shader shader; Shader *pcolor_tri_shader_program[2]{&shader,&shader};
namespace opennav::integration {
bool enabled=true;
bool ChartActiveRouteInk(ChartCanvas &c,wxColour &ink) {
  if(!enabled) return false;
  const unsigned colors[]{0x267c76,0xb0dfc8,0x71937e}; auto v=colors[c.theme];
  ink=wxColour(v>>16,(v>>8)&255,v&255); return true;
}
#define max(a,b) WINDOWS_MAX_MACRO_MUST_NOT_EXPAND
#include "production-route.h"
#undef max
}
int checks=0;
void Check(bool value,const char *reason) {
  ++checks; if(!value) throw std::runtime_error(reason);
}
bool Contains(const std::vector<float>&m,double x,double y) {
  for(size_t i=0;i<m.size();i+=6) {
    double c[3];
    for(int j=0;j<3;++j) {
      int a=i+j*2,b=i+(j+1)%3*2;
      c[j]=(m[b]-m[a])*(y-m[a+1])-(m[b+1]-m[a+1])*(x-m[a]);
    }
    if(!((c[0]<0||c[1]<0||c[2]<0)&&(c[0]>0||c[1]>0||c[2]>0))) return true;
  }
  return false;
}
int main(int argc,char **argv) {
  if(!wxEntryStart(argc,argv)||!wxTheApp->CallOnInit()) return 2;
  try {
    using namespace opennav::integration;
    Route r;
    Check(DefaultChartRouteStyle(r),"untouched default eligible");
    for(bool *v : {&r.m_bRtIsSelected,&r.m_bIsBeingEdited}) {
      *v=true; Check(!DefaultChartRouteStyle(r),"special route state eligible"); *v=false;
    }
    r.m_bVisible=false; Check(!DefaultChartRouteStyle(r),"hidden eligible"); r.m_bVisible=true;
    r.m_bRtIsActive=false; Check(!DefaultChartRouteStyle(r),"inactive eligible"); r.m_bRtIsActive=true;
    r.m_width=2; Check(!DefaultChartRouteStyle(r),"explicit route width eligible"); r.m_width=-1;
    r.m_style=wxPENSTYLE_SOLID; Check(!DefaultChartRouteStyle(r),"explicit style eligible"); r.m_style=wxPENSTYLE_INVALID;
    r.m_Colour="Red"; Check(!DefaultChartRouteStyle(r),"custom color eligible"); r.m_Colour.clear();
    r.m_hiliteWidth=20; Check(!DefaultChartRouteStyle(r),"highlight eligible"); r.m_hiliteWidth=0;
    r.list.first.point.icon="mob"; Check(!DefaultChartRouteStyle(r),"manual/AIS MOB eligible"); r.list.first.point.icon.clear();
    g_route_line_width=3; Check(!DefaultChartRouteStyle(r),"global custom width eligible"); g_route_line_width=2;
    for(double dpi : {1.,1.25,1.5,2.}) {
      auto mesh=ChartRouteSegmentMesh(20,40,120,40,dpi,640,360,false,false);
      double lo=1e9,hi=-1e9; for(size_t i=1;i<mesh.size();i+=2){lo=std::min(lo,double(mesh[i]));hi=std::max(hi,double(mesh[i]));}
      Check(std::abs(hi-lo-reference_route_width*dpi)<1e-5,"fractional width lost");
      Check(!Contains(mesh,19.9,40)&&!Contains(mesh,120.1,40),"route end lost butt cap");
      auto round=ChartRouteSegmentMesh(20,40,120,40,dpi,640,360,true,true);
      Check(Contains(round,19.5,40)&&Contains(round,120.5,40),"round join missing");
      Check(!Contains(round,20-1.3*dpi,40-1.3*dpi),"join grew square corner");
      auto diagonal=ChartRouteSegmentMesh(30,30,130,130,dpi,640,360,false,false);
      Check(Contains(diagonal,80,80)&&!Contains(diagonal,80+2*dpi,80-2*dpi),"diagonal stroke extent wrong");
    }
    Check(ChartRouteSegmentMesh(2,2,2,2,1,640,360,true,true).empty(),"zero segment created artwork");
    for(double bad : {NAN,INFINITY}) Check(ChartRouteSegmentMesh(bad,0,1,1,1,640,360,false,false).empty(),"nonfinite accepted");
    auto clipped=ChartRouteSegmentMesh(-1e7,50,1e7,50,1,640,360,true,true);
    Check(!clipped.empty(),"visible crossing clipped away");
    for(float v:clipped) Check(std::isfinite(v)&&v>=-2&&v<=642,"clipping failed to bound geometry");
    Check(ChartRouteSegmentMesh(-100,-100,-20,-20,1,640,360,true,true).empty(),"offscreen segment retained");
    ChartCanvas canvas; ocpnDC gl;
    for(bool old_blend : {false,true}) {
      blend=old_blend; program=17;
      Check(DrawChartRouteSegment(gl,canvas,20,40,120,40,true,true),"GL path refused");
      Check(blend==old_blend&&program==17,"GL state leaked");
      Check(vertices==198&&shader.color[3]==1,"GL mesh/opacity changed");
    }
    enabled=false; const int before=draws;
    Check(!DrawChartRouteSegment(gl,canvas,20,40,120,40,true,true)&&draws==before,"Standard/Legacy/Safe painted"); enabled=true;
    wxInitAllImageHandlers(); wxBitmap bmp(640,360,24); wxMemoryDC target(bmp);
    target.SetBackground(wxBrush(wxColour(213,229,229)));target.Clear();
    ocpnDC dc; dc.native=&target;
    target.SetPen(wxPen(*wxRED,5)); target.SetBrush(*wxBLUE_BRUSH);
    for(int theme=0;theme<3;++theme) for(int dpi : {100,125,150}) {
      canvas.theme=theme;canvas.dpi=dpi;
      double y=30+theme*110+(dpi-100)*1.2;
      Check(DrawChartRouteSegment(dc,canvas,30,y,300,y,true,true),"software painter refused");
      Check(DrawChartRouteSegment(dc,canvas,300,y,350,y+25,true,false),"software turn refused");
    }
    Check(target.GetPen().GetWidth()==5&&target.GetPen().GetColour()==*wxRED&&target.GetBrush()==*wxBLUE_BRUSH,"software paint state leaked");
    target.SelectObject(wxNullBitmap); auto image=bmp.ConvertToImage();
    for(int theme=0;theme<3;++theme) {
      const unsigned expected[]{0x267c76,0xb0dfc8,0x71937e}; const int y=30+theme*110;
      unsigned pixel=(image.GetRed(100,y)<<16)|(image.GetGreen(100,y)<<8)|image.GetBlue(100,y);
      Check(pixel==expected[theme],"theme foreground ink changed");
    }
    Check(image.SaveFile(argv[1],wxBITMAP_TYPE_PNG),"could not save painter evidence");
    const auto started=std::chrono::steady_clock::now();
    size_t floats=0;
    for(int i=0;i<10000;++i) floats+=ChartRouteSegmentMesh(30,30,600,320,1.5,1920,1080,true,true).size();
    auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    std::cout<<checks<<" checks passed; 10000 bounded meshes "<<ms<<" ms; "<<floats<<" floats (no raster/texture allocation)\n";
  } catch(const std::exception &e) { std::cerr<<e.what()<<"\n"; return 1; }
  wxTheApp->OnExit();wxEntryCleanup();return 0;
}

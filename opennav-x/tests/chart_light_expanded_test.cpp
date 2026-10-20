#include <GL/glew.h>
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <array>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include "integration/ChartCaFan.h"
#include "integration/ChartLightHover.h"
#include "S57Sector.h"
#include "linmath.h"

using namespace opennav::integration;
constexpr double PI=3.14159265358979323846;
static unsigned checks=0;
void Check(bool value,const char* why){++checks;if(!value)throw std::runtime_error(why);}
static bool active=true;
namespace opennav::integration {bool XNavChartPresentationActive(){return active;}}
struct Library {wxString scheme="DAY";wxString GetPLIBColorScheme(){return scheme;}} library,*ps52plib=&library;
struct Request {double lat,lon,bearing,range;};
static std::vector<Request> requests;
void ll_gc_ll(double lat,double lon,double bearing,double range,double* y,double* x) {
  requests.push_back({lat,lon,bearing,range});
  *x=lon+range*std::sin(bearing*PI/180);*y=lat-range*std::cos(bearing*PI/180);
}
struct ViewPort {
  int pix_width=400,pix_height=400;double rotation=0;
  wxPoint GetPixFromLL(double y,double x) {
    return {int(std::lround(200+(x-200)*std::cos(rotation)-(y-200)*std::sin(rotation))),
            int(std::lround(200+(x-200)*std::sin(rotation)+(y-200)*std::cos(rotation)))};
  }
};
struct ocpnDC {
  wxDC* dc=nullptr;unsigned lines=0,arcs=0,cones=0;
  wxDC* GetDC(){return dc;}
  void SetPen(const wxPen& pen){if(dc)dc->SetPen(pen);}
  void SetBrush(const wxBrush& brush){if(dc)dc->SetBrush(brush);}
  void StrokeLines(int n,wxPoint* points){++arcs;if(dc)dc->DrawLines(n,points);}
  void StrokePolygon(int n,wxPoint* points,int x,int y){++cones;if(dc)dc->DrawPolygon(n,points,x,y);}
  void StrokeLine(wxPoint a,wxPoint b){++lines;if(dc)dc->DrawLine(a,b);}
};
GLint S52texture_2D_shader_program=0;
// Stock GL arc calls are counted and exercised only with the no-paint program;
// the production overlay uses the actual pinned texture shader below.
struct GLShaderProgram {
  unsigned binds=0;void Bind(){++binds;glUseProgram(0);}
  void UnBind(){}GLuint programId(){return 0;}
} ring,*pring_shader_program[]={&ring};
#include "extended-methods.inc"
#include "shaders.inc"
#include "fixture-gl.inc"
class App:public wxApp {public:bool OnInit()override{return true;}};
wxIMPLEMENT_APP_NO_MAIN(App);

int main(int argc,char** argv) {
  try {
    const std::string out=argv[1];
    Check(wxEntryStart(argc,argv),"wx start");Check(wxTheApp->CallOnInit(),"wx init");wxInitAllImageHandlers();
    auto* frame=new wxFrame(nullptr,wxID_ANY,"Expanded sector fixture",wxDefaultPosition,wxSize(400,400));
    int attrs[]={WX_GL_RGBA,WX_GL_DOUBLEBUFFER,WX_GL_DEPTH_SIZE,0,0};
    auto* canvas=new wxGLCanvas(frame,wxID_ANY,attrs);frame->Show();wxTheApp->Yield();
    wxGLContext context(canvas);Check(context.IsOK() && canvas->SetCurrent(context),"GL context");
    glewExperimental=GL_TRUE;const auto glew=glewInit();
    Check(glew==GLEW_OK || glew==GLEW_ERROR_NO_GLX_DISPLAY,"GLEW");
    while(glGetError()!=GL_NO_ERROR){}
    S52texture_2D_shader_program=Program(S52texture_2D_vertex_shader_source,S52texture_2D_fragment_shader_source);
    std::cout<<"renderer="<<glGetString(GL_RENDERER)<<"\n";
    ViewPort viewport;
    for(const auto* theme:{"DAY","DUSK","NIGHT"})for(unsigned color:{1,3,4})for(double rotation:{0.,.43,-1.2}) {
      library.scheme=theme;viewport.rotation=rotation;
      s57Sector_t sector{};sector.pos={200,200};sector.sector1=350;sector.sector2=410;sector.range=120;
      sector.color=color==3?wxColour(255,0,0,100):color==4?wxColour(0,255,0,100):wxColour(255,255,0,130);
      sector.iswhite=color==1;std::vector<s57Sector_t> sectors{sector};
      wxBitmap bitmap(400,400,24);wxMemoryDC memory(bitmap);
      memory.SetBackground(wxBrush(wxColour(51,77,102)));memory.Clear();ocpnDC sw{&memory};
      requests.clear();s57_DrawExtendedLightSectors(sw,viewport,sectors);
      Check(!sw.lines && !sw.arcs && !sw.cones,"eligible software uses complete overlay");
      Check(requests.size()==2 && requests[0].bearing==530 && requests[1].bearing==590 &&
            requests[0].range==120 && requests[1].range==120,"native bearings and ranges feed projection");
      memory.SelectObject(wxNullBitmap);auto software=bitmap.ConvertToImage();
      ocpnDC gl;ClearGL();s57_DrawExtendedLightSectorsGL(gl,viewport,sectors);glFinish();
      Check(!gl.lines && !gl.cones && !ring.binds,"eligible GL uses complete overlay");
      Same(software,ReadGL(),1);
      Check(sectors[0].sector1==350 && sectors[0].sector2==410 && sectors[0].range==120 &&
            sectors[0].color==sector.color && !sectors[0].isleading,"source sector unchanged");
      if(rotation==0)software.SaveFile(out+"/"+theme+"-"+std::to_string(color)+".png",wxBITMAP_TYPE_PNG);
    }
    viewport.rotation=0;library.scheme="DAY";
    s57Sector_t sector{};sector.pos={200,200};sector.sector1=0;sector.sector2=40;sector.range=120;
    sector.color=wxColour(255,0,0,100);std::vector<s57Sector_t> sectors{sector};
    wxBitmap bitmap(400,400,24);wxMemoryDC memory(bitmap);ocpnDC sw{&memory};
    active=false;s57_DrawExtendedLightSectors(sw,viewport,sectors);
    Check(sw.arcs==1 && sw.lines==2,"Legacy retains native arc and rays");active=true;
    sectors[0].isleading=true;sw={&memory};s57_DrawExtendedLightSectors(sw,viewport,sectors);
    Check(sw.cones==1 && sw.lines==2 && !sw.arcs,"leading cone retains native branch");
    ocpnDC gl;s57_DrawExtendedLightSectorsGL(gl,viewport,sectors);
    Check(gl.cones==1 && gl.lines==2,"GL leading cone retains native branch");
    sectors[0].isleading=false;sectors[0].color=wxColour(2,3,4,55);sw={&memory};
    s57_DrawExtendedLightSectors(sw,viewport,sectors);
    Check(sw.arcs==1 && sw.lines==2,"unknown class falls back before painting");
    sectors[0]=sector;sw={&memory};viewport.pix_width=9000;
    s57_DrawExtendedLightSectors(sw,viewport,sectors);
    Check(sw.arcs==1 && sw.lines==2,"work refusal takes complete native path");
    viewport.pix_width=400;
    // A painted ordinary sector owns its shared ray before a leading neighbour.
    sectors[0].sector2=90;sectors.push_back(sector);sectors[1].isleading=true;
    sectors[1].sector1=90;sectors[1].sector2=120;sw={&memory};
    s57_DrawExtendedLightSectors(sw,viewport,sectors);
    Check(sw.cones==1 && sw.lines==1 && !sw.arcs,"mixed sectors retain native shared-ray ownership");
    memory.SelectObject(wxNullBitmap);
    CaFanPaint paint;Check(CaFanColors(3,"DAY",paint),"red paint");
    CaFanTile tile;CaFanGeometry geometry{{200,200},{200,320},{80,200},40,180,270,1};
    Check(BuildExtendedLightSectorTile(tile,geometry,paint,400,400,100,false,false,true,true),"direct expanded tile");
    const auto alpha=[&](int x,int y){return tile.image.GetAlpha(x-tile.bounds.x,y-tile.bounds.y);};
    int dash=0,gap=0;for(int x=198;x<=202;++x){dash+=alpha(x,271);gap+=alpha(x,275);}
    Check(dash>50 && gap==0,"three/four dash phase retained beyond arc");
    Check(alpha(185,215)==18,"expanded red wash is seven percent");
    Check(!alpha(220,220),"no wash outside native angular sector");
    Check(BuildExtendedLightSectorTile(tile,geometry,paint,400,400,20,false,true,true,true),"Night tile");
    Check(alpha(185,215)==20,"Night wash capped by native opacity");
    for(int y=0;y<tile.image.GetHeight();++y)for(int x=0;x<tile.image.GetWidth();++x)
      if(tile.image.GetAlpha(x,y)>20)throw std::runtime_error("Night layer overlap exceeds native opacity");
    ++checks;
    const auto start=std::chrono::steady_clock::now();
    Check(BuildExtendedLightSectorTile(tile,{{200,200},{-9000000,200},{200,9000000},40,180,270,1},
          paint,400,400,100,false,false,true,true),"distant endpoints clip raster work");
    Check(tile.image.GetWidth()<=400 && tile.image.GetHeight()<=400,"clipped allocation bound");
    Check(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<2,"distant rays bounded work");
    Check(!BuildExtendedLightSectorTile(tile,{{4000,4000},{0,0},{8191,8191},2000,180,270,1},
          paint,8192,8192,100,false,false,true,true) && !tile.image.IsOk(),"allocation refusal leaves no partial image");
    std::cout<<"checks="<<checks<<" actual software/GL methods and bounded overlay passed\n";
    glDeleteProgram(S52texture_2D_shader_program);frame->Destroy();wxTheApp->Yield();
    wxTheApp->OnExit();wxEntryCleanup();return 0;
  } catch(const std::exception& error){std::cerr<<error.what()<<"\n";return 1;}
}

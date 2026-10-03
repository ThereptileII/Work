// Runs production painter code with real wx software rendering and a recording
// GL interface. The latter checks submission/state only, never driver output.
#include "integration/ChartCogPredictor.h"
#include "reference-cog.h"
#include <wx/wx.h>
#include <wx/graphics.h>
#include <wx/fileconf.h>
#include <wx/sstream.h>
#include <memory>
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
class App : public wxApp { public: bool OnInit() override { return true; } };
wxIMPLEMENT_APP_NO_MAIN(App);
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
constexpr int GL_CURRENT_PROGRAM=1, GL_BLEND=2, GL_TRIANGLES=3,
GL_BLEND_SRC_RGB=4,GL_BLEND_DST_RGB=5,GL_BLEND_SRC_ALPHA=6,GL_BLEND_DST_ALPHA=7,
GL_BLEND_EQUATION_RGB=8,GL_BLEND_EQUATION_ALPHA=9,GL_SRC_ALPHA=10,
GL_ONE_MINUS_SRC_ALPHA=11,GL_FUNC_ADD=12;
int src_rgb=21,dst_rgb=22,src_alpha=23,dst_alpha=24,eq_rgb=25,eq_alpha=26;
void glBlendFunc(int a,int b) {src_rgb=src_alpha=a;dst_rgb=dst_alpha=b;}
void glBlendFuncSeparate(int a,int b,int c,int d){src_rgb=a;dst_rgb=b;src_alpha=c;dst_alpha=d;}
void glBlendEquationSeparate(int a,int b){eq_rgb=a;eq_alpha=b;}
int program=17, draws=0, vertices=0; bool blend=true;
void glGetIntegerv(int name,int *v) {
  switch(name) {case GL_CURRENT_PROGRAM:*v=program;break;
  case GL_BLEND_SRC_RGB:*v=src_rgb;break;case GL_BLEND_DST_RGB:*v=dst_rgb;break;
  case GL_BLEND_SRC_ALPHA:*v=src_alpha;break;case GL_BLEND_DST_ALPHA:*v=dst_alpha;break;
  case GL_BLEND_EQUATION_RGB:*v=eq_rgb;break;case GL_BLEND_EQUATION_ALPHA:*v=eq_alpha;break;}
}
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
bool enabled=true,xnav_mode=true,active=true;
CogPredictorStyleOwnership cog_style;
bool ChartActiveRouteInk(ChartCanvas &c,wxColour &ink) {
  if(!enabled) return false;
  const unsigned colors[]{0x267c76,0xb0dfc8,0x71937e}; auto v=colors[c.theme];
  ink=wxColour(v>>16,(v>>8)&255,v&255); return true;
}
#define max(a,b) WINDOWS_MAX_MACRO_MUST_NOT_EXPAND
#include "production-cog.h"
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
    wxStringInputStream empty(""); wxFileConfig config(empty);
    auto capture=[&]{CaptureChartCogPredictorStyle(config);};
    auto owns=[&](int width=3,int density=3,int style=105,const wxString& color="rgb(255,0,0)") {
      return UseChartCogPredictorStyle(width,style,color,density);
    };
    capture();Check(owns(),"fresh factory paint not owned");
    config.Write("/Settings/OwnshipCOGPredictorWidth",3L);
    config.Write("/Settings/OwnshipCOGPredictorStyle",105L);
    config.Write("/Settings/OwnshipCOGPredictorColor","rgb(255,0,0)");
    capture();Check(owns(),"saved factory appearance not owned");
    Check(owns(3,5)&&owns(5,5)&&owns(5,2),"density mutation incorrectly treated as preference change");
    Check(!owns(6,5)&&!owns(5,5),"runtime custom width did not revoke ownership");
    capture();Check(!owns(3,3,100)&&!owns(),"runtime custom style did not revoke");
    capture();Check(!owns(3,3,105,"blue")&&!owns(),"runtime custom color did not revoke");
    config.Write("/Settings/OwnshipCOGPredictorWidth",4L);capture();Check(!owns(4),"saved custom width owned");
    config.Write("/Settings/OwnshipCOGPredictorWidth","invalid");capture();Check(!owns(),"invalid width owned");
    config.Write("/Settings/OwnshipCOGPredictorWidth",3L);
    config.Write("/Settings/OwnshipCOGPredictorStyle",100L);capture();Check(!owns(3,3,100),"saved custom style owned");
    config.Write("/Settings/OwnshipCOGPredictorStyle",105L);
    config.Write("/Settings/OwnshipCOGPredictorColor","blue");capture();Check(!owns(3,3,105,"blue"),"saved custom color owned");
    config.Write("/Settings/OwnshipCOGPredictorColor","rgb(255,0,0)");capture();
    xnav_mode=false;Check(!owns(),"Legacy/Safe owns predictor");xnav_mode=true;
    active=false;Check(!owns(),"Standard/unverified owns predictor");active=true;
    Check(config.ReadLong("/Settings/OwnshipCOGPredictorWidth",-1)==3,"paint gate changed persisted width");
    for(double dpi : {1.,1.25,1.5,2.}) {
      auto mesh=ChartCogPredictorMesh(20,40,120,40,dpi,640,360);
      double lo=1e9,hi=-1e9;
      for(size_t i=1;i<mesh.size();i+=2){lo=std::min(lo,double(mesh[i]));hi=std::max(hi,double(mesh[i]));}
      Check(std::abs(hi-lo-ref_width*dpi)<1e-5,"fractional COG width changed");
      Check(Contains(mesh,20+ref_dash*dpi/2,40),"first dash missing");
      Check(!Contains(mesh,20+(ref_dash+ref_gap/2)*dpi,40),"gap filled");
      Check(Contains(mesh,20+(ref_dash+ref_gap+ref_dash/2)*dpi,40),"second dash phase wrong");
      Check(!Contains(mesh,19.9,40),"start cap exceeds real origin");
    }
    auto reversed=ChartCogPredictorMesh(120,40,20,40,1,640,360);
    Check(Contains(reversed,117,40)&&!Contains(reversed,112,40),"reversed real bearing changed dash phase");
    auto clipped=ChartCogPredictorMesh(-7,50,100,50,1,640,360);
    Check(!Contains(clipped,1,50)&&Contains(clipped,4,50),"clipping reset dash phase");
    auto diagonal=ChartCogPredictorMesh(30,30,130,130,1,640,360);
    Check(Contains(diagonal,31,31)&&!Contains(diagonal,35,35),"diagonal dash distance wrong");
    bool valid=false; auto gap=ChartCogPredictorMesh(-7,50,2,50,1,640,360,&valid);
    Check(valid&&gap.empty(),"offscreen/gap distinguishable from invalid geometry");
    for(double bad : {NAN,INFINITY}) Check(ChartCogPredictorMesh(bad,0,1,1,1,640,360).empty(),"nonfinite accepted");
    Check(ChartCogPredictorMesh(2,2,2,2,1,640,360).empty(),"zero predictor drawn");
    auto huge=ChartCogPredictorMesh(-1e8,50,1e8,50,1,640,360);
    Check(huge.size()<1000,"mesh grows with offscreen length");
    ChartCanvas canvas;ocpnDC gl;
    for(bool old_blend : {false,true}) {
      blend=old_blend;program=17;
      Check(DrawChartCogPredictor(gl,canvas,20,40,120,40),"GL painter refused");
      Check(blend==old_blend&&program==17&&src_rgb==21&&dst_rgb==22&&src_alpha==23&&dst_alpha==24&&eq_rgb==25&&eq_alpha==26,"GL state leaked");
      Check(vertices==60&&std::abs(shader.color[3]-ref_alpha)<1e-6,"GL dash count/alpha changed");
    }
    int before=draws;Check(DrawChartCogPredictor(gl,canvas,-7,50,2,50)&&draws==before,"empty visible dash fell back to a solid line");
    enabled=false;Check(!DrawChartCogPredictor(gl,canvas,20,40,120,40),"disabled presentation drew");enabled=true;
    wxInitAllImageHandlers();wxBitmap bmp(640,360,24);wxMemoryDC target(bmp);
    const wxColour backdrop(213,229,229);target.SetBackground(wxBrush(backdrop));target.Clear();
    target.SetPen(wxPen(*wxRED,5));target.SetBrush(*wxBLUE_BRUSH);
    ocpnDC dc;dc.native=&target;
    for(int theme=0;theme<3;++theme) for(int dpi : {100,125,150}) {
      canvas.theme=theme;canvas.dpi=dpi;
      double y=30.5+theme*110+(dpi-100)*1.2;
      Check(DrawChartCogPredictor(dc,canvas,30,y,420,y),"software painter refused");
    }
    Check(target.GetPen().GetWidth()==5&&target.GetBrush()==*wxBLUE_BRUSH,"software state leaked");
    target.SelectObject(wxNullBitmap);auto image=bmp.ConvertToImage();
    for(int theme=0;theme<3;++theme) {
      int y=30+theme*110;const unsigned colors[]{0x267c76,0xb0dfc8,0x71937e};
      unsigned color=colors[theme];int channels[]{int(color>>16),int((color>>8)&255),int(color&255)};
      int bg[]{213,229,229},actual[]{image.GetRed(32,y),image.GetGreen(32,y),image.GetBlue(32,y)};
      for(int i=0;i<3;++i) Check(std::abs(actual[i]-std::lround(channels[i]*(166./255)+bg[i]*(89./255)))<=1,"software alpha composition changed");
      Check(image.GetRed(37,y)==213&&image.GetGreen(37,y)==229&&image.GetBlue(37,y)==229,"software dash gap painted");
    }
    Check(image.SaveFile(argv[1],wxBITMAP_TYPE_PNG),"cannot save painter evidence");
    auto began=std::chrono::steady_clock::now();size_t count=0;
    for(int i=0;i<10000;++i) count+=ChartCogPredictorMesh(-1e8,50,1e8,50,1,1920,1080).size();
    std::cout<<checks<<" checks passed; 10000 clipped long predictors "<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()<<" ms; "<<count<<" floats\n";
  }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
  wxTheApp->OnExit();wxEntryCleanup();return 0;
}

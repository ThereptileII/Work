#include <GL/glew.h>
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <chrono>
#include <iostream>
#include <array>
#include <stdexcept>
#include "integration/ChartCableWave.h"
#include "Cs52_shaders.h"
#include "line_clip.h"
#include "linmath.h"
using namespace opennav::integration;
constexpr double PI=3.14159265358979323846;
static int checks=0;void Check(bool b,const char*s){++checks;if(!b)throw std::runtime_error(s);}
const GLchar* Cpreamble="#version 120\n#define highp\n#define mediump\n#define lowp\n";
CGLShaderProgram* pCcolor_tri_shader_program[2]={};
GLint S52texture_2D_shader_program=0,S52ring_shader_program=0;
float g_scaminScale=1;
// The actual traversal method is executed verbatim. Fallback records its full
// HPGL call arguments; this test does not claim stock HPGL raster acceptance.
struct Fallback {
 std::vector<std::string> calls;
 void SetTargetDC(wxDC*){} void SetTargetOpenGl(){} template<class V>void SetVP(V*){}
 void Render(char*s,char*c,wxPoint&r,wxPoint&p,wxPoint o,float scale,double angle,bool symbol){
  calls.push_back(std::string(s)+"|"+c+"|"+std::to_string(r.x)+","+std::to_string(r.y)+"|"+std::to_string(p.x)+","+std::to_string(p.y)+"|"+std::to_string(o.x)+","+std::to_string(o.y)+"|"+std::to_string(scale)+"|"+std::to_string(angle)+"|"+std::to_string(symbol));
 }
};
class s52plib {
 public:
 bool m_presentationLightSymbols=true,m_GLLineSmoothing=false;float m_GLMinCartographicLineWidth=1;
 wxDC*m_pdc=nullptr;Fallback*HPGL;double ppmm=96.0/25.4;
 struct View {double clat=0,clon=0;int pix_width=400,pix_height=400;wxRect rv_rect{0,0,400,400};}vp_plib;
 wxPoint GetPixFromLL(double,double){return{200,200};}float GetPPMM(){return ppmm;}
 void draw_lc_poly(wxDC*,wxColor&,int,wxPoint*,int*,int,float,float,Rule*);
 void stock_lc_poly(wxDC*,wxColor&,int,wxPoint*,int*,int,float,float,Rule*);
};
#include "cable-methods.inc"
#include "shaders.inc"
#include "matrix.inc"
#include "ca-upload-before.inc"
class App:public wxApp{public:bool OnInit()override{return true;}};wxIMPLEMENT_APP_NO_MAIN(App);
GLuint Program(const char*vs,const char*fs){
 auto c=[](GLenum type,const char*source){GLuint s=glCreateShader(type);const char*prefix="#version 120\n#define highp\n#define mediump\n#define lowp\n";std::string body=source;auto p=body.find("precision highp float;");if(p!=std::string::npos)body.erase(p,22);const char*t[]={prefix,body.c_str()};glShaderSource(s,2,t,nullptr);glCompileShader(s);GLint ok;glGetShaderiv(s,GL_COMPILE_STATUS,&ok);if(!ok){char log[4096];glGetShaderInfoLog(s,4096,nullptr,log);throw std::runtime_error(log);}return s;};
 GLuint p=glCreateProgram(),v=c(GL_VERTEX_SHADER,vs),f=c(GL_FRAGMENT_SHADER,fs);glAttachShader(p,v);glAttachShader(p,f);glLinkProgram(p);GLint ok;glGetProgramiv(p,GL_LINK_STATUS,&ok);Check(ok,"shader link");glDeleteShader(v);glDeleteShader(f);return p;
}
void Clear(){glViewport(0,0,400,400);glClearColor(51.f/255,77.f/255,102.f/255,1);glClear(GL_COLOR_BUFFER_BIT);}
wxImage Read(){wxImage i(400,400);glPixelStorei(GL_PACK_ALIGNMENT,1);glReadPixels(0,0,400,400,GL_RGB,GL_UNSIGNED_BYTE,i.GetData());return i.Mirror(false);}
Rule Owned(){Rule r{};r.RCID=2012;std::memcpy(r.name.LINM,"CBLSUB06",8);r.colRef.LCRF=const_cast<char*>("AXNCBL");r.vector.LVCT=const_cast<char*>(kCableWaveHpgl);r.pos.line.bnbox_w.PAHL=635;r.pos.line.bnbox_h.PAVL=168;r.pos.line.bnbox_y.SBXR=-84;return r;}
void Matrix(GLuint shader,double rotation){GLfloat m[16],i[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};ActualMatrix(m,400,400,rotation);glUseProgram(shader);glUniformMatrix4fv(glGetUniformLocation(shader,"MVMatrix"),1,GL_FALSE,m);glUniformMatrix4fv(glGetUniformLocation(shader,"TransformMatrix"),1,GL_FALSE,i);glUseProgram(0);}
int main(int argc,char**argv){try{
 Check(argc==2,"output arg");std::string output=argv[1];Check(wxEntryStart(argc,argv),"wx start");Check(wxTheApp->CallOnInit(),"wx init");wxInitAllImageHandlers();
 auto*f=new wxFrame(nullptr,wxID_ANY,"Owned cable fixture",wxDefaultPosition,wxSize(400,400));int attrs[]={WX_GL_RGBA,WX_GL_DOUBLEBUFFER,WX_GL_DEPTH_SIZE,0,0};auto*c=new wxGLCanvas(f,wxID_ANY,attrs);f->Show();wxTheApp->Yield();wxGLContext context(c);Check(context.IsOK()&&c->SetCurrent(context),"GL context");glewExperimental=GL_TRUE;auto glew=glewInit();Check(glew==GLEW_OK||glew==GLEW_ERROR_NO_GLX_DISPLAY,"GLEW");while(glGetError()!=GL_NO_ERROR){}
 S52texture_2D_shader_program=Program(S52texture_2D_vertex_shader_source,S52texture_2D_fragment_shader_source);
 std::string fragment=S52color_tri_fragment_shader_source;auto precision=fragment.find("precision highp float;");if(precision!=std::string::npos)fragment.erase(precision,22);
 CGLShaderProgram line;Check(line.addShaderFromSource(S52color_tri_vertex_shader_source,GL_VERTEX_SHADER),"line vertex");Check(line.addShaderFromSource(fragment,GL_FRAGMENT_SHADER),"line fragment");Check(line.linkProgram(),"line link");pCcolor_tri_shader_program[0]=&line;S52ring_shader_program=line.programId();
 Matrix(line.programId(),0);Matrix(S52texture_2D_shader_program,0);Clear();
 Rule rule=Owned();Check(CableWaveRule(true,&rule),"owned exact signature");Check(!CableWaveRule(false,&rule)&&!CableWaveRule(true,nullptr),"disabled/null refusal");
 for(int n=0;n<7;++n){Rule wrong=rule;if(n==0)wrong.RCID++;if(n==1)wrong.pos.line.bnbox_w.PAHL++;if(n==2)wrong.pos.line.pivot_x.PACL++;if(n==3)wrong.pos.line.maxDist.PAMA++;if(n==4)wrong.colRef.LCRF=const_cast<char*>("ACHMGD");if(n==5)wrong.vector.LVCT=const_cast<char*>("SPA;SW1;");if(n==6)wrong.name.LINM[0]='Q';Check(!CableWaveRule(true,&wrong),"wrong owned signature");}
 CableWaveTile wave;Check(wave.Prepare(1,0,wxColour(156,134,150),{100,100}),"build exact wave");Check(wave.tile.bounds.width==30&&wave.tile.bounds.height==16,"conservative curve hull plus stroke bounds");
 const auto*cached=wave.tile.image.GetData();Check(wave.Prepare(1,0,wxColour(156,134,150),{130,100})&&wave.tile.image.GetData()==cached,"local repeated tile cache");
 for(double scale:{0.,.24,8.01,std::numeric_limits<double>::infinity()})Check(!wave.Prepare(scale,0,*wxWHITE,{0,0}),"out-of-cap scale refuses");
 // Actual method: original fallback call sequences for disabled/foreign metadata,
 // forward/reversed/masked/invisible/short/remainder boundaries, SW and real GL.
 wxColour cableInk(156,134,150);Fallback fallback;s52plib p;p.HPGL=&fallback;
 for(bool gl:{false,true})for(int shape=0;shape<6;++shape){
  wxPoint points[3]={{30,100},{170,100},{200,140}};int mask[3]={1,1,1};int count=3;
  if(shape==1)std::reverse(points,points+3);
  if(shape==2)mask[0]=0;
  if(shape==3){points[0]={-500,-500};points[1]={-400,-500};count=2;}if(shape==4){points[1]={45,100};count=2;}if(shape==5){points[1]={80,100};count=2;}
  wxBitmap bitmap(400,400);wxMemoryDC dc(bitmap);p.m_pdc=gl?nullptr:&dc;auto*target=gl?nullptr:&dc;
  p.m_presentationLightSymbols=false;fallback.calls.clear();p.stock_lc_poly(target,cableInk,1,points,mask,count,24,1,&rule);auto expected=fallback.calls;
  fallback.calls.clear();p.draw_lc_poly(target,cableInk,1,points,mask,count,24,1,&rule);Check(expected==fallback.calls,"disabled actual traversal identical");
  p.m_presentationLightSymbols=true;Rule foreign=rule;foreign.RCID=2013;fallback.calls.clear();p.draw_lc_poly(target,cableInk,1,points,mask,count,24,1,&foreign);Check(expected==fallback.calls,"foreign rule actual fallback identical");
  fallback.calls.clear();p.draw_lc_poly(target,cableInk,1,points,mask,count,24,1,&rule);Check(fallback.calls.empty(),"eligible motif draws without fallback");
  g_scaminScale=.1f;fallback.calls.clear();p.draw_lc_poly(target,cableInk,1,points,mask,count,24,1,&rule);Check(expected==fallback.calls,"tile refusal preserves original fallback arguments");g_scaminScale=1;
 }
 for(int theme=0;theme<3;++theme){wxColour ink=theme==0?wxColour(156,134,150):theme==1?wxColour(184,160,177):wxColour(113,99,110);
  for(int orientation=0;orientation<2;++orientation){double angle=orientation?.73:0;Check(wave.Prepare(1,angle,ink,{140,180}),"themed tile");
   wxBitmap bm(400,400);wxMemoryDC dc(bm);dc.SetBackground(wxBrush(wxColour(51,77,102)));dc.Clear();Check(wave.tile.Draw(dc),"SW tile");dc.SelectObject(wxNullBitmap);auto sw=bm.ConvertToImage();
   Matrix(line.programId(),0);Clear();Check(DrawCableWaveGL(wave.tile,S52texture_2D_shader_program,line.programId(),400,400),"GL tile");auto gl=Read();int maxDelta=0;for(int n=0;n<400*400*3;++n)maxDelta=std::max(maxDelta,std::abs(int(sw.GetData()[n])-int(gl.GetData()[n])));Check(maxDelta<=1,"same straight RGBA SW/GL blend");
   if(!orientation){sw.SaveFile(output+"/wave-"+std::to_string(theme)+"-software.png",wxBITMAP_TYPE_PNG);gl.SaveFile(output+"/wave-"+std::to_string(theme)+"-opengl.png",wxBITMAP_TYPE_PNG);}
  }
 }
 // Borrow an intentionally rotated actual upstream matrix, then compare to
 // the same tile drawn through that matrix directly using the shared helper.
 Matrix(line.programId(),.41);Clear();Check(DrawCableWaveGL(wave.tile,S52texture_2D_shader_program,line.programId(),400,400),"loaded matrix draw");auto rotated=Read();GLfloat mv[16],tr[16];glGetUniformfv(line.programId(),glGetUniformLocation(line.programId(),"MVMatrix"),mv);glGetUniformfv(line.programId(),glGetUniformLocation(line.programId(),"TransformMatrix"),tr);Clear();Check(DrawCaFanGL(wave.tile,S52texture_2D_shader_program,400,400,mv,tr),"direct actual matrix oracle");auto same=Read();Check(!std::memcmp(rotated.GetData(),same.GetData(),400*400*3),"exact loaded-matrix equivalence");
 auto&tile=wave.tile;
#include "cable-state.inc"
 // Keep the existing CA default matrix path covered after the optional inputs.
 Clear();Check(DrawCaFanGL(tile,S52texture_2D_shader_program,400,400),"unchanged CA default draw");auto caNew=Read();Clear();Check(DrawCaFanGLBefore(tile,S52texture_2D_shader_program,400,400),"original CA upload");auto caOld=Read();Check(!std::memcmp(caNew.GetData(),caOld.GetData(),400*400*3),"original CA default pixels exact");
 for(double scale:{1.,8.}){
  auto start=std::chrono::steady_clock::now();size_t pixels=0;double worstBuild=0;
  for(int i=0;i<200;++i){auto began=std::chrono::steady_clock::now();CableWaveTile fresh;Check(fresh.Prepare(scale,i*PI/199,wxColour(156,134,150),{100,100}),"bounded fresh build");auto us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-began).count();worstBuild=std::max(worstBuild,us);pixels=std::max(pixels,size_t(fresh.tile.image.GetWidth())*fresh.tile.image.GetHeight());}
  auto us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
  Check(wave.Prepare(scale,PI/4,wxColour(156,134,150),{100,100}),"upload scale");Matrix(line.programId(),0);Clear();glFinish();start=std::chrono::steady_clock::now();for(int i=0;i<20;++i)Check(DrawCableWaveGL(wave.tile,S52texture_2D_shader_program,line.programId(),400,400),"bounded upload");glFinish();auto upload=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count()/20;
  std::cout<<"scale="<<scale<<" angleRange=0..pi maxPixels="<<pixels<<" averageBuildUs="<<us/200<<" slowestBuildUs="<<worstBuild<<" averageMesaUploadFinishUs="<<upload<<"\n";
 }
 {wxBitmap bm(400,400);wxMemoryDC dc(bm);p.m_pdc=&dc;p.m_presentationLightSymbols=false;wxPoint points[]={{10,10},{11,10}};
  for(bool old:{true,false}){auto start=std::chrono::steady_clock::now();for(int i=0;i<1000;++i){if(old)p.stock_lc_poly(&dc,cableInk,1,points,nullptr,2,24,1,&rule);else p.draw_lc_poly(&dc,cableInk,1,points,nullptr,2,24,1,&rule);}std::cout<<(old?"original":"disabledHook")<<" shortSWMeanUs="<<std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count()/1000<<"\n";}
 }
 std::cout<<checks<<" actual traversal/helper/Mesa checks passed; fallback HPGL is call-observed, not raster-qualified\n";
 glDeleteProgram(S52texture_2D_shader_program);glDeleteProgram(line.programId());f->Destroy();wxTheApp->Yield();wxTheApp->OnExit();wxEntryCleanup();return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}

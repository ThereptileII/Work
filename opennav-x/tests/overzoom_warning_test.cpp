#include <GL/glew.h>
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <wx/fontenum.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include "linmath.h"
#include "ui/Theme.h"
static int checks=0;
void Check(bool v,const char* s){++checks;if(!v)throw std::runtime_error(s);}
static wxString translation="OverZoom";
#undef _
#define _(s) translation
#include "preamble.inc"
#include "shader-class.inc"
#include "shaders.inc"
GLShaderProgram *pcolor_tri_shader_program[2]{},*ptexture_2D_shader_program[2]{};
float g_GLMinSymbolLineWidth=1;
struct {bool m_GLLineSmoothing=false;}g_GLOptions;
struct VP {double ref_scale=390,chart_scale=100;float vp_matrix_transform[16];};
struct ChartBase {virtual ~ChartBase()=default;};
struct ChartMbTiles:ChartBase {double factor=0;int calls=0;double GetZoomFactor(){++calls;return factor;}};
enum ChartTypeEnum {CHART_TYPE_UNKNOWN,CHART_TYPE_MBTILES};
struct ChartTableEntry {int type=CHART_TYPE_UNKNOWN;int GetChartType()const{return type;}};
struct DB {ChartTableEntry entry;const ChartTableEntry& GetChartTableEntry(int){return entry;}} database,*ChartData=&database;
struct Quilt {ChartBase* chart=nullptr;ChartBase* GetRefChart(){return chart;}};
struct Toolbar {wxRect rect{0,0,90,40};wxRect GetToolbarRect(){return rect;}}toolbar,*g_MainToolbar=nullptr;
struct emboss_data {int x=0,y=0;};
enum ColorScheme {GLOBAL_COLOR_SCHEME_DAY,GLOBAL_COLOR_SCHEME_DUSK,GLOBAL_COLOR_SCHEME_NIGHT};
class ocpnDC;
class ChartCanvas:public wxFrame {
 public:
 ChartCanvas():wxFrame(nullptr,wxID_ANY,"overzoom fixture",wxDefaultPosition,wxSize(640,300)){SetClientSize(640,300);}
 VP vp{};Quilt quilt;Quilt* m_pQuilt=&quilt;bool quiltMode=true,primary=true;int refIndex=0;ChartBase* m_singleChart=nullptr;emboss_data map;emboss_data* m_pEM_OverZoom=&map;ColorScheme scheme=GLOBAL_COLOR_SCHEME_DAY;
 VP& GetVP(){return vp;}VP* GetpVP(){return &vp;}bool GetQuiltMode(){return quiltMode;}int GetQuiltRefChartdbIndex(){return refIndex;}bool IsPrimaryCanvas(){return primary;}ColorScheme GetColorScheme(){return scheme;}
 emboss_data* EmbossOverzoomIndicator(ocpnDC&);
};
struct glChartCanvas {ChartCanvas* m_pParentCanvas;};
// The pinned atlas branch is compile-time disabled in DrawText. Refuse if used.
struct DisabledTexFont {template<class...T>void Build(T...){throw std::runtime_error("unexpected atlas");}template<class...T>void GetTextExtent(T...){throw std::runtime_error("unexpected atlas");}template<class...T>void SetColor(T...){throw std::runtime_error("unexpected atlas");}template<class...T>void RenderString(T...){throw std::runtime_error("unexpected atlas");}};
int NextPow2(int x){int n=1;while(n<x)n*=2;return n;}
class ocpnDC {
 public:
 wxDC* dc=nullptr;glChartCanvas* m_glchartCanvas=nullptr;int m_canvasIndex=0;
 wxPen m_pen=*wxBLACK_PEN;wxBrush m_brush=*wxWHITE_BRUSH;wxFont m_font=*wxNORMAL_FONT;wxColour m_textforegroundcolour=*wxBLACK;
 float* workBuf=nullptr;size_t workBufSize=0;int workBufIndex=0;DisabledTexFont m_texfont;float m_dpi_factor=1;VP m_vp;
 GLShaderProgram *m_ptexture_2D_shader_program=nullptr,*m_pcolor_tri_shader_program=nullptr;
 ~ocpnDC(){free(workBuf);}wxDC* GetDC(){return dc;}
 wxColour GetTextForeground()const{return dc?dc->GetTextForeground():m_textforegroundcolour;}
 void SetFont(const wxFont&);void SetTextForeground(const wxColour&);void SetPen(const wxPen&);void SetBrush(const wxBrush&);
 const wxFont& GetFont()const;const wxPen& GetPen()const;const wxBrush& GetBrush()const;
 bool ConfigurePen();bool ConfigureBrush();void drawrrhelperGLES2(wxCoord,wxCoord,wxCoord,int,int);
 void DrawRoundedRectangle(wxCoord,wxCoord,wxCoord,wxCoord,wxCoord);
 void DrawLine(wxCoord,wxCoord,wxCoord,wxCoord,bool=true);void DrawText(const wxString&,wxCoord,wxCoord,float=0);
 void SetGLStipple()const;
 void DrawGLThickLine(int,int,int,int,wxPen,bool){throw std::runtime_error("unexpected thick fallback");}
};
#include "dc.inc"
#include "trigger.inc"
namespace opennav::ui {
wxColour Colour(std::uint32_t x){return wxColour((x>>16)&255,(x>>8)&255,x&255);}
#include "font.inc"
}
namespace opennav::integration {
bool xnav_mode=true,active=true;
#include "warning.inc"
}
using opennav::integration::DrawChartOverzoomWarning;
static int stocks=0;static emboss_data* lastStock=nullptr;
void DrawEmboss(ocpnDC&,emboss_data* p){++stocks;lastStock=p;}
// Exact changed caller is executed with a real original indicator.
void SwCaller(ChartCanvas& canvas,ocpnDC& dc){
 auto Invoke=[&](){return canvas.EmbossOverzoomIndicator(dc);};
 // Its member spelling is adapted only for this free-function test host.
 #include "caller-test.inc"
}
std::array<GLint,8> GLState(){std::array<GLint,8>s{};glGetIntegerv(GL_CURRENT_PROGRAM,&s[0]);s[1]=glIsEnabled(GL_BLEND);s[2]=glIsEnabled(GL_SCISSOR_TEST);glGetIntegerv(GL_FRAMEBUFFER_BINDING,&s[3]);glGetIntegerv(GL_VIEWPORT,&s[4]);return s;}
wxImage ReadGL(){wxImage im(640,300);glReadPixels(0,0,640,300,GL_RGB,GL_UNSIGNED_BYTE,im.GetData());return im.Mirror(false);}
bool Equal(const wxImage&a,const wxImage&b){return a.GetSize()==b.GetSize()&&!memcmp(a.GetData(),b.GetData(),size_t(a.GetWidth())*a.GetHeight()*3);}
void Trigger(ChartCanvas& c,ocpnDC& dc){
 ChartBase normal;ChartMbTiles mb;
 for(bool quilt:{false,true})for(bool single:{false,true})for(double z:{3.89,3.9,3.90001,10.}){
  c.quiltMode=quilt;c.m_singleChart=single?&normal:nullptr;c.vp.ref_scale=z*100;c.vp.chart_scale=100;
  Check(bool(c.EmbossOverzoomIndicator(dc))==((quilt||single)&&z>3.9),"exact threshold/single/quilt");
 }
 c.quiltMode=true;c.refIndex=0;database.entry.type=CHART_TYPE_MBTILES;c.quilt.chart=&mb;c.vp.ref_scale=10000;
 for(double z:{3.9,3.90001}){mb.factor=z;Check(bool(c.EmbossOverzoomIndicator(dc))==(z>3.9),"MBTiles zoom overrides viewport");}
 Check(mb.calls==2,"MBTiles called once per query");c.quilt.chart=&normal;Check(c.EmbossOverzoomIndicator(dc)==&c.map,"failed MBTiles cast keeps viewport");
 c.refIndex=-1;Check(c.EmbossOverzoomIndicator(dc)==&c.map,"missing quilt ref keeps viewport");c.refIndex=0;database.entry.type=CHART_TYPE_UNKNOWN;
 g_MainToolbar=&toolbar;Check(c.EmbossOverzoomIndicator(dc)->x==94,"primary toolbar anchor");c.primary=false;Check(c.EmbossOverzoomIndicator(dc)->x==4,"secondary anchor");c.primary=true;g_MainToolbar=nullptr;
 c.m_pEM_OverZoom=nullptr;Check(!c.EmbossOverzoomIndicator(dc),"missing stock map remains absent");c.m_pEM_OverZoom=&c.map;
}
class App:public wxApp {public:bool OnInit()override{return true;}};
wxIMPLEMENT_APP_NO_MAIN(App);
int main(int argc,char**argv){try{
 Check(argc==2,"output");std::string out=argv[1];wxEntryStart(argc,argv);wxTheApp->CallOnInit();wxInitAllImageHandlers();auto*c=new ChartCanvas;c->Show();wxTheApp->Yield();
 int attrs[]={WX_GL_RGBA,WX_GL_DEPTH_SIZE,0,0};wxGLCanvas glc(c,wxID_ANY,attrs,wxPoint(0,0),wxSize(640,300));wxGLContext context(&glc);glc.Show();wxTheApp->Yield();glc.SetCurrent(context);glewInit();glGetError();
 GLuint fbo=0,target=0;glGenTextures(1,&target);glBindTexture(GL_TEXTURE_2D,target);glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,640,300,0,GL_RGBA,GL_UNSIGNED_BYTE,nullptr);glGenFramebuffers(1,&fbo);glBindFramebuffer(GL_FRAMEBUFFER,fbo);glFramebufferTexture2D(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D,target,0);Check(glCheckFramebufferStatus(GL_FRAMEBUFFER)==GL_FRAMEBUFFER_COMPLETE,"fixture FBO");
 std::cout<<"renderer="<<glGetString(GL_RENDERER)<<" version="<<glGetString(GL_VERSION)<<"\n";
 GLShaderProgram color,texture;Check(color.addShaderFromSource(color_tri_vertex_shader_source,GL_VERTEX_SHADER)&&color.addShaderFromSource(color_tri_fragment_shader_source,GL_FRAGMENT_SHADER)&&color.linkProgram(),"color shader");Check(texture.addShaderFromSource(texture_2D_vertex_shader_source,GL_VERTEX_SHADER)&&texture.addShaderFromSource(texture_2D_fragment_shader_source,GL_FRAGMENT_SHADER)&&texture.linkProgram(),"texture shader");pcolor_tri_shader_program[0]=&color;ptexture_2D_shader_program[0]=&texture;
 mat4x4 matrix,identity;mat4x4_identity(identity);mat4x4_ortho(matrix,0,640,300,0,-1,1);memcpy(c->vp.vp_matrix_transform,matrix,sizeof(matrix));
 for(auto*p:{&color,&texture}){p->Bind();p->SetUniformMatrix4fv("MVMatrix",(float*)matrix);p->SetUniformMatrix4fv("TransformMatrix",(float*)identity);p->UnBind();}
 ocpnDC dc;glChartCanvas host{c};dc.m_glchartCanvas=&host;Trigger(*c,dc);
 wxImage swDay,glDay;
 for(bool gl:{false,true})for(int theme:{0,1,2,0}){
  c->scheme=static_cast<ColorScheme>(theme);wxBitmap bitmap(640,300);wxMemoryDC memory(bitmap);memory.SetBackground(wxBrush(wxColour(100,130,150)));memory.Clear();dc.dc=gl?nullptr:&memory;
  glViewport(0,0,640,300);glDisable(GL_SCISSOR_TEST);glClearColor(100/255.f,130/255.f,150/255.f,1);glClear(GL_COLOR_BUFFER_BIT);
  dc.SetFont(*wxNORMAL_FONT);dc.SetTextForeground(*wxRED);dc.SetPen(*wxBLUE_PEN);dc.SetBrush(*wxGREEN_BRUSH);
  const auto oldfont=dc.GetFont();const auto oldink=dc.GetTextForeground();const auto oldpen=dc.GetPen();const auto oldbrush=dc.GetBrush();
  Check(DrawChartOverzoomWarning(dc,*c,4,0),"warning painted");Check(dc.GetFont()==oldfont&&dc.GetTextForeground()==oldink&&dc.GetPen()==oldpen&&dc.GetBrush()==oldbrush,"DC state restored");
  wxImage image=gl?ReadGL():bitmap.ConvertToImage();std::string name=(gl?"gl-":"software-")+std::to_string(theme);if(theme==0){auto&day=gl?glDay:swDay;if(day.IsOk()){Check(Equal(day,image),"exact Day return");name+="-return";}else day=image.Copy();}image.SaveFile(out+"/"+name+".png",wxBITMAP_TYPE_PNG);
  Check(image.GetRed(10,20)!=100,"visible warning backing");Check(image.GetRed(500,200)==100,"outside warning untouched");
  for(int refusal=0;refusal<6;++refusal){
   opennav::integration::xnav_mode=refusal!=0;opennav::integration::active=refusal!=1;
   translation=refusal==2?wxString('W',150):wxString(refusal==5?"":"OverZoom");
   int x=refusal==3?635:4,y=refusal==4?-1:0;
   const auto state=GLState();wxImage before=gl?ReadGL():bitmap.ConvertToImage();Check(!DrawChartOverzoomWarning(dc,*c,x,y),"mode/translation/fit refusal");Check(Equal(before,gl?ReadGL():bitmap.ConvertToImage()),"refusal has no paint");Check(dc.GetFont()==oldfont&&dc.GetTextForeground()==oldink&&dc.GetPen()==oldpen&&dc.GetBrush()==oldbrush,"refusal state restored");Check(GLState()==state,"refusal GL state unchanged");
  }
  opennav::integration::xnav_mode=opennav::integration::active=true;translation="OverZoom";
  if(!gl){memory.SetClippingRegion(0,0,8,8);Check(!DrawChartOverzoomWarning(dc,*c,4,0),"software clip refusal");memory.DestroyClippingRegion();}
  dc.dc=nullptr;memory.SelectObject(wxNullBitmap);
 }
 dc.dc=nullptr;opennav::integration::xnav_mode=false;stocks=0;SwCaller(*c,dc);Check(stocks==1&&lastStock==&c->map,"same stock map on mode refusal");opennav::integration::xnav_mode=true;stocks=0;SwCaller(*c,dc);Check(stocks==0,"custom success replaces only emboss paint");
 std::cout<<checks<<" assertions passed; controlled actual-method fixture, not canvas/native acceptance\n";
 dc.m_glchartCanvas=nullptr; // GL resources destruct while context is current.
 return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}

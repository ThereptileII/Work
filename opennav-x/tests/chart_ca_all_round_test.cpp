#include <GL/glew.h>
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <wx/tokenzr.h>
#include <chrono>
#include <deque>
#include <iostream>
#include <stdexcept>
#ifdef __linux__
#include <sys/resource.h>
#endif
#include "integration/ChartCaAllRound.h"
#include "integration/ChartCableWave.h"
#include "linmath.h"
using namespace opennav::integration;
constexpr double PI=3.14159265358979323846;
S57Obj::S57Obj():Primitive_type(GEO_POINT),att_array(nullptr),attVal(nullptr),n_attr(0),x(200),y(200),m_chart_context(nullptr) {Scamin=10000;m_lat=m_lon=0;std::memcpy(FeatureName,"LIGHTS",7);}
S57Obj::~S57Obj(){}
static wxString _LITDSN01(S57Obj*){return wxString();}
#define LISTSIZE 20
#include "conditional.inc"
static unsigned checks=0;
void Check(bool v,const char* why){++checks;if(!v)throw std::runtime_error(why);}
void SetInst(wxString& s,wxString& owner){s=owner;}
void SetInst(wxString*& s,wxString& owner){s=&owner;}
struct Light:S57Obj {
 wxArrayOfS57attVal values;std::deque<S57attVal> attrs;std::deque<double> numbers;std::deque<std::string> strings;std::string names;
 wxString instruction="CS(LIGHTS05)\037";LUPrec lookup{};ObjRazRules node{};Rule cache{};Rules rule{};std::string ca;
 Light(const char* color="1",double range=20,double a=-9,double b=-9) {
  attVal=&values;lookup.RCID=31183;lookup.TNAM=SIMPLIFIED;std::memcpy(lookup.OBCL,"LIGHTS",7);SetInst(lookup.INST,instruction);node.obj=this;node.LUP=&lookup;
  String("COLOUR",color);if(a!=-9)Number("SECTR1",a);if(b!=-9)Number("SECTR2",b);Number("VALNMR",range);
  char* text=static_cast<char*>(LIGHTS06(&node));ca=text;free(text);
  Check(ca.find(";CA(")==0,"actual LIGHTS06 CA prefix");rule.INSTstr=ca.data()+4;rule.razRule=&cache;
 }
 ~Light(){if(cache.pixelPtr)delete static_cast<wxBitmap*>(cache.pixelPtr);}
 void Add(const char* n,void* p,OGRatt_t t){names+=n;att_array=names.data();++n_attr;attrs.push_back({p,t});values.Add(&attrs.back());}
 void String(const char* n,const char* s){strings.emplace_back(s);Add(n,strings.back().data(),OGR_STR);}
 void Number(const char* n,double v){numbers.push_back(v);Add(n,&numbers.back(),OGR_REAL);}
};
GLint S52ring_shader_program=0,S52texture_2D_shader_program=0;
static std::vector<wxPoint> legs;
void DrawAALine(wxDC*,int x,int y,int ex,int ey,wxColour,int,int){legs.push_back({x,y});legs.push_back({ex,ey});}
void GetGlobalColor(const wxString&,wxColour* c){*c=*wxBLACK;}
class s52plib {
 public:
 wxDC* m_pdc=nullptr;bool enabled=true;double m_dipfactor=1,m_ContentScaleFactor=1;
 float m_display_size_mm=300,canvas_pix_per_mm=3;int m_colortable_index=0;
 wxString m_ColorScheme="DAY_BRIGHT";wxColour m_unused_wxColor{255,0,255};
 struct View {int pix_width=400,pix_height=400;double rotation=0,view_scale_ppm=1;}vp_plib;
 int projections=0;bool PresentationCaLightsEnabled()const{return enabled;}
 float GetPPMM()const{return canvas_pix_per_mm;}
 wxColour getwxColour(const wxString& c){return c=="LITRD"?wxColour(255,0,0):c=="LITGN"?wxColour(0,255,0):c=="LITYW"?wxColour(255,255,0):*wxBLACK;}
 void GetPointPixSingle(ObjRazRules*,double y,double x,wxPoint* p){++projections;*p={int(x),int(y)};}
 void GetPixPointSingleNoRotate(double x,double y,double* lat,double* lon){*lat=-y/1000;*lon=x/1000;}
 void ClearRulesCache(Rule* r){if(r->pixelPtr)delete static_cast<wxBitmap*>(r->pixelPtr);r->pixelPtr=nullptr;}
 void DrawDashLine(wxPen,int x,int y,int ex,int ey){legs.push_back({x,y});legs.push_back({ex,ey});}
 int StockCARC_GLSL(ObjRazRules*,Rules*);int StockCARC_VBO(ObjRazRules*,Rules*);
 int RenderCARC_GLSL(ObjRazRules*,Rules*);int RenderCARC_VBO(ObjRazRules*,Rules*);
};
#include "original-fan-tile.inc"
#include "fan-methods.inc"
#include "shaders.inc"
#include "fan-matrix.inc"
class App:public wxApp{public:bool OnInit()override{return true;}};wxIMPLEMENT_APP_NO_MAIN(App);
GLuint Program(const char* vs,const char* fs){
 auto compile=[](GLenum type,const char* source){GLuint s=glCreateShader(type);const char* prefix="#version 120\n#define highp\n#define mediump\n#define lowp\n";std::string body=source;auto p=body.find("precision highp float;");if(p!=std::string::npos)body.erase(p,22);const char* texts[]={prefix,body.c_str()};glShaderSource(s,2,texts,nullptr);glCompileShader(s);GLint ok;glGetShaderiv(s,GL_COMPILE_STATUS,&ok);if(!ok){char log[4096];glGetShaderInfoLog(s,4096,nullptr,log);throw std::runtime_error(log);}return s;};
 GLuint p=glCreateProgram(),v=compile(GL_VERTEX_SHADER,vs),f=compile(GL_FRAGMENT_SHADER,fs);glAttachShader(p,v);glAttachShader(p,f);glLinkProgram(p);GLint ok;glGetProgramiv(p,GL_LINK_STATUS,&ok);Check(ok,"shader link");glDeleteShader(v);glDeleteShader(f);return p;
}
wxImage ReadGL(){wxImage image(400,400);glPixelStorei(GL_PACK_ALIGNMENT,1);glReadPixels(0,0,400,400,GL_RGB,GL_UNSIGNED_BYTE,image.GetData());return image.Mirror(false);}
void ClearGL(){glViewport(0,0,400,400);glClearColor(51.f/255,77.f/255,102.f/255,1);glClear(GL_COLOR_BUFFER_BIT);}
std::array<int,3> RGB(const wxImage& i,int x,int y){return {i.GetRed(x,y),i.GetGreen(x,y),i.GetBlue(x,y)};}
void Same(const wxImage& a,const wxImage& b,int tolerance){for(int y=0;y<400;++y)for(int x=0;x<400;++x){auto aa=RGB(a,x,y),bb=RGB(b,x,y);for(int k=0;k<3;++k)if(std::abs(aa[k]-bb[k])>tolerance)throw std::runtime_error("SW/GL raster mismatch "+std::to_string(x)+","+std::to_string(y));}++checks;}
void Matrices(double rotation=0) {
 GLfloat matrix[16],identity[16];ActualMatrix(matrix,400,400,rotation);mat4x4_identity(reinterpret_cast<float(*)[4]>(identity));
 for(GLuint shader:{GLuint(S52ring_shader_program),GLuint(S52texture_2D_shader_program)}) {
  glUseProgram(shader);glUniformMatrix4fv(glGetUniformLocation(shader,"MVMatrix"),1,GL_FALSE,matrix);
  glUniformMatrix4fv(glGetUniformLocation(shader,"TransformMatrix"),1,GL_FALSE,identity);
 }
 glUseProgram(0);
}
wxImage Render(s52plib& lib,Light& l,bool gl,bool stock=false) {
 legs.clear();
 if(gl){lib.m_pdc=nullptr;ClearGL();Matrices(lib.vp_plib.rotation);Check((stock?lib.StockCARC_GLSL(&l.node,&l.rule):lib.RenderCARC_GLSL(&l.node,&l.rule))==1,"GL method return");glFinish();return ReadGL();}
 wxBitmap bitmap(400,400,24);wxMemoryDC dc(bitmap);dc.SetBackground(wxBrush(wxColour(51,77,102)));dc.Clear();lib.m_pdc=&dc;
 Check((stock?lib.StockCARC_VBO(&l.node,&l.rule):lib.RenderCARC_VBO(&l.node,&l.rule))==1,"software method return");dc.SelectObject(wxNullBitmap);lib.m_pdc=nullptr;return bitmap.ConvertToImage();
}
void SameBounds(const Light& a,const Light& b) {
 Check(a.BBObj.GetMinLat()==b.BBObj.GetMinLat() && a.BBObj.GetMaxLat()==b.BBObj.GetMaxLat() &&
 a.BBObj.GetMinLon()==b.BBObj.GetMinLon() && a.BBObj.GetMaxLon()==b.BBObj.GetMaxLon(),"original full-circle geographic redraw bounds");
}
int main(int argc,char** argv) {
 try {
  std::string out=argv[1];Check(wxEntryStart(argc,argv),"wx start");Check(wxTheApp->CallOnInit(),"wx init");wxInitAllImageHandlers();
  auto* frame=new wxFrame(nullptr,wxID_ANY,"All-round CA paint fixture",wxDefaultPosition,wxSize(400,400));int attrs[]={WX_GL_RGBA,WX_GL_DOUBLEBUFFER,WX_GL_DEPTH_SIZE,0,0};auto* canvas=new wxGLCanvas(frame,wxID_ANY,attrs);frame->Show();wxTheApp->Yield();wxGLContext context(canvas);Check(context.IsOK()&&canvas->SetCurrent(context),"real GL context");glewExperimental=GL_TRUE;auto glew=glewInit();if(glew!=GLEW_OK&&glew!=GLEW_ERROR_NO_GLX_DISPLAY)throw std::runtime_error("GLEW");while(glGetError()!=GL_NO_ERROR){}
  Check(GLEW_VERSION_2_0,"GL shaders");std::cout<<"renderer="<<glGetString(GL_RENDERER)<<" version="<<glGetString(GL_VERSION)<<"\n";
  S52texture_2D_shader_program=Program(S52texture_2D_vertex_shader_source,S52texture_2D_fragment_shader_source);S52ring_shader_program=Program(S52ring_vertex_shader_source,S52ring_fragment_shader_source);
  s52plib lib;Light white;Check(CaAllRoundLight(true,&white.node).color==1,"ordinary white accepted");Check(!CaAllRoundLight(false,&white.node).color,"Standard disabled");
  for(const char* field:{"ORIENT","CATLIT","LITVIS","STATUS","QUAPOS","QUASOU"}){Light l;l.String(field,"1");Check(!CaAllRoundLight(true,&l.node).color,"nonordinary stock fallback");}
  for(const char* color:{"6","1,3","12",""}){Light l;l.strings[0]=color;l.attrs[0].value=l.strings[0].data();Check(!CaAllRoundLight(true,&l.node).color,"other/unknown color fallback");}
  {Light l;l.lookup.TNAM=PAPER_CHART;Check(!CaAllRoundLight(true,&l.node).color,"Paper stock");}
  {Light l;l.Number("SECTR1",0);Check(!CaAllRoundLight(true,&l.node).color,"partial sector unknown");}
  {Light l("1",20,164,236);Check(!CaAllRoundLight(true,&l.node).color && CaFanColor(true,&l.node)==1,"finite sector unaffected");}
  {Light l;l.numbers.back()=std::numeric_limits<double>::quiet_NaN();Check(!CaAllRoundLight(true,&l.node).color,"unknown range");}
  {Light l;l.Number("VALNMR",20);Check(!CaAllRoundLight(true,&l.node).color,"duplicate range");}
  {Light l;l.numbers.back()=9;Check(!CaAllRoundLight(true,&l.node).color,"absent-sector short range remains flare");}
  const auto style=CaAllRoundLight(true,&white.node);
  Check(!CaAllRoundInstruction(style,"OUTLW",4,"LITYW",2,0,360,18,0),"wrong range-band radius refused");
  Check(!CaAllRoundInstruction(style,"OUTLW",4,"LITYW",2,0,359,17,0),"finite CA refused");
  Check(!CaAllRoundInstruction(style,"OUTLW",4,"LITYW",2,0,360,17,25),"sector legs refused");
  CaFanPaint paint;CaFanTile tile;
  Check(CaFanColors(1,"DAY_BRIGHT",paint),"prototype white arc ink");
  Check(tile.BuildAllRound({200,200},51,1,paint),"outline circle coverage");
  Check(!tile.image.GetAlpha(200-tile.bounds.x,200-tile.bounds.y),"no wash or rays at center");
  int sum=0;for(int x=245;x<=255;++x)sum+=tile.image.GetAlpha(x-tile.bounds.x,200-tile.bounds.y);
  Check(std::abs(sum-1.2*.8*255)<5,"prototype 1.2 width and .8 opacity coverage");
  Check(!tile.BuildAllRound({200,200},10000,1,paint),"bounded fallback");
  // Original range bands are checked using actual LIGHTS06 output and painters.
  for(double range:{3.,6.9,7.,14.9,15.,29.9,30.,50.})for(const char* color:{"1","3","4"}) {
    Light a(color,range,0,360);auto info=CaAllRoundLight(true,&a.node);Check(info.color==unsigned(color[0]-'0'),"explicit full sweep eligibility");
    const auto expected=std::string(",0,360,")+std::to_string(info.radius)+",0";Check(a.ca.find(expected)!=std::string::npos,"actual pinned range-band CA");
  }
  for(const char* theme:{"DAY_BRIGHT","DUSK","NIGHT"})for(const char* color:{"1","3","4"}) {
    lib.m_ColorScheme=theme;Light a(color),b(color);auto instruction=a.ca;
    auto sw=Render(lib,a,false),gl=Render(lib,b,true);Same(sw,gl,1);
    Check(!a.cache.pixelPtr&&!b.cache.pixelPtr&&a.ca==instruction&&b.ca==instruction,"source CA and shared cache unchanged");Check(legs.empty(),"no sector rays");
    Check(RGB(sw,200,200)==std::array<int,3>{51,77,102},"empty full-circle interior");
    sw.SaveFile(out+"/"+theme+"-"+color+"-software.png",wxBITMAP_TYPE_PNG);gl.SaveFile(out+"/"+theme+"-"+color+"-gl.png",wxBITMAP_TYPE_PNG);
  }
  // Radius/center remain the exact original renderer's values, including its
  // differing core/private unset-SCAMIN reduction and core display-size cap.
  for(bool gl:{false,true})for(int scenario:{0,1,2}) {
    Light original,repaint;lib.m_ColorScheme="DAY_BRIGHT";lib.m_display_size_mm=scenario==2?100:300;
    original.x=repaint.x=170;original.y=repaint.y=220;lib.vp_plib.rotation=gl?.43:0;
    lib.vp_plib.view_scale_ppm=scenario==1?.001:1;original.Scamin=repaint.Scamin=scenario==1?1e9:10000;
    Render(lib,original,gl,true);float radius,center[2];
    if(gl){glGetUniformfv(S52ring_shader_program,glGetUniformLocation(S52ring_shader_program,"circle_radius"),&radius);glGetUniformfv(S52ring_shader_program,glGetUniformLocation(S52ring_shader_program,"circle_center"),center);center[1]=400-center[1];}
    else{radius=(original.cache.parm5-8)/2.;center[0]=original.x;center[1]=original.y;}
    auto image=Render(lib,repaint,gl);SameBounds(original,repaint);Check(legs.empty(),"all-round no boundary segments");
    Check(CaFanColors(1,lib.m_ColorScheme,paint)&&tile.BuildAllRound({int(center[0]),int(center[1])},radius,1,paint),"exact original circle geometry");
    wxBitmap bitmap(400,400,24);wxMemoryDC dc(bitmap);dc.SetBackground(wxBrush(wxColour(51,77,102)));dc.Clear();Check(tile.Draw(dc),"independent circle draw");dc.SelectObject(wxNullBitmap);Same(image,bitmap.ConvertToImage(),1);
    std::cout<<"originalGeometry "<<(gl?"GL":"SW")<<" scenario="<<scenario<<" radius="<<radius<<" center="<<center[0]<<","<<center[1]<<"\n";
  }
  lib.m_display_size_mm=300;lib.vp_plib.rotation=0;lib.vp_plib.view_scale_ppm=1;
  for(const char* refusal:{"Standard","directional","obscured","uncertain","unknown-range","paint-refusal"})for(bool gl:{false,true}) {
    Light a,b;lib.enabled=std::strcmp(refusal,"Standard")!=0;
    lib.m_ContentScaleFactor=std::strcmp(refusal,"paint-refusal")==0?.1:1;
    if(std::strcmp(refusal,"directional")==0){a.Number("ORIENT",45);b.Number("ORIENT",45);}
    if(std::strcmp(refusal,"obscured")==0){a.String("LITVIS","3");b.String("LITVIS","3");}
    if(std::strcmp(refusal,"uncertain")==0){a.String("QUAPOS","4");b.String("QUAPOS","4");}
    if(std::strcmp(refusal,"unknown-range")==0){a.numbers.back()=b.numbers.back()=0;}
    auto painted=Render(lib,a,gl),stock=Render(lib,b,gl,true);Same(painted,stock,0);SameBounds(a,b);
  }
  lib.enabled=true;lib.m_ContentScaleFactor=1;lib.m_pdc=nullptr;Check(CaFanColors(1,"DAY_BRIGHT",paint)&&tile.BuildAllRound({200,200},51,1,paint),"state fixture");
#include "fan-state-check.inc"
  // Shared finite-sector default remains byte-exact against the previous class.
  CaFanGeometry finite{{200,200},{180,275},{250,275},45,344,416,1};CaFanTile now;OriginalFanTile before;
  Check(now.Build(finite,paint)&&before.Build(finite,paint),"finite sector default");Check(now.bounds==before.bounds && now.image.GetSize()==before.image.GetSize() && !std::memcmp(now.image.GetData(),before.image.GetData(),now.image.GetWidth()*now.image.GetHeight()*3) && !std::memcmp(now.image.GetAlpha(),before.image.GetAlpha(),now.image.GetWidth()*now.image.GetHeight()),"finite sector default pixel inverse");
  CableWaveTile cable;Check(cable.Prepare(1,.3,wxColour(156,134,150),{190,210}),"unchanged cable tile");ClearGL();Matrices();Check(DrawCableWaveGL(cable.tile,S52texture_2D_shader_program,S52ring_shader_program,400,400),"cable optional matrices before");auto cableBefore=ReadGL();
  Light circle;Render(lib,circle,true);ClearGL();Check(DrawCableWaveGL(cable.tile,S52texture_2D_shader_program,S52ring_shader_program,400,400),"cable optional matrices after");Same(cableBefore,ReadGL(),0);
  std::cout<<checks<<" actual-method all-round checks passed; full-circle extension, not supplied glyph or canvas qualification\n";
  glDeleteProgram(S52texture_2D_shader_program);glDeleteProgram(S52ring_shader_program);frame->Destroy();wxTheApp->Yield();wxTheApp->OnExit();wxEntryCleanup();return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}

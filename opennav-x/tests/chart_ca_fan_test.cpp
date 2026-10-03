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
#include "integration/ChartCaFan.h"
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
 Light(const char* color="1",double a=164,double b=236) {
  attVal=&values;lookup.RCID=31183;lookup.TNAM=SIMPLIFIED;std::memcpy(lookup.OBCL,"LIGHTS",7);SetInst(lookup.INST,instruction);node.obj=this;node.LUP=&lookup;
  String("COLOUR",color);Number("SECTR1",a);Number("SECTR2",b);Number("VALNMR",20);
  char* text=static_cast<char*>(LIGHTS06(&node));ca=text;free(text);
  Check(ca.find(";CA(")==0,"actual LIGHTS06 sector prefix");rule.INSTstr=ca.data()+4;rule.razRule=&cache;
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
int main(int argc,char** argv){
 try {
  std::string out=argv[1];Check(wxEntryStart(argc,argv),"wx start");Check(wxTheApp->CallOnInit(),"wx init");wxInitAllImageHandlers();
  auto* frame=new wxFrame(nullptr,wxID_ANY,"Owned CA painter fixture",wxDefaultPosition,wxSize(400,400));int attrs[]={WX_GL_RGBA,WX_GL_DOUBLEBUFFER,WX_GL_DEPTH_SIZE,0,0};auto* canvas=new wxGLCanvas(frame,wxID_ANY,attrs);frame->Show();wxTheApp->Yield();wxGLContext context(canvas);Check(context.IsOK(),"GL context created");Check(canvas->SetCurrent(context),"GL context current");glewExperimental=GL_TRUE;const auto glew=glewInit();if(glew!=GLEW_OK && glew!=GLEW_ERROR_NO_GLX_DISPLAY)throw std::runtime_error(std::string("GLEW: ")+reinterpret_cast<const char*>(glewGetErrorString(glew)));while(glGetError()!=GL_NO_ERROR){}
  Check(GLEW_VERSION_2_0 && glGetString(GL_RENDERER),"actual GL shader context");
  std::cout<<"renderer="<<glGetString(GL_RENDERER)<<" version="<<glGetString(GL_VERSION)<<"\n";
  S52texture_2D_shader_program=Program(S52texture_2D_vertex_shader_source,S52texture_2D_fragment_shader_source);S52ring_shader_program=Program(S52ring_vertex_shader_source,S52ring_fragment_shader_source);
  s52plib lib;Light white;Check(CaFanColor(true,&white.node)==1,"white eligible");Check(!CaFanColor(false,&white.node),"disabled");
  for(const char* field:{"ORIENT","CATLIT","LITVIS","STATUS","QUAPOS","QUASOU"}){Light l;l.String(field,"1");Check(!CaFanColor(true,&l.node),"special refusal");}
  for(const char* c:{"6","1,3","12",""}){Light l;l.strings[0]=c;l.attrs[0].value=l.strings[0].data();Check(!CaFanColor(true,&l.node),"other colour refusal");}
  {Light l;l.lookup.TNAM=PAPER_CHART;Check(!CaFanColor(true,&l.node),"Paper refusal");}
  {Light l;l.numbers[1]=l.numbers[0];Check(!CaFanColor(true,&l.node),"all round refusal");}
  {Light l;l.numbers[0]=std::numeric_limits<double>::quiet_NaN();Check(!CaFanColor(true,&l.node),"NaN refusal");}
  CaFanPaint paint;Check(CaFanColors(1,"DAY_BRIGHT",paint),"day color");CaFanTile tile;
  Check(!tile.Build({{200,200},{200,300},{300,200},10000,0,90,1},paint),"work cap stock fallback");
  for(const auto* theme:{"DAY_BRIGHT","DUSK","NIGHT"})for(const char* color:{"1","3","4"}) {
    Light light(color);lib.m_ColorScheme=theme;std::string before=light.ca;
    wxBitmap bitmap(400,400,24);wxMemoryDC dc(bitmap);dc.SetBackground(wxBrush(wxColour(51,77,102)));dc.Clear();lib.m_pdc=&dc;
    Check(lib.RenderCARC_VBO(&light.node,&light.rule)==1,"SW return");Check(!light.cache.pixelPtr,"owned paint leaves stock cache untouched");Check(light.ca==before,"CA unchanged");dc.SelectObject(wxNullBitmap);wxImage sw=bitmap.ConvertToImage();sw.SaveFile(out+"/"+theme+"-"+color+"-software.png",wxBITMAP_TYPE_PNG);
    lib.m_pdc=nullptr;ClearGL();Check(lib.RenderCARC_GLSL(&light.node,&light.rule)==1,"GL return");glFinish();wxImage gl=ReadGL();gl.SaveFile(out+"/"+theme+"-"+color+"-gl.png",wxBITMAP_TYPE_PNG);Same(sw,gl,1);
    // Wash interior away from boundaries: actual CA rotates 164..236 to 344..56.
    Check(CaFanColors(unsigned(color[0]-'0'),theme,paint),"theme colors");auto pixel=RGB(gl,200,180);std::array<int,3> bg{51,77,102},ink{paint.fill.Red(),paint.fill.Green(),paint.fill.Blue()};
    std::cout<<theme<<" "<<color<<" sample="<<pixel[0]<<","<<pixel[1]<<","<<pixel[2]<<" expectedInk="<<ink[0]<<","<<ink[1]<<","<<ink[2]<<"\n";
    CaFanTile diagnostic;Check(diagnostic.Build({{200,200},{180,275},{250,275},45,344,416,1},paint),"alpha diagnostic");const int dx=200-diagnostic.bounds.x,dy=180-diagnostic.bounds.y;std::cout<<"tileStraight="<<int(diagnostic.image.GetRed(dx,dy))<<","<<int(diagnostic.image.GetGreen(dx,dy))<<","<<int(diagnostic.image.GetBlue(dx,dy))<<" alpha="<<int(diagnostic.image.GetAlpha(dx,dy))<<"\n";
    for(int k=0;k<3;++k)Check(std::abs(pixel[k]-int(std::lround(ink[k]*43./255+bg[k]*212./255)))<=1,"actual .17 wash blend");
  }
  // Actual unchanged stock painter comparison with the original pinned bodies.
  for(const char* refusal:{"stock","oriented","uncertain"})for(bool glMode:{false,true}) {
    Light a,b;lib.enabled=std::strcmp(refusal,"stock")!=0;lib.m_ColorScheme="DAY_BRIGHT";
    if(std::strcmp(refusal,"oriented")==0){a.Number("ORIENT",45);b.Number("ORIENT",45);}
    if(std::strcmp(refusal,"uncertain")==0){a.String("QUAPOS","4");b.String("QUAPOS","4");}
    wxImage images[2];std::vector<wxPoint> originalLegs;
    for(int i=0;i<2;++i) {
      auto& l=i?b:a;legs.clear();
      if(glMode) {
        lib.m_pdc=nullptr;ClearGL();GLfloat matrix[16],identity[16];ActualMatrix(matrix,400,400,0);ActualMatrix(identity,2,2,0);mat4x4_identity(reinterpret_cast<float(*)[4]>(identity));
        glUseProgram(S52ring_shader_program);glUniformMatrix4fv(glGetUniformLocation(S52ring_shader_program,"MVMatrix"),1,GL_FALSE,matrix);glUniformMatrix4fv(glGetUniformLocation(S52ring_shader_program,"TransformMatrix"),1,GL_FALSE,identity);glUseProgram(0);
        Check((i?lib.StockCARC_GLSL(&l.node,&l.rule):lib.RenderCARC_GLSL(&l.node,&l.rule))==1,"stock GL return");images[i]=ReadGL();
      } else {
        wxBitmap bitmap(400,400,24);wxMemoryDC dc(bitmap);dc.SetBackground(wxBrush(wxColour(51,77,102)));dc.Clear();lib.m_pdc=&dc;
        Check((i?lib.StockCARC_VBO(&l.node,&l.rule):lib.RenderCARC_VBO(&l.node,&l.rule))==1,"stock SW return");dc.SelectObject(wxNullBitmap);images[i]=bitmap.ConvertToImage();
      }
      if(!i)originalLegs=legs;else Check(originalLegs==legs,"stock leg endpoints unchanged");
    }
    Same(images[0],images[1],0);images[0].SaveFile(out+"/"+refusal+(glMode?"-gl.png":"-software.png"),wxBITMAP_TYPE_PNG);
  }
  {
    Light a,b;wxBitmap bitmap(400,400,24);wxMemoryDC dc(bitmap);lib.m_pdc=&dc;lib.enabled=false;
    lib.RenderCARC_VBO(&a.node,&a.rule);lib.StockCARC_VBO(&b.node,&b.rule);
    auto start=std::chrono::steady_clock::now();for(int i=0;i<1000;++i)lib.RenderCARC_VBO(&a.node,&a.rule);
    const double guarded=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/1000;
    start=std::chrono::steady_clock::now();for(int i=0;i<1000;++i)lib.StockCARC_VBO(&b.node,&b.rule);
    std::cout<<"standardCachedPaintMs="<<guarded<<" originalCachedPaintMs="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/1000<<"\n";
    dc.SelectObject(wxNullBitmap);
  }
  lib.enabled=true;lib.m_pdc=nullptr;
  for(double rotation:{.43,-1.2}) {
    Light l("3",350,15);l.x=170;l.y=220;lib.vp_plib.rotation=rotation;
    GLfloat matrix[16];ActualMatrix(matrix,400,400,rotation);
    const double expectedX=(matrix[0]*l.x+matrix[4]*l.y+matrix[12]+1)*200;
    const double expectedY=(1-(matrix[1]*l.x+matrix[5]*l.y+matrix[13]))*200;
    const double centeredX=(l.x-200)*std::cos(rotation)-(l.y-200)*std::sin(rotation)+200;
    const double centeredY=(l.x-200)*std::sin(rotation)+(l.y-200)*std::cos(rotation)+200;
    Check(std::abs(expectedX-centeredX)<.0001 && std::abs(expectedY-centeredY)<.0001,"actual PrepareS52ShaderUniforms transform agrees");
    glUseProgram(S52texture_2D_shader_program);glUniformMatrix4fv(glGetUniformLocation(S52texture_2D_shader_program,"MVMatrix"),1,GL_FALSE,matrix);glUseProgram(0);
    ClearGL();Check(lib.RenderCARC_GLSL(&l.node,&l.rule)==1,"rotated actual GL paint");auto image=ReadGL();image.SaveFile(out+(rotation>0?"/rotated-positive-gl.png":"/rotated-negative-gl.png"),wxBITMAP_TYPE_PNG);
    Check(!l.cache.pixelPtr,"rotated no shared cache");
  }
  lib.vp_plib.rotation=0;
  // Rotated + wraparound geometry and duplicate per-record paint source-over.
  Check(CaFanColors(4,"NIGHT",paint),"night green");CaFanGeometry geometry{{200,200},{125,200},{200,125},45,270,360,2};Check(tile.Build(geometry,paint),"rotated tile");
  wxBitmap bitmap(400,400,24);wxMemoryDC dc(bitmap);dc.SetBackground(wxBrush(wxColour(51,77,102)));dc.Clear();Check(tile.Draw(dc)&&tile.Draw(dc),"SW overlap");dc.SelectObject(wxNullBitmap);auto sw=bitmap.ConvertToImage();
  ClearGL();Check(DrawCaFanGL(tile,S52texture_2D_shader_program,400,400)&&DrawCaFanGL(tile,S52texture_2D_shader_program,400,400),"GL overlap");auto gl=ReadGL();Same(sw,gl,1);sw.SaveFile(out+"/overlap-scale2-software.png",wxBITMAP_TYPE_PNG);gl.SaveFile(out+"/overlap-scale2-gl.png",wxBITMAP_TYPE_PNG);
  // All draw state actually touched is verified separately by a hostile snapshot.
#include "fan-state-check.inc"
  Check(CaFanColors(1,"DAY_BRIGHT",paint),"stroke fixture color");
  Check(tile.Build({{200,200},{200,100},{300,200},45,0,90,1},paint),"separate boundary coverage");
  int summed=0;for(int x=197;x<=203;++x)summed+=tile.image.GetAlpha(x-tile.bounds.x,130-tile.bounds.y);
  Check(std::abs(summed-.65*.6*255)<=3,"independent .65 px boundary coverage at .6 opacity");
  CaFanTile acute;Check(acute.Build({{200,200},{200,100},{202,100},45,0,1.1,1},paint),"acute bounded miter");
  for(int x=0;x<acute.image.GetWidth();++x)Check(!acute.image.GetAlpha(x,0)&&!acute.image.GetAlpha(x,acute.image.GetHeight()-1),"no clipped acute paint horizontal");
  for(int y=0;y<acute.image.GetHeight();++y)Check(!acute.image.GetAlpha(0,y)&&!acute.image.GetAlpha(acute.image.GetWidth()-1,y),"no clipped acute paint vertical");
  const auto uploadStart=std::chrono::steady_clock::now();
  for(int i=0;i<20;++i){Check(DrawCaFanGL(tile,S52texture_2D_shader_program,400,400),"timed upload");glFinish();}
  std::cout<<"typicalUploadAndFinishMs="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-uploadStart).count()/20<<"\n";
#ifdef __linux__
  rusage beforeMaximum{};getrusage(RUSAGE_SELF,&beforeMaximum);
#endif
  const auto maximumStart=std::chrono::steady_clock::now();
  Check(tile.Build({{0,0},{0,-507},{507,0},507,0,90,1},paint),"near cap tile");
  std::cout<<"maximumTile="<<tile.image.GetWidth()<<"x"<<tile.image.GetHeight()<<" maximumBuildMs="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-maximumStart).count()<<"\n";
  const auto maximumUpload=std::chrono::steady_clock::now();Check(DrawCaFanGL(tile,S52texture_2D_shader_program,400,400),"maximum upload");glFinish();
  std::cout<<"maximumUploadAndFinishMs="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-maximumUpload).count()<<" rgbaBytes="<<size_t(tile.image.GetWidth())*tile.image.GetHeight()*4<<"\n";
#ifdef __linux__
  rusage afterMaximum{};getrusage(RUSAGE_SELF,&afterMaximum);std::cout<<"maximumCasePeakRssIncreaseKiB="<<afterMaximum.ru_maxrss-beforeMaximum.ru_maxrss<<" (process high-water observation, not allocator upper bound)\n";
#endif
  const auto start=std::chrono::steady_clock::now();for(int i=0;i<100;++i)Check(tile.Build({{200,200},{180,275},{250,275},60,150,200,1},paint),"timed tile");
  std::cout<<"typicalTileBuildMs="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/100<<" checks="<<checks<<"\n";
  glDeleteProgram(S52texture_2D_shader_program);glDeleteProgram(S52ring_shader_program);frame->Destroy();wxTheApp->Yield();wxTheApp->OnExit();wxEntryCleanup();return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}

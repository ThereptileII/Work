// Exercises extracted production bodies and real wx drawing; GL is recording.
#include <wx/wx.h>
#include <wx/graphics.h>
#include <wx/fontenum.h>
#include <wx/bmpbndl.h>
#include <wx/arrstr.h>
#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include "ui/Theme.h"
#include "integration/ChartWaypointIcon.h"
#include "model/MarkIcon.h"
class App: public wxApp {public:bool OnInit() override{return true;}};
wxIMPLEMENT_APP_NO_MAIN(App);
constexpr int GLOBAL_COLOR_SCHEME_DAY=0,GLOBAL_COLOR_SCHEME_DUSK=1,GLOBAL_COLOR_SCHEME_NIGHT=2;
struct VP{float vp_matrix_transform[16]{};};
struct ChartCanvas:wxFrame{
 ChartCanvas():wxFrame(nullptr,wxID_ANY,"fixture"){} int dpi=100,theme=0; VP vp;
 int FromDIP(int v){return v*dpi/100;} int GetColorScheme(){return theme;}
 VP*GetpVP(){return &vp;}
};
template<class T> struct List {
 struct Node {T *data;Node*next=nullptr;T*GetData(){return data;}Node*GetNext(){return next;}};
 std::vector<Node> nodes;
 void Set(std::initializer_list<T*> items){nodes.clear();for(auto*p:items)nodes.push_back({p});for(size_t i=1;i<nodes.size();++i)nodes[i-1].next=&nodes[i];}
 Node*GetFirst(){return nodes.empty()?nullptr:&nodes[0];}
};
struct RoutePoint {
 wxString icon="diamond"; bool m_bIsInRoute=true,shared=false,m_bIsInLayer=false,
 m_bIsActive=false,m_bPtIsSelected=false,m_bBlink=false,m_bRPIsBeingEdited=false,
 drag=false,m_bShowWaypointRangeRings=false; int m_iWaypointRangeRingsNumber=0;
 wxString GetIconName(){return icon;} bool IsShared(){return shared;}bool IsDragHandleEnabled(){return drag;}
};
struct Route {List<RoutePoint> list;List<RoutePoint>*pRoutePointList=&list;bool eligible=true,m_bIsBeingCreated=false;};
struct Routeman {Route*active=nullptr;Route*GetpActiveRoute(){return active;}};
Routeman manager;Routeman*g_pRouteMan=&manager;List<Route> routes;List<Route>*pRouteList=&routes;
RoutePoint*pAnchorWatchPoint1=nullptr,*pAnchorWatchPoint2=nullptr;
float g_MarkScaleFactorExp=1;
struct ocpnDC {
 wxDC*native=nullptr;int m_canvasIndex=0;wxFont font;wxColour ink;wxString last;
 wxDC*GetDC(){return native;}void CalcBoundingBox(int x,int y){if(native)native->CalcBoundingBox(x,y);}
 wxFont GetFont(){return font;}wxColour GetTextForeground(){return ink;}
 void SetFont(const wxFont&v){font=v;}void SetTextForeground(wxColour v){ink=v;}
 void GetTextExtent(const wxString&s,wxCoord*w,wxCoord*h){wxScreenDC d;d.SetFont(font);d.GetTextExtent(s,w,h);}
 void DrawText(const wxString&s,int,int){last=s;}
};
#define ocpnUSE_GL
using GLint=int;using GLfloat=float;
constexpr int GL_CURRENT_PROGRAM=1,GL_BLEND=2,GL_TRIANGLES=3,GL_TEXTURE_BINDING_2D=4,
 GL_BLEND_SRC_RGB=5,GL_BLEND_DST_RGB=6,GL_BLEND_SRC_ALPHA=7,GL_BLEND_DST_ALPHA=8,GL_TEXTURE_2D=9;
int program=19,draws=0,texture=23;bool blended=true,textured=true;std::vector<float> radii;
void glGetIntegerv(int key,int*v){*v=key==GL_CURRENT_PROGRAM?program:key==GL_TEXTURE_BINDING_2D?texture:1;}
bool glIsEnabled(int key){return key==GL_BLEND?blended:textured;}
void glEnable(int key){(key==GL_BLEND?blended:textured)=true;}
void glDisable(int key){(key==GL_BLEND?blended:textured)=false;}
void glUseProgram(int p){program=p;}void glBindTexture(int,int v){texture=v;}
void glBlendFuncSeparate(int,int,int,int){}
void glDrawArrays(int,int,int n){if(n!=96*3)throw std::runtime_error("circle submission");++draws;}
struct Shader {void Bind(){program=29;}void UnBind(){program=0;}
 void SetUniformMatrix4fv(const char*,float*){}void SetUniform4fv(const char*,float*){}
 void SetAttributePointerf(const char*,float*v){radii.push_back(std::hypot(v[2]-v[0],v[3]-v[1]));}
} shader;Shader*pcolor_tri_shader_program[2]{&shader,&shader};
namespace opennav::ui {
wxColour Colour(std::uint32_t v){return wxColour(v>>16,(v>>8)&255,v&255);}
#include "production-marker-font.h"
}
namespace opennav::integration {
bool style=true;
bool ChartActiveRouteInk(ChartCanvas&c,wxColour&i){
 const unsigned effective[]{0x267c76,0xb0dfc8,0x71937e};
 i=ui::Colour(effective[c.theme]);return style;
}
bool DefaultChartRouteStyle(Route&r){return r.eligible;}
}
#include "production-marker.h"
struct Icons {std::vector<MarkIcon*> data;unsigned GetCount(){return data.size();}MarkIcon*Item(unsigned i){return data[i];}
 void Add(MarkIcon*p){data.push_back(p);}void Insert(MarkIcon*p,int){data.insert(data.begin(),p);}};
struct WayPointman {Icons*m_pIconArray;};
struct WayPointmanGui {WayPointman&m_waypoint_man;
 MarkIcon*ProcessIcon(wxImage image,const wxString&key,const wxString&desc,bool front=false);
 bool IsPinnedRouteDiamond(const wxString&key,const wxBitmap*bitmap)const;
};
#include "production-marker-provenance.h"
int checks=0;void Check(bool v,const char*s){++checks;if(!v)throw std::runtime_error(s);}
int main(int argc,char**argv){
 if(!wxEntryStart(argc,argv)||!wxTheApp->CallOnInit())return 2;
 wxInitAllImageHandlers();
 wxLogNull quiet;
 try{
 using namespace opennav::integration;
 auto*c=new ChartCanvas;Route a,b;RoutePoint p,q,r;routes.Set({&a});a.list.Set({&q,&p,&r});manager.active=&a;
 auto ordinal=[&](){return ChartRouteWaypointOrdinal(*c,p,true);};
 Check(ordinal()==2,"actual route ordinal");
 Check(ChartRouteWaypointOrdinal(*c,p,false)==0,"unproven icon");style=false;Check(ordinal()==0,"Standard/Legacy/Safe");style=true;
 for(bool*state:{&p.shared,&p.m_bIsInLayer,&p.m_bIsActive,&p.m_bPtIsSelected,&p.m_bBlink,&p.m_bRPIsBeingEdited,&p.drag,&a.m_bIsBeingCreated}){
 *state=true;Check(ordinal()==0,"special state remains stock");*state=false;
 }
 p.m_bIsInRoute=false;Check(!ordinal(),"standalone point");p.m_bIsInRoute=true;
 p.icon="mob";Check(!ordinal(),"MOB");p.icon="anchor";Check(!ordinal(),"anchor");p.icon="diamond";
 pAnchorWatchPoint1=&p;Check(!ordinal(),"anchor watch1");pAnchorWatchPoint1=nullptr;pAnchorWatchPoint2=&p;Check(!ordinal(),"anchor watch2");pAnchorWatchPoint2=nullptr;
 p.m_bShowWaypointRangeRings=true;p.m_iWaypointRangeRingsNumber=1;Check(!ordinal(),"range rings");p.m_bShowWaypointRangeRings=false;
 a.eligible=false;Check(!ordinal(),"custom selected emergency route");a.eligible=true;
 a.list.Set({&p,&q,&p});Check(!ordinal(),"repeat within active route");a.list.Set({&q,&p,&r});
 b.list.Set({&p});routes.Set({&a,&b});Check(!ordinal(),"shared hidden route");routes.Set({&a});
 manager.active=&b;Check(!ordinal(),"other active route");manager.active=&a;
 a.list.nodes.assign(100,{&q});a.list.nodes[99].data=&p;
 for(int i=0;i<99;++i)a.list.nodes[i].next=&a.list.nodes[i+1];
 Check(!ordinal(),"ordinal 100 retained stock");a.list.Set({&q,&p,&r});
 b.list.nodes.assign(4096,{&q});for(int i=0;i<4095;++i)b.list.nodes[i].next=&b.list.nodes[i+1];
 routes.Set({&a,&b});Check(!ordinal(),"bounded large library scan");routes.Set({&a});
 Check(IsPinnedRouteDiamond(wxString::FromUTF8(argv[2])),"exact stock SVG");
 auto fresh=wxBitmapBundle::FromSVGFile(wxString::FromUTF8(argv[2]),wxSize(68,68)).GetBitmap(wxSize(68,68)).ConvertToImage();
 const auto cache_path=wxString::FromUTF8(std::string(argv[1])+"/diamond-cache.png");
 Check(fresh.SaveFile(cache_path,wxBITMAP_TYPE_PNG),"cache PNG fixture");wxImage cached(cache_path);
 Check(SameRouteDiamondPixels(cached,fresh),"real SVG cached PNG provenance");
 std::ifstream in(argv[2],std::ios::binary);std::string bytes((std::istreambuf_iterator<char>(in)),{});bytes[100]^=1;
 std::string bad=std::string(argv[1])+"/changed-diamond.svg";std::ofstream(bad,std::ios::binary)<<bytes;
 Check(!IsPinnedRouteDiamond(wxString::FromUTF8(bad)),"same-length changed source");Check(!IsPinnedRouteDiamond("missing-icon"),"missing source");
 Icons icons;WayPointman wm{&icons};WayPointmanGui gui{wm};wxImage image(16,16);image.SetRGB(wxRect(0,0,16,16),23,45,67);
 Check(SameRouteDiamondPixels(image,image.Copy()),"decoded image match");
 auto changed=image.Copy();changed.SetRGB(1,1,0,0,0);
 Check(!SameRouteDiamondPixels(image,changed),"changed cached pixel");
 changed=image.Copy();changed.InitAlpha();Check(!SameRouteDiamondPixels(image,changed),"changed cached alpha");
 auto*icon=gui.ProcessIcon(image,"diamond","custom");icon->piconBitmap=new wxBitmap(image);
 Check(!gui.IsPinnedRouteDiamond("diamond",icon->piconBitmap),"custom matching key");
 icon->skagerPinnedRouteDiamond=true;Check(gui.IsPinnedRouteDiamond("diamond",icon->piconBitmap),"verified original instance");
 wxBitmap stranger(image);Check(!gui.IsPinnedRouteDiamond("diamond",&stranger),"foreign bitmap");
 gui.ProcessIcon(image,"diamond","plugin override");icon->piconBitmap=new wxBitmap(image);
 Check(!gui.IsPinnedRouteDiamond("diamond",icon->piconBitmap),"replacement revokes ownership");
 wxBitmap sheet(420,230);wxMemoryDC native(sheet);native.SetBackground(wxBrush(wxColour(70,80,90)));native.Clear();
 ocpnDC dc;dc.native=&native;
 for(int theme=0;theme<3;++theme){c->theme=theme;for(int dpi:{100,125,150,200}){c->dpi=100;g_MarkScaleFactorExp=dpi/100.f;int x=45+(dpi==100?0:dpi==125?100:dpi==150?200:300),y=38+theme*76;
 Check(DrawChartRouteWaypoint(dc,*c,x,y,2),"raster painter");
 }}
 native.SelectObject(wxNullBitmap);sheet.SaveFile(wxString::FromUTF8(std::string(argv[1])+"/route-markers-day-dusk-night.png"),wxBITMAP_TYPE_PNG);
 // Exact 100% software ring exterior, foreground and floating fill; no erasure beyond bounds.
 auto im=sheet.ConvertToImage();Check(im.GetRed(45,22)==70,"outside ring untouched");
 Check(im.GetRed(45,28)==38&&im.GetGreen(45,28)==124,"prototype route ring");
 Check(im.GetRed(45,31)>200,"floating interior");
 dc.native=nullptr;g_MarkScaleFactorExp=1;c->dpi=125;c->theme=0;
 for(bool state:{true,false}){blended=state;program=19;draws=0;radii.clear();Check(DrawChartRouteWaypoint(dc,*c,40,40,2),"GL painter");
 Check(draws==2&&dc.last=="02","GL geometry and true ordinal label");Check(std::abs(radii[0]-13.75)<.01&&std::abs(radii[1]-11.25)<.01,"fractional GL outline");Check(program==19&&blended==state,"GL state restore");}
 style=false;Check(!DrawChartRouteWaypoint(dc,*c,40,40,2),"fallback draws nothing");style=true;
 Check(!DrawChartRouteWaypoint(dc,*c,40,40,0),"no invented ordinal");Check(!DrawChartRouteWaypoint(dc,*c,40,40,100),"no truncated ordinal");
 g_MarkScaleFactorExp=0;Check(!DrawChartRouteWaypoint(dc,*c,40,40,2),"invalid scale");
 delete icon->piconBitmap;delete icon;delete c;std::cout<<checks<<" marker checks passed\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
 wxTheApp->OnExit();wxEntryCleanup();return 0;
}

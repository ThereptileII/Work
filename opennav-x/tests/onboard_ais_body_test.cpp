// Offline geometry/real wx painter fixture. No target is injected into OpenCPN.
#include "integration/OnboardAisPaint.h"
#include "ui/Theme.h"
#include <wx/app.h>
#include <wx/dcmemory.h>
#include <wx/image.h>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace opennav;
using namespace opennav::integration;
namespace {
unsigned checks=0;
void Check(bool ok,const char *why) { ++checks;if(!ok)throw std::runtime_error(why); }
OnboardAisAppearance Healthy() {
  return {0,0,70,0,true,false,true,false,true,false,false,false,false,false,false,true,false};
}
bool InTriangle(double x,double y,const float *p) {
  bool positive=false,negative=false;
  for(int i=0;i<3;++i) {
    const int j=(i+1)%3;
    const double cross=(p[j*2]-p[i*2])*(y-p[i*2+1])-(p[j*2+1]-p[i*2+1])*(x-p[i*2]);
    positive |= cross>1e-5;negative |= cross < -1e-5;
  }
  return !(positive&&negative);
}
bool InMesh(double x,double y,const std::vector<float> &mesh) {
  for(std::size_t i=0;i<mesh.size();i+=6)if(InTriangle(x,y,&mesh[i]))return true;
  return false;
}
// Independent even/odd reference path, not the production triangle topology.
bool InSvg(double x,double y,bool class_b) {
  std::vector<std::pair<double,double>> p={{0,-12},{6,9}};
  if(class_b)p.push_back({0,5});
  p.push_back({-6,9});
  bool inside=false;
  for(std::size_t i=0,j=p.size()-1;i<p.size();j=i++) {
    if((p[i].second>y)!=(p[j].second>y) &&
       x<(p[j].first-p[i].first)*(y-p[i].second)/(p[j].second-p[i].second)+p[i].first)
      inside=!inside;
  }
  return inside;
}
wxColour Color(std::uint32_t n){return wxColour((n>>16)&255,(n>>8)&255,n&255);}
class App:public wxApp {public:bool OnInit() override{return true;}};
}
wxIMPLEMENT_APP_NO_MAIN(App);
int main(int argc,char **argv) {
  if(argc!=2)return 2;
  const std::string output=argv[1];
  if(!wxEntryStart(argc,argv)||!wxTheApp->CallOnInit())return 2;
  int result=0;
  try {
    wxInitAllImageHandlers();Check(!UseOnboardAisBody({}),"Missing appearance must fail closed");
    Check(OnboardAisInk(0x916477,ui::LightMode::Day)==Color(0x916477),"Exact Day SVG stroke");
    Check(OnboardAisInk(0x916477,ui::LightMode::Night)==Color(0x714E5D),"Night ancestor brightness applies to body ink");
    Check(OnboardAisInk(0x152129,ui::LightMode::Night)==Color(0x101A20),"Night ancestor brightness applies to floating fill");
    auto a=Healthy();Check(UseOnboardAisBody(a),"Normal Class A eligible");
    for(int kind=-1;kind<12;++kind)for(int nav=-1;nav<23;++nav) {
      a=Healthy();a.target_class=kind;a.navigation_status=nav;
      Check(UseOnboardAisBody(a)==((kind==0||kind==1)&&(nav==0||nav==8||(kind==1&&nav==15))),
            "Only ordinary A/B navigation status eligible");
    }
    for(int alert=-1;alert<4;++alert) {a=Healthy();a.alert=alert;
      Check(UseOnboardAisBody(a)==(alert==0),"All non-normal alarm states retained");}
    for(int ship=39;ship<=50;++ship) {a=Healthy();a.ship_type=ship;
      Check(UseOnboardAisBody(a)==(ship<40||ship>=50),"HSC ship-type overlay retained");}
    bool OnboardAisAppearance::*members[]={&OnboardAisAppearance::active,
      &OnboardAisAppearance::lost,&OnboardAisAppearance::position_valid,&OnboardAisAppearance::doubtful,
      &OnboardAisAppearance::name_valid,&OnboardAisAppearance::cached_name,&OnboardAisAppearance::inland,
      &OnboardAisAppearance::euro_inland,&OnboardAisAppearance::aircraft,&OnboardAisAppearance::follower,
      &OnboardAisAppearance::blue_paddle,&OnboardAisAppearance::direction_valid,&OnboardAisAppearance::realtime_prediction};
    for(auto member:members) {a=Healthy();a.*member=!(a.*member);
      Check(!UseOnboardAisBody(a),"Every abnormal state takes stock fallback");}
    for(bool class_b:{false,true})for(double angle:{0.,.731,2.4})for(double scale:{1.,1.25,1.5,2.}) {
      const auto m=OnboardAisBodyMesh(class_b,80,70,angle,scale);
      Check(!m.fill.empty()&&!m.outline.empty(),"Finite body and outline mesh");
      for(double x=-7.37;x<7;x+=.57)for(double y=-13.27;y<10;y+=.61) {
        const double px=80+scale*(x*std::cos(angle)-y*std::sin(angle));
        const double py=70+scale*(x*std::sin(angle)+y*std::cos(angle));
        Check(InMesh(px,py,m.fill)==InSvg(x,y,class_b),"GL triangles match independent SVG interior");
      }
    }
    for(double bad:{0.,-.1,17.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
      Check(OnboardAisBodyMesh(true,0,0,0,bad).fill.empty(),"Unsafe scale rejected");
    Check(OnboardAisBodyMesh(true,1e8,0,0,1).fill.empty(),"Unbounded coordinate rejected");
    Check(OnboardAisBodyMesh(true,0,0,std::numeric_limits<double>::quiet_NaN(),1).fill.empty(),"Invalid angle rejected");
    wxBitmap picture(720,260,24);wxMemoryDC dc(picture);
    for(int theme=0;theme<3;++theme) {
      const auto mode=static_cast<ui::LightMode>(theme);
      const auto palette=ui::OnlineChartTheme(mode);
      const auto fill=OnboardAisInk(palette.fill,mode),outline=OnboardAisInk(palette.stroke,mode);
      const auto background=Color(theme==0?0xD5E5E5:theme==1?0x344F59:0x121E24);
      dc.SetPen(*wxTRANSPARENT_PEN);dc.SetBrush(wxBrush(background));dc.DrawRectangle(theme*240,0,240,260);
      dc.SetTextForeground(Color(theme==0?0x233E3E:0xE1E5D8));
      dc.DrawText(theme==0?"DAY / OFFLINE FIXTURE":theme==1?"DUSK / OFFLINE FIXTURE":"NIGHT / OFFLINE FIXTURE",theme*240+8,10);
      dc.DrawText("Class A         Class B",theme*240+24,38);
      for(int kind=0;kind<2;++kind) {
        const int x=theme*240+65+kind*110,y=126;
        const auto pen=dc.GetPen();const auto brush=dc.GetBrush();
        Check(PaintOnboardAisBody(dc,OnboardAisBodyMesh(kind==1,x,y,0,4),fill,outline),"Actual wx painter draws");
        Check(dc.GetPen()==pen&&dc.GetBrush()==brush,"Native drawing state restored");
        wxColour pixel;dc.GetPixel(x,y,&pixel);Check(pixel==fill,"Body interior has prototype fill");
        dc.GetPixel(x,y+30,&pixel);Check(pixel==(kind==0?fill:background),"A stern stays solid, B notch stays empty");
        // Default DIP-size and rotated/user-enlarged samples below the large proof.
        Check(PaintOnboardAisBody(dc,OnboardAisBodyMesh(kind==1,x,212,0,1),fill,outline),"Default logical size draws");
        Check(PaintOnboardAisBody(dc,OnboardAisBodyMesh(kind==1,x+28,212,1.5707963267948966,1.5),fill,outline),"Rotation and scale draw");
      }
    }
    dc.SelectObject(wxNullBitmap);Check(picture.ConvertToImage().SaveFile(output,wxBITMAP_TYPE_PNG),"Fixture image saved");
    std::cout<<checks<<" checks passed\n";
  }catch(const std::exception &e){std::cerr<<e.what()<<" after "<<checks<<" checks\n";result=1;}
  wxTheApp->OnExit();wxEntryCleanup();return result;
}

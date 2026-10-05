// Offline native renderer fixture; production painter and current prepared
// ring function are compiled unchanged. Input adapters have no output transport.
#include "integration/ChartAnchorWatch.h"
#include "ui/Controls.h"
#include "chcanv.h"
#include "ocpndc.h"
#include "waypointman_gui.h"
#include <wx/app.h>
#include <wx/log.h>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

bool AnchorAlertOn1=false, AnchorAlertOn2=false, bGPSValid=true;
RoutePoint *pAnchorWatchPoint1=nullptr, *pAnchorWatchPoint2=nullptr;
float g_MarkScaleFactorExp=1;
WayPointman *pWayPointMan=nullptr;
namespace { bool xnav=true, requested=true; }
namespace opennav::integration {
bool XNavChartPresentationActive() { return xnav && requested; }
}
wxColour GetGlobalColor(const wxString &name) {
  return name == "UGREN" ? wxColour(0,255,0) : wxColour(255,0,0);
}
#include "anchor_ring_painters.inc"
namespace {
using namespace opennav;
void Check(bool value,const char *why) { if(!value) throw std::runtime_error(why); }
bool Geometry(const ocpnDC &a,const ocpnDC &b) {
  if(a.circles.size()!=b.circles.size()) return false;
  for(std::size_t i=0;i<a.circles.size();++i)
    if(a.circles[i].x!=b.circles[i].x || a.circles[i].y!=b.circles[i].y ||
       a.circles[i].radius!=b.circles[i].radius) return false;
  return true;
}
void RingChecks(ChartCanvas &canvas,RoutePoint &first,RoutePoint &second,ui::LightMode mode) {
  const auto colors=ui::Theme(mode);
  for(int sign : {-1,1}) {
    first.radius=sign*70.; second.radius=-sign*90.;
    for(int alerts=0;alerts<4;++alerts) for(bool gps : {false,true}) {
      AnchorAlertOn1=(alerts&1)!=0; AnchorAlertOn2=(alerts&2)!=0; bGPSValid=gps;
      for(int fallback=0;fallback<3;++fallback) {
        xnav=fallback!=1; requested=fallback!=2;
        ocpnDC actual,stock;
        canvas.DrawAnchorWatchPoints(actual); canvas.DrawOriginalAnchorWatchPoints(stock);
        Check(Geometry(actual,stock),"Watch1/2 center, signed entry/exit radius and count match pinned renderer");
        Check(actual.circles.size()==2,"Both selected watches remain visible");
        for(std::size_t index=0;index<2;++index) {
          const bool watch_two=actual.circles[index].x==static_cast<int>(second.m_lon);
          const bool alarm=watch_two ? AnchorAlertOn2 : AnchorAlertOn1;
          const bool entry=(watch_two ? second.radius : first.radius)<0;
          const auto &pen=actual.circles[index].pen;
          if(fallback) {
            Check(pen==stock.circles[index].pen,"Legacy and Standard retain original ring ink, width and style");
          } else {
            const auto expected=alarm ? colors.alarm : !gps ? colors.attention :
                entry ? colors.alarm : ui::ActiveRouteInk(mode);
            Check(pen.GetColour()==ui::Colour(expected),"Each watch paints its own alarm/GPS/entry state in current theme");
            Check(pen.GetWidth()==(alarm?4:2),"Alarm emphasis derives from corresponding upstream watch flag");
            Check(pen.GetStyle()==(!alarm&&!gps ? wxPENSTYLE_SHORT_DASH : wxPENSTYLE_SOLID),
                  "GPS loss is uncertain and dashed; a real alarm retains priority");
          }
        }
      }
    }
  }
}
bool DrawObservedMark(ocpnDC &dc, ChartCanvas &canvas, RoutePoint &point) {
  const bool pinned=pWayPointMan && WayPointmanGui(*pWayPointMan).IsPinnedAnchor(point.GetIconName(),point.m_pbmIcon);
  return integration::DrawChartAnchorMark(dc,canvas,point,100,120,pinned);
}
void MarkChecks(ChartCanvas &canvas,RoutePoint &first,RoutePoint &second,ui::LightMode mode) {
  xnav=requested=true;
  const auto colors=ui::Theme(mode);
  for(auto *point : {&first,&second}) for(bool alarm : {false,true}) for(bool gps : {false,true}) for(bool entry : {false,true}) {
    AnchorAlertOn1=point==&first && alarm; AnchorAlertOn2=point==&second && alarm; bGPSValid=gps;
    point->name=entry ? "-100" : "100";
    ocpnDC actual;
    Check(DrawObservedMark(actual,canvas,*point),"Actual production anchor mark paints both selected watches");
    Check(actual.marks==1 && actual.last_mark.IsOk() && actual.last_mark.GetSize()==wxSize(28,28),
          "Production SVG produces an actual centered native bitmap");
    Check(actual.bounds.size()==2 && actual.bounds[0]==wxPoint(86,106) && actual.bounds[1]==wxPoint(114,134),
          "Native bounding box includes complete centered anchor artwork");
    const auto expected=ui::Colour(alarm ? colors.alarm : !gps ? colors.attention : entry ? colors.alarm : ui::ActiveRouteInk(mode));
    bool has_ink=false;
    for(int y=0;y<28;++y) for(int x=0;x<28;++x)
      if(actual.last_mark.GetRed(x,y)==expected.Red() && actual.last_mark.GetGreen(x,y)==expected.Green() &&
         actual.last_mark.GetBlue(x,y)==expected.Blue()) has_ink=true;
    Check(has_ink,"Painted icon pixels contain the correct watch state ink (cache must not leak prior state)");
  }
  for(int gate=0;gate<10;++gate) {
    RoutePoint other=first;
    auto *point=&first;
    first.icon="anchor"; first.m_bIsActive=first.m_bBlink=first.m_bRPIsBeingEdited=first.dragging=false;
    xnav=requested=true; g_MarkScaleFactorExp=1;
    if(gate==0) xnav=false;
    if(gate==1) requested=false;
    if(gate==2) first.icon="user-custom-anchor";
    if(gate==3) first.m_bIsActive=true;
    if(gate==4) first.m_bBlink=true;
    if(gate==5) first.m_bRPIsBeingEdited=true;
    if(gate==6) first.dragging=true;
    if(gate==7) point=&other;
    if(gate==8) pWayPointMan->icons.icons.front()->skagerPinnedAnchor=false;
    wxBitmap replacement(28,28);
    if(gate==9) first.m_pbmIcon=&replacement;
    ocpnDC actual;
    Check(!DrawObservedMark(actual,canvas,*point) && actual.marks==0,
          "Legacy, Standard, custom, active, blinking, editing, dragging, unselected and unverified same-key marks fall through");
    pWayPointMan->icons.icons.front()->skagerPinnedAnchor=true;
    first.m_pbmIcon=pWayPointMan->icons.icons.front()->piconBitmap;
  }
  first.icon="anchor"; first.m_bIsActive=first.m_bBlink=first.m_bRPIsBeingEdited=first.dragging=false;
  xnav=requested=true;
  for(float scale : {std::numeric_limits<float>::quiet_NaN(),0.f,9.f}) {
    g_MarkScaleFactorExp=scale; ocpnDC actual;
    Check(!DrawObservedMark(actual,canvas,first),"Invalid/unbounded scale retains original marker");
  }
  g_MarkScaleFactorExp=1;
}
} // namespace
class TestApp final : public wxApp {
public:
  bool OnInit() override {
    wxLog::SetActiveTarget(new wxLogStderr());
    try {
      RoutePoint first,second; first.m_lat=120; first.m_lon=100; second.m_lat=240; second.m_lon=250;
      wxBitmap pinned(28,28);
      MarkIcon icon; icon.piconBitmap=&pinned; icon.skagerPinnedAnchor=true;
      WayPointman manager; manager.icons.icons.push_back(&icon); pWayPointMan=&manager;
      first.m_pbmIcon=second.m_pbmIcon=&pinned;
      pAnchorWatchPoint1=&first; pAnchorWatchPoint2=&second;
      ChartCanvas canvas;
      for(int scheme=GLOBAL_COLOR_SCHEME_DAY;scheme<=GLOBAL_COLOR_SCHEME_NIGHT;++scheme) {
        canvas.scheme=scheme;
        const auto mode=scheme==GLOBAL_COLOR_SCHEME_NIGHT ? ui::LightMode::Night :
            scheme==GLOBAL_COLOR_SCHEME_DUSK ? ui::LightMode::Dusk : ui::LightMode::Day;
        RingChecks(canvas,first,second,mode); MarkChecks(canvas,first,second,mode);
      }
      std::cout<<"Anchor watch production renderer checks passed\n";
    } catch(const std::exception &error) { result_=1; std::cerr<<error.what()<<'\n'; }
    pAnchorWatchPoint1=pAnchorWatchPoint2=nullptr; pWayPointMan=nullptr;
    CallAfter([this]{ExitMainLoop();}); return true;
  }
  int OnRun() override { wxApp::OnRun(); return result_; }
  int OnExit() override { return result_; }
private: int result_=0;
};
wxIMPLEMENT_APP(TestApp);

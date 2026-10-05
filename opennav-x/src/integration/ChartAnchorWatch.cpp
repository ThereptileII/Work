#include "integration/ChartAnchorWatch.h"
#include "integration/ChartPresentation.h"
#include "ui/PrototypeIcons.h"
#include "chcanv.h"
#include "ocpndc.h"
#include "model/route_point.h"
#include "model/own_ship.h"
#include <wx/bmpbndl.h>
#include <wx/thread.h>
#include <cmath>
#include <map>
#include <tuple>
extern bool AnchorAlertOn1, AnchorAlertOn2;
extern RoutePoint *pAnchorWatchPoint1, *pAnchorWatchPoint2;
extern float g_MarkScaleFactorExp;
namespace opennav::integration {
namespace {
ui::LightMode Mode(ChartCanvas &canvas) {
  return canvas.GetColorScheme()==GLOBAL_COLOR_SCHEME_NIGHT ? ui::LightMode::Night
      : canvas.GetColorScheme()==GLOBAL_COLOR_SCHEME_DUSK ? ui::LightMode::Dusk : ui::LightMode::Day;
}
}
bool ChartAnchorWatchInk(ChartCanvas &canvas, bool alarm, bool position_valid,
                         bool entry_watch, wxColour &ink) {
  if(!wxIsMainThread() || !XNavChartPresentationActive()) return false;
  const auto mode=Mode(canvas);
  ink=ui::Colour(AnchorWatchInk(mode,alarm,position_valid,entry_watch));
  return true;
}
int ChartAnchorWatchExtent(ChartCanvas &canvas, RoutePoint &point, bool pinned_icon) {
  if(!wxIsMainThread() || !XNavChartPresentationActive() ||
      (&point!=pAnchorWatchPoint1 && &point!=pAnchorWatchPoint2) ||
      !pinned_icon || point.GetIconName()!="anchor" ||
      point.m_bIsActive || point.m_bBlink ||
      point.m_bRPIsBeingEdited || point.IsDragHandleEnabled()) return 0;
  const double scale=canvas.FromDIP(100)/100.*g_MarkScaleFactorExp;
  return std::isfinite(scale) && scale>=.25 && scale<=8 ? int(std::ceil(14*scale)) : 0;
}
bool DrawChartAnchorMark(ocpnDC &dc, ChartCanvas &canvas, RoutePoint &point,int x,int y,bool pinned_icon) {
  const int radius=ChartAnchorWatchExtent(canvas,point,pinned_icon);
  if(!radius) return false;
  const auto mode=Mode(canvas);
  const bool alarm=(&point==pAnchorWatchPoint1 && AnchorAlertOn1) ||
                   (&point==pAnchorWatchPoint2 && AnchorAlertOn2);
  double distance=0;
  const bool entry=point.GetName().ToDouble(&distance) && distance<0;
  const auto color=AnchorWatchInk(mode,alarm,bGPSValid,entry);
  const auto fill=ui::FloatingTheme(mode).surface;
  // Small bounded artwork cache. It stores only prototype pixels, no route or
  // waypoint pointers; theme/quality/DPI changes select a different key.
  using Key=std::tuple<int,std::uint32_t,std::uint32_t>;
  static std::map<Key,wxBitmap> cache;
  const Key key{radius,color,fill};
  auto found=cache.find(key);
  if(found==cache.end()) {
    if(cache.size()>=12) cache.clear();
    const auto svg=wxString::Format(
      "<svg xmlns='http://www.w3.org/2000/svg' width='28' height='28' viewBox='0 0 28 28'>"
      "<circle cx='14' cy='14' r='13' fill='#%06x'/>"
      "<path transform='translate(2 2)' d='%s' fill='none' stroke='#%06x' stroke-width='1.65' stroke-linecap='round' stroke-linejoin='round'/></svg>",
      static_cast<unsigned>(fill),wxString::FromUTF8(ui::PrototypeIconPath(ui::XNavIcon::Anchor)),
      static_cast<unsigned>(color));
    const auto bundle=wxBitmapBundle::FromSVG(svg.utf8_str(),wxSize(radius*2,radius*2));
    if(!bundle.IsOk()) return false;
    found=cache.emplace(key,bundle.GetBitmap(wxSize(radius*2,radius*2))).first;
  }
  if(!found->second.IsOk()) return false;
  dc.DrawBitmap(found->second,x-radius,y-radius,true);
  dc.CalcBoundingBox(x-radius,y-radius); dc.CalcBoundingBox(x+radius,y+radius);
  return true;
}
}

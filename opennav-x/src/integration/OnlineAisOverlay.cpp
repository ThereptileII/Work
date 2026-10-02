#include "integration/OnlineAisOverlay.h"
#include "integration/ChartPresentation.h"
#include "integration/OnlineAisLabels.h"
#include "ui/Theme.h"
#include "chcanv.h"
#include "ocpndc.h"
#include "viewport.h"
#include "model/georef.h"
#include <cmath>
#include <wx/thread.h>

extern ColorScheme global_color_scheme;
namespace opennav::integration {
namespace {
wxColour Color(std::uint32_t c) {
  return {static_cast<unsigned char>(c>>16),static_cast<unsigned char>(c>>8),
          static_cast<unsigned char>(c)};
}
bool Project(ChartCanvas &canvas, ViewPort &vp, const ais::ChartTarget &t,
             wxPoint &point) {
  if (!vp.IsValid() || !canvas.GetCanvasPointPixVP(vp,t.latitude,t.longitude,&point))
    return false;
  // Upstream may return true with INVALID_COORD; clip before arithmetic/drawing.
  return point.x>=-32 && point.y>=-32 && point.x<=vp.pix_width+32 &&
         point.y<=vp.pix_height+32;
}
std::optional<double> Direction(ChartCanvas &canvas, ViewPort &vp,
                                const ais::ChartTarget &t, wxPoint point) {
  if (!t.direction_true || !std::isfinite(vp.view_scale_ppm) ||
      vp.view_scale_ppm<=0) return {};
  // Plot a short bearing with the existing OpenCPN georeferencing library and
  // actual canvas projection, including raster georeferencing. This is solely
  // symbol orientation: no new vessel position, route distance or COG vector.
  const double distance=(100.0/vp.view_scale_ppm)/1852.0;
  if (!std::isfinite(distance) || distance<=0 || distance>100) return {};
  double lat=0,lon=0;
  ll_gc_ll(t.latitude,t.longitude,*t.direction_true,distance,&lat,&lon);
  if (!std::isfinite(lat)||!std::isfinite(lon)||std::abs(lat)>90) return {};
  wxPoint ahead;
  if (!canvas.GetCanvasPointPixVP(vp,lat,lon,&ahead) ||
      std::abs(static_cast<double>(ahead.x))>1e6 ||
      std::abs(static_cast<double>(ahead.y))>1e6 || ahead==point) return {};
  return std::atan2(static_cast<double>(ahead.y)-point.y,
                    static_cast<double>(ahead.x)-point.x)+3.14159265358979323846/2;
}
} // namespace
bool OnlineAisOverlay::Update(const vessel::AisState &display, vessel::Time now,
                              int selected) {
  if (!wxIsMainThread()) return false;
  auto next=ais::OnlineChartTargets(display,now,selected);
  if (targets_==next) return false;
  targets_=std::move(next);return true;
}
void OnlineAisOverlay::Clear() { if(wxIsMainThread()) targets_.clear(); }
void OnlineAisOverlay::Draw(ocpnDC &dc, ViewPort &vp, ChartCanvas &canvas) const {
  if(!wxIsMainThread()||!canvas.GetShowAIS())return;
  const auto mode=global_color_scheme==GLOBAL_COLOR_SCHEME_NIGHT?ui::LightMode::Night:
      global_color_scheme==GLOBAL_COLOR_SCHEME_DUSK?ui::LightMode::Dusk:ui::LightMode::Day;
  const auto colors=ui::OnlineChartTheme(mode);
  const auto pen=dc.GetPen();const auto brush=dc.GetBrush();
  const double scale=canvas.FromDIP(100)/100.0;
  const auto now=vessel::Clock::now();
  std::vector<OnlineAisLabelTarget> positioned;
  for(const auto &stored:targets_) {
    const auto current=ais::CurrentChartMark(stored,now);if(!current)continue;
    wxPoint point;if(Project(canvas,vp,*current,point))positioned.push_back({*current,point});
  }
  // Labels are only part of verified XNav chart presentation. Standard keeps
  // its existing supplemental symbols, with no new label styling.
  wxColour land,water;
  if(ChartBackground(canvas.GetColorScheme(),land,water)) {
    DrawOnlineAisLabels(dc,canvas,mode,{vp.pix_width,vp.pix_height},positioned);
  }
  // Symbols, age/provenance marks and selection rings stay above their own
  // optional labels. Labels never enlarge the existing native target hit area.
  for(const auto &p:positioned) {
    const auto &t=p.mark;const auto point=p.point;
    const bool old=t.age==ais::TargetAge::Stale||t.age==ais::TargetAge::Lost;
    const auto stroke=Color(old?colors.stale:colors.stroke);
    dc.SetPen(wxPen(stroke,canvas.FromDIP(2),
        old||t.age==ais::TargetAge::Aging?wxPENSTYLE_SHORT_DASH:wxPENSTYLE_SOLID));
    dc.SetBrush(wxBrush(Color(t.selected?colors.selected:colors.fill)));
    const auto angle=Direction(canvas,vp,t,point);
    if(angle) {
      // Exact reference path M0-12 6 9 0 5-6 9Z, scaled only by Windows DPI.
      wxPoint vertices[4];const int x[4]={0,6,0,-6},y[4]={-12,9,5,9};
      for(int i=0;i<4;++i)vertices[i]={point.x+wxRound(scale*(x[i]*std::cos(*angle)-y[i]*std::sin(*angle))),
        point.y+wxRound(scale*(x[i]*std::sin(*angle)+y[i]*std::cos(*angle)))};
      dc.DrawPolygon(4,vertices);
    } else {
      dc.SetBrush(*wxTRANSPARENT_BRUSH);
      dc.DrawCircle(point,canvas.FromDIP(6));
    }
    if(old) {
      const int r=canvas.FromDIP(8);
      dc.SetPen(wxPen(stroke,canvas.FromDIP(2)));
      dc.DrawLine(point.x-r,point.y-r,point.x+r,point.y+r);
      if(t.age==ais::TargetAge::Lost)dc.DrawLine(point.x-r,point.y+r,point.x+r,point.y-r);
    } else {
      // One small provenance dot, not an ONLINE label over every target.
      dc.SetPen(*wxTRANSPARENT_PEN);dc.SetBrush(wxBrush(stroke));
      dc.DrawCircle(point.x+canvas.FromDIP(10),point.y+canvas.FromDIP(10),canvas.FromDIP(2));
    }
    if(t.selected) {
      dc.SetBrush(*wxTRANSPARENT_BRUSH);dc.SetPen(wxPen(Color(colors.selected),canvas.FromDIP(1)));
      dc.DrawCircle(point,canvas.FromDIP(18));
    }
  }
  dc.SetBrush(brush);dc.SetPen(pen);
}
int OnlineAisOverlay::HitTest(ViewPort &vp, ChartCanvas &canvas,int x,int y) const {
  if(!wxIsMainThread()||!canvas.GetShowAIS())return 0;
  const auto radius=canvas.FromDIP(24);double best=static_cast<double>(radius)*radius;
  int result=0;bool ambiguous=false;
  const auto now=vessel::Clock::now();
  for(const auto &stored:targets_) {
    const auto current=ais::CurrentChartMark(stored,now);if(!current)continue;
    const auto &t=*current;
    wxPoint p;if(!Project(canvas,vp,t,p))continue;
    const double dx=static_cast<double>(p.x)-x,dy=static_cast<double>(p.y)-y,d=dx*dx+dy*dy;
    if(d<best){best=d;result=t.mmsi;ambiguous=false;}
    else if(d==best&&result&&result!=t.mmsi)ambiguous=true;
  }
  return ambiguous?0:result;
}
} // namespace opennav::integration

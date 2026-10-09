#include "ui/PreviewPanel.h"
#include "application/Version.h"
#include "integration/BuildFeatures.h"
#include "vessel/DataItems.h"
#include <algorithm>
#include <cmath>
#include <wx/dcbuffer.h>

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
wxString Dash() { return wxString::FromUTF8("—"); }
struct Painter : XNavPainter {
  wxDC &dc;
  vessel::Time now;
  Painter(wxWindow &window, wxDC &drawing, LightMode mode, vessel::Time at)
      : XNavPainter(window, drawing, mode), dc(drawing), now(at) {}
  void Value(const vessel::Sample &sample, int x, int y, const wxString &unit,
             int precision = 1, int size = 48, int width = 200) {
    const auto reading = vessel::Assess(sample, now);
    const bool stale = reading.quality == vessel::Quality::Stale;
    const auto number = reading.value && !stale
                            ? wxString::Format("%.*f", precision, *reading.value)
                            : Dash();
    Text(number, x, y, size, stale ? c.muted : c.primary, false, width);
    Text(unit, x, y + size + 4, 12, c.secondary, false, width);
    if (reading.quality != vessel::Quality::Live) {
      auto status = reading.quality == vessel::Quality::Unavailable
                        ? wxString("NO DATA")
                        : W(vessel::QualityName(reading.quality));
      if (stale && reading.age)
        status += wxString::Format(" / %.0f s", reading.age->count() / 1000.0);
      Text(status, x, y + size + 24, 11,
           stale || reading.quality == vessel::Quality::Uncertain
               ? c.attention : c.muted, false, width);
    }
  }
  void Estimate(std::optional<double> value, int x, int y, const wxString &unit,
                int precision = 0, int size = 48, int width = 220) {
    Text(value ? wxString::Format("%.*f", precision, *value) : Dash(), x, y,
         size, value ? c.accent : c.muted, false, width);
    Text(unit, x, y + size + 4, 12, c.secondary, false, width);
  }
};
wxString ApproxTime(std::optional<double> seconds) {
  if (!seconds || !std::isfinite(*seconds) || *seconds < 0 || *seconds >= 3600000)
    return "Time unavailable";
  const auto minutes = static_cast<unsigned>(*seconds / 60.0);
  if (minutes < 1) return "Less than a minute";
  if (minutes < 60) return wxString::Format("~%u min", minutes);
  return wxString::Format("~%u h %02u min", minutes / 60, minutes % 60);
}
wxString EnergyMessage(const smartnav::Prediction<smartnav::ArrivalEstimate> &p) {
  if (p.estimate) {
    if (p.estimate->energy_shortfall_kwh > 0) return "Not enough energy to reach destination";
    if (p.estimate->below_reserve) return "Arrival below your reserve";
    return "Above your configured reserve";
  }
  if (p.reason == smartnav::EnergyReason::InvalidModel)
    return "Set battery capacity and reserve in Settings";
  if (p.input == smartnav::EnergyInput::BatteryIdentity)
    return "Battery source not identified";
  // Data names are human-facing; protocol/source details belong in Diagnostics.
  return W(smartnav::EnergyStatus(p.reason, p.input));
}
const smartnav::AdvisoryEvent *Event(const smartnav::NavigationAdvice &advice,
                                   smartnav::EventKind kind) {
  if (!advice.route_valid) return nullptr;
  for (const auto &event : advice.events)
    if (event.kind == kind) return &event;
  return nullptr;
}
std::optional<double> Distance(const vessel::VesselState &s, vessel::Time now) {
  return s.navigation.route ? vessel::AssessRoute(*s.navigation.route, now)
                                  .remaining_distance_nm
                            : std::nullopt;
}
wxString DestinationName(const vessel::VesselState &s) {
  if (!s.navigation.route || s.navigation.route->route_id.empty() ||
      s.navigation.route->state == vessel::RouteState::NoActiveRoute)
    return "No active route";
  const auto &route = *s.navigation.route;
  if (!route.remaining_steps.empty() &&
      !route.remaining_steps.back().name.empty())
    return W(route.remaining_steps.back().name);
  return route.route_name.empty() ? "OpenCPN active route"
                                  : W(route.route_name);
}
wxString PointName(const vessel::VesselState &s) {
  if (!s.navigation.route || s.navigation.route->active_waypoint_id.empty())
    return "No active waypoint";
  const auto &route = *s.navigation.route;
  if (!route.remaining_steps.empty() &&
      route.remaining_steps.front().waypoint_id == route.active_waypoint_id &&
      !route.remaining_steps.front().name.empty())
    return W(route.remaining_steps.front().name);
  return "Unnamed waypoint";
}
} // namespace
PreviewPanel::PreviewPanel(wxWindow *parent)
    : XNavScroll(parent) {
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  SetScrollRate(0, FromDIP(24));
  close_=new XNavButton(this,wxID_ANY,"Close","Close this page");
  close_->SetRole(ButtonRole::Quiet);
  close_->SetIcon(XNavIcon::Close);
  close_->SetInlineIcon();
  close_->SetMinSize(wxSize(close_->InlineWidth(80),FromDIP(44)));
  close_->Bind(wxEVT_BUTTON,[this](wxCommandEvent&){if(close_action_)close_action_();});
  Bind(wxEVT_PAINT, &PreviewPanel::Paint, this);
  Bind(wxEVT_SIZE, [this](wxSizeEvent &e) {
    // Child positions are viewport-relative, but the header belongs to the
    // scrolling content. A resize must not add the current scroll offset.
    const int close_width = close_->InlineWidth(80);  // SCRUM-356
    const auto header = CalcScrolledPosition(
        wxPoint(GetClientSize().x-FromDIP(32)-close_width,FromDIP(28)));
    close_->SetSize(header.x,header.y,close_width,FromDIP(44));
    Refresh();
    e.Skip();
  });
}
void PreviewPanel::Update(PreviewPage page, LightMode mode,
                          const vessel::VesselState &s, vessel::Time now,
                          const smartnav::EnergyModel &model,
                          const smartnav::EnergyPrediction &energy,
                          const std::vector<std::string> &info,
                          const smartnav::NavigationAdvice &advice) {
  if (page != page_)
    Scroll(0, 0);
  page_ = page;
  SetLabel(page == PreviewPage::Route    ? "SKAGER page: Route"
           : page == PreviewPage::Energy ? "SKAGER page: Energy"
                                         : "SKAGER page: Diagnostics");
  mode_ = mode;
  SetBackgroundColour(Colour(Theme(mode).background));
  close_->SetLightMode(mode);
  state_ = s;
  now_ = now;
  model_ = model;
  energy_ = energy;
  energy_view_ = application::PresentEnergy(s, model, energy, advice, now);
  info_ = info;
  advice_ = advice;
  Refresh();
}
void PreviewPanel::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);
  PrepareDC(dc);
  const auto c = Theme(mode_);
  dc.SetBackground(wxBrush(Colour(c.background)));
  dc.Clear();
  Painter p{*this, dc, mode_, now_};
  const int width = std::max(320, ToDIP(GetClientSize().x)), margin = 24,
            gap = 16;
  const auto &model = model_;
  const auto &energy = page_ == PreviewPage::Energy ? energy_view_.prediction : energy_;
  const auto distance = page_ == PreviewPage::Energy
                            ? energy_view_.passage.distance_nm : Distance(state_, now_);
  const auto arrival = energy.arrival.estimate;
  const wxString title = page_ == PreviewPage::Energy  ? "Energy"
                         : page_ == PreviewPage::Route ? "Passage"
                                                       : "Diagnostics";
  if(page_!=PreviewPage::Energy) p.Text(title, margin, 20, 28, c.primary);
  wxString subtitle = page_ == PreviewPage::Energy
                          ? "Battery, propulsion and estimated arrival"
                      : page_ == PreviewPage::Route
                          ? "Your destination and the next maneuver"
                          : "System details and data health";
  if (state_.replayed) subtitle = "REPLAY / Historical data / Controls disabled";
#if XNAV_ENABLE_TEST_FIXTURES
  else if (state_.simulated) subtitle = "DEMO / Synthetic test trip / No device output";
#endif
  if(page_!=PreviewPage::Energy) p.Text(subtitle, margin, 60, 13,
         state_.simulated || state_.replayed ? c.attention : c.secondary, false,
         width - 2 * margin);
  const auto *destination = Event(advice_, smartnav::EventKind::Destination);
  const auto *turn = Event(advice_, smartnav::EventKind::Turn);
  int bottom = 0;
  if (page_ == PreviewPage::Energy) {
    // Exact v8 composition: two unequal main cards, then four compact values.
    // Content comes only from assessed Vessel Data and the existing model.
    const int inset=32, space=18, available=width-2*inset;
    const bool wide=available>=720;
    const int left=wide?static_cast<int>(std::lround((available-space)*1.1/2.1)):available;
    const int right=wide?available-space-left:available;
    const int y=146, height=423, rx=wide?inset+left+space:inset;
    const int ry=wide?y:y+height+space;
    p.TextTracked("ELECTRIC PASSAGE",inset,36,9,c.accent,650,1.17);
    p.TextTracked("More horizon. Less uncertainty.",inset,56,30,c.primary,400,-1.2,available-95);
    p.Text(state_.replayed ? "REPLAY / Historical data / Controls disabled" :
      state_.simulated ? "TEST DATA / Not live" :
      "Your energy picture, from this moment to your destination.",
      inset,104,12,c.muted,false,available);
    p.Card(inset,y,left,height,"");
    p.Card(rx,ry,right,height,"");
    const auto usable=[](const vessel::Assessment &a){
      return a.value && (a.quality==vessel::Quality::Live || a.quality==vessel::Quality::Aging ||
                         a.quality==vessel::Quality::Estimated);
    };
    const auto &soc=energy_view_.soc;
    const bool valid_soc=usable(soc)&&*soc.value>=0&&*soc.value<=100;
    const auto big = [&](std::optional<double> value, int bx, int by,
                         const wxString &unit, std::uint32_t ink, int maximum) {
      const auto text = value ? wxString::Format("%.0f", *value) : Dash();
      dc.SetFont(UiFontWeight(*this,60,350));
      const auto extent = dc.GetTextExtent(text);
      const int top = by - (ToDIP(extent.y) - 72) / 2;
      p.TextTracked(text,bx,top,60,ink,350,-2.0,maximum);
      const int unit_x = bx + ToDIP(extent.x) - 2*(static_cast<int>(text.size())-1) + 6;
      p.Text(unit,unit_x,by+43,14,c.secondary,false,std::max(1,maximum-unit_x+bx));
    };
    p.TextWeight("Energy on board",inset+25,y+25,12,c.secondary,500);
    big(soc.value,inset+25,y+59,"%",valid_soc?c.accent:c.muted,left-140);
    p.Text(energy_view_.remaining_kwh?wxString::Format("~%.1f kWh remaining",*energy_view_.remaining_kwh):
           "Usable energy unavailable",inset+25,y+137,10,c.secondary,false,left-120);
    wxString battery_health=valid_soc?wxString(soc.quality==vessel::Quality::Aging?"AGING":
           soc.quality==vessel::Quality::Estimated?"ESTIMATED":"CURRENT"):
           W(vessel::QualityName(soc.quality));
    if(soc.age) battery_health+=wxString::Format(W(" · %.1f s"),soc.age->count()/1000.);
    p.Tag(battery_health,inset+25,y+157,left-120,!valid_soc,valid_soc);
    dc.SetPen(wxPen(Colour(c.border)));dc.SetBrush(wxBrush(Colour(c.surface)));
    dc.DrawRoundedRectangle(p.D(inset+left-90),p.D(y+64),p.D(66),p.D(114),p.D(11));
    dc.SetPen(*wxTRANSPARENT_PEN);dc.SetBrush(wxBrush(Colour(c.border)));
    dc.DrawRoundedRectangle(p.D(inset+left-68),p.D(y+59),p.D(22),p.D(6),p.D(2));
    if(valid_soc) {
      const int fill=static_cast<int>(100*std::clamp(*soc.value,0.,100.)/100.);
      dc.SetBrush(wxBrush(Colour(c.accent)));
      dc.DrawRoundedRectangle(p.D(inset+left-83),p.D(y+172-fill),p.D(52),p.D(fill),p.D(3));
    }
    p.TextTracked("ESTIMATED BATTERY OVER THIS PASSAGE",inset+25,y+208,10,c.secondary,400,1.5,left-50);
    // The model returns endpoint energy, not a sampled voyage profile. Draw
    // that honest steady-consumption segment, never the mock's curved history.
    if(valid_soc&&arrival&&arrival->soc_percent) {
      const int gx=inset+78,gy=y+247,gw=std::max(1,left-156),gh=88;
      const auto ordinate=[&](double v){return gy+gh-static_cast<int>(gh*std::clamp(v,0.,100.)/100.);};
      // Prototype's literal #b6efce0c area ink, with an honest linear forecast
      // endpoint instead of the reference's illustrative curved history.
      const auto fill=Colour(prototype_ink::active),back=Colour(c.surface);
      dc.SetBrush(wxBrush(wxColour((fill.Red()*12+back.Red()*243+127)/255,
           (fill.Green()*12+back.Green()*243+127)/255,
           (fill.Blue()*12+back.Blue()*243+127)/255)));
      dc.SetPen(*wxTRANSPARENT_PEN);
      const wxPoint area[]={{p.D(gx),p.D(ordinate(*soc.value))},
        {p.D(gx+gw),p.D(ordinate(*arrival->soc_percent))},
        {p.D(gx+gw),p.D(gy+gh)},{p.D(gx),p.D(gy+gh)}};
      dc.DrawPolygon(4,area);
      dc.SetPen(wxPen(Colour(c.border),1,wxPENSTYLE_DOT));
      for(int percent:{25,50,75})
        dc.DrawLine(p.D(gx),p.D(ordinate(percent)),p.D(gx+gw),p.D(ordinate(percent)));
      dc.DrawLine(p.D(gx),p.D(gy+gh),p.D(gx+gw),p.D(gy+gh));
      if(std::isfinite(model.reserve_soc_percent)) {
        dc.SetPen(wxPen(Colour(c.attention),1,wxPENSTYLE_DOT));
        dc.DrawLine(p.D(gx),p.D(ordinate(model.reserve_soc_percent)),p.D(gx+gw),p.D(ordinate(model.reserve_soc_percent)));
        p.Text(wxString::Format("%.0f%% reserve",model.reserve_soc_percent),gx+gw-74,
               ordinate(model.reserve_soc_percent)-14,8,c.muted);
      }
      dc.SetPen(wxPen(Colour(c.accent),p.D(2)));
      dc.DrawLine(p.D(gx),p.D(ordinate(*soc.value)),p.D(gx+gw),p.D(ordinate(*arrival->soc_percent)));
      dc.SetBrush(wxBrush(Colour(c.accent)));dc.SetPen(*wxTRANSPARENT_PEN);
      dc.DrawCircle(p.D(gx+gw),p.D(ordinate(*arrival->soc_percent)),p.D(3));
      p.Text("NOW",gx,gy+gh+6,9,c.muted);
      p.Text("ESTIMATED ARRIVAL",gx+gw-105,gy+gh+6,9,c.muted);
      p.Text("Steady consumption estimate",inset+24,y+377,10,c.secondary,false,left-48);
    } else {
      p.Text("Passage estimate unavailable",inset+24,y+270,16,c.muted,false,left-48);
      p.Text(EnergyMessage(energy.arrival),inset+24,y+300,12,c.secondary,false,left-48);
    }
    p.TextWeight(distance?"At "+DestinationName(state_):wxString("At destination"),rx+25,ry+25,12,c.secondary,500,right-50);
    big(arrival?arrival->soc_percent:std::nullopt,rx+25,ry+59,"% estimated",
         !arrival?c.muted:arrival->below_reserve?c.attention:c.accent,right-50);
    const bool reserve=arrival&&arrival->soc_percent&&std::isfinite(model.reserve_soc_percent);
    p.Text(reserve?wxString::Format("%.0f%% ",std::abs(*arrival->soc_percent-model.reserve_soc_percent))+
            (*arrival->soc_percent>=model.reserve_soc_percent?"above":"below")+" your reserve":
           "Reserve estimate unavailable",rx+25,ry+137,10,c.secondary,false,right-50);
    dc.SetPen(wxPen(Colour(c.border)));dc.SetBrush(wxBrush(Colour(c.selected)));
    dc.DrawRoundedRectangle(p.D(rx+24),p.D(ry+171),p.D(right-48),p.D(72),p.D(8));
    const auto advisory_color=!arrival?c.muted:arrival->below_reserve?c.attention:c.accent;
    dc.SetPen(wxPen(Colour(advisory_color),p.D(2)));
    dc.DrawLine(p.D(rx+25),p.D(ry+171),p.D(rx+25),p.D(ry+243));
    const bool shortfall=arrival&&arrival->energy_shortfall_kwh>0;
    p.Text(arrival?(shortfall?"Insufficient passage energy":arrival->below_reserve?"Energy below your reserve":"Estimated energy margin"):
           "Energy estimate unavailable",rx+40,ry+188,12,arrival?advisory_color:c.primary,false,right-80);
    p.Text(arrival&&!shortfall?"Advisory estimate; conditions can change.":EnergyMessage(energy.arrival),
           rx+40,ry+213,12,c.secondary,false,right-80);
    const wxString values[]={distance?wxString::Format("%.1f nm",*distance):Dash(),
      energy.range.estimate?wxString::Format("%.1f nm",energy.range.estimate->range_nm):Dash(),
      !arrival?"Unavailable":energy.arrival.quality==smartnav::EnergyQuality::Aging?"Aging inputs":
      energy.arrival.quality==smartnav::EnergyQuality::Modeled?"Modeled consumption":"Current inputs"};
    const char *labels[]={"Remaining passage","Estimated range","Forecast quality"};
    for(int i=0;i<3;++i) {
      const int line=ry+278+45*i;
      p.Text(labels[i],rx+24,line,12,c.secondary,false,right/2-24);
      p.TextWeight(values[i],rx+right/2,line,12,c.primary,500,right/2-25,true);
      p.Rule(rx+24,line+29,right-48);
    }
    const int compact_y=ry+height+space;
    const int count=wide?4:2,cw=(available-10*(count-1))/count;
    const vessel::Sample *samples[]={&state_.propulsion.electrical_power_kw,&state_.propulsion.motor_rpm,
      &state_.battery.voltage_v,&state_.propulsion.motor_temperature_c};
    const char *titles[]={"PROPULSION","MOTOR","HV BATTERY","MOTOR TEMPERATURE"};
    const wxString units[]={"kW","RPM","V",W("°C")};
    for(int i=0;i<4;++i) {
      const int x=inset+(i%count)*(cw+10),cy=compact_y+(i/count)*85;
      dc.SetPen(wxPen(Colour(c.border)));dc.SetBrush(wxBrush(Colour(c.surface)));
      dc.DrawRoundedRectangle(p.D(x),p.D(cy),p.D(cw),p.D(75),p.D(9));
      p.Text(titles[i],x+14,cy+14,8,c.muted,false,cw-28);
      const auto a=vessel::Assess(*samples[i],now_);
      p.Text(usable(a)?wxString::Format(i==0?"%.1f":"%.0f",*a.value):Dash(),
             x+14,cy+34,20,usable(a)?c.primary:c.muted,false,cw-75);
      dc.SetFont(UiFont(*this,20));
      const auto number=usable(a)?wxString::Format(i==0?"%.1f":"%.0f",*a.value):Dash();
      const int unit_x=x+14+ToDIP(dc.GetTextExtent(number).x)+4;
      p.Text(usable(a)?units[i]:W(vessel::QualityName(a.quality)),unit_x,cy+44,9,c.secondary,false,std::max(1,x+cw-unit_x-14));
    }
    const int detail_y=compact_y+(4/count)*85+8;
    p.Card(inset,detail_y,available,118,"OPERATING STATE");
    const auto current=vessel::Assess(state_.battery.current_a,now_);
    const auto gear=vessel::AssessText(state_.propulsion.gear,now_);
    const auto regen=vessel::AssessText(state_.propulsion.regeneration,now_);
    p.Text(usable(current)?wxString::Format("Battery current %.1f A",*current.value):"Battery current unavailable",
           inset+24,detail_y+45,13,c.secondary,false,available-48);
    p.Text("Drive: "+(gear.value&&gear.quality!=vessel::Quality::Stale?W(*gear.value):"Unavailable")+
           " / Regeneration: "+(regen.value&&regen.quality!=vessel::Quality::Stale?W(*regen.value):"Unavailable"),
           inset+24,detail_y+75,13,c.secondary,false,available-48);
    bottom=detail_y+142;
  } else if (page_ == PreviewPage::Route) {
    const int columns = width >= 660 ? 2 : 1;
    const int cw = (width - 2 * margin - gap * (columns - 1)) / columns;
    p.Card(margin, 100, cw, 308, "DESTINATION");
    p.Text(DestinationName(state_), margin + 24, 148, 26, c.primary, false, cw - 48);
    p.Text(distance ? wxString::Format("%.1f", *distance) : Dash(),
           margin + 24, 192, 60, distance ? c.accent : c.muted, false, cw - 48);
    p.Text("NM REMAINING", margin + 24, 260, 12, c.secondary);
    p.Rule(margin + 24, 298, cw - 48);
    p.Text("ARRIVAL IN", margin + 24, 318, 12, c.secondary);
    p.Text(destination ? ApproxTime(destination->seconds_from_now) : "Time unavailable",
           margin + 24, 342, 26, c.primary, false, cw - 48);
    const int x = columns == 2 ? margin + cw + gap : margin;
    const int y = columns == 2 ? 100 : 424;
    p.Card(x, y, cw, 308, "NEXT WAYPOINT");
    p.Text(PointName(state_), x + 24, y + 48, 26, c.primary, false, cw - 48);
    const auto route = state_.navigation.route;
    const bool valid = route && vessel::AssessRoute(*route, now_).remaining_distance_nm.has_value();
    const auto next = valid && !route->remaining_steps.empty()
                          ? std::optional<double>{route->remaining_steps.front().distance_from_previous_nm}
                          : std::nullopt;
    p.Text(next ? wxString::Format("%.1f NM", *next) : "Distance unavailable",
           x + 24, y + 92, 24, next ? c.primary : c.muted, false, cw - 48);
    p.Rule(x + 24, y + 136, cw - 48);
    p.Text("NEXT TURN", x + 24, y + 156, 12, c.secondary);
    p.Text(turn && turn->course_change_deg
               ? wxString::Format(W("%+.0f°"), *turn->course_change_deg)
           : valid && route->remaining_steps.size() == 1 ? "Final leg" : Dash(),
           x + 24, y + 180, 36, turn ? c.accent : c.muted, false, cw - 48);
    p.Text(turn ? ApproxTime(turn->seconds_from_now) : "Turn timing unavailable",
           x + 24, y + 230, 17, c.secondary, false, cw - 48);
    p.Text(turn && turn->course_true_deg
               ? wxString::Format(W("New course %.0f° true / advisory"), *turn->course_true_deg)
               : "Steering remains under human control",
           x + 24, y + 268, 11, c.muted, false, cw - 48);
    const int end = y + 324;
    p.Card(margin, end, width - 2 * margin, 148, "ESTIMATED ARRIVAL CHARGE");
    p.Estimate(arrival ? arrival->soc_percent : std::nullopt, margin + 24,
               end + 44, "% SOC", 0, 40, cw - 48);
    const int detail_x = width >= 660 ? margin + cw + gap + 24 : margin + cw / 2;
    const int detail_width = width - margin - detail_x - 24;
    p.Text(EnergyMessage(energy.arrival), detail_x, end + 52, 14,
           arrival && arrival->below_reserve ? c.attention : c.secondary,
           false, detail_width);
    p.Text(valid ? "Navigation data current" : "Navigation data unavailable",
           detail_x, end + 86, 13, valid ? c.healthy : c.attention, false, detail_width);
    p.Text("Times and charge are advisory estimates at current conditions.",
           margin, end + 164, 11, c.muted, false, width - 2 * margin);
    bottom = end + 200;
  } else {
    int y = 96;
    p.Card(margin, y, width - 2 * margin,
           static_cast<int>(info_.size()) * 22 + 62,
           W(std::string(application::Edition) + " / COMMISSIONING BUILD"));
    for (const auto &line : info_) {
      p.Text(W(line), margin + 20, y + 44, 12, c.secondary, false,
             width - 2 * margin - 40);
      y += 22;
    }
    y += 82;
    const auto r = state_.navigation.route;
    p.Text("ROUTE  /  " + (r ? W(vessel::RouteStateName(
                                   vessel::AssessRoute(*r, now_).state))
                             : "Unavailable"),
           margin, y, 13, c.accent, true);
    y += 27;
    p.Text(r ? "ID: " + W(r->route_id) + "  /  revision " +
                   wxString::Format("%llu", static_cast<unsigned long long>(
                                                r->route_revision)) +
                   "  /  " + W(r->revision_scope)
             : "No route observation",
           margin, y, 11, c.secondary, false, width - 2 * margin);
    y += 24;
    if (r) {
      p.Text("Progress source: " + W(r->source), margin, y, 11, c.secondary,
             false, width - 2 * margin);
      y += 24;
      const auto a = vessel::AssessRoute(*r, now_);
      p.Text("Progress age: " +
                 (a.observation_age
                      ? wxString::Format("%.1f s",
                                         a.observation_age->count() / 1000.0)
                      : Dash()) +
                 "  /  position age: " +
                 (a.position_age
                      ? wxString::Format("%.1f s",
                                         a.position_age->count() / 1000.0)
                      : Dash()),
             margin, y, 11, c.secondary);
      y += 24;
    }
    p.Text("ENERGY  /  " + W(smartnav::EnergyStatus(energy.arrival.reason, energy.arrival.input)),
           margin, y, 13, c.attention, true);
    y += 25;
    p.Text(model.source.empty()
               ? "Battery capacity and reserve have not been configured."
               : W(model.source),
           margin, y, 11, c.secondary, false, width - 2 * margin);
    y += 38;
    const bool wide = width >= 940;
    p.Text("DATUM / VALUE", margin, y, 11, c.secondary, true);
    p.Text("VALIDITY / AGE", wide ? margin + 355 : width / 2, y, 11,
           c.secondary, true);
    if (wide)
      p.Text("SOURCE", margin + 590, y, 11, c.secondary, true);
    y += 27;
    for (const auto &item : vessel::DataItems(state_)) {
      const auto a = vessel::Assess(*item.sample, now_);
      const wxString value =
          a.value ? wxString::Format("%.2f ", *a.value) + item.unit : Dash();
      p.Text(W(item.name) + "  " + value, margin, y, 12, c.primary, false,
             wide ? 345 : width / 2 - margin - 8);
      const wxString age =
          a.age ? wxString::Format(" %.1fs", a.age->count() / 1000.0) : "";
      p.Text(W(vessel::ValidityName(item.sample->validity)) + " / " +
                 W(vessel::QualityName(a.quality)) + age,
             wide ? margin + 355 : width / 2, y, 10,
             a.quality == vessel::Quality::Stale ? c.attention : c.secondary,
             false, wide ? 225 : width / 2 - margin);
      p.Text(item.sample->source.empty() ? "No source" : W(item.sample->source),
             wide ? margin + 590 : margin, wide ? y : y + 20, 10, c.muted,
             false, wide ? width - margin * 2 - 590 : width - margin * 2);
      y += wide ? 32 : 48;
    }
    for (const auto &item : vessel::TextDataItems(state_)) {
      const auto &s = *item.sample;
      const auto a = vessel::AssessText(s, now_);
      p.Text(W(item.name) + " / " + (a.value ? W(*s.value) : "Unavailable") +
                 " / " + W(vessel::ValidityName(s.validity)) + " / " +
                 W(vessel::QualityName(a.quality)) +
                 (a.age ? wxString::Format(" %.1fs", a.age->count() / 1000.0)
                        : ""),
             margin, y, 11, c.secondary, false, width - 2 * margin);
      y += 22;
      p.Text(s.source.empty() ? "No source" : W(s.source), margin, y, 10,
             c.muted, false, width - 2 * margin);
      y += 30;
    }
    bottom = y + 24;
  }
  const auto target = FromDIP(wxSize(0, bottom));
  if (GetVirtualSize().y != target.y)
    SetVirtualSize(wxSize(GetClientSize().x, target.y));
}
} // namespace opennav::ui

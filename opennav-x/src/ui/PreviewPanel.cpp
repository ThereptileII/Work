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
  Bind(wxEVT_PAINT, &PreviewPanel::Paint, this);
  Bind(wxEVT_SIZE, [this](wxSizeEvent &e) {
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
  SetLabel(page == PreviewPage::Route    ? "OpenNav page: Route"
           : page == PreviewPage::Energy ? "OpenNav page: Energy"
                                         : "OpenNav page: Diagnostics");
  mode_ = mode;
  state_ = s;
  now_ = now;
  model_ = model;
  energy_ = energy;
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
  const auto &energy = energy_;
  const auto distance = Distance(state_, now_);
  const auto arrival = energy.arrival.estimate;
  const wxString title = page_ == PreviewPage::Energy  ? "Energy"
                         : page_ == PreviewPage::Route ? "Passage"
                                                       : "Diagnostics";
  p.Text(title, margin, 20, 28, c.primary);
  wxString subtitle = page_ == PreviewPage::Energy
                          ? "Battery, propulsion and estimated arrival"
                      : page_ == PreviewPage::Route
                          ? "Your destination and the next maneuver"
                          : "System details and data health";
  if (state_.replayed) subtitle = "REPLAY / Historical data / Controls disabled";
#if XNAV_ENABLE_TEST_FIXTURES
  else if (state_.simulated) subtitle = "DEMO / Synthetic test trip / No device output";
#endif
  p.Text(subtitle, margin, 60, 13,
         state_.simulated || state_.replayed ? c.attention : c.secondary, false,
         width - 2 * margin);
  const auto *destination = Event(advice_, smartnav::EventKind::Destination);
  const auto *turn = Event(advice_, smartnav::EventKind::Turn);
  int bottom = 0;
  if (page_ == PreviewPage::Energy) {
    const int columns = width >= 800 ? 3 : width >= 580 ? 2 : 1;
    const int cw = (width - margin * 2 - gap * (columns - 1)) / columns;
    const int card_height = 276;
    const int bx = margin, by = 100;
    p.Card(bx, by, cw, card_height, "BATTERY");
    p.Value(state_.battery.soc_percent, bx + 24, by + 44, "% CHARGE", 0, 60, cw - 48);
    const auto soc = vessel::Assess(state_.battery.soc_percent, now_);
    dc.SetPen(*wxTRANSPARENT_PEN);
    dc.SetBrush(wxBrush(Colour(c.border)));
    dc.DrawRoundedRectangle(p.D(bx + 24), p.D(by + 154), p.D(cw - 48), p.D(4), p.D(2));
    if (soc.value && soc.quality != vessel::Quality::Stale) {
      const auto color = soc.quality == vessel::Quality::Uncertain ? c.attention
                         : std::isfinite(model.reserve_soc_percent) &&
                                   *soc.value <= model.reserve_soc_percent ? c.attention
                                                                           : c.healthy;
      dc.SetBrush(wxBrush(Colour(color)));
      dc.DrawRoundedRectangle(p.D(bx + 24), p.D(by + 154),
          p.D(static_cast<int>((cw - 48) * std::clamp(*soc.value, 0.0, 100.0) / 100)),
          p.D(4), p.D(2));
    }
    p.Value(state_.battery.voltage_v, bx + 24, by + 178, "VOLT", 0, 28, cw / 2 - 32);
    p.Value(state_.battery.current_a, bx + cw / 2 + 8, by + 178, "AMPERE", 1, 28, cw / 2 - 32);
    const auto net = vessel::Assess(state_.battery.net_discharge_kw, now_);
    p.Text(net.value && net.quality != vessel::Quality::Stale
               ? wxString::Format("Battery load  %.1f kW", *net.value)
               : "Battery load unavailable",
           bx + 24, by + 248, 12, c.secondary, false, cw - 48);
    const int px = columns >= 2 ? margin + cw + gap : margin;
    const int py = columns >= 2 ? by : by + card_height + gap;
    p.Card(px, py, cw, card_height, "PROPULSION");
    p.Value(state_.propulsion.electrical_power_kw, px + 24, py + 44,
            "kW / MOTOR POWER", 1, 60, cw - 48);
    p.Rule(px + 24, py + 154, cw - 48);
    p.Value(state_.propulsion.motor_rpm, px + 24, py + 178, "RPM", 0, 28, cw / 2 - 32);
    p.Value(state_.propulsion.motor_temperature_c, px + cw / 2 + 8, py + 178,
            "MOTOR / °C", 0, 28, cw / 2 - 32);

    const int full = width - 2 * margin;
    int range_y = by + card_height + gap;
    if (columns == 3) {
      const int x = margin + 2 * (cw + gap);
      p.Card(x, by, cw, card_height, "DESTINATION");
      p.Text(DestinationName(state_), x + 24, by + 44, 20, c.primary, false, cw - 48);
      p.Text(distance ? wxString::Format("%.1f NM", *distance) : "Distance unavailable",
             x + 24, by + 76, 21, distance ? c.primary : c.muted, false, cw - 48);
      p.Text(destination ? ApproxTime(destination->seconds_from_now) : "Time unavailable",
             x + 24, by + 106, 14, c.secondary, false, cw - 48);
      p.Rule(x + 24, by + 136, cw - 48);
      p.Text("ESTIMATED ARRIVAL", x + 24, by + 152, 11, c.secondary);
      p.Estimate(arrival ? arrival->soc_percent : std::nullopt, x + 24,
                 by + 174, "% CHARGE", 0, 40, cw - 48);
      wxString reserve = arrival && arrival->soc_percent && std::isfinite(model.reserve_soc_percent)
                             ? wxString::Format("Reserve %+.0f%%", *arrival->soc_percent - model.reserve_soc_percent)
                             : "Estimate unavailable";
      if (arrival && arrival->energy_shortfall_kwh > 0) reserve = "Insufficient energy";
      p.Text(reserve, x + 24, by + 244, 13,
             arrival && arrival->below_reserve ? c.attention : c.secondary, false, cw - 48);
    } else {
      const int y = py + card_height + gap;
      const int destination_height = columns == 2 ? 244 : 408;
      p.Card(margin, y, full, destination_height, "DESTINATION");
      const int half = columns == 2 ? full / 2 : full;
      p.Text(DestinationName(state_), margin + 24, y + 46, 24, c.primary, false, half - 48);
      p.Text(distance ? wxString::Format("%.1f", *distance) : Dash(),
             margin + 24, y + 84, 42, distance ? c.primary : c.muted, false, half - 48);
      p.Text("NM REMAINING", margin + 24, y + 136, 12, c.secondary);
      p.Text(destination ? ApproxTime(destination->seconds_from_now) : "Time unavailable",
             margin + 24, y + 164, 20, c.primary, false, half - 48);
      p.Text("At current speed / approximate", margin + 24, y + 198, 11, c.secondary, false, half - 48);
      const int ax = columns == 2 ? margin + half + 24 : margin + 24;
      const int ay = columns == 2 ? y + 46 : y + 230;
      p.Text("ESTIMATED ARRIVAL", ax, ay, 12, c.secondary);
      p.Estimate(arrival ? arrival->soc_percent : std::nullopt, ax, ay + 24,
                 "% CHARGE", 0, 48, half - 48);
      wxString reserve = EnergyMessage(energy.arrival);
      if (arrival && arrival->soc_percent && std::isfinite(model.reserve_soc_percent))
        reserve = wxString::Format("Reserve %+.0f%%", *arrival->soc_percent - model.reserve_soc_percent);
      p.Text(reserve, ax, ay + 110, 14,
             arrival && arrival->below_reserve ? c.attention : c.secondary,
             false, half - 48);
      p.Text(energy.arrival.quality == smartnav::EnergyQuality::Aging
                 ? "Estimate uses aging data"
             : energy.arrival.quality == smartnav::EnergyQuality::Modeled
                 ? "Estimate uses modeled consumption"
             : arrival ? "Advisory / current conditions" : "Estimate unavailable",
             ax, ay + 140, 11, c.muted, false, half - 48);
      range_y = y + destination_height + gap;
    }
    p.Card(margin, range_y, full, 152, "ESTIMATED RANGE ABOVE RESERVE");
    p.Estimate(energy.range.estimate ? std::optional<double>{energy.range.estimate->range_nm}
                                    : std::nullopt,
               margin + 24, range_y + 44, "NM", 0, 40, full / 2 - 40);
    const int sx = margin + full / 2;
    const int sw = full / 2 - 24;
    const auto gear = vessel::AssessText(state_.propulsion.gear, now_);
    const auto regen = vessel::AssessText(state_.propulsion.regeneration, now_);
    p.Text("DRIVE", sx, range_y + 24, 11, c.secondary);
    p.Text(gear.value && gear.quality != vessel::Quality::Stale ? W(*gear.value) : "Unavailable",
           sx, range_y + 46, 17, c.primary, false, sw);
    p.Text("REGENERATION", sx, range_y + 82, 11, c.secondary);
    p.Text(regen.value && regen.quality != vessel::Quality::Stale ? W(*regen.value) : "Unavailable",
           sx, range_y + 104, 17, c.primary, false, sw);
    p.Text(!arrival ? EnergyMessage(energy.arrival)
           : energy.arrival.quality == smartnav::EnergyQuality::Aging
               ? wxString("Limited estimate: aging inputs. Assumptions are in Settings.")
           : energy.arrival.quality == smartnav::EnergyQuality::Modeled
               ? wxString("Estimated consumption. Assumptions are in Settings.")
               : wxString("Advisory estimates at current conditions. Assumptions are in Settings."),
           margin, range_y + 168, 12, c.secondary, false, full);
    bottom = range_y + 208;
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
               ? wxString::Format("%+.0f°", *turn->course_change_deg)
           : valid && route->remaining_steps.size() == 1 ? "Final leg" : Dash(),
           x + 24, y + 180, 36, turn ? c.accent : c.muted, false, cw - 48);
    p.Text(turn ? ApproxTime(turn->seconds_from_now) : "Turn timing unavailable",
           x + 24, y + 230, 17, c.secondary, false, cw - 48);
    p.Text(turn && turn->course_true_deg
               ? wxString::Format("New course %.0f° true / advisory", *turn->course_true_deg)
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

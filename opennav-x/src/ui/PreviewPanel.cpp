#include "ui/PreviewPanel.h"
#include "vessel/DataItems.h"
#include <algorithm>
#include <cmath>
#include <wx/dcbuffer.h>

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
wxString Dash() { return wxString::FromUTF8("—"); }
struct Painter {
  wxWindow &window;
  wxDC &dc;
  Palette c;
  int D(int n) { return window.FromDIP(n); }
  void Text(wxString s, int x, int y, int size, std::uint32_t color,
            bool bold = false, int width = 0) {
    dc.SetFont(UiFont(window, size, bold));
    dc.SetTextForeground(Colour(color));
    if (width > 0 && dc.GetTextExtent(s).x > D(width)) {
      while (!s.empty() && dc.GetTextExtent(s + "...").x > D(width))
        s.RemoveLast();
      s += "...";
    }
    dc.DrawText(s, D(x), D(y));
  }
  void Card(int x, int y, int w, int h, const wxString &title) {
    dc.SetPen(wxPen(Colour(c.border)));
    dc.SetBrush(wxBrush(Colour(c.surface)));
    dc.DrawRoundedRectangle(D(x), D(y), D(w), D(h), D(8));
    Text(title, x + 20, y + 18, 12, c.secondary, true, w - 40);
  }
  void Value(const vessel::Sample &s, int x, int y, const wxString &unit,
             int precision = 1, int size = 36) {
    const auto a = vessel::Assess(s, now);
    const bool stale = a.quality == vessel::Quality::Stale;
    Text(a.value ? wxString::Format("%.*f", precision, *a.value) : Dash(), x, y,
         size, stale ? c.muted : c.primary, true);
    Text(unit, x, y + size + 7, 13, c.secondary);
    wxString q = W(vessel::QualityName(a.quality));
    if (a.age && a.quality != vessel::Quality::Live)
      q += wxString::Format(" %.0fs", a.age->count() / 1000.0);
    Text(q, x, y + size + 28, 10, stale ? c.attention : c.muted, true);
  }
  void Estimate(std::optional<double> v, int x, int y, const wxString &unit,
                int precision = 1) {
    Text(v ? wxString::Format("%.*f", precision, *v) : Dash(), x, y, 40,
         c.accent, true);
    Text(unit, x, y + 48, 13, c.secondary);
    Text(v ? "ESTIMATED / ADVISORY" : "UNAVAILABLE", x, y + 70, 10, c.attention,
         true);
  }
  vessel::Time now;
};
std::optional<double> Distance(const vessel::VesselState &s, vessel::Time now) {
  return s.navigation.route ? vessel::AssessRoute(*s.navigation.route, now)
                                  .remaining_distance_nm
                            : std::nullopt;
}
wxString DestinationName(const vessel::VesselState &s) {
  if (!s.navigation.route || s.navigation.route->route_id.empty() ||
      s.navigation.route->state == vessel::RouteState::NoActiveRoute)
    return "No active route";
  return s.simulated ? "Sheltered bay" : "OpenCPN active route";
}
wxString PointName(const vessel::VesselState &s) {
  if (!s.navigation.route || s.navigation.route->active_waypoint_id.empty())
    return "No active waypoint";
  auto id = s.navigation.route->active_waypoint_id;
  if (id.rfind("DEMO-", 0) == 0)
    id = id.substr(5);
  std::replace(id.begin(), id.end(), '-', ' ');
  return W(id);
}
} // namespace
PreviewPanel::PreviewPanel(wxWindow *parent)
    : wxScrolledWindow(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                       wxBORDER_NONE | wxVSCROLL) {
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
                          const std::vector<std::string> &info) {
  if (page != page_)
    Scroll(0, 0);
  page_ = page;
  SetLabel(page == PreviewPage::Route ? "OpenNav page: Route"
           : page == PreviewPage::Energy ? "OpenNav page: Energy"
                                         : "OpenNav page: Diagnostics");
  mode_ = mode;
  state_ = s;
  now_ = now;
  info_ = info;
  Refresh();
}
void PreviewPanel::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);
  PrepareDC(dc);
  const auto c = Theme(mode_);
  dc.SetBackground(wxBrush(Colour(c.background)));
  dc.Clear();
  Painter p{*this, dc, c, now_};
  const int width = std::max(320, ToDIP(GetClientSize().x)), margin = 24,
            gap = 16;
  const auto model = smartnav::PreviewEnergyModel(state_.simulated);
  const auto energy = smartnav::PredictVesselEnergy(model, state_, now_);
  const auto distance = Distance(state_, now_);
  const auto arrival = energy.arrival.estimate;
  const wxString title = page_ == PreviewPage::Energy  ? "Propulsion & energy"
                         : page_ == PreviewPage::Route ? "Route & destination"
                                                       : "System & diagnostics";
  p.Text(title, margin, 20, 26, c.primary, true);
  p.Text(state_.simulated ? "DEMO  /  Synthetic trip  /  No device output"
                          : "OPENCPN  /  Read-only vessel data",
         margin, 58, 13, state_.simulated ? c.attention : c.secondary, true);
  int bottom = 0;
  if (page_ == PreviewPage::Energy) {
    const int columns = width >= 940 ? 3 : width >= 660 ? 2 : 1;
    const int cw = (width - margin * 2 - gap * (columns - 1)) / columns,
              ch = 308;
    auto xy = [&](int i) {
      return wxPoint(margin + (i % columns) * (cw + gap),
                     96 + (i / columns) * (ch + gap));
    };
    auto b = xy(0);
    p.Card(b.x, b.y, cw, ch, "BATTERY");
    p.Value(state_.battery.soc_percent, b.x + 20, b.y + 48,
            "State of charge / %", 0, 52);
    const auto soc = vessel::Assess(state_.battery.soc_percent, now_);
    dc.SetPen(*wxTRANSPARENT_PEN);
    dc.SetBrush(wxBrush(Colour(c.border)));
    dc.DrawRoundedRectangle(p.D(b.x + 20), p.D(b.y + 164), p.D(cw - 40), p.D(6),
                            p.D(3));
    if (soc.value) {
      dc.SetBrush(wxBrush(Colour(soc.quality == vessel::Quality::Stale ? c.muted
                                 : *soc.value < 15 ? c.attention
                                                   : c.healthy)));
      dc.DrawRoundedRectangle(
          p.D(b.x + 20), p.D(b.y + 164),
          p.D(static_cast<int>((cw - 40) * std::clamp(*soc.value, 0.0, 100.0) /
                               100)),
          p.D(6), p.D(3));
    }
    p.Value(state_.battery.voltage_v, b.x + 20, b.y + 188, "V", 0, 27);
    p.Value(state_.battery.current_a, b.x + cw / 2, b.y + 188,
            "A / + discharge", 1, 27);
    p.Text(state_.simulated ? "48 kWh usable  /  15% reserve"
                            : "Capacity / reserve unconfigured",
           b.x + 20, b.y + 284, 11, c.secondary, false, cw - 40);
    b = xy(1);
    p.Card(b.x, b.y, cw, ch, "PROPULSION");
    p.Value(state_.propulsion.electrical_power_kw, b.x + 20, b.y + 48,
            "kW / motor electrical", 1, 52);
    p.Value(state_.propulsion.motor_rpm, b.x + 20, b.y + 188, "RPM", 0, 27);
    p.Value(state_.propulsion.motor_temperature_c, b.x + cw / 2, b.y + 188,
            "Motor / C", 0, 27);
    const auto gear = vessel::AssessText(state_.propulsion.gear, now_);
    p.Text("Gear: " + (gear.value ? W(*gear.value) + " / " +
                                        W(vessel::QualityName(gear.quality))
                                  : "Unavailable"),
           b.x + 20, b.y + 284, 11, c.secondary);
    b = xy(2);
    p.Card(b.x, b.y, cw, ch, "DESTINATION");
    p.Text(DestinationName(state_), b.x + 20, b.y + 48, 19, c.primary, true,
           cw - 40);
    p.Text(distance ? wxString::Format("%.1f NM remaining", *distance)
                    : "Remaining distance unavailable",
           b.x + 20, b.y + 80, 15, c.secondary, false, cw - 40);
    p.Estimate(arrival ? arrival->soc_percent : std::nullopt, b.x + 20,
               b.y + 119, "Arrival SOC / %");
    wxString reason = W(smartnav::EnergyReasonName(energy.arrival.reason));
    if (arrival && arrival->energy_shortfall_kwh > 0)
      reason = wxString::Format("SHORTFALL  %.1f kWh",
                                arrival->energy_shortfall_kwh);
    else if (arrival)
      reason = arrival->below_reserve ? "BELOW CONFIGURED RESERVE"
                                      : "Above configured reserve";
    p.Text(reason, b.x + 20, b.y + 241, 13,
           arrival && arrival->below_reserve ? c.attention : c.secondary, true,
           cw - 40);
    if (arrival && arrival->soc_percent)
      p.Text(
          wxString::Format("Reserve margin  %+.1f%%",
                           *arrival->soc_percent - model.reserve_soc_percent),
          b.x + 20, b.y + 271, 13, c.secondary);
    const int y = 96 + ((3 + columns - 1) / columns) * (ch + gap);
    p.Card(margin, y, width - margin * 2, 188,
           "RANGE AT PRESENT CONDITIONS  /  ADVISORY");
    p.Estimate(energy.range.estimate
                   ? std::optional<double>{energy.range.estimate->range_nm}
                   : std::nullopt,
               margin + 20, y + 51, "NM above reserve");
    const int x = width >= 660 ? margin + 280 : margin + 175;
    p.Text("Whole-pack discharge (includes hotel load)", x, y + 52, 12,
           c.secondary, false, width - x - 44);
    const auto net = vessel::Assess(state_.battery.net_discharge_kw, now_);
    p.Text(net.value ? wxString::Format("%.1f kW  /  ", *net.value) +
                           W(vessel::QualityName(net.quality))
                     : "Unavailable",
           x, y + 78, 20, c.primary, true, width - x - 44);
    p.Text(arrival ? wxString::Format("Passage energy  %.1f kWh",
                                      arrival->energy_required_kwh)
                   : W(smartnav::EnergyReasonName(energy.arrival.reason)),
           x, y + 112, 13, c.secondary, false, width - x - 44);
    p.Text("Constant speed and net power. No weather/current forecast.",
           margin + 20, y + 160, 11, c.muted, false, width - margin * 2 - 40);
    bottom = y + 212;
  } else if (page_ == PreviewPage::Route) {
    const int columns = width >= 760 ? 2 : 1,
              cw = (width - 2 * margin - gap * (columns - 1)) / columns;
    p.Card(margin, 96, cw, 325, "DESTINATION");
    p.Text(DestinationName(state_), margin + 20, 145, 24, c.primary, true,
           cw - 40);
    p.Text(distance ? wxString::Format("%.2f", *distance) : Dash(), margin + 20,
           187, 52, c.accent, true);
    p.Text("NM remaining along route", margin + 20, 249, 14, c.secondary);
    p.Text("Estimated passage time", margin + 20, 300, 12, c.secondary);
    const auto speed = vessel::Assess(state_.navigation.sog_kn, now_);
    const bool reliable = speed.value && *speed.value >= 0.5 &&
                          (speed.quality == vessel::Quality::Live ||
                           speed.quality == vessel::Quality::Aging);
    if (distance && reliable) {
      const double minutes = *distance / (*speed.value) * 60;
      p.Text(minutes < 60000
                 ? wxString::Format("%dh %02dm", static_cast<int>(minutes) / 60,
                                    static_cast<int>(minutes) % 60)
                 : "Outside preview range",
             margin + 20, 325, 28, c.primary, true);
      p.Text("At current SOG / advisory", margin + 20, 371, 11, c.attention);
    } else
      p.Text("Unavailable", margin + 20, 330, 24, c.muted);
    const int x = columns == 2 ? margin + cw + gap : margin,
              y = columns == 2 ? 96 : 437;
    p.Card(x, y, cw, 325, "NEXT WAYPOINT");
    p.Text(PointName(state_), x + 20, y + 49, 22, c.primary, true, cw - 40);
    const auto route = state_.navigation.route;
    const auto assessment =
        route ? vessel::AssessRoute(*route, now_) : vessel::RouteAssessment{};
    p.Text(route && route->active_waypoint_index
               ? wxString::Format(
                     "Waypoint %u of %u",
                     static_cast<unsigned>(*route->active_waypoint_index + 1),
                     static_cast<unsigned>(route->waypoint_count))
               : "No active route",
           x + 20, y + 86, 13, c.secondary);
    p.Text("NAVIGATION VALIDITY", x + 20, y + 127, 11, c.secondary, true);
    p.Text(W(vessel::RouteStateName(assessment.state)), x + 20, y + 153, 17,
           assessment.remaining_distance_nm ? c.healthy : c.attention, true,
           cw - 40);
    p.Text("ESTIMATED ARRIVAL SOC", x + 20, y + 207, 11, c.secondary, true);
    p.Text(arrival && arrival->soc_percent
               ? wxString::Format("%.1f %%", *arrival->soc_percent)
               : "Unavailable",
           x + 20, y + 232, 30, c.accent, true);
    p.Text("Advisory / inspect assumptions in Energy", x + 20, y + 287, 11,
           c.muted, false, cw - 40);
    const int end = y + 345;
    p.Card(margin, end, width - 2 * margin, 114, "READ-ONLY ROUTE PROGRESS");
    p.Text(route ? W(route->source) : "No route source", margin + 20, end + 46,
           12, c.secondary, false, width - 2 * margin - 40);
    p.Text(state_.simulated ? "Demo route and GPS are synthetic. The chart "
                              "remains OpenCPN's real canvas."
                            : "OpenCPN owns route activation, arrival and "
                              "waypoint changes. No automatic steering.",
           margin + 20, end + 77, 11, c.muted, false, width - 2 * margin - 40);
    bottom = end + 138;
  } else {
    int y = 96;
    p.Card(margin, y, width - 2 * margin,
           static_cast<int>(info_.size()) * 22 + 62,
           "DEVELOPER PREVIEW 0.1 / NOT FOR NAVIGATION");
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
    p.Text("ENERGY  /  " + W(smartnav::EnergyReasonName(energy.arrival.reason)),
           margin, y, 13, c.attention, true);
    y += 25;
    p.Text(model.source.empty()
               ? "Live model is unconfigured; no demo battery defaults apply."
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

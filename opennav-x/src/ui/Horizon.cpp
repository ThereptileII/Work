#include "ui/Horizon.h"
#include <wx/dcbuffer.h>
#include <cmath>

namespace opennav::ui {
XNavHorizon::XNavHorizon(wxWindow *parent) : wxPanel(parent, wxID_ANY) {
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  SetName("Navigation horizon / advisory only");
  Bind(wxEVT_PAINT, &XNavHorizon::Paint, this);
}

void XNavHorizon::Update(const vessel::VesselState &state,
                         const smartnav::NavigationAdvice &advice,
                         vessel::Time now, LightMode mode) {
  const auto c = Theme(mode);
  std::array<Item, 4> next{};
  next[0] = {"NOW", "Navigation unavailable", "Check vessel input", c.accent};
  const auto speed = vessel::Assess(state.navigation.sog_kn, now);
  const auto course = vessel::Assess(state.navigation.cog_deg, now);
  const auto current = [](const vessel::Assessment &a) {
    return a.value && (a.quality == vessel::Quality::Live || a.quality == vessel::Quality::Aging);
  };
  if (current(speed)) {
    next[0].title = "Current motion";
    next[0].detail = wxString::Format("%.1f kn", *speed.value);
    if (current(course)) next[0].detail += wxString::Format(wxString::FromUTF8(" · %03.0f° COG"), *course.value);
  }
  int slot = 1;
  // Existing event order/semantics remain owned by SmartNav. Show a bounded
  // overview; no fabricated waypoint, time, destination or CPA fills a slot.
  for (const auto &event : advice.events) {
    if (slot == 4) break;
    if (event.kind == smartnav::EventKind::ArrivalSoc || event.kind == smartnav::EventKind::Reserve)
      continue; // destination/detail presents these without duplicate rows
    auto &item = next[slot++];
    item.title = wxString::FromUTF8(event.title);
    item.detail = wxString::FromUTF8(event.detail);
    item.color = event.severity == smartnav::Severity::Information ? c.accent : c.attention;
    item.time = "TIME UNAVAILABLE";
    if (event.seconds_from_now && std::isfinite(*event.seconds_from_now) && *event.seconds_from_now >= 0)
      item.time = wxString::Format("~%.0f min", *event.seconds_from_now / 60.0);
  }
  if (slot == 1) {
    next[1] = {"PASSAGE", "No current advisory", wxString::FromUTF8(advice.reason), c.muted};
  }
  std::string signature = std::to_string(static_cast<int>(mode));
  for (const auto &item : next)
    signature += "|" + item.time.ToStdString() + "|" + item.title.ToStdString() + "|" + item.detail.ToStdString();
  if (signature != signature_) {
    items_ = std::move(next); mode_ = mode; signature_ = std::move(signature); Refresh();
  }
}

void XNavHorizon::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);
  XNavPainter p(*this, dc, mode_);
  dc.SetBackground(wxBrush(Colour(p.c.background))); dc.Clear();
  const int width = ToDIP(GetClientSize().x);
  p.Rule(0, 0, width);
  p.Text("YOUR HORIZON", 25, 18, 9, p.c.accent);
  p.Text(wxString::FromUTF8("SmartNav · advisory"), 151, 18, 9, p.c.muted);
  p.Rule(29, 47, std::max(0, width - 64));
  const int cell = std::max(1, (width - 50)/4);
  for (int i = 0; i < 4; ++i) {
    const auto &item = items_[i];
    if (item.title.empty()) continue;
    const int x = 25 + cell*i;
    dc.SetPen(wxPen(Colour(item.color), FromDIP(2)));
    dc.SetBrush(wxBrush(Colour(i ? p.c.background : item.color)));
    dc.DrawCircle(FromDIP(x+4), FromDIP(47), FromDIP(3));
    p.Text(item.time, x, 58, 10, p.c.secondary, false, cell-18);
    p.Text(item.title, x, 74, 13, p.c.primary, false, cell-18);
    p.Text(item.detail, x, 94, 10, p.c.muted, false, cell-18);
  }
}
} // namespace opennav::ui

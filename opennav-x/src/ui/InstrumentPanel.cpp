#include "ui/InstrumentPanel.h"
#include "ui/PrototypeGeometry.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <wx/dcbuffer.h>
#include <wx/graphics.h>

namespace opennav::ui {
namespace {
using application::InstrumentReading;
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
wxString Number(const InstrumentReading &r) {
  if (!r.value)
    return W("—");
  if (r.key == "heading" || r.key == "cog")
    return wxString::Format("%03.0f", *r.value);
  const bool angle = r.key == "twa" || r.key == "awa";
  return wxString::Format("%.*f", r.decimals,
                          angle ? std::abs(*r.value) : *r.value);
}
wxString Unit(const InstrumentReading &r) {
  if ((r.key == "twa" || r.key == "awa") && r.value)
    return *r.value < 0   ? W("° port")
           : *r.value > 0 ? W("° starboard")
                          : W("° ahead");
  return W(r.unit);
}
wxString Health(const InstrumentReading &r) {
  if (!r.selected)
    return "Hidden in instrument settings";
  wxString source = W(r.source);
  if (source.StartsWith("NMEA2000/"))
    source = "NMEA 2000";
  else if (source.StartsWith("NMEA0183/"))
    source = "NMEA 0183";
  else if (source.StartsWith("SignalK/"))
    source = "Signal K";
  if (r.quality != vessel::Quality::Live)
    source = W(vessel::QualityName(r.quality));
  if (r.age)
    source += wxString::Format(W(" · %.1f s"), r.age->count() / 1000.);
  return source;
}
struct InstrumentLayout {
  int width, left, right, rx, ry, grid_bottom;
  bool wide;
  InstrumentLayout(int w, std::size_t tiles) {
    width = std::max(280, w - 2 * prototype::page_inset);
    wide = width >= 720;
    left = wide ? static_cast<int>(std::lround(
                      (width - prototype::dashboard_gap) * 1.1 / 2.1))
                : width;
    right = wide ? width - prototype::dashboard_gap - left : width;
    rx = wide ? prototype::page_inset + left + prototype::dashboard_gap
              : prototype::page_inset;
    ry = wide ? prototype::dashboard_y
              : prototype::dashboard_y + prototype::wind_card_height + 18;
    const int tile_height =
        static_cast<int>((tiles + 1) / 2) * (prototype::instrument_tile_height +
                                             prototype::instrument_tile_gap) -
        prototype::instrument_tile_gap;
    grid_bottom = std::max(prototype::dashboard_y + prototype::wind_card_height,
                           ry + tile_height);
  }
};
} // namespace
XNavInstrumentPanel::XNavInstrumentPanel(wxWindow *parent)
    : wxPanel(parent, wxID_ANY) {
  SetName("Vessel instrument presentation");
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  EnableScrollGesture(*this);
  const auto button = [this](const wxString &label,
                             std::function<void()> callback) {
    auto *b = new XNavButton(this, wxID_ANY, label, label);
    b->Bind(wxEVT_BUTTON,
            [this, callback](wxCommandEvent &) { CallAfter(callback); });
    return b;
  };
  close_ = button("Close", [this] {
    if (on_close)
      on_close();
  });
  close_->SetIcon(XNavIcon::Close);
  close_->SetInlineIcon();
  close_->SetRole(ButtonRole::Quiet);
  rail_ = button("Configure data rail", [this] {
    if (on_rail)
      on_rail();
  });
  health_ = button("Inspect source quality", [this] {
    if (on_health)
      on_health();
  });
  configure_ = button("Configure instruments", [this] {
    if (on_configure)
      on_configure();
  });
  configure_->SetRole(ButtonRole::Quiet);
  Bind(wxEVT_PAINT, &XNavInstrumentPanel::Paint, this);
  Bind(wxEVT_SIZE, [this](wxSizeEvent &e) {
    Reflow();
    Refresh(false);
    e.Skip();
  });
  Reflow();
}
void XNavInstrumentPanel::Update(const vessel::VesselState &s,
                                 const std::vector<std::string> &keys,
                                 vessel::Time now, LightMode light) {
  const auto count = view_.tiles.size();
  view_ = application::PresentInstruments(s, keys, now);
  light_ = light;
  SetBackgroundColour(Colour(Theme(light_).background));
  for (auto *button : Controls())
    button->SetLightMode(light);
  close_->Enable(static_cast<bool>(on_close));
  rail_->Enable(static_cast<bool>(on_rail));
  health_->Enable(static_cast<bool>(on_health));
  configure_->Enable(static_cast<bool>(on_configure));
  if (count != view_.tiles.size())
    Reflow();
  Refresh(false);
}
void XNavInstrumentPanel::Reflow() {
  const int width = ToDIP(GetClientSize().x);
  InstrumentLayout l(width, view_.tiles.size());
  SetMinSize(FromDIP(wxSize(320, l.grid_bottom + 148)));
  close_->SetSize(FromDIP(width - 112), FromDIP(28), FromDIP(80), FromDIP(44));
  const int split = (l.width - 9) / 2;
  rail_->SetSize(FromDIP(32), FromDIP(l.grid_bottom + 20), FromDIP(split),
                 FromDIP(48));
  health_->SetSize(FromDIP(32 + split + 9), FromDIP(l.grid_bottom + 20),
                   FromDIP(l.width - split - 9), FromDIP(48));
  configure_->SetSize(FromDIP(32), FromDIP(l.grid_bottom + 80),
                      FromDIP(l.width), FromDIP(48));
}
std::vector<std::pair<wxString, wxRect>> XNavInstrumentPanel::Regions() const {
  InstrumentLayout l(ToDIP(GetClientSize().x), view_.tiles.size());
  std::vector<std::pair<wxString, wxRect>> result{
      {"Wind and heading",
       {32, prototype::dashboard_y, l.left, prototype::wind_card_height}}};
  const int split = (l.right - 12) / 2;
  for (std::size_t i = 0; i < view_.tiles.size(); ++i)
    result.push_back({W(view_.tiles[i].title),
                      {l.rx + static_cast<int>(i % 2) * (split + 12),
                       l.ry + static_cast<int>(i / 2) * 138,
                       i % 2 ? l.right - split - 12 : split, 126}});
  return result;
}
void XNavInstrumentPanel::Reading(wxDC &dc, XNavPainter &p,
                                  const InstrumentReading &r, int x, int y,
                                  int width, bool tile) {
  p.Text(W(r.title), x, y, 9, p.c.muted, false, width);
  const auto value = Number(r);
  const int size = tile ? 29 : 22;
  const int vy = y + (tile ? 27 : 24);
  p.Text(value, x, vy, size, r.value ? p.c.primary : p.c.muted);
  dc.SetFont(UiFont(*this, size));
  const int offset = ToDIP(dc.GetTextExtent(value).x);
  p.Text(Unit(r), x + offset + 4, vy + size - 12, tile ? 11 : 9, p.c.secondary,
         false, std::max(1, width - offset - 4));
  const auto ink = r.quality == vessel::Quality::Stale ||
                           r.quality == vessel::Quality::Uncertain
                       ? p.c.attention
                       : p.c.muted;
  p.Text(Health(r), x, y + (tile ? 70 : 53), 8, ink, false, width);
}
void XNavInstrumentPanel::Wind(wxDC &dc, XNavPainter &p, int x, int y,
                               int width) {
  const int size = std::min(270, width - 50), sx = x + (width - size) / 2,
            sy = y + 59;
  auto gc = std::unique_ptr<wxGraphicsContext>(
      wxGraphicsContext::CreateFromUnknownDC(dc));
  if (gc) {
    gc->PushState();
    gc->Translate(p.D(sx), p.D(sy));
    gc->Scale(p.D(size) / 280., p.D(size) / 280.);
    gc->SetBrush(*wxTRANSPARENT_BRUSH);
    gc->SetPen(wxPen(Colour(p.c.border), 1));
    gc->DrawEllipse(20, 20, 240, 240);
    gc->DrawEllipse(46, 46, 188, 188);
    gc->SetPen(wxPen(Colour(p.c.muted), 1));
    for (int i = 0; i < 36; ++i) {
      gc->PushState();
      gc->Translate(140, 140);
      gc->Rotate(i * 3.14159265358979323846 / 18.);
      gc->StrokeLine(0, -120, 0, -120 + (i % 3 ? 5 : 12));
      gc->PopState();
    }
    if (view_.heading.value) {
      gc->PushState();
      gc->Translate(140, 140);
      gc->Rotate(*view_.heading.value * 3.14159265358979323846 / 180.);
      gc->Translate(-140, -140);
      auto hull = gc->CreatePath();
      hull.MoveToPoint(140, 101);
      hull.AddQuadCurveToPoint(123, 127, 126, 162);
      hull.AddLineToPoint(154, 162);
      hull.AddQuadCurveToPoint(157, 127, 140, 101);
      hull.CloseSubpath();
      gc->SetPen(wxPen(Colour(p.c.secondary), 1));
      gc->SetBrush(wxBrush(Colour(p.c.elevated)));
      gc->DrawPath(hull);
      gc->PopState();
    }
    if (view_.wind_bearing_true_deg) {
      gc->PushState();
      gc->Translate(140, 140);
      gc->Rotate(*view_.wind_bearing_true_deg * 3.14159265358979323846 / 180.);
      gc->Translate(-140, -140);
      auto arrow = gc->CreatePath();
      arrow.MoveToPoint(140, 20);
      arrow.AddLineToPoint(131, 44);
      arrow.AddLineToPoint(149, 44);
      arrow.CloseSubpath();
      gc->SetPen(*wxTRANSPARENT_PEN);
      gc->SetBrush(wxBrush(Colour(NavigationContextInk(light_))));
      gc->DrawPath(arrow);
      gc->SetPen(wxPen(Colour(NavigationContextInk(light_)), 1));
      for (int py = 40; py < 93; py += 7)
        gc->StrokeLine(140, py, 140, std::min(93, py + 3));
      gc->PopState();
    }
    gc->PopState();
  }
  gc.reset(); // Restore shared native DC transform before drawing labels.
  const auto centered = [&](wxString text, int px, int py, int font,
                            std::uint32_t ink) {
    dc.SetFont(UiFont(*this, font));
    text = wxControl::Ellipsize(text, dc, wxELLIPSIZE_END, p.D(size - 16));
    const int tw = ToDIP(dc.GetTextExtent(text).x);
    p.Text(text, sx + px * size / 280 - tw / 2, sy + py * size / 280, font,
           ink);
  };
  centered("N", 140, 48, 9, p.c.secondary);
  centered("S", 140, 223, 9, p.c.secondary);
  centered("W", 56, 134, 9, p.c.secondary);
  centered("E", 226, 134, 9, p.c.secondary);
  centered(Number(view_.heading) + W("°"), 140, 171, 23, p.c.primary);
  centered("HEADING / TRUE", 140, 198, 8, p.c.secondary);
  // These provenance lines are required where the illustrative HTML is silent.
  centered(Health(view_.heading) + (view_.wind_bearing_true_deg
                                        ? wxString{}
                                        : W(" · wind direction unavailable")),
           140, 277, 8, p.c.muted);
}
void XNavInstrumentPanel::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);
  XNavPainter p(*this, dc, light_);
  dc.SetBackground(wxBrush(Colour(p.c.background)));
  dc.Clear();
  InstrumentLayout l(ToDIP(GetClientSize().x), view_.tiles.size());
  p.TextTracked("VESSEL INSTRUMENTS", 32, 36, 9, p.c.accent, 650, 1.17);
  p.TextTracked("Feel the passage. Read the details.", 32, 56, 30, p.c.primary,
                400, -1.2, l.width - 90);
  p.Text(view_.replayed ? "REPLAY / Historical readings"
         : view_.simulated
             ? "TEST DATA / Not live"
             : "A focused view of the boat, the wind and the water.",
         32, 104, 12, p.c.muted, false, l.width);
  dc.SetPen(wxPen(Colour(p.c.border)));
  dc.SetBrush(wxBrush(Colour(p.c.surface)));
  dc.DrawRoundedRectangle(p.D(32), p.D(146), p.D(l.left), p.D(540), p.D(14));
  p.TextWeight("Wind & heading", 57, 171, 12, p.c.secondary, 500);
  Wind(dc, p, 32, 146, l.left);
  const int stats = (l.left - 50 - 16) / 2;
  Reading(dc, p, view_.true_speed, 57, 494, stats, false);
  Reading(dc, p, view_.true_angle, 57 + stats + 16, 494, stats, false);
  const auto regions = Regions();
  for (std::size_t i = 0; i < view_.tiles.size(); ++i) {
    const auto &r = regions[i + 1].second;
    dc.SetPen(wxPen(Colour(p.c.border)));
    dc.SetBrush(wxBrush(Colour(p.c.surface)));
    dc.DrawRoundedRectangle(p.D(r.x), p.D(r.y), p.D(r.width), p.D(r.height),
                            p.D(10));
    Reading(dc, p, view_.tiles[i], r.x + 18, r.y + 25, r.width - 36, true);
  }
  if (view_.tiles.empty())
    p.Text("No instruments selected", l.rx + 18, l.ry + 25, 12, p.c.muted);
}
} // namespace opennav::ui

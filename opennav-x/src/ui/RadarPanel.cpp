#include "ui/RadarPanel.h"
#include "ui/PrototypeGeometry.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <wx/dcbuffer.h>
#include <wx/graphics.h>
#include <wx/sizer.h>
namespace opennav::ui {
namespace {
struct RadarLayout {
  int width, left, right = 265, gap = 28, x, y = 144, height = 508, scope = 428;
  bool narrow;
  explicit RadarLayout(int w, int viewport, int window_width) {
    height = std::max(200, viewport - 190);
    width = std::max(280, w - 64);
    narrow = window_width <= 760;
    if (window_width <= 1100) {
      right = 224;
      gap = 18;
    }
    left = narrow ? width : std::max(250, width - right - gap);
    if (window_width >= 1500) {
      right = 300;
      left = std::min(640, width - right - gap);
    }
    x = narrow ? 32 : 32 + left + gap;
    if (narrow)
      right = width;
    scope = std::min(std::max(120, viewport - 270), left - 42);
  }
  int ControlsY() const { return narrow ? y + height + 23 : y; }
};
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
wxColour Alpha(std::uint32_t rgb, unsigned char alpha) {
  return wxColour((rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255, alpha);
}
} // namespace
XNavRadarPanel::XNavRadarPanel(wxWindow *parent) : wxPanel(parent, wxID_ANY) {
  SetName("Radar presentation");
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  EnableScrollGesture(*this);
  controls_ = new XNavScroll(this);
  control_body_ = new wxPanel(controls_, wxID_ANY);
  control_body_->SetName("Radar controls");
  control_body_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  control_body_->Bind(wxEVT_PAINT, &XNavRadarPanel::PaintControls, this);
  EnableScrollGesture(*control_body_);
  auto *content = new wxBoxSizer(wxVERTICAL);
  content->Add(control_body_, 0, wxEXPAND | wxRIGHT, FromDIP(8));
  controls_->SetSizer(content);
  close_ = new XNavButton(this, wxID_ANY, "Close", "Close radar");
  close_->SetIcon(XNavIcon::Close);
  close_->SetInlineIcon();
  close_->SetRole(ButtonRole::Quiet);
  close_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
    if (on_close)
      CallAfter([this] { on_close(); });
  });
  const auto unavailable = [this](const wxString &name) {
    auto *b =
        new XNavButton(control_body_, wxID_ANY, name, name + " unavailable");
    b->SetToggle();
    b->Enable(false);
    return b;
  };
  active_ = unavailable("Radar active");
  guard_ = unavailable("Guard zone");
  pause_ = unavailable("Pause sweep");
  plugins_ = new XNavButton(control_body_, wxID_ANY, "OpenCPN radar plugins",
                            "OpenCPN radar plugins");
  plugins_->SetRole(ButtonRole::Quiet);
  plugins_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
    if (on_plugins)
      CallAfter([this] { on_plugins(); });
  });
  Bind(wxEVT_PAINT, &XNavRadarPanel::Paint, this);
  Bind(wxEVT_SIZE, [this](wxSizeEvent &e) {
    Reflow();
    Refresh(false);
    e.Skip();
  });
  Reflow();
}
void XNavRadarPanel::Update(const adapters::RadarState &state, vessel::Time now,
                            bool replay, LightMode light) {
  const bool current = state.available && !replay &&
                       state.observed_at != vessel::Time{} &&
                       state.observed_at <= now &&
                       now - state.observed_at < std::chrono::seconds(3);
  const bool changed = !initialized_ || state_.source != state.source ||
                       status_current_ != current || replay_ != replay ||
                       light_ != light;
  state_ = state;
  if (!changed)
    return;
  initialized_ = true;
  status_current_ = current;
  replay_ = replay;
  light_ = light;
  SetBackgroundColour(Colour(Theme(light).background));
  controls_->SetBackgroundColour(Colour(Theme(light).background));
  control_body_->SetBackgroundColour(Colour(Theme(light).background));
  for (auto *b : Controls())
    b->SetLightMode(light);
  // Capability/status is not an owned radar image or verified command path.
  // Even a fresh, capability-rich report cannot enable controls in this view.
  for (auto *b : {active_, guard_, pause_}) {
    b->SetSelected(false);
    b->Enable(false);
  }
  plugins_->Enable(bool(on_plugins) && !replay);
  Refresh(false);
  control_body_->Refresh(false);
}
void XNavRadarPanel::Reflow() {
  const RadarLayout l(ToDIP(GetClientSize().x),
                      ToDIP(GetParent()->GetClientSize().y),
                      ToDIP(wxGetTopLevelParent(GetParent())->GetClientSize().x));
  const auto rect = [this](int x, int y, int w, int h) {
    return wxRect(FromDIP(wxPoint(x, y)), FromDIP(wxSize(w, h)));
  };
  close_->SetSize(rect(32 + l.width - 86, 28, 86, 44));
  const int y = l.ControlsY(), right = l.right - 8;
  active_->SetSize(rect(right - 48, 40, 48, 52));
  guard_->SetSize(rect(right - 48, 414, 48, 58));
  pause_->SetSize(rect(right - 48, 472, 48, 52));
  plugins_->SetSize(rect(0, 584, right, 48));
  controls_->SetSize(rect(l.x, y, l.right, l.height));
  control_body_->SetMinSize(FromDIP(wxSize(right, 692)));
  controls_->Layout();
  controls_->FitInside();
  SetMinSize(FromDIP(wxSize(300, y + l.height + 46)));
  // The parent owns this panel's size. Re-entering its sizer from this size
  // event recursively resizes XNavScroll and exhausts the application stack.
}
std::vector<std::pair<wxString, wxRect>> XNavRadarPanel::Regions() const {
  const RadarLayout l(ToDIP(GetClientSize().x),
                      ToDIP(GetParent()->GetClientSize().y),
                      ToDIP(wxGetTopLevelParent(GetParent())->GetClientSize().x));
  return {{"Radar display", {32, l.y, l.left, l.height}},
          {"Radar controls", {l.x, l.ControlsY(), l.right, l.height}}};
}
void XNavRadarPanel::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);
  XNavPainter p(*this, dc, light_);
  dc.SetBackground(wxBrush(Colour(p.c.background)));
  dc.Clear();
  const RadarLayout l(ToDIP(GetClientSize().x),
                      ToDIP(GetParent()->GetClientSize().y),
                      ToDIP(wxGetTopLevelParent(GetParent())->GetClientSize().x));
  p.TextTracked("RADAR / SITUATIONAL AWARENESS", 32, 36, 9, p.c.accent, 650,
                1.17);
  p.TextTracked("Read beyond the chart.", 32, 56, 30, p.c.primary, 400, -1.2,
                l.width - 100);
  p.Text(replay_ ? "REPLAY / Historical status. No live radar display."
                 : "Radar unavailable. Continue to monitor the chart and "
                   "surroundings.",
         32, 104, 12, p.c.muted, false, l.width);
  const auto ink = RadarTheme();
  dc.SetPen(wxPen(Colour(p.c.border)));
  dc.SetBrush(wxBrush(Colour(ink.surface)));
  dc.DrawRoundedRectangle(p.D(32), p.D(l.y), p.D(l.left), p.D(l.height),
                          p.D(18));
  // Flex centering includes the 18px legend margin and 13.5px line box.
  const int sx = 32 + (l.left - l.scope) / 2,
            sy = l.y + int(std::lround((l.height - l.scope - 31.5) / 2));
  {
    std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(dc));
    if (gc) {
      const double dip = FromDIP(1024) / 1024.;
      gc->Scale(dip, dip);
      const double cx = sx + l.scope * .5, cy = sy + l.scope * .5,
                   r = l.scope * .5;
      wxGraphicsGradientStops stops(Colour(ink.center), Colour(ink.surface));
      stops.Add(Colour(ink.middle), .45);
      stops.Add(Colour(ink.surface), .69);
      // CSS circle defaults to farthest-corner, not the inscribed radius.
      gc->SetBrush(gc->CreateRadialGradientBrush(cx, cy, cx, cy,
                                                r * std::sqrt(2.), stops));
      gc->SetPen(wxPen(Alpha(ink.border, ink.border_alpha)));
      gc->DrawEllipse(sx, sy, l.scope, l.scope);
      // Neutral reference grid only: no synthetic returns, bearing vector,
      // ownship heading or calibrated range can exist without an image source.
      gc->SetBrush(*wxTRANSPARENT_BRUSH);
      wxGraphicsPenInfo rings(Alpha(ink.rings, ink.rings_alpha));
      rings.Width(.7);
      gc->SetPen(gc->CreatePen(rings));
      for (int ring = 1; ring <= 3; ++ring) {
        const double rr = l.scope / 2.12 * ring / 3.;
        gc->DrawEllipse(cx - rr, cy - rr, 2 * rr, 2 * rr);
      }
      gc->SetPen(wxPen(Alpha(ink.crosshair, ink.crosshair_alpha)));
      gc->DrawEllipse(cx - r * .84, cy - r * .84, 2 * r * .84, 2 * r * .84);
      gc->StrokeLine(cx - r * .84, cy, cx + r * .84, cy);
      gc->StrokeLine(cx, cy - r * .84, cx, cy + r * .84);
    }
  }
  const wxString caption = "NO RADAR IMAGE", detail = "NO VALIDATED RECEIVE SOURCE";
  const int label_width = ToDIP(int(UiTextWidth(*this, caption, 15, 500))) + 13;
  const int detail_width = ToDIP(int(UiTextWidth(*this, detail, 8))) +
                           int((detail.size() - 1) * .8);
  p.TextTracked(caption, sx + (l.scope - label_width) / 2,
                sy + l.scope - 58, 15, ink.caption, 500, 1.);
  p.TextTracked(detail, sx + (l.scope - detail_width) / 2,
                sy + l.scope - 33, 8, ink.caption, 400, .8);
  const int legend = std::min(470, l.left - 42), lx = 32 + (l.left - legend) / 2;
  p.Text("No returns available", lx, sy + l.scope + 18, 9, ink.legend);
  p.TextWeight("Range unavailable", lx + legend / 2, sy + l.scope + 18, 9,
               ink.legend, 400, legend / 2, true);
}
void XNavRadarPanel::PaintControls(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(control_body_);
  XNavPainter p(*control_body_, dc, light_);
  dc.SetBackground(wxBrush(Colour(p.c.background)));
  dc.Clear();
  const int x = 0, y = 0,
            w = control_body_->ToDIP(control_body_->GetClientSize().x);
  p.Tag(replay_ ? "REPLAY" : "UNAVAILABLE", x, y, w, true);
  p.Text("Radar active", x, y + 60, 11, p.c.secondary);
  p.Rule(x, y + 92, w);
  p.Text("Range", x, y + 108, 12, p.c.secondary);
  dc.SetPen(wxPen(Colour(p.c.border)));
  dc.SetBrush(wxBrush(Colour(p.c.background)));
  dc.DrawRoundedRectangle(p.D(x), p.D(y + 132), p.D(w), p.D(46), p.D(9));
  p.Text(W("—"), x + 18, y + 145, 13, p.c.muted);
  int yy = y + 196;
  for (const char *label :
       {"Gain", "Sea clutter", "Rain suppression", "Overlay opacity"}) {
    p.Text(label, x, yy, 12, p.c.secondary);
    p.TextWeight(W("—"), x + w - 28, yy, 12, p.c.muted, 400, 28, true);
    dc.SetPen(wxPen(Colour(p.c.border)));
    dc.SetBrush(wxBrush(Colour(p.c.surface)));
    dc.DrawRoundedRectangle(p.D(x + 2), p.D(yy + 20), p.D(w - 4), p.D(7),
                            p.D(3));
    yy += 57;
  }
  p.Text("Guard zone", x, y + 426, 11, p.c.secondary);
  p.Text("Unavailable", x, y + 444, 10, p.c.muted);
  p.Rule(x, y + 472, w);
  p.Text("Pause sweep", x, y + 490, 11, p.c.secondary);
  p.Rule(x, y + 524, w);
  p.Wrapped("No validated radar display adapter is connected. Scanner controls "
            "remain disabled. Compatible plugin interfaces are available "
            "through OpenCPN.",
            x, y + 536, 11, 18, w, p.c.muted, 3);
  wxString detail = "Source unavailable";
  if (!state_.source.empty()) {
    detail = (status_current_ ? "Status only / "
                              : "Status unavailable or stale / ") +
             W(state_.source);
  }
  p.Wrapped(detail, x, y + 646, 11, 18, w, p.c.muted, 2);
}
} // namespace opennav::ui

#include "ui/Range.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <wx/dcbuffer.h>

namespace opennav::ui {
XNavRange::XNavRange(wxWindow *parent, const wxString &name, int minimum,
                     int maximum, int step, int value)
    : wxControl(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                wxBORDER_NONE),
      minimum_(minimum), maximum_(maximum), step_(step), value_(value) {
  if (maximum <= minimum || step <= 0)
    throw std::invalid_argument("Invalid range limits");
  SetName(name);
  SetLabel(name);
  SetValue(value);
  SetMinSize(FromDIP(wxSize(160, 46)));
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  Bind(wxEVT_PAINT, &XNavRange::Paint, this);
  Bind(wxEVT_LEFT_DOWN, [this](wxMouseEvent &e) {
    if (!IsEnabled())
      return;
    SetFocus();
    CaptureMouse();
    Choose(e.GetX());
  });
  Bind(wxEVT_MOTION, [this](wxMouseEvent &e) {
    if (HasCapture() && e.LeftIsDown())
      Choose(e.GetX());
  });
  Bind(wxEVT_LEFT_UP, [this](wxMouseEvent &e) {
    if (HasCapture()) {
      Choose(e.GetX());
      ReleaseMouse();
    }
  });
  Bind(wxEVT_MOUSE_CAPTURE_LOST,
       [this](wxMouseCaptureLostEvent &) { Refresh(false); });
  Bind(wxEVT_KEY_DOWN, [this](wxKeyEvent &e) {
    if (!IsEnabled())
      return;
    const int key = e.GetKeyCode(), old = value_;
    if (key == WXK_LEFT || key == WXK_DOWN)
      SetValue(value_ - step_);
    else if (key == WXK_RIGHT || key == WXK_UP)
      SetValue(value_ + step_);
    else if (key == WXK_HOME)
      SetValue(minimum_);
    else if (key == WXK_END)
      SetValue(maximum_);
    else {
      e.Skip();
      return;
    }
    if (old != value_ && on_change)
      on_change(value_);
  });
  Bind(wxEVT_SET_FOCUS, [this](wxFocusEvent &e) {
    Refresh(false);
    e.Skip();
  });
  Bind(wxEVT_KILL_FOCUS, [this](wxFocusEvent &e) {
    Refresh(false);
    e.Skip();
  });
}
void XNavRange::SetValue(int value) {
  const auto clamped = std::clamp(value, minimum_, maximum_);
  value_ = std::clamp(minimum_ + static_cast<int>(std::round(
                                     double(clamped - minimum_) / step_)) *
                                     step_,
                      minimum_, maximum_);
  Refresh(false);
}
void XNavRange::Choose(int x) {
  if (!IsEnabled())
    return;
  const int margin = FromDIP(13),
            length = (std::max)(1, GetClientSize().x - 2 * margin),
            old = value_;
  const double fraction = std::clamp(double(x - margin) / length, 0., 1.);
  SetValue(minimum_ +
           static_cast<int>(std::round(fraction * (maximum_ - minimum_))));
  if (old != value_ && on_change)
    on_change(value_);
}
void XNavRange::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);
  const auto c = Theme(light_);
  dc.SetBackground(wxBrush(Colour(c.background)));
  dc.Clear();
  const int margin = FromDIP(13), width = GetClientSize().x - 2 * margin,
            y = GetClientSize().y / 2;
  const int x = margin + static_cast<int>(double(value_ - minimum_) /
                                          (maximum_ - minimum_) * width);
  dc.SetPen(wxPen(Colour(c.border), FromDIP(4)));
  dc.DrawLine(margin, y, margin + width, y);
  const auto ink = IsEnabled() ? c.accent : c.muted;
  dc.SetPen(wxPen(Colour(ink), FromDIP(4)));
  dc.DrawLine(margin, y, x, y);
  dc.SetPen(*wxTRANSPARENT_PEN);
  dc.SetBrush(wxBrush(Colour(ink)));
  dc.DrawCircle(x, y, FromDIP(8));
  if (HasFocus()) {
    dc.SetBrush(*wxTRANSPARENT_BRUSH);
    dc.SetPen(wxPen(Colour(c.secondary)));
    dc.DrawCircle(x, y, FromDIP(11));
  }
}
} // namespace opennav::ui

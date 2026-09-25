#include "ui/Controls.h"

#include <wx/dcbuffer.h>
#include <wx/graphics.h>
#include <wx/sizer.h>
#ifdef __WXMSW__
#include <wx/msw/wrapwin.h>
#endif

#include <memory>

namespace opennav::ui {

XNavScroll::XNavScroll(wxWindow *parent)
    : wxScrolledWindow(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                       wxVSCROLL | wxBORDER_NONE) {
  SetScrollRate(0, FromDIP(24));
  ShowScrollbars(wxSHOW_SB_NEVER, wxSHOW_SB_NEVER);
  EnableScrollGesture(*this);
}

bool XNavScroll::Layout() {
  if (!GetSizer()) return wxScrolledWindow::Layout();
  // wx 3.2 ScrollLayout treats a hidden native scrollbar as disabled scrolling
  // and shrinks the sizer to the viewport. Keep full-height content when using
  // our explicit controls, otherwise quality labels and touch rows are clipped.
  const wxSize content(GetClientSize().x,
      std::max(GetVirtualSize().y, GetSizer()->GetMinSize().y));
  GetSizer()->SetDimension(CalcScrolledPosition(wxPoint(0, 0)), content);
  return true;
}

bool XNavScroll::CanScroll(int direction) const {
  int x, y, ux, uy;
  GetViewStart(&x, &y);
  GetScrollPixelsPerUnit(&ux, &uy);
  return direction < 0 ? y > 0
      : y * uy + GetClientSize().y < GetVirtualSize().y;
}

void XNavScroll::Step(int direction) {
  int x, y, ux, uy;
  GetViewStart(&x, &y);
  GetScrollPixelsPerUnit(&ux, &uy);
  if (uy > 0)
    Scroll(0, std::max(0, y + direction * std::max(1, GetClientSize().y * 2 / (3 * uy))));
}

void XNavScroll::Pan(wxPanGestureEvent &event) {
  if (event.IsGestureStart()) pan_remainder_ = 0;
  int x, y, ux, uy;
  GetViewStart(&x, &y);
  GetScrollPixelsPerUnit(&ux, &uy);
  if (uy <= 0) return;
  pan_remainder_ -= event.GetDelta().y;
  const int steps = pan_remainder_ / uy;
  pan_remainder_ %= uy;
  Scroll(0, std::max(0, y + steps));
}

void EnableScrollGesture(wxWindow &window) {
  window.EnableTouchEvents(wxTOUCH_VERTICAL_PAN_GESTURE);
  window.Bind(wxEVT_GESTURE_PAN, [&window](wxPanGestureEvent &event) {
    for (auto *parent = &window; parent; parent = parent->GetParent())
      if (auto *scroll = dynamic_cast<XNavScroll *>(parent)) {
        scroll->Pan(event);
        return;
      }
    event.Skip();
  });
}

bool ThemeWindowChrome(wxWindow &window, LightMode mode) {
#ifdef __WXMSW__
  // Documented DWM attributes; unsupported older Windows retain their native
  // caption. No undocumented ordinals, global theme or registry changes.
  const auto library = LoadLibraryExW(L"dwmapi.dll", nullptr, LOAD_LIBRARY_SEARCH_SYSTEM32);
  if (!library) return false;
  using SetAttribute = HRESULT(WINAPI *)(HWND, DWORD, LPCVOID, DWORD);
  const auto set = reinterpret_cast<SetAttribute>(GetProcAddress(library, "DwmSetWindowAttribute"));
  bool applied = false;
  if (set) {
    const auto c = Theme(mode);
    const BOOL dark = TRUE;
    const COLORREF caption = RGB((c.surface >> 16) & 255, (c.surface >> 8) & 255, c.surface & 255);
    const COLORREF text = RGB((c.secondary >> 16) & 255, (c.secondary >> 8) & 255, c.secondary & 255);
    const auto handle = static_cast<HWND>(window.GetHandle());
    set(handle, 20, &dark, sizeof(dark));
    const auto a = set(handle, 35, &caption, sizeof(caption));
    const auto b = set(handle, 36, &text, sizeof(text));
    applied = SUCCEEDED(a) && SUCCEEDED(b);
  }
  FreeLibrary(library);
  return applied;
#else
  return false;
#endif
}

wxColour Colour(std::uint32_t rgb) {
  return wxColour((rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255);
}

wxFont UiFont(wxWindow& window, int pixels, bool bold) {
#ifdef __WXMSW__
  const wxString face = "Segoe UI";
#else
  const wxString face = "Sans";
#endif
  return wxFont(wxFontInfo(wxSize(0, window.FromDIP(pixels)))
                    .FaceName(face).Weight(bold ? wxFONTWEIGHT_BOLD : wxFONTWEIGHT_NORMAL));
}

XNavButton::XNavButton(wxWindow* parent, wxWindowID id, const wxString& label,
                       const wxString& accessible_name)
    : wxControl(parent, id, wxDefaultPosition, wxDefaultSize, wxBORDER_NONE) {
  SetLabel(label);
  SetName(accessible_name);
  SetToolTip(accessible_name);
  SetMinSize(FromDIP(wxSize(spacing::touch, spacing::touch)));
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  EnableTouchEvents(wxTOUCH_VERTICAL_PAN_GESTURE);
  Bind(wxEVT_GESTURE_PAN, [this](wxPanGestureEvent &e) {
    pressed_ = false;
    if (HasCapture()) ReleaseMouse();
    Refresh();
    for (auto *p = GetParent(); p; p = p->GetParent())
      if (auto *scroll = dynamic_cast<XNavScroll *>(p)) { scroll->Pan(e); return; }
    e.Skip();
  });
  Bind(wxEVT_PAINT, &XNavButton::Paint, this);
  Bind(wxEVT_SET_FOCUS, [this](wxFocusEvent& e) { Refresh(); e.Skip(); });
  Bind(wxEVT_KILL_FOCUS, [this](wxFocusEvent& e) { pressed_ = false; Refresh(); e.Skip(); });
  Bind(wxEVT_LEFT_DOWN, [this](wxMouseEvent&) {
    if (!IsEnabled()) return;
    SetFocus();
    pressed_ = true;
    CaptureMouse();
    Refresh();
  });
  Bind(wxEVT_LEFT_UP, [this](wxMouseEvent& e) {
    const bool activate = pressed_ && GetClientRect().Contains(e.GetPosition());
    pressed_ = false;
    if (HasCapture()) ReleaseMouse();
    Refresh();
    if (activate) Activate();
  });
  Bind(wxEVT_MOUSE_CAPTURE_LOST, [this](wxMouseCaptureLostEvent&) {
    pressed_ = false;
    Refresh();
  });
  Bind(wxEVT_KEY_DOWN, [this](wxKeyEvent& e) {
    if (e.GetKeyCode() == WXK_SPACE || e.GetKeyCode() == WXK_RETURN) {
      pressed_ = IsEnabled();
      Refresh();
    } else e.Skip();
  });
  Bind(wxEVT_KEY_UP, [this](wxKeyEvent& e) {
    if (e.GetKeyCode() == WXK_SPACE || e.GetKeyCode() == WXK_RETURN) {
      const bool activate = pressed_;
      pressed_ = false;
      Refresh();
      if (activate) Activate();
    } else e.Skip();
  });
}

void XNavButton::SetLabel(const wxString &label) {
  if(GetLabel() == label) return;
  wxControl::SetLabel(label);
  Refresh();
}

void XNavButton::SetLightMode(LightMode mode) { mode_ = mode; Refresh(); }

void XNavButton::Activate() {
  if (!IsEnabled()) return;
  wxCommandEvent event(wxEVT_BUTTON, GetId());
  event.SetEventObject(this);
  // Queue dispatch so a restart callback cannot destroy the active input handler.
  wxPostEvent(this, event);
}

void XNavButton::Paint(wxPaintEvent&) {
  wxAutoBufferedPaintDC dc(this);
  const auto colors = Theme(mode_);
  dc.SetBackground(wxBrush(Colour(colors.surface)));
  dc.Clear();
  const auto size = GetClientSize();
  const auto edge = Colour(HasFocus() ? colors.accent : colors.border);
  dc.SetPen(wxPen(edge, FromDIP(HasFocus() ? 2 : 1)));
  dc.SetBrush(wxBrush(Colour(pressed_ ? colors.selected : colors.surface)));
  dc.DrawRoundedRectangle(1, 1, size.x - 2, size.y - 2, FromDIP(spacing::control_radius));
  dc.SetFont(UiFont(*this, 14, false));
  dc.SetTextForeground(Colour(IsEnabled() ? colors.primary : colors.muted));
  const auto label=wxControl::Ellipsize(GetLabel(),dc,wxELLIPSIZE_END,std::max(1,size.x-FromDIP(20)));
  const auto extent = dc.GetTextExtent(label);
  dc.DrawText(label, (size.x - extent.x) / 2, (size.y - extent.y) / 2);
}

XNavDataValue::XNavDataValue(wxWindow* parent, const wxString& label,
                             const wxString& unit, int decimals)
    : wxPanel(parent, wxID_ANY), label_(label), unit_(unit), decimals_(decimals) {
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  SetMinSize(FromDIP(wxSize(120, 120)));
  SetName(label + " unavailable");
  EnableScrollGesture(*this);
  Bind(wxEVT_PAINT, &XNavDataValue::Paint, this);
}

void XNavDataValue::SetLightMode(LightMode mode) { mode_ = mode; Refresh(); }

void XNavDataValue::SetReading(const vessel::Sample& sample, vessel::Time now) {
  sample_ = sample;
  reading_ = vessel::Assess(sample, now);
  const wxString source = sample.source.empty() ? "No source" : wxString::FromUTF8(sample.source);
  const wxString age = reading_.age
      ? wxString::Format("%.1f s old", reading_.age->count() / 1000.0) : "Age unavailable";
  SetToolTip(source + "\n" + age);
  SetName(label_ + ": " + (reading_.value ? wxString::Format("%.*f", decimals_, *reading_.value) : "Unavailable"));
  Refresh();
}

void XNavDataValue::Paint(wxPaintEvent&) {
  wxAutoBufferedPaintDC dc(this);
  const auto colors = Theme(mode_);
  dc.SetBackground(wxBrush(Colour(colors.surface)));
  dc.Clear();
  const int x = FromDIP(12);
  dc.SetPen(wxPen(Colour(colors.border)));
  dc.DrawLine(x, GetClientSize().y - 1, GetClientSize().x - x, GetClientSize().y - 1);
  dc.SetFont(UiFont(*this, 11, true));
  dc.SetTextForeground(Colour(colors.secondary));
  dc.DrawText(label_, x, FromDIP(12));
  const bool stale = reading_.quality == vessel::Quality::Stale;
  dc.SetFont(UiFont(*this, 32, true));
  dc.SetTextForeground(Colour(stale ? colors.muted : colors.primary));
  dc.DrawText(reading_.value ? wxString::Format("%.*f", decimals_, *reading_.value) : wxString::FromUTF8("—"),
              x, FromDIP(30));
  dc.SetFont(UiFont(*this, 13));
  dc.SetTextForeground(Colour(colors.secondary));
  dc.DrawText(unit_, x, FromDIP(69));
  wxString status;
  switch (reading_.quality) {
    case vessel::Quality::Live: status = "LIVE"; break;
    case vessel::Quality::Aging: status = "AGING"; break;
    case vessel::Quality::Stale: status = "STALE"; break;
    case vessel::Quality::Unavailable: status = "UNAVAILABLE"; break;
    case vessel::Quality::Estimated: status = "EST"; break;
    case vessel::Quality::Uncertain: status = "UNCERTAIN"; break;
  }
  if (reading_.age && reading_.quality != vessel::Quality::Live) {
    status += wxString::Format(" %.0fs", reading_.age->count() / 1000.0);
  }
  dc.SetFont(UiFont(*this, 10, true));
  dc.SetTextForeground(Colour(stale ? colors.attention : colors.muted));
  dc.DrawText(status, x, FromDIP(96));
}

}  // namespace opennav::ui

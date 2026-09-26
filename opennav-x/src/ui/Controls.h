#pragma once

#include "ui/Theme.h"
#include "vessel/VesselState.h"

#include <wx/control.h>
#include <wx/panel.h>
#include <wx/scrolwin.h>
#include <wx/dc.h>

namespace opennav::ui {

wxColour Colour(std::uint32_t rgb);
wxFont UiFont(wxWindow& window, int pixels, bool bold = false);

// Shared drawing primitives. All geometry is in logical DIP; semantic palette
// roles, radii and typography remain identical across painted product pages.
class XNavPainter {
 public:
  XNavPainter(wxWindow &window, wxDC &dc, LightMode mode)
      : window_(window), dc_(dc), c(Theme(mode)) {}
  int D(int value) const { return window_.FromDIP(value); }
  void Text(wxString text, int x, int y, int size, std::uint32_t color,
            bool bold = false, int width = 0);
  void Card(int x, int y, int width, int height, const wxString &title);
  void Rule(int x, int y, int width);
 private:
  wxWindow &window_;
  wxDC &dc_;
 public:
  const Palette c;
};

enum class ButtonRole { Normal, Quiet, Primary, Critical };
enum class XNavIcon { None, Plus, Minus, Ownship, Menu, Back, Close, Route, Compass, Settings };

// Shared touch/wheel scrolling with no bright native scrollbar. Persistent
// XNav navigation buttons are supplied outside the scrolling content.
class XNavScroll : public wxScrolledWindow {
 public:
  explicit XNavScroll(wxWindow *parent);
  bool Layout() override;
  bool CanScroll(int direction) const;
  void Step(int direction);
  void Pan(wxPanGestureEvent &event);
 private:
  int pan_remainder_ = 0;
};
void EnableScrollGesture(wxWindow &window);
bool ThemeWindowChrome(wxWindow &window, LightMode mode);

class XNavButton : public wxControl {
 public:
  XNavButton(wxWindow* parent, wxWindowID id, const wxString& label,
             const wxString& accessible_name);
  void SetLightMode(LightMode mode);
  void SetLabel(const wxString &label) override;
  void SetRole(ButtonRole role) { role_ = role; Refresh(); }
  void SetSelected(bool selected) { if(selected_ != selected) { selected_ = selected; Refresh(); } }
  void SetIcon(XNavIcon icon) { icon_ = icon; Refresh(); }

 private:
  void Paint(wxPaintEvent& event);
  void Activate();
  LightMode mode_ = LightMode::Day;
  bool pressed_ = false;
  bool selected_ = false;
  ButtonRole role_ = ButtonRole::Normal;
  XNavIcon icon_ = XNavIcon::None;
};

class XNavIconButton final : public XNavButton {
 public:
  XNavIconButton(wxWindow *parent, wxWindowID id, XNavIcon icon,
                 const wxString &label, const wxString &accessible_name)
      : XNavButton(parent, id, label, accessible_name) {
    SetIcon(icon); SetRole(ButtonRole::Quiet);
  }
};

class XNavDataValue final : public wxPanel {
 public:
  XNavDataValue(wxWindow* parent, const wxString& label, const wxString& unit,
                int decimals = 1);
  void SetLightMode(LightMode mode);
  void SetReading(const vessel::Sample& sample, vessel::Time now);
  void SetCompact(bool compact);

 private:
  void Paint(wxPaintEvent& event);
  LightMode mode_ = LightMode::Day;
  wxString label_, unit_;
  int decimals_;
  vessel::Sample sample_;
  vessel::Assessment reading_;
  bool compact_ = false;
};

}  // namespace opennav::ui

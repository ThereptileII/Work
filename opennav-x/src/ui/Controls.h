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
wxFont UiFontWeight(wxWindow& window, int pixels, int weight);
// Natural text advance in physical pixels. Windows uses fractional DirectWrite
// metrics, avoiding cumulative GDI glyph rounding in wrapping component rows.
double UiTextWidth(wxWindow& window, const wxString& text, int pixels, int weight = 400);

// Shared drawing primitives. All geometry is in logical DIP; semantic palette
// roles, radii and typography remain identical across painted product pages.
class XNavPainter {
 public:
  XNavPainter(wxWindow &window, wxDC &dc, LightMode mode)
      : window_(window), dc_(dc), c(Theme(mode)) {}
  int D(int value) const { return window_.FromDIP(value); }
  void Text(wxString text, int x, int y, int size, std::uint32_t color,
            bool bold = false, int width = 0);
  void TextWeight(wxString text, int x, int y, int size, std::uint32_t color,
                  int weight, int width = 0, bool right = false);
  void TextTracked(wxString text, int x, int y, int size, std::uint32_t color,
                   int weight, double tracking, int width = 0);
  void Wrapped(const wxString &text, int x, int y, int size, int line_height,
               int width, std::uint32_t color, int maximum_lines = 3);
  void Stat(const wxString &label, const wxString &value, const wxString &unit,
            int x, int y, int width);
  int Tag(const wxString &text, int x, int y, int maximum, bool attention = false,
          bool active = false);
  void Callout(const wxString &title, const wxString &body, int width, int height,
               bool attention = false);
  void Card(int x, int y, int width, int height, const wxString &title);
  void Rule(int x, int y, int width);
 private:
  wxWindow &window_;
  wxDC &dc_;
 public:
  const Palette c;
};

enum class ButtonRole { Normal, Quiet, Primary, Critical, Segment };
enum class XNavIcon { None, Plus, Minus, Ownship, Menu, Back, Close, Route, Compass, Settings,
  Chart, Traffic, Energy, Instruments, Anchor, Radar, Sun, Dusk, Moon, Bell,
  Search, Layers, Ruler, Pin, Sliders, Chevron, Edit, Shield };

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
  void SetHint(const wxString &hint);
#if wxUSE_HELP
  wxString GetHelpTextAtPoint(const wxPoint &, wxHelpEvent::Origin) const override {
    return hint_;
  }
#endif
  void SetLabel(const wxString &label) override;
  void SetRole(ButtonRole role) { role_ = role; Refresh(); }
  void SetSelected(bool selected) { if(selected_ != selected) { selected_ = selected; Refresh(); } }
  bool IsSelected() const { return selected_; }
  void SetIcon(XNavIcon icon) { icon_ = icon; Refresh(); }
  void SetNavigationItem(bool value = true) { navigation_item_ = value; Refresh(); }
  void SetIconOnly(bool value = true) { icon_only_ = value; Refresh(); }
  void SetIconSize(int pixels) { if (pixels >= 8 && pixels <= 32) { icon_size_ = pixels; Refresh(); } }
  void SetFloating(bool value = true) { floating_ = value; Refresh(); }
  void SetInlineIcon(bool value = true) { inline_icon_ = value; Refresh(); }
  void SetCompassRotation(double radians) { if (compass_rotation_ != radians) { compass_rotation_ = radians; Refresh(); } }
  void SetSummary(const wxString &value, const wxString &detail);
  void SetSettingsTab(bool value = true) { settings_tab_ = value; Refresh(); }
  void SetVesselProfile() { vessel_profile_ = true; Refresh(); }
  // Prototype 42x25 toggle inside its extended 48x49 touch target. State is
  // supplied by the owner; activation never optimistically changes it.
  void SetToggle() { toggle_ = true; SetMinSize(FromDIP(wxSize(48,49))); Refresh(); }
  void SetTextSize(int value) { if(value>=9&&value<=24) { text_size_=value; Refresh(); } }
  void SetSuiteLink(const wxString &detail, XNavIcon icon) {
    suite_link_ = true; suite_detail_ = detail; icon_ = icon; Refresh();
  }

 private:
  void Paint(wxPaintEvent& event);
  void Activate();
  LightMode mode_ = LightMode::Day;
  wxString hint_;
  wxString summary_value_, summary_detail_;
  bool summary_ = false;
  bool pressed_ = false;
  bool keyboard_focus_ = false;
  bool selected_ = false;
  bool hovered_ = false, navigation_item_ = false, icon_only_ = false, floating_ = false;
  bool inline_icon_ = false;
  bool settings_tab_ = false, suite_link_ = false, vessel_profile_ = false;
  bool toggle_ = false;
  int text_size_ = 12;
  wxString suite_detail_;
  double compass_rotation_ = 0;
  ButtonRole role_ = ButtonRole::Normal;
  XNavIcon icon_ = XNavIcon::None;
  int icon_size_ = 22;
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
#if wxUSE_HELP
  wxString GetHelpTextAtPoint(const wxPoint &, wxHelpEvent::Origin) const override {
    return hint_;
  }
#endif

 private:
  void Paint(wxPaintEvent& event);
  LightMode mode_ = LightMode::Day;
  wxString label_, unit_, hint_;
  int decimals_;
  vessel::Sample sample_;
  vessel::Assessment reading_;
  bool compact_ = false;
};

// Equal CSS-style rows use cumulative rounding, avoiding per-row integer
// truncation that displaces later readings at the 1280x800 target.
class XNavDataRail final : public wxPanel {
 public:
  explicit XNavDataRail(wxWindow *parent);
  bool Layout() override;
};

}  // namespace opennav::ui

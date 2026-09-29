#pragma once
#include "ui/Drawer.h"
#include "ui/ProductPanel.h"

namespace opennav::ui {
enum class SettingsSection { Vessel, Navigation, Sensors, Autopilot, Radar, Display, System, Help };
struct SettingsDrawerActions {
  std::function<void(ProductPage)> page;
  std::function<void()> advanced, plugins, diagnostics, fullscreen, legacy, safe;
  std::function<void(LightMode)> theme;
};
// Presentation/navigation only. Configuration mutations continue through the
// existing validated product settings flows; no protocol or actuator methods.
class XNavSettingsDrawer final : public XNavDrawer {
public:
  XNavSettingsDrawer(wxWindow &, SettingsDrawerActions);
  void Select(SettingsSection);
  SettingsSection Section() const { return section_; }
  void Update(const ProductState &, LightMode);
private:
  void Build();
  void CopyBlock(int height, std::function<void(XNavPainter &,int)>);
  void Link(const wxString &, const wxString &, XNavIcon, std::function<void()>);
  void Page(const wxString &, const wxString &, XNavIcon, ProductPage);
  void Button(const wxString &, std::function<void()>, ButtonRole = ButtonRole::Normal);
  SettingsDrawerActions actions_;
  ProductState state_;
  SettingsSection section_ = SettingsSection::Vessel;
  wxPanel *tabs_ = nullptr;
  std::vector<XNavButton *> tabs_buttons_, buttons_;
  std::vector<std::pair<XNavButton *, LightMode>> light_buttons_;
  std::vector<wxPanel *> copies_;
};
} // namespace opennav::ui

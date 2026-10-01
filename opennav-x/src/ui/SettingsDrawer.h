#pragma once
#include "ui/Drawer.h"
#include "ui/ProductPanel.h"
#include "ui/ChoiceField.h"
#include "application/DisplayPreferences.h"
#include <array>
#include <wx/stattext.h>
#include <wx/textctrl.h>

namespace opennav::ui {
enum class SettingsSection { Vessel, Navigation, Sensors, Autopilot, Radar, Display, System, Help };
struct SettingsDrawerActions {
  std::function<void(ProductPage)> page;
  std::function<void()> advanced, plugins, diagnostics, fullscreen, legacy, safe;
  std::function<void(LightMode)> theme;
  std::function<application::DisplayPreferences()> display;
  std::function<application::CommandResult(const application::DisplayPreferences &)> save_display;
  std::function<application::Settings()> settings;
  std::function<application::CommandResult(const application::Settings &,
                                           const std::string &, double)> save_vessel;
};
// UI form and navigation only. The save callback owns validated profile writes;
// this drawer has no protocol or actuator methods.
class XNavSettingsDrawer final : public XNavDrawer {
public:
  XNavSettingsDrawer(wxWindow &, SettingsDrawerActions);
  void Select(SettingsSection);
  SettingsSection Section() const { return section_; }
  void Update(const ProductState &, LightMode);
  void ResetDraft();
  void SetDisplayPreferences(const application::DisplayPreferences &value);
private:
  void Build();
  void CopyBlock(int height, std::function<void(XNavPainter &,int)>);
  void Link(const wxString &, const wxString &, XNavIcon, std::function<void()>);
  void Page(const wxString &, const wxString &, XNavIcon, ProductPage);
  void Button(const wxString &, std::function<void()>, ButtonRole = ButtonRole::Normal);
  void VesselForm();
  void SaveVessel();
  void DisplayForm();
  void SaveDisplay();
  void ApplyScale();
  SettingsDrawerActions actions_;
  ProductState state_;
  SettingsSection section_ = SettingsSection::Vessel;
  wxPanel *tabs_ = nullptr;
  std::vector<XNavButton *> tabs_buttons_, buttons_;
  std::vector<std::pair<XNavButton *, LightMode>> light_buttons_;
  std::vector<wxPanel *> copies_;
  std::array<wxTextCtrl *,5> fields_{};
  std::array<wxString,5> draft_{};
  std::array<bool,5> touched_{};
  std::vector<wxPanel *> input_frames_;
  std::vector<wxPanel *> field_containers_;
  std::vector<wxStaticText *> field_captions_;
  wxStaticText *message_ = nullptr;
  bool draft_initialized_ = false;
  bool draft_dirty_ = false;
  bool loading_ = false;
  wxString feedback_;
  application::DisplayPreferences display_draft_;
  application::DisplayPreferences display_applied_;
  bool display_dirty_ = false;
  wxStaticText *display_message_ = nullptr;
  XNavChoiceField *scale_field_ = nullptr;
  XNavChoiceField *layout_field_ = nullptr;
};
} // namespace opennav::ui

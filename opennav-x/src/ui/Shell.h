#pragma once

#include "ui/Controls.h"
#include "ui/PreviewPanel.h"
#include "ui/ProductPanel.h"
#include "ui/ContextCard.h"
#include "ui/Horizon.h"
#include "ui/FloatingSurface.h"
#include "ui/AisDrawer.h"
#include "ui/PassageDrawer.h"
#include "ui/SettingsDrawer.h"
#include "ui/AnchorDrawer.h"
#include "ui/PilotDrawer.h"
#include "ui/AlertDrawer.h"
#include "ui/HealthDrawer.h"
#include "ui/StatusFooter.h"
#include "integration/BuildFeatures.h"
#if XNAV_ENABLE_TEST_FIXTURES
#include "vessel/DemoSource.h"
#endif
#include "vessel/AisSelection.h"

#include <wx/aui/aui.h>
#include <wx/frame.h>
#include <wx/stattext.h>
#include <wx/timer.h>
#include <wx/weakref.h>

#include <functional>
#include <memory>
#include <vector>

namespace opennav::ui {

struct ShellActions {
  std::function<diagnostics::FieldEnvironment()> field_environment;
  std::shared_ptr<diagnostics::Commissioning> commissioning;
  // Integration supplies AUI identities, never chart objects. Page visibility
  // is restored before OpenCPN saves its normal perspective on close.
  std::vector<wxString> navigation_panes;
  std::function<void()> zoom_in, zoom_out, follow, legacy;
  std::function<std::string()> chart_orientation;
  std::function<double()> chart_rotation;
  application::NavigationActions navigation;
  application::OnlineAisActions online_ais;
  std::function<void(bool)> online_ais_tick;
  std::function<application::CommandResult(int)> view_online_ais;
  std::function<bool()> route_creating;
  std::function<adapters::PilotView(bool, vessel::Time)> pilot_tick;
  std::function<std::vector<adapters::PilotCommand>(bool)> pilot_log;
  std::function<void(bool, adapters::PilotAction, double)> pilot_command;
  std::function<void(bool, bool)> pilot_enable;
  std::function<application::CommandResult()> pilot_identity;
  std::function<std::vector<std::string>()> pilot_sources;
  std::function<void()> restart_xnav, safe, diagnostics_folder;
  std::function<vessel::RouteProgress()> route;
  std::function<vessel::VesselState()> live_state;
  std::function<std::string()> chart_style_status;
  std::function<bool()> chart_style_requested;
  std::function<application::CommandResult(bool)> set_chart_style;
  std::function<application::Settings()> settings;
  std::function<std::string()> settings_status;
  std::function<std::string()> boat_bridge_status;
  std::function<application::CommandResult(const application::Settings &)>
      save_settings;
  std::function<std::vector<vessel::SourceHealth>()> source_health;
  std::function<adapters::RadarState()> radar;
  std::function<std::vector<std::string>()> build_info;
  std::function<void(const vessel::VesselState &,
                     const smartnav::EnergyPrediction &, const std::string &)>
      diagnostic_snapshot;
  std::function<void()> demo_chart;
  std::function<void(LightMode)> theme;
};

// The existing OpenCPN ChartCanvas stays in its original parent/AUI pane.
// Destroy before the owning frame's AUI manager is uninitialized.
class Shell final : public wxEvtHandler {
public:
  Shell(wxFrame &frame, wxAuiManager &manager, ShellActions actions,
        LightMode mode, bool simulation);
  ~Shell() override;
  struct UpdateMetrics {
    std::uint64_t ticks = 0;
    double last_ms = 0, mean_ms = 0, maximum_ms = 0;
  };
  UpdateMetrics Metrics() const { return metrics_; }
  const std::vector<application::Alert> &Alerts() const { return alerts_.Current(); }
  void UpdateState(const vessel::VesselState &state);
  void ShowAis(int mmsi);
  const smartnav::NavigationAdvice &Advice() const { return field_snapshot_.advice; }
  int SelectedAis() const { return ais_selection_.Selected(vessel::Clock::now()); }
  int PageScrollPosition() const;
  bool CanScrollPage(int direction) const;
  bool NativeCaptionThemed() const { return caption_themed_; }
  int MinimumValueHeight() const { return product_ && product_->IsShown() ? product_->MinimumValueHeight() : 0; }
  std::vector<ProductGeometry> ProductControls() const { return product_ && product_->IsShown() ? product_->ControlGeometry() : std::vector<ProductGeometry>{}; }
  std::vector<ProductGeometry> ProductRegions() const { return product_ && product_->IsShown() ? product_->RegionGeometry() : std::vector<ProductGeometry>{}; }
  std::vector<ProductGeometry> RailRegions() const;
  wxRect FooterRegion() const { return footer_->GetScreenRect(); }
  bool FooterMiddleVisible() const { return footer_->MiddleVisible(); }
  const application::FooterView &NavigationFooter() const { return footer_->View(); }
  std::optional<wxRect> DrawerRegion() const {
    if (health_drawer_ && health_drawer_->IsShown()) return health_drawer_->GetScreenRect();
    if (alert_drawer_ && alert_drawer_->IsShown()) return alert_drawer_->GetScreenRect();
    if (pilot_drawer_ && pilot_drawer_->IsShown()) return pilot_drawer_->GetScreenRect();
    if (anchor_drawer_ && anchor_drawer_->IsShown()) return anchor_drawer_->GetScreenRect();
    if (settings_drawer_ && settings_drawer_->IsShown()) return settings_drawer_->GetScreenRect();
    if (passage_drawer_ && passage_drawer_->IsShown()) return passage_drawer_->GetScreenRect();
    return ais_drawer_ && ais_drawer_->IsShown()
        ? std::make_optional(ais_drawer_->GetScreenRect()) : std::nullopt;
  }
  std::vector<ProductGeometry> InteractionControls() const;
  bool HasTransientSurface() const;
  bool RouteCreationActive() const { return actions_.route_creating && actions_.route_creating(); }
  const char *LightName() const;
  void ShowObject(const std::string &id, bool route);
  void AfterCanvasLayoutChanged();
  bool OwnsPane(const wxWindow *window) const;
  bool OwnsManager(const wxAuiManager &manager) const { return &manager == &manager_; }
  bool LoadPersistentPerspective(const wxString &perspective);
  void ShowChartContext(application::Coordinate position);

private:
  wxPanel *MakePane(const wxString &name, wxAuiPaneInfo placement);
  XNavButton *Button(wxWindow *parent, const wxString &text,
                     const wxString &name, std::function<void()> callback);
  wxStaticText *Text(wxWindow *parent, const wxString &text, int size,
                     bool bold = false);
  void ApplyTheme();
  void SetLight(LightMode mode);
  void UpdateRail(const std::vector<std::string> &keys, vessel::Time now);
  void Tick();
  void ApplyResponsiveLayout();
  void UpdateAlerts();
  void PlaceChartControls();
  void CloseContext();
  void UpdateContext(vessel::Time now);
  XNavScroll *CurrentScroll() const;
  void UpdateScrollControls();
  std::string PageTitle() const;
  void ShowSystem();
  void ShowProduct(ProductPage page);
  void ShowDemo();
  void ShowPage(PreviewPage page);
  void ShowNavigation();
  void ShowTraffic(int mmsi = 0);
  void ShowPassage();
  void ShowSettings();
  void ShowAnchor();
  void ShowPilot();
  void ShowAlerts();
  void ShowHealth();
  wxRect DrawerWorkspace() const;
  void OnCommand(wxCommandEvent &event);
  void StartDemo();
#if XNAV_ENABLE_TEST_FIXTURES
  void SelectDemo(vessel::DemoScenario scenario);
#endif
  wxString InputSummary() const;
  vessel::AisSelection ais_selection_;
  vessel::AisState ais_state_;
  application::OnlineAisState online_ais_state_;
  XNavAisDrawer *ais_drawer_ = nullptr;
  XNavPassageDrawer *passage_drawer_ = nullptr;
  XNavSettingsDrawer *settings_drawer_ = nullptr;
  XNavAnchorDrawer *anchor_drawer_ = nullptr;
  XNavPilotDrawer *pilot_drawer_ = nullptr;
  XNavAlertDrawer *alert_drawer_ = nullptr;
  XNavHealthDrawer *health_drawer_ = nullptr;
  PilotDrawerActions pilot_actions_;
  application::AlertCenter alerts_;
  wxPanel *alert_pane_ = nullptr;
  wxStaticText *alert_label_ = nullptr;
  XNavButton *alert_button_ = nullptr;
  UpdateMetrics metrics_;
  diagnostics::FieldSnapshot field_snapshot_;
  diagnostics::FieldJournal field_journal_;
  wxFrame &frame_;
  wxAuiManager &manager_;
  int original_pane_border_ = 0;
  int original_sash_size_ = 0;
  ShellActions actions_;
  LightMode mode_;
  vessel::VesselState state_;
  bool simulation_ = false;
  bool simulation_paused_ = false;
#if XNAV_ENABLE_TEST_FIXTURES
  vessel::DemoSource demo_{vessel::Clock::now()};
#endif
  PreviewPanel *page_ = nullptr;
  ProductPanel *product_ = nullptr;
  wxWeakRef<XNavContextCard> context_;
  std::string context_waypoint_;
  int context_mmsi_ = 0;
  std::optional<application::Coordinate> context_position_;
  std::shared_ptr<int> context_lifetime_ = std::make_shared<int>(0);
  std::vector<XNavButton *> navigation_page_buttons_;
  XNavButton *finish_route_ = nullptr;
  XNavButton *standby_ = nullptr;
  XNavButton *pilot_summary_ = nullptr;
  XNavButton *theme_button_ = nullptr;
  XNavButton *orientation_button_ = nullptr;
  XNavButton *undo_route_ = nullptr, *cancel_route_ = nullptr;
  PreviewPage current_page_ = PreviewPage::Route;
  std::vector<std::pair<wxString, bool>> navigation_visibility_;
  wxTimer timer_;
  std::vector<wxPanel *> panes_;
  std::vector<XNavButton *> buttons_;
  std::vector<std::pair<int, std::function<void()>>> commands_;
  std::vector<wxStaticText *> labels_;
  wxPanel *rail_scroll_ = nullptr;
  wxPanel *brand_panel_ = nullptr, *navigation_divider_ = nullptr, *rail_header_ = nullptr;
  XNavButton *vessel_profile_ = nullptr;
  XNavButton *rail_configure_ = nullptr;
  int responsive_class_ = -1, responsive_dpi_ = -1;
  XNavHorizon *horizon_ = nullptr;
  XNavStatusFooter *footer_ = nullptr;
  wxPanel *route_actions_ = nullptr;
  XNavFloatingSurface *chart_tools_ = nullptr, *chart_orientation_ = nullptr, *chart_follow_ = nullptr;
  std::vector<wxWindow *> chart_overlays_;
  XNavButton *page_up_ = nullptr, *page_down_ = nullptr;
  XNavButton *rail_up_ = nullptr, *rail_down_ = nullptr;
  wxPanel *rail_actions_ = nullptr;
  bool caption_themed_ = false;
  std::vector<std::string> rail_keys_;
  std::vector<std::pair<std::string, XNavDataValue *>> rail_values_;
  wxStaticText *clock_, *route_summary_;
  XNavButton *source_;
};

} // namespace opennav::ui

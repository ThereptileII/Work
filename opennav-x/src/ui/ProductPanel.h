#pragma once
#include "adapters/Autopilot.h"
#include "adapters/Radar.h"
#include "application/NavigationObjects.h"
#include "application/Settings.h"
#include "application/Alerts.h"
#include "diagnostics/Commissioning.h"
#include "diagnostics/FieldReport.h"
#include "smartnav/Advisories.h"
#include "ui/Controls.h"
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
namespace opennav::ui {
enum class ProductPage {
  Home,
  Routes,
  Waypoints,
  RouteDetail,
  WaypointDetail,
  Ais,
  AisDetail,
  Instruments,
  Advice,
  Pilot,
  Anchor,
  Settings,
  EnergySettings,
  Sources,
  SourceDetail,
  VesselSettings,
  Radar,
  Display,
  RailLayout,
  InstrumentLayout,
  Commissioning,
  PilotSettings,
  FieldReport,
  Alerts
};
struct ProductState {
  std::vector<application::Alert> alerts;
  vessel::VesselState vessel;
  vessel::AisState ais;
  application::AnchorState anchor;
  adapters::PilotView pilot;
  std::vector<adapters::PilotCommand> pilot_log;
  std::vector<std::string> pilot_sources;
  smartnav::NavigationAdvice advice;
  application::Settings settings;
  std::string settings_status;
  std::string boat_bridge_status;
  std::vector<vessel::SourceHealth> sources;
  adapters::RadarState radar;
  vessel::Time now{};
};
struct ProductActions {
  std::function<void(const std::string &, std::uint64_t)> acknowledge_alert;
  std::function<std::vector<diagnostics::BundleEntry>(const std::optional<std::string> &)> field_bundle;
  std::shared_ptr<diagnostics::Commissioning> commissioning;
  application::NavigationActions navigation;
  std::function<void()> chart, route_summary, energy, diagnostics;
  std::function<void(LightMode)> theme;
  std::function<void(adapters::PilotAction, double)> pilot_command;
  std::function<void(bool)> pilot_enable;
  std::function<application::CommandResult()> pilot_identity;
  std::function<application::Settings()> settings;
  std::function<application::CommandResult(const application::Settings &)>
      save_settings;
};
class ProductPanel final : public XNavScroll {
public:
  ProductPanel(wxWindow *parent, ProductActions actions);
  void ShowPage(ProductPage page, LightMode mode);
  std::string PageTitle() const;
  int MinimumValueHeight() const;
  void ShowAis(int mmsi, LightMode mode);
  void ShowObject(const std::string &id, bool route, LightMode mode);
  void Update(const ProductState &state, LightMode mode);

private:
  void Build();
  void BeginActions(int columns);
  void EndActions() { actions_grid_ = nullptr; }
  void Heading(const wxString &title, const wxString &subtitle);
  void Text(const wxString &text, int size = 14);
  void LiveText(std::function<wxString(const ProductState &)> text);
  XNavButton *Action(const wxString &label, std::function<void()> action,
                     bool enabled = true);
  void Value(const wxString &title, const wxString &unit,
             std::function<vessel::Sample(const ProductState &)> value,
             int decimals = 1);
  void Result(application::CommandResult result);
  void RouteActions();
  void PointActions();
  void CreateMark();
  void PilotActions();
  void PilotSettings();
  void EnergySettings();
  void CommissioningPanel();
  void FieldReportPanel();
  void AlertsPanel();
  void ExportFieldReport(bool include_recording);
  void Sources();
  void SourceDetail();
  void DisplaySettings();
  void InstrumentSelection(bool rail);
  void SaveSettings(application::Settings settings);
  ProductActions actions_;
  ProductState state_;
  ProductPage page_ = ProductPage::Home;
  LightMode mode_ = LightMode::Day;
  application::Route route_;
  application::Waypoint point_;
  int mmsi_ = 0;
  vessel::Quantity source_quantity_ = vessel::Quantity::Depth;
  wxBoxSizer *body_ = nullptr;
  wxGridSizer *grid_ = nullptr;
  wxGridSizer *actions_grid_ = nullptr;
  wxStaticText *notice_ = nullptr;
  int layout_width_ = 0;
  std::vector<std::pair<wxStaticText *, wxString>> static_text_;
  std::vector<std::pair<wxGridSizer *, int>> action_grids_;
  std::vector<std::pair<XNavButton *, adapters::PilotAction>> pilot_buttons_;
  std::vector<
      std::pair<wxStaticText *, std::function<wxString(const ProductState &)>>>
      text_;
  std::vector<std::pair<XNavDataValue *,
                        std::function<vessel::Sample(const ProductState &)>>>
      values_;
};
} // namespace opennav::ui

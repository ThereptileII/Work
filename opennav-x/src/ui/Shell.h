#pragma once

#include "ui/Controls.h"
#include "ui/PreviewPanel.h"
#include "ui/ProductPanel.h"
#include "vessel/DemoSource.h"

#include <wx/aui/aui.h>
#include <wx/frame.h>
#include <wx/stattext.h>
#include <wx/timer.h>

#include <functional>
#include <vector>

namespace opennav::ui {

struct ShellActions {
  // Integration supplies AUI identities, never chart objects. Page visibility
  // is restored before OpenCPN saves its normal perspective on close.
  std::vector<wxString> navigation_panes;
  std::function<void()> zoom_in, zoom_out, follow, legacy;
  application::NavigationActions navigation;
  std::function<bool()> route_creating;
  std::function<adapters::PilotView(bool)> pilot_tick;
  std::function<std::vector<adapters::PilotCommand>(bool)> pilot_log;
  std::function<void(bool, adapters::PilotAction, double)> pilot_command;
  std::function<void(bool, bool)> pilot_enable;
  std::function<void()> restart_xnav, safe, diagnostics_folder;
  std::function<vessel::RouteProgress()> route;
  std::function<vessel::VesselState()> live_state;
  std::function<application::Settings()> settings;
  std::function<std::string()> settings_status;
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
  void UpdateState(const vessel::VesselState &state);
  void ShowAis(int mmsi);
  void ShowObject(const std::string &id, bool route);

private:
  wxPanel *MakePane(const wxString &name, wxAuiPaneInfo placement);
  XNavButton *Button(wxWindow *parent, const wxString &text,
                     const wxString &name, std::function<void()> callback);
  wxStaticText *Text(wxWindow *parent, const wxString &text, int size,
                     bool bold = false);
  void ApplyTheme();
  void Tick();
  std::string PageTitle() const;
  void ShowSystem();
  void ShowProduct(ProductPage page);
  void ShowDemo();
  void ShowPage(PreviewPage page);
  void ShowNavigation();
  void OnCommand(wxCommandEvent &event);
  void StartDemo();
  void SelectDemo(vessel::DemoScenario scenario);
  wxString InputSummary() const;
  UpdateMetrics metrics_;
  wxFrame &frame_;
  wxAuiManager &manager_;
  ShellActions actions_;
  LightMode mode_;
  vessel::VesselState state_;
  bool simulation_ = false;
  bool simulation_paused_ = false;
  vessel::DemoSource demo_{vessel::Clock::now()};
  PreviewPanel *page_ = nullptr;
  ProductPanel *product_ = nullptr;
  XNavButton *finish_route_ = nullptr;
  PreviewPage current_page_ = PreviewPage::Route;
  std::vector<std::pair<wxString, bool>> navigation_visibility_;
  wxTimer timer_;
  std::vector<wxPanel *> panes_;
  std::vector<XNavButton *> buttons_;
  std::vector<std::pair<int, std::function<void()>>> commands_;
  std::vector<wxStaticText *> labels_;
  XNavDataValue *wind_, *depth_, *speed_, *course_, *heading_;
  wxStaticText *clock_, *source_, *route_summary_;
};

} // namespace opennav::ui

#pragma once
#include "adapters/Autopilot.h"
#include "application/NavigationObjects.h"
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
  Settings
};
struct ProductState {
  vessel::VesselState vessel;
  vessel::AisState ais;
  application::AnchorState anchor;
  adapters::PilotView pilot;
  std::vector<adapters::PilotCommand> pilot_log;
  smartnav::NavigationAdvice advice;
  vessel::Time now{};
};
struct ProductActions {
  application::NavigationActions navigation;
  std::function<void()> chart, route_summary, energy, diagnostics;
  std::function<void(adapters::PilotAction, double)> pilot_command;
  std::function<void(bool)> pilot_enable;
};
class ProductPanel final : public wxScrolledWindow {
public:
  ProductPanel(wxWindow *parent, ProductActions actions);
  void ShowPage(ProductPage page, LightMode mode);
  std::string PageTitle() const;
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
  ProductActions actions_;
  ProductState state_;
  ProductPage page_ = ProductPage::Home;
  LightMode mode_ = LightMode::Day;
  application::Route route_;
  application::Waypoint point_;
  int mmsi_ = 0;
  wxBoxSizer *body_ = nullptr;
  wxGridSizer *grid_ = nullptr;
  wxGridSizer *actions_grid_ = nullptr;
  wxStaticText *notice_ = nullptr;
  std::vector<
      std::pair<wxStaticText *, std::function<wxString(const ProductState &)>>>
      text_;
  std::vector<std::pair<XNavDataValue *,
                        std::function<vessel::Sample(const ProductState &)>>>
      values_;
};
} // namespace opennav::ui

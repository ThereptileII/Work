#pragma once
#include "application/NavigationObjects.h"
#include "ui/Controls.h"
#include <functional>
#include <wx/dialog.h>
#include <wx/eventfilter.h>

namespace opennav::ui {
enum class ContextKind { Waypoint, Ais, ChartPosition };
enum class ContextAction { Details, ShowAis, GoTo, Edit, Remove, CreateWaypoint, Measure, Info };
// A modeless owned window, bounded to the chart. It never grabs the pointer or
// disables global alerts/manual controls. State contains no OpenCPN pointers.
class XNavContextCard final : public wxDialog, public wxEventFilter {
public:
  using Action = std::function<void(ContextAction, std::optional<application::Waypoint>)>;
  XNavContextCard(wxWindow &owner, ContextKind kind, Action action);
  ~XNavContextCard() override;
  void UpdateAis(std::optional<vessel::AisTarget> target, vessel::Time now,
                 bool live, LightMode mode);
  void UpdateWaypoint(application::WaypointContext point, vessel::Time now,
                      bool live, LightMode mode);
  void UpdateChartPosition(application::Coordinate position, LightMode mode,
                           bool live = false, bool position_current = false);
  bool Place(const wxRect &chart);
  void Dismiss();
  int FilterEvent(wxEvent &event) override;
private:
  void Paint(wxPaintEvent &event);
  void ThemeControls(LightMode mode);
  ContextKind kind_;
  bool closing_ = false, filter_added_ = false;
  application::Coordinate position_{};
  Action action_;
  LightMode mode_ = LightMode::Day;
  vessel::Time now_{};
  std::optional<vessel::AisTarget> target_;
  application::WaypointContext point_;
  wxPanel *content_ = nullptr;
  std::vector<std::pair<XNavButton *, ContextAction>> actions_;
  XNavIconButton *close_ = nullptr;
};

// Shared mutation sheets used by the compact card and full waypoint Details.
// Cancellation returns no result and performs no mutation.
std::optional<application::CommandResult> WaypointSheet(
    wxWindow &parent, LightMode mode, ContextAction action,
    const application::Waypoint &point,
    const application::NavigationActions &navigation);
} // namespace opennav::ui

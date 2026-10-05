#pragma once
#include "application/NavigationObjects.h"
#include "vessel/RouteProgress.h"

class MyFrame;
namespace opennav::integration {
application::ChartPresentationState CopyChartPresentation(MyFrame &frame);
application::ChartPresentationResult SetChartAis(MyFrame &frame, bool show);
application::ChartPresentationResult SetChartEncText(MyFrame &frame, bool show);
application::ChartPresentationResult SetChartSoundings(MyFrame &frame, bool show);
application::ChartPresentationResult SetChartOrientation(
    MyFrame &frame, application::ChartOrientation orientation);
application::Catalog CopyNavigationCatalog();
application::NavigationNameSuggestion CopyNavigationNameSuggestion(
    MyFrame &frame, application::Coordinate position, bool route);
std::optional<application::Route> CopyNavigationRoute(const std::string &id);
application::WaypointContext CopyWaypointContext(
    const std::string &id, const vessel::Navigation &position, vessel::Time now);
vessel::AisState CopyAisState(const vessel::Navigation &selected,
                              vessel::Time now);
application::CommandResult ActivateRoute(const application::Route &selected,
                                         const vessel::Navigation &position);
application::AnchorWatchSelection CopyAnchorWatchSelection();
application::CommandResult ActivateRouteAfterAnchor(
    const application::Route &selected, const vessel::Navigation &position,
    const application::AnchorWatchSelection &confirmed);
application::CommandResult StopRoute(const application::Route &selected);
application::CommandResult ReverseRoute(const application::Route &selected);
application::CommandResult EditRoute(const application::Route &selected,
                                     const std::string &name,
                                     const std::string &description);
application::CommandResult EditWaypoint(const application::Waypoint &selected,
                                        const std::string &name,
                                        const std::string &description);
application::CommandResult
DeleteWaypoint(const application::Waypoint &selected);
application::CommandResult CreateWaypoint(application::Coordinate position,
                                          const std::string &name,
                                          const std::string &description);
application::CommandResult GoTo(application::Coordinate destination,
                                 const std::string &name,
                                 const vessel::Navigation &position);
application::CommandResult GoToWaypoint(const application::Waypoint &selected,
                                         const vessel::Navigation &position);
// Read-only observation after normal ProcessAnchorWatch or on a UI tick.
// Copies the last upstream alarm; never calls ProcessAnchorWatch. The UI
// supplies its tick time so the observation and presentation share one clock.
application::AnchorState ObserveAnchor(const vessel::Navigation &position,
                                       vessel::Time now);
application::CommandResult StartAnchor(const vessel::Navigation &position,
                                       double radius_m);
application::CommandResult ClearAnchor(const std::string &id);
} // namespace opennav::integration

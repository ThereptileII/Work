#pragma once
#include "application/NavigationObjects.h"
#include "vessel/RouteProgress.h"

namespace opennav::integration {
application::Catalog CopyNavigationCatalog();
vessel::AisState CopyAisState(const vessel::Navigation &selected,
                              vessel::Time now);
application::CommandResult ActivateRoute(const application::Route &selected,
                                         const vessel::Navigation &position);
application::CommandResult StopRoute();
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
// Called after OpenCPN's normal ProcessAnchorWatch, not from a getter.
application::AnchorState ObserveAnchor(const vessel::Navigation &position,
                                       vessel::Time now);
application::CommandResult StartAnchor(const vessel::Navigation &position,
                                       double radius_m);
application::CommandResult ClearAnchor(const std::string &id);
} // namespace opennav::integration

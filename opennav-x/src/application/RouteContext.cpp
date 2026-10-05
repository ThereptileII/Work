#include "application/RouteContext.h"
#include <cmath>

namespace opennav::application {
RouteContextView PresentRouteContext(const std::string &selected_id,
                                    const std::optional<Route> &route) {
  RouteContextView view;
  view.id = selected_id;
  view.name = "Route unavailable";
  view.status = "Removed or no longer identifiable";
  if (selected_id.empty() || !route || route->id != selected_id) return view;
  view.available = true;
  view.active = route->active;
  view.name = route->name.empty() ? "Unnamed route" : route->name;
  view.status = route->active ? "ACTIVE ROUTE" : "SAVED ROUTE";
  if (!route->visible) view.status += " / HIDDEN";
  if (!route->editable) view.status += " / READ ONLY";
  view.points = route->points.size();
  if (!route->points.empty()) {
    const auto &first = route->points.front();
    const auto &last = route->points.back();
    view.departure = first.name.empty() ? "Unnamed waypoint" : first.name;
    view.destination = last.name.empty() ? "Unnamed waypoint" : last.name;
    view.can_view = std::isfinite(first.latitude_deg) &&
        std::isfinite(first.longitude_deg) && std::abs(first.latitude_deg) <= 90 &&
        std::abs(first.longitude_deg) <= 180;
  }
  return view;
}
} // namespace opennav::application

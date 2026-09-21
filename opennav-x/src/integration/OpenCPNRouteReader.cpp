#include "integration/OpenCPNRouteReader.h"
#include "model/route.h"
#include "model/route_point.h"
#include "model/routeman.h"
#include "model/own_ship.h"
#include <wx/thread.h>
#include <stdexcept>

extern bool bGPSValid;

namespace opennav::integration {
RouteCopy CopyActiveRoute(Routeman* manager) {
  if (!wxIsMainThread()) throw std::logic_error("Route observation requires the application thread");
  RouteCopy r;
  if (!manager) return r;
  auto* route = manager->GetpActiveRoute();
  if (!route) return r;
  r.active = true;
  // A removed route must not be dereferenced even if a retained pointer exists.
  r.registered = pRouteList && manager->IsRouteValid(route);
  if (!r.registered) return r;
  r.id = route->GetGUID().ToStdString(wxConvUTF8);
  auto* active = manager->GetpActivePoint();
  r.active_point_consistent = active && active == route->m_pRouteActivePoint;
  std::size_t matches = 0;
  for (auto* node = route->pRoutePointList->GetFirst(); node; node = node->GetNext()) {
    const auto* point = node->GetData();
    if (!point) { r.registered = false; return r; }
    const auto id = point->m_GUID.ToStdString(wxConvUTF8);
    if (point == active) {
      ++matches; r.active_index = r.points.size(); r.active_point_id = id;
    }
    // The first point has no incoming route leg. Do not expose an irrelevant
    // retained segment value left on it by a reversed/shared route.
    r.points.push_back({id, point->m_lat, point->m_lon,
                       r.points.empty() ? 0.0 : point->m_seg_len});
  }
  r.active_point_consistent = r.active_point_consistent && matches == 1;
  return r;
}

RouteRead ReadRouteProgress(const vessel::Navigation& position) {
  RouteRead r;
  r.route = CopyActiveRoute(g_pRouteMan);
  r.position = position;
  r.upstream_position_valid = bGPSValid;
  r.upstream_latitude_deg = gLat; r.upstream_longitude_deg = gLon;
  if (r.route.active && r.route.registered && r.route.active_point_consistent &&
      g_pRouteMan->m_bDataValid)
    r.range_to_active_nm = g_pRouteMan->GetCurrentRngToActivePoint();
  return r;
}
}  // namespace opennav::integration

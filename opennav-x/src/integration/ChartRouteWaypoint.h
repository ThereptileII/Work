#pragma once
#include <wx/gdicmn.h>
class ChartCanvas;
class RoutePoint;
class ocpnDC;
namespace opennav::integration {
// A copied ordinal is valid for this application-thread paint only.
int ChartRouteWaypointOrdinal(ChartCanvas &canvas, RoutePoint &point, bool pinned_icon);
// Label-only eligibility may include the actual active point; its icon remains stock.
int ChartRouteLabelOrdinal(ChartCanvas &canvas, RoutePoint &point, bool pinned_icon);
int ChartRouteWaypointExtent(ChartCanvas &canvas);
// Plain numbered route marker (ordinal 1..99).
bool DrawChartRouteWaypoint(ocpnDC &dc, ChartCanvas &canvas,
                            int x, int y, int ordinal);

// XNav waypoint marker for one application-thread paint. 0 keeps the stock
// icon (Standard/Legacy/Safe, MOB, anchor watch, layers, meaningful icons,
// custom-styled routes, active drag handle).
enum class WaypointMarkerKind { RoutePoint = 1, Active = 2, Visited = 3, Standalone = 4 };
// Same route-state ink as the route line: active/default, inactive, selected.
enum class WaypointMarkerRole { Route = 0, Inactive = 1, SelectedRoute = 2 };
struct DecodedWaypointMarker {
  bool valid = false;
  WaypointMarkerKind kind = WaypointMarkerKind::RoutePoint;
  WaypointMarkerRole role = WaypointMarkerRole::Route;
  int ordinal = 0;  // 0 = unnumbered (shared, repeated or >99 points)
  bool selected = false;
};
constexpr int EncodeWaypointMarker(WaypointMarkerKind kind, WaypointMarkerRole role,
                                   int ordinal, bool selected) {
  return ordinal < 0 || ordinal > 99 ? 0
      : static_cast<int>(kind) * 10000 + static_cast<int>(role) * 1000 +
        (selected ? 100 : 0) + ordinal;
}
constexpr DecodedWaypointMarker DecodeWaypointMarker(int marker) {
  DecodedWaypointMarker d;
  const int kind = marker / 10000, role = marker / 1000 % 10,
            selected = marker / 100 % 10, ordinal = marker % 100;
  if (marker <= 0 || kind < 1 || kind > 4 || role > 2 || selected > 1) return d;
  d.valid = true;
  d.kind = static_cast<WaypointMarkerKind>(kind);
  d.role = static_cast<WaypointMarkerRole>(role);
  d.ordinal = d.kind == WaypointMarkerKind::Standalone ? 0 : ordinal;
  d.selected = selected == 1;
  return d;
}
int ChartWaypointMarker(ChartCanvas &canvas, RoutePoint &point, bool pinned_icon);
// Bounds relative to the point, including an owned standalone name label.
wxRect ChartWaypointMarkerBounds(ChartCanvas &canvas, RoutePoint &point, int marker);
// Standalone markers draw their own XNav name label; skip the stock name.
bool ChartWaypointMarkerOwnsName(int marker);
bool DrawChartWaypointMarker(ocpnDC &dc, ChartCanvas &canvas, RoutePoint *point,
                             int x, int y, int marker);
} // namespace opennav::integration

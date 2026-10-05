#pragma once
class ChartCanvas;
class RoutePoint;
class ocpnDC;
namespace opennav::integration {
// A copied ordinal is valid for this application-thread paint only.
int ChartRouteWaypointOrdinal(ChartCanvas &canvas, RoutePoint &point, bool pinned_icon);
// Label-only eligibility may include the actual active point; its icon remains stock.
int ChartRouteLabelOrdinal(ChartCanvas &canvas, RoutePoint &point, bool pinned_icon);
int ChartRouteWaypointExtent(ChartCanvas &canvas);
bool DrawChartRouteWaypoint(ocpnDC &dc, ChartCanvas &canvas,
                            int x, int y, int ordinal);
} // namespace opennav::integration

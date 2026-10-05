#pragma once
class ChartCanvas;
class RoutePoint;
class ocpnDC;
namespace opennav::integration {
bool PrepareChartRouteLabel(ChartCanvas& canvas, RoutePoint& point, int ordinal);
bool DrawChartRouteLabel(ocpnDC& dc, ChartCanvas& canvas, RoutePoint& point,
                         int x, int y, bool prepared);
} // namespace opennav::integration

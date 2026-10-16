#pragma once

class ocpnDC;
class ChartCanvas;
class ViewPort;

namespace opennav::integration {
// Application-thread, paint-only overlay for the actual active route. Called
// before route lines/markers in both backends; no navigation or control output.
void DrawActiveRoutePassage(ocpnDC& dc, ChartCanvas& canvas, ViewPort& viewport);
} // namespace opennav::integration

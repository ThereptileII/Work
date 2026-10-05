#pragma once
#include "model/route_point.h"
#include <wx/gdicmn.h>
class ocpnDC;
enum { GLOBAL_COLOR_SCHEME_DAY, GLOBAL_COLOR_SCHEME_DUSK, GLOBAL_COLOR_SCHEME_NIGHT };
// Observation boundary only: no navigation or alarm decisions in this fixture.
class ChartCanvas {
public:
  int GetColorScheme() const { return scheme; }
  int FromDIP(int value) const { return value * density; }
  double GetAnchorWatchRadiusPixels(RoutePoint *point) { return point->radius; }
  void GetCanvasPointPix(double latitude, double longitude, wxPoint *result) {
    *result = wxPoint(static_cast<int>(longitude), static_cast<int>(latitude));
  }
  void DrawAnchorWatchPoints(ocpnDC &);
  void DrawOriginalAnchorWatchPoints(ocpnDC &);
  int scheme = GLOBAL_COLOR_SCHEME_DAY, density = 1;
};

#pragma once
#include "ui/Theme.h"
class ChartCanvas;
class RoutePoint;
class ocpnDC;
class wxColour;
namespace opennav::integration {
// Presentation of upstream watch state, never an independent alarm decision.
inline std::uint32_t AnchorWatchInk(ui::LightMode mode, bool alarm,
                                     bool position_valid, bool entry_watch) {
  const auto c=ui::Theme(mode);
  return alarm ? c.alarm : !position_valid ? c.attention
       : entry_watch ? c.alarm : ui::ActiveRouteInk(mode);
}
bool ChartAnchorWatchInk(ChartCanvas &, bool alarm, bool position_valid,
                         bool entry_watch, wxColour &ink);
int ChartAnchorWatchExtent(ChartCanvas &, RoutePoint &, bool pinned_icon = false);
bool DrawChartAnchorMark(ocpnDC &, ChartCanvas &, RoutePoint &, int x, int y, bool pinned_icon = false);
}

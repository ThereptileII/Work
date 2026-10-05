#pragma once
#include "model/waypointman.h"
class WayPointmanGui {
public:
  explicit WayPointmanGui(WayPointman &manager) : m_waypoint_man(manager) {}
  bool IsPinnedAnchor(const wxString &,const wxBitmap *) const;
private: WayPointman &m_waypoint_man;
};

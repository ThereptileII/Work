#pragma once
#include "application/NavigationObjects.h"

namespace opennav::application {
struct AnchorView {
  bool active=false, alarm=false, can_start=false;
  bool inner_alarm=false;
  std::string identity, reason;
  std::optional<double> distance_m, radius_m, gps_age_s, history_minutes;
  std::optional<double> display_distance;
  std::string distance_unit;
  int distance_decimals = 0;
  std::optional<AnchorFix> vessel_position;
  std::vector<AnchorFix> history;
  vessel::Assessment depth, wind, battery;
};
AnchorView PresentAnchor(const AnchorState &, const vessel::VesselState &,
                         vessel::Time now);
// Bounded owned history: a changed/deleted/moved mark starts a new history.
// Observation reads neither duplicate points nor make their ages fresh.
void RetainAnchorHistory(AnchorState &current, const AnchorState &previous);
}

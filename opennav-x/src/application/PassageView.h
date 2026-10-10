#pragma once
#include "smartnav/Advisories.h"

namespace opennav::application {
struct PassagePointView {
  std::string id, name;
  std::size_t ordinal = 0;
  std::optional<double> distance_nm, seconds, turn_deg, course_true_deg;
};
struct PassageView {
  std::string route_id, destination, reason;
  bool active = false, current = false, below_reserve = false;
  std::size_t waypoint_count = 0;
  std::optional<double> distance_nm, seconds, arrival_soc;
  std::vector<PassagePointView> points;
};
// Read-only presentation of the current observation batch. Distances and times
// come from the accepted route/SmartNav contracts, never chart geometry here.
PassageView PresentPassage(const vessel::VesselState &state,
                           const smartnav::NavigationAdvice &advice,
                           const smartnav::EnergyPrediction &energy,
                           vessel::Time now);
// SCRUM-362: the prototype's next-turn card ("NEXT · LÅNGHOLMEN / Starboard
// 32° / 0.7 nm · in 7 min · new course 075°"), derived only from a current
// passage view. Not visible without a current active route.
struct NextTurnView {
  bool visible = false;
  std::string eyebrow, headline, detail;  // UTF-8.
  std::optional<int> turn_deg;            // Absolute, with headline side.
  int direction = 0;                      // -1 port, +1 starboard, 0 none.
};
NextTurnView PresentNextTurn(const PassageView &passage);
} // namespace opennav::application

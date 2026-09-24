#pragma once
#include "vessel/VesselState.h"
#include <vector>

namespace opennav::vessel {
// Copies of OpenCPN's AIS result, not a second CPA/TCPA calculator.
struct AisTarget {
  int mmsi = 0;
  std::string name, status, source;
  bool active = false, lost = false, doubtful = false, upstream_alarm = false;
  Sample latitude_deg, longitude_deg, sog_kn, cog_deg, heading_true_deg;
  Sample range_nm, bearing_true_deg, cpa_nm, tcpa_minutes;
  Time observed_at{};
};
struct AisState {
  std::vector<AisTarget> targets;
  std::string source;
  Time observed_at{};
  bool available = false, simulated = false;
};
} // namespace opennav::vessel

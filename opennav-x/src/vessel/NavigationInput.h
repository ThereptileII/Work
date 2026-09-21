#pragma once

#include "vessel/VesselState.h"

namespace opennav::vessel {

// A delta from an already selected navigation source, not a full fresh snapshot.
// Timestamps describe receipt by that source; omitted fields keep their age.
struct NavigationUpdate {
  std::string source;
  std::optional<Time> observed_at;
  bool position_updated = false, position_valid = false;
  bool sog_updated = false, cog_updated = false;
  std::optional<double> latitude_deg, longitude_deg, sog_kn, cog_deg;
};

class NavigationInput {
 public:
  void Apply(const NavigationUpdate& update);
  const VesselState& State() const { return state_; }
 private:
  VesselState state_;
};

}  // namespace opennav::vessel

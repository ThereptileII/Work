#pragma once
#include "smartnav/Energy.h"
#include "vessel/AisState.h"
#include "vessel/RouteProgress.h"
#include <vector>

namespace opennav::smartnav {
enum class EventKind {
  Waypoint,
  Turn,
  Destination,
  ArrivalSoc,
  Reserve,
  EnergyShortfall,
  AisEncounter
};
enum class Severity { Information, Caution, Warning };
struct AdvisoryEvent {
  EventKind kind = EventKind::Waypoint;
  Severity severity = Severity::Information;
  std::string identity, title, detail, source;
  std::optional<double> seconds_from_now, distance_nm, course_true_deg,
      course_change_deg;
  vessel::Time observed_at{};
};
struct NavigationAdvice {
  std::vector<AdvisoryEvent> events;
  std::string route_id, revision_scope, reason;
  std::uint64_t route_revision = 0;
  vessel::Time calculated_at{};
  bool route_valid = false;
};
// Advisory value output only. No dependency on adapters or command interfaces.
NavigationAdvice Advise(const vessel::VesselState &state,
                        const EnergyPrediction &energy,
                        const vessel::AisState &ais, vessel::Time now);
} // namespace opennav::smartnav

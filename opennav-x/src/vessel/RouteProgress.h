#pragma once

#include "vessel/VesselState.h"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace opennav::vessel {

enum class RouteState {
  Valid, NoActiveRoute, InvalidRoute, InvalidActivePoint, AmbiguousPoint,
  RouteChanged, RouteEditing, ActivePointChanged, PositionChanged, InvalidLeg, InvalidRange,
  ArithmeticLimit, MissingPosition, StalePosition, UncertainPosition,
  PositionMismatch, OutOfOrder, InterruptedPass, AwaitingProgress
};
const char* RouteStateName(RouteState state);

// Owned values only. Published as shared_ptr<const ...>; safe to retain after
// OpenCPN edits/deletes a route. This is evidence as of observation, not a live
// route handle. Consumers must use the current publication and assess its age.
struct RouteStep {
  std::string waypoint_id, name;
  double latitude_deg = 0, longitude_deg = 0;
  // First step uses normal progress range; later steps use stored route legs.
  double distance_from_previous_nm = 0;
  std::optional<double> course_true_deg;
};
struct RouteProgressSnapshot {
  std::string route_id;
  std::string revision_scope;  // process-local revision namespace, not persistent
  std::uint64_t route_revision = 0;
  std::string active_waypoint_id;
  std::optional<std::size_t> active_waypoint_index;  // zero based
  std::size_t waypoint_count = 0;
  std::optional<double> remaining_distance_nm;  // nautical miles, never default 0
  Time observed_at{};  // completed upstream progress observation
  std::optional<Time> position_observed_at;
  RouteState state = RouteState::AwaitingProgress;
  std::string source;
  std::string position_source;
  std::string route_name;
  std::vector<RouteStep> remaining_steps;
};
using RouteProgress = std::shared_ptr<const RouteProgressSnapshot>;

struct RouteAssessment {
  RouteState state = RouteState::AwaitingProgress;
  Quality quality = Quality::Unavailable;
  std::optional<double> remaining_distance_nm;
  std::optional<Duration> observation_age, position_age;
};
// Reading/assessing never renews either timestamp. Stale distances are withheld
// from consumers even though the immutable historical snapshot can be retained.
RouteAssessment AssessRoute(const RouteProgressSnapshot& snapshot, Time now,
                            Freshness freshness = {});
Sample RouteDistanceSample(const RouteProgressSnapshot& snapshot, Time now,
                           Freshness freshness = {});

}  // namespace opennav::vessel

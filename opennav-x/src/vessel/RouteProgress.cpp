#include "vessel/RouteProgress.h"
#include <algorithm>
#include <cmath>

namespace opennav::vessel {
const char* RouteStateName(RouteState state) {
  switch (state) {
#define STATE(x) case RouteState::x: return #x
    STATE(Valid); STATE(NoActiveRoute); STATE(InvalidRoute);
    STATE(InvalidActivePoint); STATE(AmbiguousPoint); STATE(RouteChanged);
    STATE(ActivePointChanged); STATE(PositionChanged); STATE(InvalidLeg);
    STATE(InvalidRange); STATE(ArithmeticLimit); STATE(MissingPosition);
    STATE(StalePosition); STATE(UncertainPosition); STATE(PositionMismatch);
    STATE(OutOfOrder); STATE(InterruptedPass); STATE(AwaitingProgress);
#undef STATE
  }
  return "Unknown";
}

RouteAssessment AssessRoute(const RouteProgressSnapshot& s, Time now, Freshness f) {
  RouteAssessment a;
  a.state = s.state;
  if (s.observed_at <= now) a.observation_age =
      std::chrono::duration_cast<Duration>(now - s.observed_at);
  if (s.position_observed_at && *s.position_observed_at <= now) a.position_age =
      std::chrono::duration_cast<Duration>(now - *s.position_observed_at);
  if (s.state == RouteState::StalePosition) a.quality = Quality::Stale;
  if (s.state == RouteState::UncertainPosition) a.quality = Quality::Uncertain;
  if (s.state != RouteState::Valid) return a;
  if (s.position_observed_at && *s.position_observed_at > s.observed_at) {
    a.state = RouteState::UncertainPosition; a.quality = Quality::Uncertain;
    return a;
  }
  if (s.source.empty() || s.position_source.empty() || s.route_id.empty() ||
      s.revision_scope.empty() || !s.route_revision || s.active_waypoint_id.empty() ||
      !s.active_waypoint_index || *s.active_waypoint_index >= s.waypoint_count ||
      !s.position_observed_at || !s.remaining_distance_nm ||
      !std::isfinite(*s.remaining_distance_nm) || *s.remaining_distance_nm < 0) {
    a.state = RouteState::InvalidRoute;
    return a;
  }
  const auto p = Assess({s.remaining_distance_nm, s.source, *s.position_observed_at,
                         Validity::Measured}, now, f);
  const auto o = Assess({s.remaining_distance_nm, s.source, s.observed_at,
                         Validity::Measured}, now, f);
  if (p.quality == Quality::Uncertain || o.quality == Quality::Uncertain) {
    a.state = RouteState::UncertainPosition;
    a.quality = Quality::Uncertain;
  } else if (p.quality == Quality::Stale || o.quality == Quality::Stale) {
    a.state = RouteState::StalePosition;
    a.quality = Quality::Stale;
  } else {
    a.quality = p.quality == Quality::Aging || o.quality == Quality::Aging
                    ? Quality::Aging : Quality::Live;
    a.remaining_distance_nm = s.remaining_distance_nm;
  }
  return a;
}

Sample RouteDistanceSample(const RouteProgressSnapshot& s, Time now, Freshness f) {
  const auto a = AssessRoute(s, now, f);
  return {a.remaining_distance_nm, s.source,
          s.position_observed_at ? std::min(s.observed_at, *s.position_observed_at) : s.observed_at,
          a.remaining_distance_nm ? Validity::Estimated : Validity::Invalid};
}
}  // namespace opennav::vessel

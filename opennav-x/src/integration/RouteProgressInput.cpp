#include "integration/RouteProgressInput.h"

#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace opennav::integration {
using namespace vessel;
namespace {
bool SameNumber(double a, double b) { return a == b || (std::isnan(a) && std::isnan(b)); }
bool SameCourse(const std::optional<double>& a, const std::optional<double>& b) {
  return a.has_value() == b.has_value() && (!a || SameNumber(*a, *b));
}
bool SameSample(const Sample& a, const Sample& b) {
  return a.value == b.value && a.observed_at == b.observed_at &&
         a.source == b.source && a.validity == b.validity;
}
RouteState CheckPosition(const RouteRead& r, Time now) {
  const auto& lat = r.position.latitude_deg;
  const auto& lon = r.position.longitude_deg;
  if (!lat.value || !lon.value || lat.source.empty() || lon.source != lat.source ||
      lat.observed_at != lon.observed_at) return RouteState::MissingPosition;
  if (!std::isfinite(*lat.value) || !std::isfinite(*lon.value) ||
      std::abs(*lat.value) > 90 || std::abs(*lon.value) > 180 ||
      lat.validity != Validity::Measured || lon.validity != Validity::Measured ||
      lat.observed_at > now) return RouteState::UncertainPosition;
  if (Assess(lat, now).quality == Quality::Stale) return RouteState::StalePosition;
  if (!r.upstream_position_valid || *lat.value != r.upstream_latitude_deg ||
      *lon.value != r.upstream_longitude_deg) return RouteState::PositionMismatch;
  return RouteState::Valid;
}
RouteState CheckRoute(const RouteCopy& r) {
  if (!r.active) return RouteState::NoActiveRoute;
  if (!r.registered || r.id.empty() || r.points.empty()) return RouteState::InvalidRoute;
  if (r.editing) return RouteState::RouteEditing;
  if (!r.active_point_consistent || !r.active_index || *r.active_index >= r.points.size() ||
      r.active_point_id.empty() || r.points[*r.active_index].id != r.active_point_id)
    return RouteState::InvalidActivePoint;
  std::set<std::string> ids;
  for (std::size_t i = 0; i < r.points.size(); ++i) {
    const auto& p = r.points[i];
    if (p.id.empty() || !ids.insert(p.id).second) return RouteState::AmbiguousPoint;
    if (!std::isfinite(p.latitude_deg) || !std::isfinite(p.longitude_deg) ||
        std::abs(p.latitude_deg) > 90 || std::abs(p.longitude_deg) > 180)
      return RouteState::InvalidRoute;
    if (i && (!std::isfinite(p.incoming_leg_nm) || p.incoming_leg_nm < 0))
      return RouteState::InvalidLeg;
  }
  return RouteState::Valid;
}
}  // namespace

bool SameRoute(const RouteCopy& a, const RouteCopy& b) {
  if (a.active != b.active || a.registered != b.registered || a.editing != b.editing || a.id != b.id ||
      a.points.size() != b.points.size() || a.name != b.name) return false;
  for (std::size_t i = 0; i < a.points.size(); ++i) {
    const auto& x = a.points[i]; const auto& y = b.points[i];
    if (x.id != y.id || !SameNumber(x.latitude_deg, y.latitude_deg) ||
        !SameNumber(x.longitude_deg, y.longitude_deg) ||
        !SameNumber(x.incoming_leg_nm, y.incoming_leg_nm) || x.name != y.name ||
        !SameCourse(x.incoming_course_true_deg, y.incoming_course_true_deg)) return false;
  }
  return true;
}
bool SameActivePoint(const RouteCopy& a, const RouteCopy& b) {
  return a.active_point_id == b.active_point_id && a.active_index == b.active_index &&
         a.active_point_consistent == b.active_point_consistent;
}

RouteProgressInput::RouteProgressInput(std::string scope) : scope_(std::move(scope)) {
  current_ = std::make_shared<const RouteProgressSnapshot>();
}
RouteProgressSnapshot RouteProgressInput::Describe(const RouteRead& r, Time now) const {
  RouteProgressSnapshot s;
  s.route_id = r.route.id; s.revision_scope = scope_; s.route_revision = revision_;
  s.route_name = r.route.name;
  s.active_waypoint_id = r.route.active_point_id;
  s.active_waypoint_index = r.route.active_index; s.waypoint_count = r.route.points.size();
  s.observed_at = now;
  if (r.position.latitude_deg.value && r.position.longitude_deg.value)
    s.position_observed_at = r.position.latitude_deg.observed_at;
  s.position_source = r.position.latitude_deg.source;
  s.source = "OpenCPN 5.12.4 normal route progress: active range + subsequent stored legs (NM)";
  return s;
}
void RouteProgressInput::Publish(RouteProgressSnapshot result) {
  current_ = std::make_shared<const RouteProgressSnapshot>(std::move(result));
}

void RouteProgressInput::Complete(const RouteRead& before, const RouteRead& after, Time now) {
  const auto position_time = after.position.latitude_deg.observed_at;
  if ((observation_watermark_ && now < *observation_watermark_) ||
      (position_watermark_ && after.position.latitude_deg.value && position_time < *position_watermark_)) {
    auto rejected = *current_;
    rejected.state = RouteState::OutOfOrder; rejected.remaining_distance_nm.reset();
    rejected.remaining_steps.clear();
    Publish(std::move(rejected));
    return;
  }
  observation_watermark_ = now;
  if (after.position.latitude_deg.value && position_time <= now) position_watermark_ = position_time;
  const bool changed = last_route_ && !SameRoute(*last_route_, after.route);
  const bool advanced = last_route_ && !SameActivePoint(*last_route_, after.route);
  if (!last_route_ || changed) {
    if (revision_ == std::numeric_limits<std::uint64_t>::max()) {
      auto s = Describe(after, now); s.state = RouteState::ArithmeticLimit;
      Publish(std::move(s)); return;
    }
    ++revision_;
  }
  last_route_ = after.route;
  auto s = Describe(after, now);
  s.state = CheckRoute(after.route);
  if (s.state == RouteState::Valid) {
    if (after.interrupted) s.state = RouteState::InterruptedPass;
    else if (changed || !SameRoute(before.route, after.route)) s.state = RouteState::RouteChanged;
    else if (advanced || !SameActivePoint(before.route, after.route)) s.state = RouteState::ActivePointChanged;
    else if (!SameSample(before.position.latitude_deg, after.position.latitude_deg) ||
             !SameSample(before.position.longitude_deg, after.position.longitude_deg) ||
             before.upstream_position_valid != after.upstream_position_valid ||
             before.upstream_latitude_deg != after.upstream_latitude_deg ||
             before.upstream_longitude_deg != after.upstream_longitude_deg) s.state = RouteState::PositionChanged;
    else s.state = CheckPosition(after, now);
  }
  if (s.state == RouteState::Valid) {
    if (scope_.empty()) s.state = RouteState::InvalidRoute;
    else if (!after.range_to_active_nm || !std::isfinite(*after.range_to_active_nm) ||
             *after.range_to_active_nm < 0) s.state = RouteState::InvalidRange;
    else {
      // Mirror the upstream route-total traversal. The incoming segment of the
      // active waypoint is NOT included; no geometry is calculated here.
      double distance = *after.range_to_active_nm;
      for (std::size_t i = *after.route.active_index + 1; i < after.route.points.size(); ++i) {
        const double leg = after.route.points[i].incoming_leg_nm;
        if (leg > std::numeric_limits<double>::max() - distance) {
          s.state = RouteState::ArithmeticLimit; break;
        }
        distance += leg;
      }
      if (!std::isfinite(distance)) s.state = RouteState::ArithmeticLimit;
      if (s.state == RouteState::Valid) {
        s.remaining_distance_nm = distance;
        for (std::size_t i = *after.route.active_index; i < after.route.points.size(); ++i) {
          const auto& p = after.route.points[i];
          const bool first = i == *after.route.active_index;
          auto course = first ? after.bearing_to_active_true_deg : p.incoming_course_true_deg;
          if (course && (!std::isfinite(*course) || *course < 0 || *course > 360)) course.reset();
          if (course == 360) course = 0;
          s.remaining_steps.push_back({p.id, p.name, p.latitude_deg, p.longitude_deg,
              first ? *after.range_to_active_nm : p.incoming_leg_nm, course});
        }
      }
    }
  }
  Publish(std::move(s));
}

void RouteProgressInput::CheckCurrent(const RouteCopy& current, Time now) {
  if (!last_route_ || (SameRoute(*last_route_, current) && SameActivePoint(*last_route_, current))) return;
  // Do not acknowledge the new geometry here: the next completed normal pass
  // will assign its revision and validate it. This read can only invalidate.
  auto s = *current_;
  s.state = CheckRoute(current);
  if (s.state == RouteState::Valid) s.state = SameRoute(*last_route_, current)
      ? RouteState::ActivePointChanged : RouteState::RouteChanged;
  s.remaining_distance_nm.reset();
  s.remaining_steps.clear();
  (void)now;  // A consumer request cannot renew the observation timestamp.
  Publish(std::move(s));
}
}  // namespace opennav::integration

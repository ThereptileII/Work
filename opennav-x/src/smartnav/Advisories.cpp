#include "smartnav/Advisories.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>

namespace opennav::smartnav {
namespace {
std::string Rounded(double value, int decimals) {
  std::ostringstream s;
  s.imbue(std::locale::classic());
  s << std::fixed << std::setprecision(decimals) << value;
  return s.str();
}
bool Fresh(const vessel::Sample &s, vessel::Time now) {
  const auto q = vessel::Assess(s, now).quality;
  return q == vessel::Quality::Live || q == vessel::Quality::Aging ||
         q == vessel::Quality::Estimated;
}
bool Course(std::optional<double> n) {
  return n && std::isfinite(*n) && *n >= 0 && *n < 360;
}
double Turn(double next, double previous) {
  double angle = std::fmod(next - previous + 540.0, 360.0) - 180.0;
  return angle == -180 ? 180 : angle;
}
bool StepsValid(const vessel::RouteProgressSnapshot &route) {
  if (route.remaining_steps.empty() || route.remaining_steps.size() > 10000 ||
      route.remaining_steps.front().waypoint_id != route.active_waypoint_id ||
      route.remaining_steps.size() !=
          route.waypoint_count - *route.active_waypoint_index)
    return false;
  double total = 0;
  std::set<std::string> ids;
  for (const auto &step : route.remaining_steps) {
    if (step.waypoint_id.empty() || !ids.insert(step.waypoint_id).second ||
        !std::isfinite(step.distance_from_previous_nm) ||
        step.distance_from_previous_nm < 0)
      return false;
    total += step.distance_from_previous_nm;
  }
  return std::isfinite(total) &&
         std::abs(total - *route.remaining_distance_nm) <=
             1e-9 * std::max(1.0, total);
}
} // namespace
NavigationAdvice Advise(const vessel::VesselState &s,
                        const EnergyPrediction &energy,
                        const vessel::AisState &ais, vessel::Time now) {
  NavigationAdvice advice;
  advice.calculated_at = now;
  advice.reason = "No coherent active route";
  const auto &r = s.navigation.route;
  if (r && vessel::AssessRoute(*r, now).remaining_distance_nm &&
      StepsValid(*r)) {
    advice.route_valid = true;
    advice.route_id = r->route_id;
    advice.revision_scope = r->revision_scope;
    advice.route_revision = r->route_revision;
    const auto &speed = s.navigation.sog_kn;
    const bool moving =
        Fresh(speed, now) && *speed.value >= 0.5 && *speed.value <= 200;
    const bool course =
        Fresh(s.navigation.cog_deg, now) && Course(s.navigation.cog_deg.value);
    advice.reason =
        moving ? "Estimated at constant present SOG; human review required"
               : "Timing unavailable: fresh underway speed required";
    double distance = 0;
    for (std::size_t i = 0;
         i < r->remaining_steps.size() && advice.events.size() < 128; ++i) {
      const auto &step = r->remaining_steps[i];
      distance += step.distance_from_previous_nm;
      AdvisoryEvent event;
      event.identity = step.waypoint_id;
      event.kind = i + 1 == r->remaining_steps.size() ? EventKind::Destination
                                                      : EventKind::Waypoint;
      event.title = step.name.empty() ? step.waypoint_id : step.name;
      event.detail =
          event.kind == EventKind::Destination ? "Destination" : "Waypoint";
      event.distance_nm = distance;
      event.source = r->source;
      event.observed_at = std::min(r->observed_at, *r->position_observed_at);
      if (moving) {
        const double seconds = distance / *speed.value * 3600;
        if (std::isfinite(seconds))
          event.seconds_from_now = seconds;
      }
      advice.events.push_back(event);
      if (i + 1 < r->remaining_steps.size() && moving && course &&
          r->remaining_steps[i + 1].distance_from_previous_nm > 0 &&
          Course(r->remaining_steps[i + 1].course_true_deg)) {
        const auto incoming =
            i == 0 ? s.navigation.cog_deg.value : step.course_true_deg;
        if (Course(incoming)) {
          event.kind = EventKind::Turn;
          event.title = "Planned course change";
          event.course_true_deg = r->remaining_steps[i + 1].course_true_deg;
          event.course_change_deg = Turn(*event.course_true_deg, *incoming);
          event.detail =
              i == 0
                  ? "Relative to present COG; advisory only"
                  : "Between stored OpenCPN route-leg courses; advisory only";
          if (std::abs(*event.course_change_deg) >= 1.0)
            advice.events.push_back(std::move(event));
        }
      }
    }
  }
  // Predictions must have been calculated for this evaluation epoch. A cached
  // prediction cannot re-enter the event stream after its inputs expire.
  if (energy.calculated_at == now && energy.input_route == r &&
      advice.route_valid && energy.arrival.estimate) {
    const auto &a = *energy.arrival.estimate;
    AdvisoryEvent e;
    e.identity = advice.route_id;
    e.source = energy.model_source;
    e.observed_at = std::min(r->observed_at, *r->position_observed_at);
    e.kind = EventKind::ArrivalSoc;
    e.title = "Estimated destination SOC";
    e.detail = a.soc_percent
                   ? Rounded(*a.soc_percent, 1) + "% (constant conditions)"
                   : "Unavailable: energy exhausted before destination";
    if (std::isfinite(a.passage_hours))
      e.seconds_from_now = a.passage_hours * 3600;
    advice.events.push_back(e);
    if (a.energy_shortfall_kwh > 0) {
      e.kind = EventKind::EnergyShortfall;
      e.severity = Severity::Warning;
      e.title = "Insufficient route energy";
      e.detail =
          Rounded(a.energy_shortfall_kwh, 1) + " kWh estimated shortfall";
      advice.events.push_back(e);
    } else if (a.below_reserve) {
      e.kind = EventKind::Reserve;
      e.severity = Severity::Caution;
      e.title = "Destination below configured reserve";
      advice.events.push_back(e);
    }
    if (energy.range.estimate && a.below_reserve) {
      const auto &range = *energy.range.estimate;
      if (std::isfinite(range.endurance_hours)) {
        e.kind = EventKind::Reserve;
        e.severity = Severity::Caution;
        e.title = "Projected reserve threshold";
        e.detail = "Estimated at current consumption";
        e.seconds_from_now = range.endurance_hours * 3600;
        e.distance_nm = range.range_nm;
        advice.events.push_back(e);
      }
    }
  }
  if (ais.available && ais.observed_at <= now &&
      now - ais.observed_at < std::chrono::seconds(5)) {
    for (const auto &target : ais.targets) {
      if (advice.events.size() >= 160)
        break;
      if (!target.active || target.lost || target.doubtful ||
          !target.upstream_alarm || !Fresh(target.cpa_nm, now) ||
          !Fresh(target.tcpa_minutes, now) || *target.cpa_nm.value < 0 ||
          *target.tcpa_minutes.value < 0)
        continue;
      AdvisoryEvent e;
      e.kind = EventKind::AisEncounter;
      e.severity = Severity::Warning;
      e.identity = std::to_string(target.mmsi);
      e.title = target.name.empty() ? e.identity : target.name;
      e.detail = std::string(ais.simulated ? "DEMO encounter; CPA "
                                           : "OpenCPN AIS alarm; CPA ") +
                 Rounded(*target.cpa_nm.value, 2) + " NM";
      e.seconds_from_now = *target.tcpa_minutes.value * 60;
      e.source = target.source;
      e.observed_at =
          std::min(target.cpa_nm.observed_at, target.tcpa_minutes.observed_at);
      if (std::isfinite(*e.seconds_from_now))
        advice.events.push_back(std::move(e));
    }
  }
  std::stable_sort(advice.events.begin(), advice.events.end(),
                   [](const auto &a, const auto &b) {
                     if (!a.seconds_from_now)
                       return false;
                     return !b.seconds_from_now ||
                            *a.seconds_from_now < *b.seconds_from_now;
                   });
  return advice;
}
} // namespace opennav::smartnav

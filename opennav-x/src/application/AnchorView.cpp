#include "application/AnchorView.h"
#include <cmath>

namespace opennav::application {
namespace {
bool CoordinateValid(Coordinate p) {
  return std::isfinite(p.latitude_deg) && std::abs(p.latitude_deg) <= 90 &&
         std::isfinite(p.longitude_deg) && std::abs(p.longitude_deg) <= 180;
}
bool Current(const vessel::Sample &s, vessel::Time now) {
  const auto a = vessel::Assess(s, now);
  return s.validity == vessel::Validity::Measured &&
         (a.quality == vessel::Quality::Live ||
          a.quality == vessel::Quality::Aging);
}
bool PlotValid(const AnchorFix &p) {
  return CoordinateValid(p.position) && p.east_m && p.north_m &&
         std::isfinite(*p.east_m) && std::isfinite(*p.north_m) &&
         std::isfinite(std::hypot(*p.east_m, *p.north_m));
}
bool SameMark(const AnchorState &a, const AnchorState &b) {
  return !a.waypoint_id.empty() && a.waypoint_id == b.waypoint_id && a.anchor &&
         b.anchor && a.anchor->latitude_deg == b.anchor->latitude_deg &&
         a.anchor->longitude_deg == b.anchor->longitude_deg;
}
} // namespace
void RetainAnchorHistory(AnchorState &current, const AnchorState &previous) {
  current.recent_positions.clear();
  if (current.waypoint_id.empty() || !current.anchor ||
      !CoordinateValid(*current.anchor))
    return;
  if (SameMark(current, previous))
    current.recent_positions = previous.recent_positions;
  if (current.vessel_position && PlotValid(*current.vessel_position) &&
      current.vessel_position->observed_at <= current.observed_at &&
      (current.recent_positions.empty() ||
       current.vessel_position->observed_at >
           current.recent_positions.back().observed_at))
    current.recent_positions.push_back(*current.vessel_position);
  if (current.recent_positions.size() > 300)
    current.recent_positions.erase(current.recent_positions.begin(),
                                   current.recent_positions.end() - 300);
}
AnchorView PresentAnchor(const AnchorState &watch,
                         const vessel::VesselState &state, vessel::Time now) {
  AnchorView v;
  v.reason = "A current GPS position is required to set an anchor watch.";
  v.depth = vessel::Assess(state.environment.depth_below_transducer_m, now);
  v.wind = vessel::Assess(state.wind.true_speed_kn, now);
  v.battery = vessel::Assess(state.battery.soc_percent, now);
  const auto &lat = state.navigation.latitude_deg,
             &lon = state.navigation.longitude_deg;
  const bool position = Current(lat, now) && Current(lon, now) &&
                        lat.source == lon.source &&
                        lat.observed_at == lon.observed_at &&
                        CoordinateValid({*lat.value, *lon.value});
  if (position)
    v.gps_age_s = std::chrono::duration<double>(now - lat.observed_at).count();
  v.active = !watch.waypoint_id.empty();
  v.identity = watch.waypoint_id;
  v.alarm = v.active && watch.alarm; // Upstream alarm survives stale GPS.
  v.can_start = !v.active && position && !state.simulated && !state.replayed;
  if (!v.active)
    return v;
  v.reason =
      v.alarm
          ? "Anchor alarm. Check the vessel and GPS position."
          : "Anchor watch active. Recorded movement is relative to the anchor.";
  if (!watch.anchor || !CoordinateValid(*watch.anchor) ||
      watch.source.empty() || watch.observed_at > now) {
    v.reason = "Anchor observation unavailable";
    return v;
  }
  if (watch.radius_m && std::isfinite(*watch.radius_m) &&
      *watch.radius_m != 0) {
    v.radius_m = std::abs(*watch.radius_m);
    v.inner_alarm = *watch.radius_m < 0; // Preserve upstream too-close watch.
  }
  // Do not publish a retained distance for a newer/different selected fix.
  if (position && Current(watch.distance_m, now) &&
      *watch.distance_m.value >= 0 && watch.vessel_position &&
      PlotValid(*watch.vessel_position) &&
      watch.distance_m.observed_at == lat.observed_at &&
      watch.vessel_position->observed_at == lat.observed_at &&
      watch.vessel_position->position_source == lat.source &&
      watch.vessel_position->position.latitude_deg == *lat.value &&
      watch.vessel_position->position.longitude_deg == *lon.value) {
    v.distance_m = watch.distance_m.value;
    v.vessel_position = watch.vessel_position;
  } else
    v.reason = "Position unavailable or changed; watch needs attention.";
  vessel::Time last{};
  for (const auto &fix : watch.recent_positions) {
    if (v.history.size() == 300)
      break;
    if (!PlotValid(fix) || fix.observed_at > now || fix.observed_at <= last)
      continue;
    v.history.push_back(fix);
    last = fix.observed_at;
  }
  if (v.history.size() > 1)
    v.history_minutes =
        std::chrono::duration<double>(v.history.back().observed_at -
                                      v.history.front().observed_at)
            .count() /
        60.;
  return v;
}
} // namespace opennav::application

#include "integration/AnchorGeometry.h"
#include "model/georef.h"
#include <cmath>

namespace opennav::integration {
void ObserveAnchorPosition(application::AnchorState &watch,
                           const vessel::Navigation &position, vessel::Time now) {
  watch.distance_m = {};
  watch.vessel_position.reset();
  const auto &lat = position.latitude_deg, &lon = position.longitude_deg;
  const auto current = [now](const vessel::Sample &sample) {
    const auto quality = vessel::Assess(sample, now).quality;
    return sample.validity == vessel::Validity::Measured &&
           (quality == vessel::Quality::Live || quality == vessel::Quality::Aging);
  };
  if (watch.waypoint_id.empty() || !watch.anchor || watch.source.empty() ||
      !current(lat) || !current(lon) || lat.source != lon.source ||
      lat.observed_at != lon.observed_at)
    return;
  const application::Coordinate fix{*lat.value, *lon.value};
  auto projection = ProjectAnchorPosition(*watch.anchor, fix, lat.observed_at,
                                          lat.source);
  if (!projection)
    return;
  double bearing = 0, distance = 0;
  // Same argument order and distance routine as ProcessAnchorWatch. The plot
  // uses the reverse bearing, but its norm must not replace watch distance.
  DistanceBearingMercator(watch.anchor->latitude_deg, watch.anchor->longitude_deg,
                          fix.latitude_deg, fix.longitude_deg, &bearing,
                          &distance);
  const double metres = distance * 1852.;
  if (!std::isfinite(metres) || metres < 0)
    return;
  watch.distance_m = {metres, watch.source, lat.observed_at,
                      vessel::Validity::Measured};
  watch.distance_m.freshness = lat.freshness;
  watch.vessel_position = std::move(projection);
}
std::optional<application::AnchorFix> ProjectAnchorPosition(
    application::Coordinate anchor, application::Coordinate position,
    vessel::Time observed_at, const std::string &position_source) {
  const auto valid = [](application::Coordinate p) {
    return std::isfinite(p.latitude_deg) && std::abs(p.latitude_deg) < 90 &&
           std::isfinite(p.longitude_deg) && std::abs(p.longitude_deg) <= 180;
  };
  if (!valid(anchor) || !valid(position) || position_source.empty())
    return {};
  double bearing = 0, range = 0;
  // This routine's first pair is destination, second pair is origin.
  // Preserve its antimeridian and Mercator behavior; no alternate geodesy.
  DistanceBearingMercator(position.latitude_deg, position.longitude_deg,
                          anchor.latitude_deg, anchor.longitude_deg, &bearing,
                          &range);
  const double radians = bearing * std::acos(-1.) / 180.,
               metres = range * 1852.;
  if (!std::isfinite(radians) || !std::isfinite(metres) || metres < 0)
    return {};
  return application::AnchorFix{position, observed_at,
                                metres * std::sin(radians),
                                metres * std::cos(radians), position_source};
}
} // namespace opennav::integration

#include "integration/AnchorGeometry.h"
#include "model/georef.h"
#include <cmath>

namespace opennav::integration {
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

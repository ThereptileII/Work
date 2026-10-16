#pragma once
#include <cmath>
#include <optional>
#include <string>

namespace opennav::application {
// Route-GUID profile metadata. Shared waypoint properties and GPX stay intact.
// An explicit radius opts into radial proximity in every mode of this profile;
// absence preserves OpenCPN's waypoint/crossing-line arrival policy.
struct RoutePassageSettings {
  std::optional<double> arrival_radius_m;
  double corridor_width_m = 100; // Full display width, not a steering limit.
  bool valid = true;
  std::string reason;
  // Opaque, bounded identity copied from storage, never a user-facing value.
  // Editors submit fresh values; integration compares the selected read first.
  std::string storage_revision;
};
inline std::string ValidateRoutePassage(const RoutePassageSettings& value) {
  if (!value.valid)
    return value.reason.empty() ? "Route passage settings are unavailable" : value.reason;
  if (value.arrival_radius_m && (!std::isfinite(*value.arrival_radius_m) ||
      *value.arrival_radius_m < 2 || *value.arrival_radius_m > 1852))
    return "Waypoint passage distance must be 2 to 1852 m, or blank for OpenCPN behavior";
  if (!std::isfinite(value.corridor_width_m) || value.corridor_width_m < 2 ||
      value.corridor_width_m > 20000)
    return "Full route corridor width must be 2 to 20000 m";
  return {};
}
} // namespace opennav::application

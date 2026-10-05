#include "integration/AisViewport.h"
#include <algorithm>
#include <cmath>

namespace opennav::integration {
std::optional<ais::Viewport> AisViewport(bool valid, double south, double north,
                                       double west, double east,
                                       std::optional<ais::AreaCenter> center) {
  if (!valid || !std::isfinite(south) || !std::isfinite(north) ||
      !std::isfinite(west) || !std::isfinite(east) || south >= north || west >= east)
    return {};
  south = (std::max)(-90.0, south);
  north = (std::min)(90.0, north);
  const double span = east-west;
  if (south >= north || !std::isfinite(span)) return {};
  if (center && (!std::isfinite(center->latitude) || !std::isfinite(center->longitude) ||
                 std::abs(center->latitude) > 90)) return {};
  if (center) center->longitude = std::remainder(center->longitude,360.0);
  if (span >= 360.0) return ais::Viewport{south,north,-180,180,center};
  const auto normalize = [](double longitude) {
    return std::remainder(longitude,360.0);
  };
  const auto w=normalize(west), e=normalize(east);
  if (w == e) return {}; // precision loss must not mean an all-world area
  return ais::Viewport{south,north,w,e,center};
}
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double earth_radius_nm = 3440.065;
bool ValidRadius(int radius) { return radius >= ais::MinimumRadiusNm && radius <= ais::MaximumRadiusNm; }
bool ValidPoint(ais::AreaCenter p) {
  return std::isfinite(p.latitude) && std::isfinite(p.longitude) &&
         std::abs(p.latitude) <= 90 && std::abs(p.longitude) <= 180;
}
}
std::optional<ais::Viewport> AisRadiusViewport(ais::Viewport chart, int radius_nm) {
  if (!ValidRadius(radius_nm) || ais::SubscriptionArea(chart).empty()) return {};
  // Legacy callers without a copied projection center retain a geographic
  // bbox-center fallback. The application bridge supplies the actual center.
  const double span = chart.east >= chart.west ? chart.east-chart.west
                                              : chart.east-chart.west+360;
  const auto center = chart.center.value_or(ais::AreaCenter{
      (chart.south+chart.north)/2, std::remainder(chart.west+span/2,360.0)});
  if (!ValidPoint(center)) return {};
  const double angular = radius_nm/earth_radius_nm;
  const double latitude = center.latitude*pi/180;
  const double south = std::max(-90.0, center.latitude-angular*180/pi);
  const double north = std::min(90.0, center.latitude+angular*180/pi);
  if (south <= -90 || north >= 90)
    return ais::Viewport{south,north,-180,180,center,true};
  const double longitude_delta = std::asin(std::clamp(std::sin(angular)/std::cos(latitude),-1.,1.))*180/pi;
  return ais::Viewport{south,north,
      std::remainder(center.longitude-longitude_delta,360.0),
      std::remainder(center.longitude+longitude_delta,360.0),center,true};
}
bool WithinAisRadius(ais::AreaCenter center, int radius_nm,
                     double latitude, double longitude) {
  if (!ValidRadius(radius_nm) || !ValidPoint(center) || !ValidPoint({latitude,longitude})) return false;
  const double a = center.latitude*pi/180, b = latitude*pi/180;
  const double dlat = (b-a)/2, dlon = (longitude-center.longitude)*pi/360;
  const double h = std::sin(dlat)*std::sin(dlat) +
      std::cos(a)*std::cos(b)*std::sin(dlon)*std::sin(dlon);
  return 2*earth_radius_nm*std::asin(std::sqrt(std::clamp(h,0.,1.))) <= radius_nm+1e-9;
}
} // namespace opennav::integration

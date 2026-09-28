#include "integration/AisViewport.h"
#include <algorithm>
#include <cmath>

namespace opennav::integration {
std::optional<ais::Viewport> AisViewport(bool valid, double south, double north,
                                       double west, double east) {
  if (!valid || !std::isfinite(south) || !std::isfinite(north) ||
      !std::isfinite(west) || !std::isfinite(east) || south >= north || west >= east)
    return {};
  south = (std::max)(-90.0, south);
  north = (std::min)(90.0, north);
  const double span = east-west;
  if (south >= north || !std::isfinite(span)) return {};
  if (span >= 360.0) return ais::Viewport{south,north,-180,180};
  const auto normalize = [](double longitude) {
    return std::remainder(longitude,360.0);
  };
  const auto w=normalize(west), e=normalize(east);
  if (w == e) return {}; // precision loss must not mean an all-world area
  return ais::Viewport{south,north,w,e};
}
} // namespace opennav::integration

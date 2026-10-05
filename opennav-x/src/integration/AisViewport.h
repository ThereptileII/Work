#pragma once
#include "ais/Subscription.h"
#include <optional>

namespace opennav::integration {
// The pinned ViewPort::SetBoxes publishes an LLBBox with ordered, potentially
// unwrapped longitudes. This only normalizes geographic subscription bounds;
// it does not compute route geometry, range, bearing or collision risk.
std::optional<ais::Viewport> AisViewport(bool valid, double south, double north,
                                       double west, double east,
                                       std::optional<ais::AreaCenter> center = {});
// Subscription geometry only, never navigation, CPA or collision calculations.
std::optional<ais::Viewport> AisRadiusViewport(ais::Viewport chart, int radius_nm);
bool WithinAisRadius(ais::AreaCenter center, int radius_nm,
                     double latitude, double longitude);
} // namespace opennav::integration

#pragma once
#include "ais/Subscription.h"
#include <optional>

namespace opennav::integration {
// The pinned ViewPort::SetBoxes publishes an LLBBox with ordered, potentially
// unwrapped longitudes. This only normalizes geographic subscription bounds;
// it does not compute route geometry, range, bearing or collision risk.
std::optional<ais::Viewport> AisViewport(bool valid, double south, double north,
                                       double west, double east);
} // namespace opennav::integration

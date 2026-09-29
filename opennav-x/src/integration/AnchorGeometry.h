#pragma once
#include "application/NavigationObjects.h"

namespace opennav::integration {
// Presentation projection only. Never processes progress, arms a watch or
// calculates an alarm. The caller supplies a coherent selected position.
std::optional<application::AnchorFix> ProjectAnchorPosition(
    application::Coordinate anchor, application::Coordinate position,
    vessel::Time observed_at, const std::string &position_source);
}

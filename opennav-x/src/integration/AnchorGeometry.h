#pragma once
#include "application/NavigationObjects.h"

namespace opennav::integration {
// Presentation projection only. Never processes progress, arms a watch or
// calculates an alarm. The caller supplies a coherent selected position.
std::optional<application::AnchorFix> ProjectAnchorPosition(
    application::Coordinate anchor, application::Coordinate position,
    vessel::Time observed_at, const std::string &position_source);
// Refresh an owned presentation snapshot from the selected fix. The upstream
// caller additionally verifies bGPSValid and equality with its accepted fix.
// Never processes or changes the alarm; a read cannot refresh GPS timestamps.
void ObserveAnchorPosition(application::AnchorState &watch,
                           const vessel::Navigation &position, vessel::Time now);
}

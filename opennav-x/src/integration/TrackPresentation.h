#pragma once

#include "application/NavigationObjects.h"

class wxFileConfig;

namespace opennav::integration {
// Application-thread lifecycle. Pass nullptr/false before the profile is
// destroyed. This preference does not depend on the selected chart palette.
void ConfigureTrackPresentation(wxFileConfig *config, bool xnav);

// Master chart-display filter only. Track recording, database contents and
// every individual track/imported-layer visibility flag remain upstream-owned.
bool TrackLayerVisible();
application::CommandResult SetTrackLayerVisible(bool visible);

// Narrow GUI paint/hit-test hook. Always false in ordinary OpenCPN/Legacy/Safe
// and outside the application thread; no model-side callback/link dependency.
bool XNavTrackLayerHidden();
} // namespace opennav::integration

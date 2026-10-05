#pragma once

#include "application/StartupUpdate.h"
#include "ui/Theme.h"

class wxWindow;

namespace opennav::ui {
// Call on the wx application thread before normal startup. Parent may be null:
// no OpenCPN frame, plugins, adapters or navigation model are needed. Returns
// only the exact candidate selected by the user, never a launch instruction.
std::optional<application::StartupUpdateCandidate> ShowStartupUpdateDialog(
    wxWindow* parent, application::StartupUpdate& update,
    LightMode light = LightMode::Day);
}  // namespace opennav::ui

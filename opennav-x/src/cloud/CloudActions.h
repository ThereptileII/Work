#pragma once
// UI-facing SKAGER Cloud actions (SCRUM-44, SCRUM-369), mirroring
// WeatherActions. Callbacks return immediately; outcomes appear in read().
#include "application/NavigationObjects.h"
#include "cloud/CloudService.h"
#include <functional>

namespace opennav::application {
struct CloudActions {
  std::function<cloud::CloudSnapshot()> read;
  std::function<CommandResult()> link;          // Request a pairing code.
  std::function<CommandResult()> cancel_link;
  std::function<CommandResult()> unlink;        // Forget the account on this PC.
  std::function<CommandResult(bool)> share;     // Boat data sharing on/off.
  std::function<void()> refresh;                // Re-read account and Cloud status.
};
}  // namespace opennav::application

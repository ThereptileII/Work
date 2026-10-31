#pragma once
// UI-facing SKAGER Cloud actions (SCRUM-44, SCRUM-369), mirroring
// WeatherActions. Callbacks return immediately; outcomes appear in read().
#include "application/NavigationObjects.h"
#include "cloud/CloudService.h"
#include <functional>
#include <string>
#include <vector>

namespace opennav::application {
// A QR code as rows of '1' (dark) and '0' (light) modules, without the quiet zone.
struct CloudQr {
  int size = 0;
  std::vector<std::string> rows;
};
struct CloudActions {
  std::function<cloud::CloudSnapshot()> read;
  std::function<CommandResult()> link;          // Request a pairing code.
  std::function<CommandResult()> cancel_link;
  std::function<CommandResult()> unlink;        // Forget the account on this PC.
  std::function<CommandResult(bool)> share;     // Boat data sharing on/off.
  std::function<void()> refresh;                // Re-read account and Cloud status.
  // The pairing page as a QR code for the phone's camera; empty when it cannot be made.
  std::function<CloudQr(const std::string &user_code)> qr;
  // Active critical alerts as (id, title) from the live alert tick, for
  // SKAGER Companion notifications (SCRUM-387). Never called for demo/replay.
  std::function<void(const std::vector<std::pair<std::string, std::string>> &)> alerts;
};
}  // namespace opennav::application

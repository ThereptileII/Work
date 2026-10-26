#pragma once
// UI-facing SKAGER Helm actions (SCRUM-379), mirroring CloudActions.
// Callbacks return immediately; outcomes appear in read().
#include "helm/HelmHost.h"
#include <functional>
#include <string>

namespace opennav::application {
struct HelmActions {
  std::function<helm::HelmSnapshot()> read;
  std::function<bool()> enabled;                 // Switched on in this profile.
  std::function<void(bool)> enable;              // Starts or stops the Helm Server.
  std::function<void()> pair;                    // Opens a five-minute pairing window.
  std::function<void()> cancel_pair;
  std::function<void(const std::string &)> revoke;  // Removes a paired phone.
  std::function<void()> disconnect;              // Ends the current phone session.
};
}  // namespace opennav::application

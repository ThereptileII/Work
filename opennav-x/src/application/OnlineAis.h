#pragma once
#include "ais/Credentials.h"
#include "ais/Provider.h"
#include "ais/Subscription.h"
#include "application/NavigationObjects.h"
#include <functional>

namespace opennav::application {
struct OnlineAisState {
  bool enabled = false, credential_present = false, credential_writable = false;
  int radius_nm = ais::DefaultRadiusNm;
  ais::ProviderSnapshot feed;
};
struct OnlineAisActions {
  // Copied status only. No socket, provider or OpenCPN model pointer in the UI.
  std::function<OnlineAisState(vessel::Time)> read;
  std::function<CommandResult(bool)> enable;
  std::function<CommandResult(int)> set_radius_nm;
  std::function<CommandResult(const ais::Secret &)> store_key;
  std::function<CommandResult()> remove_key;
};
} // namespace opennav::application

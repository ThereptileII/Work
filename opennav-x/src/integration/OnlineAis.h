#pragma once
#include "ais/AisStreamProvider.h"
#include "ais/Credentials.h"
#include "application/NavigationObjects.h"
#include <memory>
#include <optional>
class wxFileConfig;

namespace opennav::integration {
// Application-thread settings/lifecycle boundary. The OpenCPN bridge copies
// and normalizes its viewport before passing it here. No chart pointers enter.
class OnlineAis final {
public:
  OnlineAis(wxFileConfig &config, std::unique_ptr<ais::IAisCredentials> credentials,
            std::unique_ptr<ais::IOnlineAisProvider> provider);
  ~OnlineAis();
  void ObserveViewport(std::optional<ais::Viewport> viewport, bool live_allowed);
  ais::ProviderSnapshot Read(vessel::Time now) const;
  bool Enabled() const { return enabled_; }
  bool CredentialPresent() const { return credential_present_; }
  application::CommandResult Enable(bool enabled);
  application::CommandResult StoreKey(const ais::Secret &key);
  application::CommandResult RemoveKey();

private:
  wxFileConfig &config_;
  std::unique_ptr<ais::IAisCredentials> credentials_;
  std::unique_ptr<ais::IOnlineAisProvider> provider_;
  bool enabled_ = false, credential_present_ = false;
};
} // namespace opennav::integration

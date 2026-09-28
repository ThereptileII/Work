#pragma once
#include "ais/Provider.h"
#include "ais/Subscription.h"
#include <string>

namespace opennav::ais {
class IAisCredentials;
// Optional supplemental provider; constructing it never enables networking.
// All callbacks stay on its worker boundary. Reads return retained-safe values.
class AisStreamProvider final : public IAisProvider {
public:
  AisStreamProvider();
  ~AisStreamProvider() override;
  AisStreamProvider(const AisStreamProvider &) = delete;
  AisStreamProvider &operator=(const AisStreamProvider &) = delete;
  void SetEnabled(bool enabled);
  bool ObserveViewport(Viewport viewport);
  void CredentialChanged();
  ProviderSnapshot Read(vessel::Time now) const override;
#ifdef OPENNAV_AIS_TEST_TRANSPORT
  // Compiled exclusively into a dedicated loopback integration test driver.
  static std::unique_ptr<AisStreamProvider>
  ForTest(std::unique_ptr<IAisCredentials>, const std::string &loopback_url,
          const std::string &test_ca);
#endif
private:
  AisStreamProvider(std::unique_ptr<IAisCredentials>, std::string url,
                    std::string ca);
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace opennav::ais

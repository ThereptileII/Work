#include "ais/AisStreamProvider.h"
#include "ais/Credentials.h"
#include "ais/TargetCache.h"
#include <chrono>
#include <iostream>
#include <thread>
using namespace opennav;
class TestCredentials final : public ais::IAisCredentials {
public:
  ais::CredentialResult Read() const override {
    ais::CredentialResult r;
    r.status = ais::CredentialStatus::Ready;
    r.key.Assign("loopback-test-not-a-real-key");
    return r;
  }
  ais::CredentialStatus Store(const ais::Secret &) override {
    return ais::CredentialStatus::ReadOnly;
  }
  ais::CredentialStatus Remove() override {
    return ais::CredentialStatus::ReadOnly;
  }
};
int main(int argc, char **argv) {
  if (argc != 3)
    return 2;
  auto provider = ais::AisStreamProvider::ForTest(
      std::make_unique<TestCredentials>(), argv[1], argv[2]);
  if (!provider)
    return 3;
  provider->ObserveViewport({59, 60, 18, 19});
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  if (provider->Read(vessel::Clock::now()).health.connection !=
      ais::Connection::Disabled)
    return 4;
  provider->SetEnabled(true);
  bool panned = false;
  ais::ProviderSnapshot retained;
  const auto deadline = vessel::Clock::now() + std::chrono::seconds(20);
  while (vessel::Clock::now() < deadline) {
    const auto now = vessel::Clock::now();
    auto s = provider->Read(now);
    if (s.health.subscription_confirmed && s.targets.targets.size() == 1) {
      if (!panned) {
        retained = s;
        if (!provider->ObserveViewport({40, 41, 10, 11}))
          return 5;
        panned = true;
      }
      if (s.health.accepted >= 2 && s.health.reconnects >= 1) {
        if (s.targets.targets[0].source != ais::OnlineSource ||
            !s.health.compression_enabled)
          return 6;
        provider->SetEnabled(false);
        if (!provider->Read(now).targets.targets.empty())
          return 7;
        if (retained.targets.targets.size() != 1)
          return 8;
        std::cout
            << "PASS provider: confirmed, real transport reports, viewport "
               "replacement, reconnect, resubscribe, owned lifetime, disable\n";
        return 0;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::cerr << "Provider loopback lifecycle did not finish\n";
  return 9;
}

// Opt-in real-clock provider proof. Never installed; no real service credentials.
#include "ais/AisStreamProvider.h"
#include "ais/Credentials.h"
#include "ais/TargetCache.h"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace opennav;
using namespace std::chrono_literals;
namespace {
unsigned checks = 0;
void Check(bool ok) {
  ++checks;
  if (!ok) throw std::runtime_error("AIS outage invariant refused");
}
class Credentials final : public ais::IAisCredentials {
public:
  ais::CredentialResult Read() const override {
    ais::CredentialResult result;
    result.status = ais::CredentialStatus::Ready;
    result.key.Assign("loopback-test-not-a-real-key");
    return result;
  }
  ais::CredentialStatus Store(const ais::Secret &) override { return ais::CredentialStatus::ReadOnly; }
  ais::CredentialStatus Remove() override { return ais::CredentialStatus::ReadOnly; }
};
bool Same(const ais::ConnectionObservation &a, const ais::ConnectionObservation &b) {
  return a.family == b.family && a.generation == b.generation && a.captured_at == b.captured_at &&
         a.local.address == b.local.address && a.remote.address == b.remote.address &&
         a.local.port == b.local.port && a.remote.port == b.remote.port &&
         a.local.scope == b.local.scope && a.remote.scope == b.remote.scope;
}
void Endpoint(const ais::ConnectionObservation &c) {
  Check(c.family != ais::AddressFamily::Unavailable && c.generation && c.local.port && c.remote.port);
  std::cout << "OBS " << c.generation << ' ' << (c.family == ais::AddressFamily::IPv4 ? 4 : 6)
            << ' ' << c.local.port << ' ' << c.remote.port << ' ' << c.local.scope << ' ' << c.remote.scope;
  for (auto b : c.local.address) std::cout << ' ' << unsigned(b);
  for (auto b : c.remote.address) std::cout << ' ' << unsigned(b);
  std::cout << std::endl;
}
}
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  try {
    auto provider = ais::AisStreamProvider::ForTest(std::make_unique<Credentials>(), argv[1], argv[2]);
    Check(bool(provider));
    Check(provider->ObserveViewport({59, 60, 18, 19}));
    provider->SetEnabled(true);
    ais::ProviderSnapshot retained, stale, lost, confirmed;
    bool initial = false, saw_stale = false, saw_lost = false, cleared = false, reconfirmed = false;
    const auto deadline = vessel::Clock::now() + 150s;
    while (vessel::Clock::now() < deadline) {
      const auto now = vessel::Clock::now();
      const auto s = provider->Read(now);
      if (!initial) {
        if (s.health.accepted == 1 && s.health.subscription_confirmed && s.targets.targets.size() == 1) {
          retained = s;
          Endpoint(s.connection);
          Check(s.connection.captured_at > vessel::Time{} && s.connection.captured_at <= now);
          Check(s.targets.targets[0].active && !s.targets.targets[0].lost);
          initial = true;
          std::cout << "PHASE initial" << std::endl;
        }
      } else {
        Check(s.targets.targets.size() == 1); // no disable/cache reset is permitted
        const auto &target = s.targets.targets[0];
        if (s.health.accepted == 1) {
          Check(target.observed_at == retained.targets.targets[0].observed_at);
          Check(s.health.last_position == retained.health.last_position);
          const auto age = now - target.observed_at;
          if (s.connection.generation == retained.connection.generation) {
            Check(Same(s.connection, retained.connection));
            Check(s.health.subscription_confirmed && s.health.reconnects == 0);
            if (!saw_stale && age >= 61s) {
              Check(ais::Age(target.observed_at, now) == ais::TargetAge::Stale);
              Check(target.active && !target.lost);
              stale = s; saw_stale = true;
              std::cout << "PHASE stale " << std::chrono::duration_cast<std::chrono::milliseconds>(age).count() << std::endl;
            }
            if (!saw_lost && age >= 121s) {
              Check(saw_stale && ais::Age(target.observed_at, now) == ais::TargetAge::Lost);
              Check(!target.active && target.lost);
              lost = s; saw_lost = true;
              std::cout << "PHASE lost " << std::chrono::duration_cast<std::chrono::milliseconds>(age).count() << std::endl;
            }
          } else if (s.connection.family == ais::AddressFamily::Unavailable) {
            Check(saw_lost && target.lost && !target.active);
            Check(s.connection.generation == 0 && s.connection.captured_at == vessel::Time{});
            if (!cleared) {
              cleared = true;
              std::cout << "PHASE disconnected" << std::endl;
            }
          } else if (s.health.subscription_confirmed && !reconfirmed) {
            Check(saw_lost && cleared && target.lost && !target.active);
            Check(s.connection.generation > retained.connection.generation);
            Check(s.connection.captured_at > retained.connection.captured_at && s.connection.captured_at <= now);
            Check(s.health.reconnects == 1);
            confirmed = s; reconfirmed = true;
            Endpoint(s.connection);
            std::cout << "PHASE confirmed_still_lost" << std::endl;
          }
        } else {
          Check(s.health.accepted == 2 && reconfirmed && s.health.subscription_confirmed);
          Check(Same(s.connection, confirmed.connection));
          Check(target.active && !target.lost && target.observed_at > lost.targets.targets[0].observed_at);
          Check(s.health.last_position == target.observed_at && target.source == ais::OnlineSource);
          Check(target.latitude_deg.value && *target.latitude_deg.value == 59.2);
          Check(target.origin == vessel::AisOrigin::AisStreamOnline);
          provider.reset(); // snapshot copies survive provider and worker destruction
          Check(retained.targets.targets[0].active && !retained.targets.targets[0].lost);
          Check(stale.targets.targets[0].active && lost.targets.targets[0].lost && confirmed.targets.targets[0].lost);
          Check(Same(retained.connection, lost.connection));
          std::cout << "PHASE recovered\nPASS " << checks << " real-clock outage checks" << std::endl;
          return 0;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
  } catch (...) {
    std::cerr << "AIS loopback outage proof failed\n";
    return 1;
  }
  std::cerr << "AIS loopback outage proof deadline expired\n";
  return 1;
}

#include "ais/AisStreamProvider.h"
#include "ais/Credentials.h"
#include "ais/TargetCache.h"
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <iostream>
#include <thread>
using namespace opennav;
struct CredentialRead {
  std::mutex mutex;
  std::condition_variable changed;
  bool block = false, entered = false, release = false;
  unsigned reads = 0;
};
class TestCredentials final : public ais::IAisCredentials {
public:
  explicit TestCredentials(std::shared_ptr<CredentialRead> state) : state_(std::move(state)) {}
  ais::CredentialResult Read() const override {
    ais::CredentialResult r;
    r.status = ais::CredentialStatus::Ready;
    std::unique_lock<std::mutex> lock(state_->mutex);
    const bool blocked = state_->block && ++state_->reads == 1;
    if (blocked) {
      state_->entered = true;
      state_->changed.notify_all();
      if (!state_->changed.wait_for(lock, std::chrono::seconds(5), [&] { return state_->release; }))
        return {};
    }
    r.key.Assign(blocked ? "stale-loopback-key" : "loopback-test-not-a-real-key");
    return r;
  }
  ais::CredentialStatus Store(const ais::Secret &) override {
    return ais::CredentialStatus::ReadOnly;
  }
  ais::CredentialStatus Remove() override {
    return ais::CredentialStatus::ReadOnly;
  }
private:
  std::shared_ptr<CredentialRead> state_;
};
int main(int argc, char **argv) {
  if (argc != 4)
    return 2;
  const std::string scenario = argv[3];
  auto credentials = std::make_shared<CredentialRead>();
  credentials->block = scenario == "disable-enable" || scenario == "credential-change";
  auto open = std::make_shared<CredentialRead>();
  open->block = scenario == "open-disable-enable" || scenario == "open-credential-change";
  auto before_open = [open] {
    if (!open->block) return;
    std::unique_lock<std::mutex> lock(open->mutex);
    if (++open->reads != 1) return;
    open->entered = true;
    open->changed.notify_all();
    // A finite deadline also prevents teardown from hanging on a failed test.
    open->changed.wait_for(lock, std::chrono::seconds(5), [&] { return open->release; });
  };
  auto provider = ais::AisStreamProvider::ForTest(
      std::make_unique<TestCredentials>(credentials), argv[1], argv[2], before_open);
  if (!provider)
    return 3;
  provider->ObserveViewport({59, 60, 18, 19});
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  if (provider->Read(vessel::Clock::now()).health.connection !=
      ais::Connection::Disabled)
    return 4;
  provider->SetEnabled(true);
  if (credentials->block || open->block) {
    auto barrier = credentials->block ? credentials : open;
    std::unique_lock<std::mutex> lock(barrier->mutex);
    if (!barrier->changed.wait_for(lock, std::chrono::seconds(5), [&] { return barrier->entered; }))
      return 10;
    lock.unlock();
    if (scenario == "disable-enable" || scenario == "open-disable-enable") {
      provider->SetEnabled(false);
      if (provider->Read(vessel::Clock::now()).connection.family != ais::AddressFamily::Unavailable)
        return 11;
      provider->SetEnabled(true);
    } else if (scenario == "credential-change" || scenario == "open-credential-change")
      provider->CredentialChanged();
    else
      return 12;
    if (provider->Read(vessel::Clock::now()).connection.family != ais::AddressFamily::Unavailable)
      return 19;
    lock.lock();
    barrier->release = true;
    barrier->changed.notify_all();
  }
  std::uint64_t seen_generation = 0;
  bool saw_unavailable_after_open = false;
  bool panned = false;
  ais::ProviderSnapshot retained;
  const auto deadline = vessel::Clock::now() + std::chrono::seconds(20);
  while (vessel::Clock::now() < deadline) {
    const auto now = vessel::Clock::now();
    auto s = provider->Read(now);
    if (s.connection.family == ais::AddressFamily::Unavailable) {
      if (seen_generation) saw_unavailable_after_open = true;
    } else {
      if (s.connection.captured_at <= vessel::Time{} || s.connection.captured_at > now ||
          !s.connection.local.port || !s.connection.remote.port || !s.connection.generation)
        return 13;
      if (s.connection.generation != seen_generation) {
        if (seen_generation && (!saw_unavailable_after_open || s.connection.generation <= seen_generation))
          return 14;
        seen_generation = s.connection.generation;
        std::cout << "OBS " << seen_generation << ' '
                  << (s.connection.family == ais::AddressFamily::IPv4 ? 4 : 6) << ' '
                  << s.connection.local.port << ' ' << s.connection.remote.port << ' '
                  << s.connection.local.scope << ' ' << s.connection.remote.scope;
        for (auto byte : s.connection.local.address) std::cout << ' ' << unsigned(byte);
        for (auto byte : s.connection.remote.address) std::cout << ' ' << unsigned(byte);
        std::cout << '\n';
      }
      if (panned && s.connection.generation == retained.connection.generation &&
          s.connection.captured_at != retained.connection.captured_at)
        return 15;
    }
    if (s.health.subscription_confirmed && s.targets.targets.size() == 1) {
      if (!panned) {
        retained = s;
        if (!provider->ObserveViewport({40, 41, 10, 11}))
          return 5;
        provider->SetEnabled(true); // repeated runtime tick must preserve connection
        if (provider->Read(now).connection.generation != retained.connection.generation)
          return 16;
        panned = true;
      }
      if (s.health.accepted >= 2 && s.health.reconnects >= 1) {
        if (s.targets.targets[0].source != ais::OnlineSource ||
            !s.health.compression_enabled)
          return 6;
        provider->SetEnabled(false);
        if (!provider->Read(now).targets.targets.empty())
          return 7;
        if (provider->Read(now).connection.family != ais::AddressFamily::Unavailable)
          return 17;
        provider.reset(); // retained snapshot outlives socket threads and provider
        if (retained.connection.family == ais::AddressFamily::Unavailable ||
            retained.connection.captured_at == vessel::Time{})
          return 18;
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

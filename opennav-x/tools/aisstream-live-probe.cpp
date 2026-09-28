// Explicit commissioning tool, never installed or started by XNav. Links the
// production fixed-endpoint provider, not its loopback test override. Outputs
// only aggregate counters: no credential, MMSI, names or vessel positions.
#include "ais/AisStreamProvider.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <string_view>
#include <thread>

using namespace opennav;
using namespace std::chrono_literals;
namespace {
const char *State(ais::Connection state) {
  switch (state) {
  case ais::Connection::Disabled: return "disabled";
  case ais::Connection::CredentialMissing: return "credential_missing";
  case ais::Connection::Connecting: return "connecting";
  case ais::Connection::Subscribing: return "subscribing";
  case ais::Connection::Connected: return "connected";
  case ais::Connection::Backoff: return "backoff";
  default: return "offline";
  }
}
} // namespace
int main(int argc, char **argv) {
  if (argc == 2 && std::string_view(argv[1]) == "--describe") {
    std::cout << "{\"scope\":\"explicit read-only internet AIS commissioning\","
                 "\"endpoint\":\"wss://stream.aisstream.io/v0/stream\","
                 "\"credentials\":\"production protected store or development environment\","
                 "\"maximum_observation_seconds\":45,\"marine_equipment\":false,"
                 "\"profile_access\":false,\"secret_output\":false}\n";
    return 0;
  }
  if (argc != 3 || std::string_view(argv[1]) != "--read-only-live-ais" ||
      (std::string_view(argv[2]) != "stockholm" &&
       std::string_view(argv[2]) != "oresund")) {
    std::cerr << "Usage: --describe OR --read-only-live-ais stockholm|oresund\n";
    return 2;
  }
  try {
    ais::AisStreamProvider provider;
    const ais::Viewport area = std::string_view(argv[2]) == "stockholm"
        ? ais::Viewport{58.5, 60, 17.5, 20.5}
        : ais::Viewport{55, 56.5, 12, 13.5};
    if (!provider.ObserveViewport(area)) return 3;
    provider.SetEnabled(true);
    const auto start = vessel::Clock::now();
    std::size_t maximum = 0;
    std::uint64_t accepted = 0, rejected = 0, reconnects = 0;
    bool confirmed = false, compressed = false;
    auto previous = ais::Connection::Disabled;
    while (vessel::Clock::now() - start < 45s) {
      const auto now = vessel::Clock::now();
      const auto snapshot = provider.Read(now);
      maximum = (std::max)(maximum, snapshot.targets.targets.size());
      accepted = (std::max)(accepted, snapshot.health.accepted);
      rejected = (std::max)(rejected, snapshot.health.rejected);
      reconnects = (std::max)(reconnects, snapshot.health.reconnects);
      confirmed |= snapshot.health.subscription_confirmed;
      compressed |= snapshot.health.compression_enabled;
      if (snapshot.health.connection != previous) {
        previous = snapshot.health.connection;
        std::cout << "{\"event\":\"connection\",\"state\":\"" << State(previous)
                  << "\",\"elapsed_ms\":"
                  << std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count()
                  << "}\n" << std::flush;
      }
      if (previous == ais::Connection::CredentialMissing) break;
      std::this_thread::sleep_for(100ms);
    }
    provider.SetEnabled(false);
    const bool stopped = provider.Read(vessel::Clock::now()).health.connection ==
                             ais::Connection::Disabled &&
                         provider.Read(vessel::Clock::now()).targets.targets.empty();
    std::cout << "{\"event\":\"result\",\"region\":\"" << argv[2]
              << "\",\"subscription_confirmed\":" << (confirmed ? "true" : "false")
              << ",\"compression\":" << (compressed ? "true" : "false")
              << ",\"peak_target_count\":" << maximum
              << ",\"accepted_reports\":" << accepted << ",\"rejected_reports\":" << rejected
              << ",\"reconnects\":" << reconnects
              << ",\"disabled_and_cleared\":" << (stopped ? "true" : "false")
              << ",\"chart_or_UI_acceptance\":false}\n";
    return confirmed && maximum && stopped ? 0 : 4;
  } catch (...) {
    // Even a dependency exception must not echo server text or secrets.
    std::cerr << "AIS read-only commissioning probe failed\n";
    return 5;
  }
}

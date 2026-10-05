#pragma once
#include "vessel/AisState.h"
#include <cstdint>
#include <array>
#include <memory>

namespace opennav::ais {
enum class Connection { Disabled, CredentialMissing, Connecting, Subscribing, Connected, Backoff, Offline };
struct ProviderHealth {
  Connection connection = Connection::Disabled;
  vessel::Time state_since{}, last_position{};
  std::uint64_t accepted = 0, rejected = 0, reconnects = 0;
  // Complete data messages received by an active session, including service
  // confirmations and unsupported message kinds. Never count these as reports.
  std::uint64_t received_messages = 0, ignored_messages = 0;
  bool subscription_confirmed = false;
  bool subscription_pending = false, subscription_awaiting_confirmation = false;
  bool compression_enabled = false;
  vessel::Time retry_at{};
  // Diagnostics are enums/counters; no server text, subscription JSON or key.
};
enum class AddressFamily { Unavailable, IPv4, IPv6 };
struct ConnectionEndpoint {
  std::array<std::uint8_t, 16> address{}; // network bytes; IPv4 uses first four
  std::uint16_t port = 0;               // host order
  std::uint32_t scope = 0;              // IPv6 only
};
struct ConnectionObservation {
  AddressFamily family = AddressFamily::Unavailable;
  ConnectionEndpoint local, remote;
  std::uint64_t generation = 0; // provider-lifetime identity, not a process ID
  vessel::Time captured_at{};   // never refreshed by Read; not proof of liveness
};
struct ProviderSnapshot {
  vessel::AisState targets;
  ProviderHealth health;
  ConnectionObservation connection;
  // Eligible unexpired positioned cache entries before any presentation-radius
  // filtering. Static-only cache entries do not establish a vessel position.
  std::uint32_t cached_position_count = 0;
};
class IAisProvider {
 public:
  virtual ~IAisProvider() = default;
  // An owned value copy, no raw decoder, socket, target or GUI pointers.
  virtual ProviderSnapshot Read(vessel::Time now) const = 0;
};
struct AisFeeds {
  vessel::AisState onboard;
  ProviderSnapshot online;
  vessel::AisState display;
};
// For chart/list presentation only. Onboard health and upstream safety outputs
// continue to consume onboard, never infer receiver health from display.
AisFeeds Aggregate(vessel::AisState onboard, ProviderSnapshot online);
} // namespace opennav::ais

#pragma once
#include "vessel/AisState.h"
#include <cstdint>
#include <memory>

namespace opennav::ais {
enum class Connection { Disabled, CredentialMissing, Connecting, Subscribing, Connected, Backoff, Offline };
struct ProviderHealth {
  Connection connection = Connection::Disabled;
  vessel::Time state_since{}, last_position{};
  std::uint64_t accepted = 0, rejected = 0, reconnects = 0;
  bool subscription_confirmed = false;
  bool compression_enabled = false;
  vessel::Time retry_at{};
  // Diagnostics are enums/counters; no server text, subscription JSON or key.
};
struct ProviderSnapshot {
  vessel::AisState targets;
  ProviderHealth health;
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

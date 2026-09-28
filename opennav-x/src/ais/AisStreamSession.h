#pragma once
#include "ais/AisStreamCodec.h"

namespace opennav::ais {
// Thread-confined state machine; the runtime owns its mutex/transport. UI reads
// only an owned ProviderSnapshot. No credentials or remote text are retained.
class AisStreamSession {
public:
  void Enable(bool enabled, vessel::Time now);
  bool ObserveViewport(Viewport viewport);
  bool NeedsConnection(vessel::Time now) const;
  bool Connecting(vessel::Time now);
  void Opened(vessel::Time now);
  void CredentialMissing(vessel::Time now);
  void RetryCredentials(vessel::Time now);
  std::vector<BoundingBox> PendingSubscription(vessel::Time now) const;
  bool SubscriptionSent(vessel::Time now);
  void Receive(const std::string &message, vessel::Time now,
               std::chrono::system_clock::time_point wall, unsigned entropy);
  void Disconnected(vessel::Time now, unsigned entropy);
  void Tick(vessel::Time now, unsigned entropy);
  bool ShouldClose() const;
  ProviderSnapshot Read(vessel::Time now) const;

private:
  bool Advance(vessel::Time now);
  void State(Connection state, vessel::Time now);
  bool enabled_ = false, new_connection_ = false;
  bool compression_ = false;
  unsigned failures_ = 0;
  vessel::Time last_event_{}, retry_at_{}, sent_at_{};
  SubscriptionPolicy subscription_;
  TargetCache cache_;
  ProviderHealth health_;
};
} // namespace opennav::ais

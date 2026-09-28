#include "ais/AisStreamSession.h"
#include <algorithm>
#include <limits>

namespace opennav::ais {
namespace {
void Increment(std::uint64_t &value) {
  if (value != std::numeric_limits<std::uint64_t>::max())
    ++value;
}
vessel::Time After(vessel::Time at, vessel::Duration delay) {
  return at > vessel::Time::max() - delay ? vessel::Time::max() : at + delay;
}
} // namespace
bool AisStreamSession::Advance(vessel::Time now) {
  if (now <= vessel::Time{} || now < last_event_)
    return false;
  last_event_ = now;
  return true;
}
void AisStreamSession::State(Connection state, vessel::Time now) {
  if (health_.connection != state) {
    health_.connection = state;
    health_.state_since = now;
  }
}
void AisStreamSession::Enable(bool enabled, vessel::Time now) {
  if (!Advance(now) || enabled == enabled_)
    return;
  enabled_ = enabled;
  failures_ = 0;
  health_.subscription_confirmed = false;
  compression_ = false;
  retry_at_ = {};
  if (!enabled)
    cache_ = TargetCache{}; // explicit user disable removes online targets
  State(enabled ? Connection::Offline : Connection::Disabled, now);
}
bool AisStreamSession::ObserveViewport(Viewport viewport) {
  return subscription_.ObserveViewport(viewport);
}
bool AisStreamSession::NeedsConnection(vessel::Time now) const {
  return enabled_ && now >= last_event_ &&
         (health_.connection == Connection::Offline ||
          (health_.connection == Connection::Backoff && now >= retry_at_)) &&
         !subscription_.Pending(now, true).empty();
}
bool AisStreamSession::Connecting(vessel::Time now) {
  if (!NeedsConnection(now) || !Advance(now))
    return false;
  State(Connection::Connecting, now);
  return true;
}
void AisStreamSession::Opened(vessel::Time now) {
  if (!Advance(now) || !enabled_ ||
      health_.connection != Connection::Connecting)
    return;
  new_connection_ = true;
  health_.subscription_confirmed = false;
  State(Connection::Subscribing, now);
}
void AisStreamSession::CredentialMissing(vessel::Time now) {
  if (!Advance(now) || !enabled_)
    return;
  health_.subscription_confirmed = false;
  State(Connection::CredentialMissing, now);
}
void AisStreamSession::RetryCredentials(vessel::Time now) {
  if (!Advance(now) || !enabled_)
    return;
  failures_ = 0;
  retry_at_ = {};
  health_.subscription_confirmed = false;
  State(Connection::Offline, now);
}
std::vector<BoundingBox>
AisStreamSession::PendingSubscription(vessel::Time now) const {
  if (!enabled_ || now < last_event_ ||
      (health_.connection != Connection::Subscribing &&
       health_.connection != Connection::Connected))
    return {};
  return subscription_.Pending(now, new_connection_);
}
bool AisStreamSession::SubscriptionSent(vessel::Time now) {
  if (PendingSubscription(now).empty() || !Advance(now))
    return false;
  subscription_.Sent(now);
  sent_at_ = now;
  new_connection_ = false;
  health_.subscription_confirmed = false;
  State(Connection::Subscribing, now);
  return true;
}
void AisStreamSession::Receive(const std::string &message, vessel::Time now,
                               std::chrono::system_clock::time_point wall,
                               unsigned entropy) {
  if (!Advance(now) || !enabled_ ||
      (health_.connection != Connection::Subscribing &&
       health_.connection != Connection::Connected))
    return;
  const auto decoded = DecodeAisStream(message, now, wall);
  if (decoded.kind == DecodeKind::ServiceError) {
    Increment(health_.rejected);
    Disconnected(now, entropy);
  } else if (decoded.kind == DecodeKind::Confirmation) {
    if (health_.connection != Connection::Subscribing || new_connection_) {
      Increment(health_.rejected);
      return;
    }
    subscription_.Confirmed();
    health_.subscription_confirmed = subscription_.IsConfirmed();
    compression_ = decoded.compression;
    if (health_.subscription_confirmed) {
      failures_ = 0;
      State(Connection::Connected, now);
    }
  } else if (decoded.kind == DecodeKind::Reports) {
    if (!health_.subscription_confirmed) {
      Increment(health_.rejected);
      return;
    }
    bool accepted = false;
    if (decoded.position && cache_.Observe(*decoded.position, now)) {
      accepted = true;
      health_.last_position =
          std::max(health_.last_position, decoded.position->observed_at);
    }
    if (decoded.static_data && cache_.Observe(*decoded.static_data, now))
      accepted = true;
    Increment(accepted ? health_.accepted : health_.rejected);
  } else if (decoded.kind == DecodeKind::Invalid)
    Increment(health_.rejected);
}
void AisStreamSession::Disconnected(vessel::Time now, unsigned entropy) {
  if (!Advance(now) || !enabled_ || health_.connection == Connection::Backoff ||
      health_.connection == Connection::CredentialMissing ||
      health_.connection == Connection::Offline)
    return;
  const auto delay = ReconnectDelay(failures_, entropy);
  failures_ = std::min(10u, failures_ + 1);
  retry_at_ = After(now, delay);
  health_.subscription_confirmed = false;
  compression_ = false;
  Increment(health_.reconnects);
  State(Connection::Backoff, now);
}
void AisStreamSession::Tick(vessel::Time now, unsigned entropy) {
  if (!Advance(now) || !enabled_)
    return;
  if ((health_.connection == Connection::Connecting &&
       now >= After(health_.state_since, vessel::Duration(12000))) ||
      (health_.connection == Connection::Subscribing &&
       now >= After(new_connection_ ? health_.state_since : sent_at_,
                    vessel::Duration(new_connection_ ? 2500 : 10000))))
    Disconnected(now, entropy);
}
bool AisStreamSession::ShouldClose() const {
  return !enabled_ || health_.connection == Connection::Backoff ||
         health_.connection == Connection::CredentialMissing ||
         health_.connection == Connection::Offline;
}
ProviderSnapshot AisStreamSession::Read(vessel::Time now) const {
  ProviderSnapshot snapshot;
  snapshot.health = health_;
  snapshot.health.compression_enabled = compression_;
  snapshot.health.retry_at = retry_at_;
  if (enabled_)
    snapshot.targets = cache_.Read(now);
  return snapshot;
}
} // namespace opennav::ais

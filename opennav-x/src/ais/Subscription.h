#pragma once
#include "vessel/VesselState.h"
#include <vector>

namespace opennav::ais {
struct Viewport {
  double south = 0, north = 0, west = 0, east = 0;
};
struct BoundingBox {
  double south, west, north, east;
};
// Geographic filtering only. No route distances or navigation geometry.
std::vector<BoundingBox> SubscriptionArea(Viewport viewport);
class SubscriptionPolicy {
 public:
  bool ObserveViewport(Viewport viewport);
  // Initial connection always sends the latest complete subscription promptly.
  // Thereafter coalesce changes; a UI read never modifies the sent timestamp.
  std::vector<BoundingBox> Pending(vessel::Time now, bool new_connection) const;
  void Sent(vessel::Time at);
  void Confirmed() { confirmed_ = awaiting_confirmation_; awaiting_confirmation_ = false; }
  bool IsConfirmed() const { return confirmed_; }
 private:
  std::vector<BoundingBox> desired_, sent_;
  vessel::Time last_sent_{};
  bool confirmed_ = false, awaiting_confirmation_ = false;
};
// Injected entropy makes deterministic CI possible. Retry delay is 2s..5min
// plus up to 25% jitter; the tenth consecutive failure enters a 15min cooldown.
vessel::Duration ReconnectDelay(unsigned consecutive_failures, unsigned entropy);
} // namespace opennav::ais

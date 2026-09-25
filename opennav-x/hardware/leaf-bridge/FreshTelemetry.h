#pragma once
// Boat-side firmware helper only. No EV-CAN decoding is compiled into OpenNav.
#include <math.h>
#include <stdint.h>

namespace boat_bridge {
class FreshTelemetry {
public:
  enum Group { PowerSoc = 0, Rpm = 1, Temperature = 2, GearRegen = 3 };
  static const uint32_t expiry_ms = 2500;
  void Observe(Group group, uint32_t now) {
    if (unsigned(group) >= 4)
      return;
    seen_[group] = true;
    times_[group] = now;
  }
  uint8_t Mask(uint32_t now) {
    uint8_t result = 0;
    for (unsigned i = 0; i < 4; ++i) {
      // Latch expiry so a counter wrap after an extended dropout cannot revive
      // old telemetry. Called by every periodic snapshot under the data lock.
      if (seen_[i] && uint32_t(now - times_[i]) >= expiry_ms)
        seen_[i] = false;
      if (seen_[i])
        result |= uint8_t(1u << i);
    }
    return result;
  }
  template <class T> T Snapshot(const T &source, uint32_t now) {
    T result = source;
    result.expiryMask = Mask(now);
    if (!(result.expiryMask & 1))
      result.packV = result.packA = result.socPct = NAN;
    if (!(result.expiryMask & 2)) {
      result.rpm = 0;
      result.rpmValid = false;
    }
    if (!(result.expiryMask & 4))
      result.motorTempC = NAN;
    if (!(result.expiryMask & 8)) {
      result.gear = 0;
      result.regen = 0xff;
    }
    return result;
  }

private:
  bool seen_[4] = {false, false, false, false};
  uint32_t times_[4] = {0, 0, 0, 0};
};
} // namespace boat_bridge

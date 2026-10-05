#pragma once
#include "vessel/VesselState.h"
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace opennav::integration {
namespace pilot_traffic_detail {
inline void Increment(std::uint64_t &counter) {
  if (counter != (std::numeric_limits<std::uint64_t>::max)()) ++counter;
}
} // namespace pilot_traffic_detail

// Passive envelope diagnostics only. A counted packet proves neither a device
// identity nor valid physical feedback. No payload bytes or values are retained.
class PilotTrafficDiagnostics {
public:
  struct Source {
    std::string interface_id;
    std::uint32_t pgn = 0;
    std::uint8_t source = 255;
    std::uint64_t accepted_count = 0;
    vessel::Time last_observed{};
  };
  struct Rejections {
    std::uint64_t unsupported_pgn = 0, invalid_interface = 0;
    std::uint64_t invalid_type = 0, invalid_length = 0, pgn_mismatch = 0;
    std::uint64_t invalid_source = 0, invalid_time = 0, future = 0, stale = 0;
    std::uint64_t out_of_order = 0, capacity = 0;
  };
  struct Snapshot {
    std::vector<Source> sources;
    std::uint64_t accepted_count = 0;
    Rejections rejected;
  };

  bool Observe(const std::string &iface, std::uint32_t expected_pgn,
               const std::vector<std::uint8_t> &envelope,
               vessel::Time observed_at, vessel::Time now) {
    const unsigned data_length = DataLength(expected_pgn);
    if (!data_length) return Reject(snapshot_.rejected.unsupported_pgn);
    if (iface.empty() || iface.size() > 200)
      return Reject(snapshot_.rejected.invalid_interface);
    for (const unsigned char c : iface)
      if (c < 32 || c == 127) return Reject(snapshot_.rejected.invalid_interface);
    if (envelope.empty()) return Reject(snapshot_.rejected.invalid_length);
    // 0x94 transmit echoes are not received physical-bus observations.
    if (envelope[0] != 0x93) return Reject(snapshot_.rejected.invalid_type);
    // Pinned OpenCPN serial RX: 13 header bytes + data + checksum. Byte 1
    // declares priority through data (11 + data length), byte 12 data only.
    if (envelope.size() != 14 + data_length ||
        envelope[12] != data_length || envelope[1] != 11 + data_length)
      return Reject(snapshot_.rejected.invalid_length);
    const auto pgn = std::uint32_t(envelope[3]) |
                     (std::uint32_t(envelope[4]) << 8) |
                     (std::uint32_t(envelope[5]) << 16);
    if (pgn != expected_pgn) return Reject(snapshot_.rejected.pgn_mismatch);
    const auto source = envelope[7];
    if (source >= 254) return Reject(snapshot_.rejected.invalid_source);
    if (observed_at < vessel::Time{} || now < vessel::Time{})
      return Reject(snapshot_.rejected.invalid_time);
    if (observed_at > now) return Reject(snapshot_.rejected.future);
    if (now - observed_at >= std::chrono::seconds(3))
      return Reject(snapshot_.rejected.stale);
    for (auto &bucket : snapshot_.sources) {
      if (bucket.interface_id != iface || bucket.pgn != pgn ||
          bucket.source != source) continue;
      if (observed_at <= bucket.last_observed)
        return Reject(snapshot_.rejected.out_of_order);
      bucket.last_observed = observed_at;
      pilot_traffic_detail::Increment(bucket.accepted_count);
      pilot_traffic_detail::Increment(snapshot_.accepted_count);
      return true;
    }
    if (snapshot_.sources.size() >= 64) return Reject(snapshot_.rejected.capacity);
    snapshot_.sources.push_back({iface, pgn, source, 1, observed_at});
    pilot_traffic_detail::Increment(snapshot_.accepted_count);
    return true;
  }

  Snapshot GetSnapshot() const { return snapshot_; }

private:
  static unsigned DataLength(std::uint32_t pgn) {
    switch (pgn) {
    case 60928: case 65379: case 65360: case 127250: case 65359: return 8;
    case 126720: return 13;
    default: return 0;
    }
  }
  static bool Reject(std::uint64_t &counter) {
    pilot_traffic_detail::Increment(counter);
    return false;
  }
  Snapshot snapshot_;
};
} // namespace opennav::integration

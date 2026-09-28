#pragma once
#include "ais/Subscription.h"
#include "ais/TargetCache.h"
#include <chrono>
#include <string>
#include <string_view>

namespace opennav::ais {
enum class DecodeKind { Invalid, Ignored, Confirmation, Reports, ServiceError };
struct DecodedMessage {
  DecodeKind kind = DecodeKind::Invalid;
  std::optional<PositionReport> position;
  std::optional<StaticReport> static_data;
  bool compression = false;
};
// Pure, bounded decoder. Error results never retain the received JSON or text.
// The transport must ALSO bound frames/decompression before calling this.
DecodedMessage DecodeAisStream(const std::string &json, vessel::Time received,
                               std::chrono::system_clock::time_point wall_now);
// The returned string contains a secret: send promptly, erase, never log it.
std::optional<std::string>
AisStreamSubscription(std::string_view key,
                      const std::vector<BoundingBox> &boxes,
                      const std::vector<int> &mmsis = {});
} // namespace opennav::ais

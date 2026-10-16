#pragma once
#include "integration/OpenCPNPilot.h"
#include "integration/RouteProgressInput.h"

class Routeman;

namespace opennav::integration {
// Main-thread owner of copied route identity and serialization tokens. It never
// calculates guidance or retains a Routeman/route/point/driver pointer.
class NativeRouteOutput {
 public:
  using AllocateTokens = std::function<std::optional<std::uint16_t>()>;
  explicit NativeRouteOutput(AllocateTokens allocate) : allocate_(std::move(allocate)) {}
  bool Current(const RouteRead &read, vessel::Time now) const;
  void Update(Routeman &manager, const RouteRead &read,
              const vessel::RouteProgress &progress, OpenCPNPilot &pilot,
              vessel::Time now);
 private:
  AllocateTokens allocate_;
  std::optional<RouteRead> accepted_;
  std::optional<NavigationOutputSession> session_;
  std::uint64_t revision_ = 0;
  std::uint16_t origin_token_ = 0;
};
} // namespace opennav::integration

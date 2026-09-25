#pragma once
#include "vessel/VesselState.h"
#include <map>
#include <vector>

namespace opennav::integration {
struct N2kIdentity {
  std::string interface_id, name;
  unsigned address = 255;
  vessel::Time observed_at{};
};
enum class ClaimResult { Ignored, Unchanged, Changed, Conflict };
// Owned, bounded main-thread address claims. No interpretation of OpenCPN's
// synthetic NavAddr2000 label as a real NMEA NAME. Unknown identities remain
// explicitly address-based; no fake persistent identity is invented.
class N2kSourceIdentity {
public:
  ClaimResult Observe(const std::string &interface_id, unsigned address,
                      const std::vector<unsigned char> &data,
                      vessel::Time observed, vessel::Time now);
  std::string Label(const std::string &interface_id, unsigned address) const;
  std::vector<N2kIdentity> Observations() const;
  void Clear() { claims_.clear(); }

private:
  struct Claim {
    std::uint64_t name = 0;
    vessel::Time observed{};
    bool conflict = false;
  };
  std::map<std::pair<std::string, unsigned>, Claim> claims_;
};
} // namespace opennav::integration

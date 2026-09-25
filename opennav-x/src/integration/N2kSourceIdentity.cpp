#include "integration/N2kSourceIdentity.h"
#include <iomanip>
#include <sstream>

namespace opennav::integration {
namespace {
std::string Hex(std::uint64_t n) {
  std::ostringstream s;
  s << std::hex << std::setfill('0') << std::setw(16) << n;
  return s.str();
}
} // namespace
ClaimResult N2kSourceIdentity::Observe(const std::string &iface,
                                       unsigned address,
                                       const std::vector<unsigned char> &data,
                                       vessel::Time at, vessel::Time now) {
  if (iface.empty() || iface.size() > 200 || address >= 254 ||
      data.size() != 8 || at < vessel::Time{} || at > now ||
      now - at >= std::chrono::seconds(3))
    return ClaimResult::Ignored;
  std::uint64_t name = 0;
  for (unsigned i = 0; i < 8; ++i)
    name |= std::uint64_t(data[i]) << (i * 8);
  if (!name || name == UINT64_MAX)
    return ClaimResult::Ignored;
  const auto key = std::make_pair(iface, address);
  const auto old = claims_.find(key);
  if (old != claims_.end() && at <= old->second.observed)
    return ClaimResult::Ignored;
  if (old == claims_.end() && claims_.size() >= 256)
    return ClaimResult::Ignored;
  bool conflict = old != claims_.end() && old->second.conflict;
  // Duplicate NAME remains ambiguous. Do not silently reassign a pack identity.
  for (auto &p : claims_)
    if (p.first.first == iface && p.first.second != address &&
        p.second.name == name) {
      p.second.conflict = true;
      conflict = true;
    }
  const bool changed = old == claims_.end() || old->second.name != name;
  claims_[key] = {name, at, conflict};
  return conflict  ? ClaimResult::Conflict
         : changed ? ClaimResult::Changed
                   : ClaimResult::Unchanged;
}
std::string N2kSourceIdentity::Label(const std::string &iface,
                                     unsigned address) const {
  const auto p = claims_.find({iface, address});
  if (p == claims_.end())
    return iface + "/address-only";
  if (p->second.conflict)
    return {}; // Do not admit observations with ambiguous identity.
  return iface + "/NAME-" + Hex(p->second.name);
}
std::vector<N2kIdentity> N2kSourceIdentity::Observations() const {
  std::vector<N2kIdentity> result;
  for (const auto &p : claims_)
    if (!p.second.conflict)
      result.push_back({p.first.first, Hex(p.second.name), p.first.second,
                        p.second.observed});
  return result;
}
} // namespace opennav::integration

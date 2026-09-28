#include "ais/Provider.h"
#include <set>

namespace opennav::ais {
AisFeeds Aggregate(vessel::AisState onboard, ProviderSnapshot online) {
  AisFeeds feeds{std::move(onboard), std::move(online), {}};
  feeds.display = feeds.onboard;
  feeds.display.source = "OpenCPN AIS + supplemental AISSTREAM_ONLINE";
  feeds.display.available = feeds.onboard.available || feeds.online.targets.available;
  // An over-bound local snapshot must not be disguised by a small online set.
  if (feeds.onboard.targets.size() > 2000 || feeds.onboard.simulated ||
      feeds.online.targets.simulated) {
    feeds.display.targets.clear(); feeds.display.available = false; return feeds;
  }
  std::set<int> identities;
  for (const auto &target : feeds.onboard.targets) identities.insert(target.mmsi);
  // Preserve a local target as a whole, even stale/lost. Never transplant an
  // online position beneath local range/CPA/alarms or refresh its observation.
  // Duplicate local identities remain duplicates so existing selection rejects
  // ambiguity. Online becomes eligible only after the local model removes it.
  std::set<int> online_seen, ambiguous;
  if (feeds.online.targets.targets.size() <= 2000) {
    for (const auto &target : feeds.online.targets.targets)
      if (!online_seen.insert(target.mmsi).second) ambiguous.insert(target.mmsi);
    for (const auto &target : feeds.online.targets.targets) {
      if (feeds.display.targets.size() == 2000) break;
      if (target.origin != vessel::AisOrigin::AisStreamOnline || target.mmsi < 100000000 ||
          target.mmsi > 999999999 || identities.count(target.mmsi) || ambiguous.count(target.mmsi)) continue;
      auto copy = target;
      copy.upstream_alarm = false;
      copy.range_nm = {}; copy.bearing_true_deg = {}; copy.cpa_nm = {}; copy.tcpa_minutes = {};
      feeds.display.targets.push_back(std::move(copy));
      identities.insert(target.mmsi);
    }
  }
  return feeds;
}
} // namespace opennav::ais

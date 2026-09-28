#include "ais/ChartTargets.h"
#include <cmath>
#include <map>
#include <tuple>

namespace opennav::ais {
namespace {
bool Measured(const vessel::Sample &s, double low, double high) {
  return s.value && std::isfinite(*s.value) && *s.value >= low &&
         *s.value <= high && s.validity == vessel::Validity::Measured &&
         s.source == OnlineSource;
}
std::optional<double> Current(const vessel::Sample &s, vessel::Time now,
                              double low, double high) {
  if (!Measured(s, low, high)) return {};
  const auto age = Age(s.observed_at, now);
  return age == TargetAge::Live || age == TargetAge::Aging ? s.value
                                                         : std::nullopt;
}
} // namespace
bool ChartTarget::operator==(const ChartTarget &o) const {
  return std::tie(mmsi, latitude, longitude, direction_true, age, selected, observed_at) ==
         std::tie(o.mmsi, o.latitude, o.longitude, o.direction_true, o.age,
                  o.selected, o.observed_at);
}
std::vector<ChartTarget> OnlineChartTargets(const vessel::AisState &display,
                                          vessel::Time now, int selected) {
  std::vector<ChartTarget> result;
  if (!display.available || display.simulated || display.targets.size() > 2000)
    return result;
  std::map<int, unsigned> counts;
  for (const auto &target : display.targets) ++counts[target.mmsi];
  for (const auto &t : display.targets) {
    if (t.origin != vessel::AisOrigin::AisStreamOnline ||
        t.source != OnlineSource || t.mmsi < 100000000 ||
        t.mmsi > 999999999 || counts[t.mmsi] != 1 || t.doubtful ||
        (t.time_basis != vessel::AisTimeBasis::OnlineService &&
         t.time_basis != vessel::AisTimeBasis::OnlineReceipt) ||
        !Measured(t.latitude_deg, -90, 90) ||
        !Measured(t.longitude_deg, -180, 180) ||
        t.latitude_deg.observed_at != t.longitude_deg.observed_at ||
        t.observed_at != t.latitude_deg.observed_at)
      continue;
    const auto age = Age(t.observed_at, now);
    if (age == TargetAge::Invalid || age == TargetAge::Expired) continue;
    const bool current = age == TargetAge::Live || age == TargetAge::Aging;
    if (current && (!t.active || t.lost)) continue;
    auto direction = current ? Current(t.heading_true_deg, now, 0, 359)
                             : std::nullopt;
    if (!direction && current) {
      const auto speed = Current(t.sog_kn, now, 0, 102.2);
      if (speed && *speed >= 0.5)
        direction = Current(t.cog_deg, now, 0, 359.999999);
    }
    // A stale/lost position has no current direction or selection highlight.
    // No direction is rendered as an unoriented mark, never a northbound ship.
    result.push_back({t.mmsi, *t.latitude_deg.value, *t.longitude_deg.value,
                      direction, age, current && t.mmsi == selected, t.observed_at});
  }
  return result;
}
std::optional<ChartTarget> CurrentChartMark(ChartTarget mark, vessel::Time now) {
  const auto age = Age(mark.observed_at, now);
  if (age == TargetAge::Invalid || age == TargetAge::Expired) return {};
  // A clock reversal cannot make an older retained state fresh again.
  if (age < mark.age) return {};
  mark.age = age;
  if (age == TargetAge::Stale || age == TargetAge::Lost) {
    mark.direction_true.reset(); mark.selected = false;
  }
  return mark;
}
const char *OnlineAgeLabel(TargetAge age) {
  switch (age) {
  case TargetAge::Live: return "Live";
  case TargetAge::Aging: return "Aging";
  case TargetAge::Stale: return "Stale";
  case TargetAge::Lost: return "Lost";
  default: return "Unavailable";
  }
}
} // namespace opennav::ais

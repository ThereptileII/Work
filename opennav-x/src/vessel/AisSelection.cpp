#include "vessel/AisSelection.h"
#include <cmath>
namespace opennav::vessel {
bool AisSelection::CurrentPosition(const AisTarget &t, Time now) {
  if (t.mmsi <= 0 || t.mmsi > 999999999 || !t.active || t.lost || t.doubtful)
    return false;
  const auto lat = Assess(t.latitude_deg, now),
             lon = Assess(t.longitude_deg, now);
  const auto good = [](const Assessment &a) {
    return a.value &&
           (a.quality == Quality::Live || a.quality == Quality::Aging);
  };
  return good(lat) && good(lon) && std::abs(*lat.value) <= 90 &&
         std::abs(*lon.value) <= 180 &&
         t.latitude_deg.observed_at == t.longitude_deg.observed_at &&
         t.latitude_deg.source == t.longitude_deg.source;
}
bool AisSelection::Select(int mmsi, const AisState &s, Time now) {
  Clear();
  if (!s.available || s.simulated || s.targets.size() > 2000)
    return false;
  unsigned matches = 0;
  for (const auto &t : s.targets)
    if (t.mmsi == mmsi) {
      ++matches;
      if (CurrentPosition(t, now))
        target_ = t;
    }
  if (matches != 1)
    Clear();
  return target_.has_value();
}
void AisSelection::Observe(const AisState &s, Time now) {
  if (!target_)
    return;
  const auto id = target_->mmsi;
  const auto previous = target_->latitude_deg.observed_at;
  if (Select(id, s, now) && target_->latitude_deg.observed_at < previous)
    Clear();
}
int AisSelection::Selected(Time now) const {
  return target_ && CurrentPosition(*target_, now) ? target_->mmsi : 0;
}
} // namespace opennav::vessel

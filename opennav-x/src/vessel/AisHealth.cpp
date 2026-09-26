#include "vessel/AisHealth.h"
#include "vessel/AisSelection.h"
#include <cmath>

namespace opennav::vessel {
AisReportHealth AssessAisReports(const AisState &state, Time now) {
  if (!state.available || state.targets.size() > 2000)
    return AisReportHealth::Unavailable;
  if (state.targets.empty()) return AisReportHealth::Empty;
  bool stale = false, lost = false;
  for (const auto &target : state.targets) {
    if (AisSelection::CurrentPosition(target, now))
      return AisReportHealth::Current;
    if (target.mmsi <= 0 || target.mmsi > 999999999) continue;
    lost |= target.lost;
    if (!target.active || target.lost || target.doubtful ||
        target.latitude_deg.validity != Validity::Measured ||
        target.longitude_deg.validity != Validity::Measured ||
        target.latitude_deg.source != target.longitude_deg.source ||
        target.latitude_deg.observed_at != target.longitude_deg.observed_at)
      continue;
    const auto lat = Assess(target.latitude_deg, now);
    const auto lon = Assess(target.longitude_deg, now);
    const auto retained = [](const Assessment &a) {
      return a.value && (a.quality == Quality::Live ||
                         a.quality == Quality::Aging ||
                         a.quality == Quality::Stale);
    };
    stale |= retained(lat) && retained(lon) && std::abs(*lat.value) <= 90 &&
             std::abs(*lon.value) <= 180 &&
             (lat.quality == Quality::Stale || lon.quality == Quality::Stale);
  }
  if (stale) return AisReportHealth::Stale;
  if (lost) return AisReportHealth::Lost;
  return AisReportHealth::Unusable;
}
const char *AisReportHealthName(AisReportHealth health) {
  switch (health) {
  case AisReportHealth::Unavailable: return "AIS unavailable";
  case AisReportHealth::Empty: return "No targets received";
  case AisReportHealth::Current: return "Targets current";
  case AisReportHealth::Stale: return "Targets stale";
  case AisReportHealth::Lost: return "Targets lost";
  case AisReportHealth::Unusable: return "Target data unavailable";
  }
  return "AIS unavailable";
}
} // namespace opennav::vessel

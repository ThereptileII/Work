#pragma once
#include "vessel/VesselState.h"
#include <cmath>
#include <ctime>
#include <wx/datetime.h>

namespace opennav::integration {
// Pinned AIS decoder and its expiry timer both use Now().MakeGMT()/MakeUTC()
// before GetTicks(). Those shifted local-wall ticks are NOT Unix timestamps.
// Convert elapsed time within that same clock domain, then retain the paired
// monotonic observation. Never re-date a retained report when the UI reads it.
inline std::optional<vessel::Time> AisObservationFromClock(
    std::time_t report, std::time_t upstream_now, int milliseconds,
    vessel::Time observed_now) {
  const double age = std::difftime(upstream_now, report) + milliseconds / 1000.0;
  if (report <= 0 || milliseconds < 0 || milliseconds > 999 ||
      !std::isfinite(age) || age < 0 || age >= 86400)
    return {};
  return observed_now - std::chrono::duration_cast<vessel::Clock::duration>(
                            std::chrono::duration<double>(age));
}
inline std::optional<vessel::Time> AisObservationAt(
    std::time_t report, wxDateTime local_now, vessel::Time observed_now) {
  if (!local_now.IsValid()) return {};
  local_now.MakeUTC();
  return AisObservationFromClock(report, local_now.GetTicks(),
                                  local_now.GetMillisecond(), observed_now);
}
} // namespace opennav::integration

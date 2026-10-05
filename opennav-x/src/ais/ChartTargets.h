#pragma once
#include "ais/TargetCache.h"

namespace opennav::ais {
// Presentation-only owned values. No CPA, collision risk, decoder or socket.
struct ChartTarget {
  int mmsi = 0;
  double latitude = 0, longitude = 0;
  std::optional<double> direction_true;
  TargetAge age = TargetAge::Invalid;
  bool selected = false;
  vessel::Time observed_at{};
  // Optional display identity, copied from validated static metadata. A name
  // never supplies position, motion, freshness or collision information.
  std::string name;
  bool operator==(const ChartTarget &other) const;
};
// Input is the aggregated display snapshot: onboard identities must suppress
// matching online symbols, including stale or ambiguous onboard reports.
std::vector<ChartTarget> OnlineChartTargets(const vessel::AisState &display,
                                          vessel::Time now, int selected);
const char *OnlineAgeLabel(TargetAge age);
// Recheck a retained mark at paint/hit-test time without renewing its timestamp.
std::optional<ChartTarget> CurrentChartMark(ChartTarget mark, vessel::Time now);
// Physical-pixel label placement; keep full labels within the viewport and
// clear of other target symbols/previous labels. Never shift their anchor.
struct ChartLabelBounds { int x, y, width, height; };
bool ChartLabelFits(const ChartLabelBounds &label, const ChartLabelBounds &viewport,
                    const std::vector<ChartLabelBounds> &occupied);
} // namespace opennav::ais

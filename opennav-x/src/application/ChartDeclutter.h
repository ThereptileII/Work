#pragma once
#include <cmath>

namespace opennav::application {
// SCRUM-317: one deterministic level-of-detail hierarchy for XNav-owned chart
// overlays, shared by Day/Dusk/Night. It only reduces secondary presentation
// (names, ordinals, provenance dots); it never hides a target, a waypoint, the
// own ship, an alarm or an active-route position. S-52 chart objects follow
// the chart's own SCAMIN/super-SCAMIN declutter, enforced in XNav mode.
enum class ChartDetail { Full, Reduced, Overview };

// Scale denominators (1:N). Coastal pilotage stays Full; passage planning
// is Reduced; anything wider is an overview.
constexpr double kReducedDetailScale = 150000;
constexpr double kOverviewDetailScale = 600000;

constexpr ChartDetail ChartDetailForScale(double denominator) {
  // Unknown/invalid scale keeps full detail rather than guessing.
  return !(denominator > 0) || denominator != denominator ? ChartDetail::Full
       : denominator >= kOverviewDetailScale ? ChartDetail::Overview
       : denominator >= kReducedDetailScale  ? ChartDetail::Reduced
                                             : ChartDetail::Full;
}
// Waypoint/mark names and online AIS names.
constexpr bool ShowSecondaryLabels(ChartDetail detail) {
  return detail == ChartDetail::Full;
}
// Route ordinals and full-size waypoint discs; overview uses compact markers.
constexpr bool ShowMarkerDetail(ChartDetail detail) {
  return detail != ChartDetail::Overview;
}
} // namespace opennav::application

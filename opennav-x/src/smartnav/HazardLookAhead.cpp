#include "smartnav/HazardLookAhead.h"
#include <cmath>

namespace opennav::smartnav {
namespace {
bool ValidConfig(const HazardConfiguration &c) {
  return std::isfinite(c.draft_m) && c.draft_m > 0 && c.draft_m <= 100 &&
         std::isfinite(c.safety_margin_m) && c.safety_margin_m >= 0 &&
         c.safety_margin_m <= 100 && std::isfinite(c.corridor_half_width_m) &&
         c.corridor_half_width_m > 0 && c.corridor_half_width_m <= 10000;
}
bool Point(const GeoPoint &p) {
  return std::isfinite(p.latitude_deg) && std::abs(p.latitude_deg) <= 90 &&
         std::isfinite(p.longitude_deg) && std::abs(p.longitude_deg) <= 180;
}
bool SameQuery(const PathCorridor &a, const PathCorridor &b) {
  if (a.route_id != b.route_id || a.revision_scope != b.revision_scope ||
      a.route_revision != b.route_revision || a.observed_at != b.observed_at ||
      a.position_observed_at != b.position_observed_at ||
      a.half_width_m != b.half_width_m || a.path.size() != b.path.size())
    return false;
  for (std::size_t i = 0; i < a.path.size(); ++i)
    if (a.path[i].latitude_deg != b.path[i].latitude_deg ||
        a.path[i].longitude_deg != b.path[i].longitude_deg)
      return false;
  return true;
}
} // namespace
CorridorEvidence
UnavailableChartCorridor::Inspect(const PathCorridor &path) const {
  return {
      path,
      Coverage::Unavailable,
      "OpenCPN corridor-query integration unavailable",
      "Rendered object picking cannot establish complete future-path coverage",
      {}};
}
std::optional<PathCorridor> BuildCorridor(const vessel::VesselState &s,
                                          const HazardConfiguration &c,
                                          vessel::Time now) {
  if (!ValidConfig(c) || s.simulated || !s.navigation.route)
    return {};
  const auto &r = *s.navigation.route;
  if (!vessel::AssessRoute(r, now).remaining_distance_nm ||
      r.remaining_steps.empty() || r.remaining_steps.size() > 10000)
    return {};
  const auto &lat = s.navigation.latitude_deg;
  const auto &lon = s.navigation.longitude_deg;
  if (!lat.value || !lon.value || lat.validity != vessel::Validity::Measured ||
      lon.validity != vessel::Validity::Measured ||
      lat.observed_at != lon.observed_at ||
      lat.observed_at != r.position_observed_at || lat.source != lon.source ||
      lat.source != r.position_source)
    return {};
  PathCorridor p{r.route_id,
                 r.revision_scope,
                 r.route_revision,
                 {},
                 c.corridor_half_width_m,
                 r.observed_at,
                 *r.position_observed_at};
  p.path.push_back({*lat.value, *lon.value});
  for (const auto &step : r.remaining_steps)
    p.path.push_back({step.latitude_deg, step.longitude_deg});
  for (const auto &point : p.path)
    if (!Point(point))
      return {};
  return p;
}
HazardAdvice LookAhead(const PathCorridor &p, const HazardConfiguration &c,
                       const IChartCorridor &provider, vessel::Time now) {
  HazardAdvice a;
  a.message = "Chart look-ahead unavailable; absence of warnings is not proof "
              "of safe water";
  if (!ValidConfig(c) || p.route_id.empty() || p.revision_scope.empty() ||
      !p.route_revision || p.path.size() < 2 || p.path.size() > 10001 ||
      p.half_width_m != c.corridor_half_width_m || p.observed_at > now ||
      p.position_observed_at > p.observed_at ||
      now - p.observed_at >= std::chrono::seconds(5) ||
      now - p.position_observed_at >= std::chrono::seconds(5))
    return a;
  for (const auto &point : p.path)
    if (!Point(point))
      return a;
  const auto evidence = provider.Inspect(p);
  if (!SameQuery(p, evidence.query) || evidence.source.empty() ||
      evidence.coverage == Coverage::Unavailable)
    return a;
  a.coverage = evidence.coverage;
  a.source = evidence.source;
  for (const auto &object : evidence.objects) {
    if (object.object_id.empty() || object.chart_source.empty() ||
        object.datum.empty() ||
        (object.minimum_charted_depth_m &&
         !std::isfinite(*object.minimum_charted_depth_m))) {
      a.coverage = Coverage::Partial;
      continue;
    }
    if (object.obstruction || !object.minimum_charted_depth_m ||
        *object.minimum_charted_depth_m <= c.draft_m + c.safety_margin_m)
      a.potential_hazards.push_back(object);
  }
  a.message = a.potential_hazards.empty()
                  ? "No hazards identified in returned chart information; this "
                    "is not proof of safe water"
                  : "Potential charted hazards in future corridor; review "
                    "chart and conditions";
  a.message += "; charted depths are not measured clearance; tide/datum and "
               "survey uncertainty remain";
  if (!evidence.uncertainty.empty())
    a.message += "; " + evidence.uncertainty;
  return a;
}
} // namespace opennav::smartnav

#pragma once
#include "vessel/RouteProgress.h"
#include <limits>
#include <vector>

namespace opennav::smartnav {
struct GeoPoint {
  double latitude_deg = 0, longitude_deg = 0;
};
struct PathCorridor {
  std::string route_id, revision_scope;
  std::uint64_t route_revision = 0;
  std::vector<GeoPoint> path;
  double half_width_m = 0;
  vessel::Time observed_at{}, position_observed_at{};
};
enum class Coverage { Unavailable, Partial, Complete };
struct ChartedHazard {
  std::string object_id, description, chart_source, datum;
  std::optional<double> minimum_charted_depth_m;
  bool obstruction = false;
};
struct CorridorEvidence {
  PathCorridor query;
  Coverage coverage = Coverage::Unavailable;
  std::string source, uncertainty;
  std::vector<ChartedHazard> objects;
};
// The provider owns spatial intersection and ENC semantics. Point-picking a
// rendered viewport is not a complete corridor query. No live provider yet.
class IChartCorridor {
public:
  virtual ~IChartCorridor() = default;
  virtual CorridorEvidence Inspect(const PathCorridor &path) const = 0;
};
class UnavailableChartCorridor final : public IChartCorridor {
public:
  CorridorEvidence Inspect(const PathCorridor &path) const override;
};
struct HazardConfiguration {
  double draft_m = std::numeric_limits<double>::quiet_NaN();
  double safety_margin_m = std::numeric_limits<double>::quiet_NaN();
  double corridor_half_width_m = std::numeric_limits<double>::quiet_NaN();
};
struct HazardAdvice {
  Coverage coverage = Coverage::Unavailable;
  std::vector<ChartedHazard> potential_hazards;
  std::string message, source;
  bool advisory_only = true;
};
std::optional<PathCorridor> BuildCorridor(const vessel::VesselState &state,
                                          const HazardConfiguration &config,
                                          vessel::Time now);
HazardAdvice LookAhead(const PathCorridor &path,
                       const HazardConfiguration &config,
                       const IChartCorridor &provider, vessel::Time now);
} // namespace opennav::smartnav

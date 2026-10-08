#pragma once
// Builds the bounded forecast query (SCRUM-330/331 data side) from copied
// application-thread inputs. Priority: vessel, route samples, chart grid.
#include "weather/Weather.h"
#include <optional>
#include <vector>

namespace opennav::weather {
constexpr std::size_t kMaxRouteSamples = 16;
constexpr std::size_t kMaxGridPerAxis = 6;
constexpr std::chrono::hours kForecastHorizon{48};

struct ChartBox {
  double min_lat = 0, max_lat = 0, min_lon = 0, max_lon = 0;  // lon may be unwrapped
};
struct QueryInputs {
  std::optional<Coordinate> vessel;  // Fresh fix only; never estimated.
  std::vector<Coordinate> route;     // Active route waypoints, in order.
  std::optional<ChartBox> chart;
  WallTime now{};
};
// Route: up to kMaxRouteSamples points evenly spaced along the polyline
// (ends included). Grid: a coarse lattice anchored to multiples of a
// scale-dependent step (0.25° .. 45°) so most small pans keep the same points.
ForecastQuery BuildForecastQuery(const QueryInputs &inputs);
std::vector<Coordinate> SampleRoute(const std::vector<Coordinate> &route, std::size_t count);
std::vector<Coordinate> ChartGrid(const ChartBox &box, std::size_t budget);
} // namespace opennav::weather

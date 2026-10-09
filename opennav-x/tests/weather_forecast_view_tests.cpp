// SCRUM-327..331: pure forecast-wind presentation helpers.
#include "weather/ForecastView.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

using namespace opennav::weather;
using namespace std::chrono_literals;
namespace {
void Check(bool value, const char *why) { if (!value) throw std::runtime_error(why); }
bool Near(double a, double b, double eps = 1e-6) { return std::abs(a - b) <= eps; }
const WallTime T0 = WallTime{} + std::chrono::hours(500000);
ForecastWind Wind(double lat, double lon, WallTime valid, double mps = 5, double from = 230) {
  ForecastWind w;
  w.requested = {lat, lon};
  w.valid_time = valid;
  w.model_run = T0 - 3h;
  w.speed_mps = mps;
  w.direction_from_true_deg = from;
  return w;
}
ForecastSnapshot Snapshot(ForecastState state = ForecastState::Live) {
  ForecastSnapshot s;
  s.state = state;
  s.model = "gfs";
  s.fetched_at = T0 - 10min;
  s.model_run = T0 - 3h - 5min;
  for (int h = 0; h <= 3; ++h) {
    s.winds.push_back(Wind(57.0, 16.0, T0 + std::chrono::hours(h), 5 + h, 230));
    s.winds.push_back(Wind(57.5, 16.5, T0 + std::chrono::hours(h), 8 + h, 200));
  }
  return s;
}
} // namespace

int main() {
  try {
    // Knots conversion and labels.
    Check(Near(KnotsFromMps(1), 1.9438444924406046), "m/s to kn uses 1852 m");
    Check(Near(KnotsFromMps(0), 0), "calm stays zero");
    Check(std::string(CompassPoint(230)) == "SW" && std::string(CompassPoint(359)) == "N" &&
          std::string(CompassPoint(-90)) == "W", "16-point names wrap");
    Check(WindLabel(Wind(0, 0, T0, 7.2, 230)) == "14 kn from 230\xC2\xB0 SW",
          "Wind label states speed in kn and the FROM direction");

    // Valid time selection.
    auto s = Snapshot();
    const auto times = ValidTimes(s);
    Check(times.size() == 4 && times.front() == T0 && times.back() == T0 + 3h,
          "Distinct sorted valid times");
    Check(*NearestValidTime(times, T0 + 40min) == T0 + 1h, "Nearest valid time");
    Check(*NearestValidTime(times, T0 + 30min) == T0, "Exact tie prefers earlier step");
    Check(*NearestValidTime(times, T0 + 99h) == T0 + 3h, "Beyond range clamps to last");
    Check(!NearestValidTime({}, T0), "No steps, no time");
    Check(*ResolveDisplayTime(times, std::nullopt, T0 + 5min) == T0, "Default is nearest to now");
    Check(*ResolveDisplayTime(times, T0 + 2h, T0) == T0 + 2h, "User step kept while present");
    Check(*ResolveDisplayTime(times, T0 + 2h + 20min, T0) == T0 + 2h,
          "Removed user step falls back to nearest available");
    Check(*StepDisplayTime(times, std::nullopt, T0, 1) == T0 + 1h, "Step forward");
    Check(*StepDisplayTime(times, T0 + 3h, T0, 1) == T0 + 3h, "Step clamps at last");
    Check(*StepDisplayTime(times, T0, T0, -1) == T0, "Step clamps at first");
    Check(StepLabel(T0 + 10min, T0) == "Now" && StepLabel(T0 + 3h, T0) == "+3 h" &&
          StepLabel(T0 + 48h, T0) == "+48 h" && StepLabel(T0 - 2h, T0) == "\xE2\x88\x92" "2 h",
          "Step labels relative to now");
    {
      auto many = Snapshot();
      many.winds.clear();
      for (int h = 0; h < 80; ++h) many.winds.push_back(Wind(57, 16, T0 + std::chrono::hours(h)));
      Check(ValidTimes(many).size() == kMaxForecastSteps, "Steps bounded");
    }

    // Live / stale / unavailable labelling.
    Check(DisplayState(s, T0) == ForecastDisplay::Live, "Fresh Live is live");
    Check(ProvenanceLabel(s, T0) == "FORECAST \xC2\xB7 GRIBstream GFS \xC2\xB7 run 3 h 05 min old",
          "Live label carries FORECAST, provider/model and run age");
    Check(DisplayState(s, T0 + 2h) == ForecastDisplay::Stale,
          "Live state older than kLiveFor is shown stale, never live");
    Check(ProvenanceLabel(s, T0 + 2h).rfind("STALE FORECAST", 0) == 0 &&
          ProvenanceLabel(s, T0 + 2h).find("fetched 2 h 10 min ago") != std::string::npos,
          "Stale label states fetch age");
    auto no_fetch = s; no_fetch.fetched_at.reset();
    Check(DisplayState(no_fetch, T0) == ForecastDisplay::Stale, "Unknown fetch time is not live");
    Check(DisplayState(Snapshot(ForecastState::Stale), T0) == ForecastDisplay::Stale, "Stale state");
    Check(DisplayState(s, T0 + 25h) == ForecastDisplay::Unavailable, "Expired cache unavailable");
    Check(DisplayState(Snapshot(ForecastState::Disabled), T0) == ForecastDisplay::Unavailable &&
          DisplayState(Snapshot(ForecastState::Error), T0) == ForecastDisplay::Unavailable,
          "Disabled/error never displayed");
    ForecastSnapshot empty; empty.state = ForecastState::Live; empty.fetched_at = T0;
    Check(DisplayState(empty, T0) == ForecastDisplay::Unavailable, "No values, nothing to show");
    Check(UnavailableReason(Snapshot(ForecastState::NoCredential), T0).find("token") != std::string::npos,
          "Missing credential explained");
    Check(AgeLabel(30s) == "under 1 min" && AgeLabel(12min) == "12 min" && AgeLabel(26h) == "1 d 2 h",
          "Age labels");

    // Coverage lookup.
    auto at = WindNear(s, {57.01, 16.01}, T0 + 1h);
    Check(at && Near(at->speed_mps, 6), "Nearest covered point at the chosen step");
    Check(!WindNear(s, {58.5, 16.0}, T0), "Far point has no coverage");
    Check(!WindNear(s, {57.0, 16.0}, T0 + 6h), "Time beyond the forecast has no coverage");
    Check(WindNear(s, {57.0, 16.0}, T0 + 3h + 80min).has_value(), "Within time gap still covered");
    auto grid = s; grid.winds[0].grid = Coordinate{57.2, 16.0};
    Check(WindNear(grid, {57.2, 16.0}, T0).has_value(), "Grid coordinate also counts");

    // Arrow decimation: deterministic, spaced, bounded.
    std::vector<ScreenPoint> points;
    for (int y = 0; y < 20; ++y)
      for (int x = 0; x < 20; ++x) points.push_back({x * 20.0, y * 20.0});
    const auto full = DecimateArrows(points, 400, 400, ArrowSpacingPx(opennav::application::ChartDetail::Full));
    const auto over = DecimateArrows(points, 400, 400, ArrowSpacingPx(opennav::application::ChartDetail::Overview));
    Check(full == DecimateArrows(points, 400, 400, 64), "Decimation is deterministic");
    Check(!full.empty() && over.size() < full.size(), "Zooming out thins arrows");
    for (std::size_t i = 0; i < full.size(); ++i)
      for (std::size_t j = i + 1; j < full.size(); ++j) {
        const double dx = points[full[i]].x - points[full[j]].x, dy = points[full[i]].y - points[full[j]].y;
        Check(dx * dx + dy * dy >= 64.0 * 64.0, "Minimum on-screen spacing honoured");
      }
    Check(DecimateArrows(points, 4000, 4000, 1).size() == kMaxWindArrows, "Draw count bounded");
    Check(DecimateArrows({{-100, 5}, {5, 5}}, 400, 400, 64) == std::vector<std::size_t>{1},
          "Off-screen points skipped");
    Check(ArrowSpacingPx(opennav::application::ChartDetailForScale(20000)) == 64 &&
          ArrowSpacingPx(opennav::application::ChartDetailForScale(1e6)) == 120,
          "Spacing follows the shared chart detail levels");
    // SCRUM-355: a screen grid takes the nearest model sample, but never one
    // further than about a 0.25 degree model cell.
    {
      const std::vector<ForecastWind> grid{Wind(58.25, 15.50, T0, 4), Wind(58.50, 15.75, T0, 9)};
      const auto near = NearestWindSample(grid, {58.44, 15.62});
      Check(near && *near == 1, "Harbour cell takes the closest model sample");
      Check(!NearestWindSample(grid, {55.0, 10.0}), "Far cell refuses a stretched sample");
      Check(!NearestWindSample({}, {58.44, 15.62}), "No samples, no arrow");
      Check(!NearestWindSample(grid, {std::nan(""), 15.62}), "Invalid position refused");
      // Longitude is scaled by latitude: at 58 N a 0.5 degree east step is
      // only ~0.26 degrees of arc, so it is within reach.
      Check(NearestWindSample({Wind(58.0, 15.5, T0)}, {58.0, 15.0}).has_value(),
            "Longitude distance shrinks with latitude");
      Check(!NearestWindSample({Wind(0.0, 15.5, T0)}, {0.0, 15.0}).has_value(),
            "Same longitude step is too far at the equator");
      // Antimeridian: 179.9 E and 179.9 W are 0.2 degrees apart, not 359.8.
      Check(NearestWindSample({Wind(10.0, -179.9, T0)}, {10.0, 179.9}).has_value(),
            "Distance wraps at the antimeridian");
    }
    {
      const double pitch = ArrowGridPitch(1192, 690, 64);
      Check(pitch >= 64, "Pitch never tighter than the declutter spacing");
      Check(std::ceil(1192 / pitch) * std::ceil(690 / pitch) <= kMaxWindArrows,
            "Whole view fits under the arrow cap");
      Check(ArrowGridPitch(200, 150, 120) == 120, "Small view keeps the base spacing");
      Check(ArrowGridPitch(0, 100, 64) == 0 && ArrowGridPitch(100, 100, 0) == 0,
            "Degenerate input gives no grid");
    }
    const auto tri = ArrowTriangles(100, 100, 0, 10, 1);
    Check(tri.size() == 18, "Arrow is three triangles");
    Check(Near(tri[12], 100) && tri[13] < 100 - ArrowLengthPx(10) / 2 + 1,
          "Angle 0 points up the screen");
    Check(ArrowTriangles(0, 0, 0, -1, 1).empty(), "Invalid speed draws nothing");

    // Route forecast assembly.
    std::vector<PassPoint> route{{"Start", {57.0, 16.0}, std::nullopt},
                                 {"Mid", {57.5, 16.5}, 12.0},
                                 {"Far", {60.0, 20.0}, 6.0}};
    auto rf = AssembleRouteForecast(route, false, s, T0, 6.0, "current SOG");
    Check(rf.points.size() == 3 && rf.display == ForecastDisplay::Live, "One row per point");
    Check(rf.points[0].eta == T0 && rf.points[1].eta == T0 + 2h && rf.points[2].eta == T0 + 3h,
          "ETA from departure now at steady speed");
    Check(rf.points[0].wind && Near(rf.points[0].wind->speed_mps, 5) &&
          rf.points[1].wind && Near(rf.points[1].wind->speed_mps, 10),
          "Forecast at estimated passing time");
    Check(!rf.points[2].wind, "Point outside the grid reports no coverage");
    Check(rf.assumption.find("6.0 kn (current SOG)") != std::string::npos, "Assumption stated");
    auto now_only = AssembleRouteForecast(route, false, s, T0, std::nullopt, "");
    Check(!now_only.points[1].eta && now_only.points[1].lookup == T0 &&
          now_only.points[1].wind && Near(now_only.points[1].wind->speed_mps, 8),
          "No speed: forecast at now");
    Check(now_only.assumption.find("No speed basis") != std::string::npos, "Fallback stated");
    auto broken = route; broken[1].leg_nm.reset();
    auto rb = AssembleRouteForecast(broken, false, s, T0, 6.0, "current SOG");
    Check(rb.points[0].eta && !rb.points[1].eta && !rb.points[2].eta && rb.points[1].lookup == T0,
          "Unknown leg breaks the ETA chain; those points use now");
    auto from_vessel = AssembleRouteForecast({{"Next", {57.5, 16.5}, 6.0}}, true, s, T0, 6.0, "SOG");
    Check(from_vessel.points[0].eta == T0 + 1h, "First leg from vessel position");
    auto stale = AssembleRouteForecast(route, false, Snapshot(ForecastState::Disabled), T0, 6.0, "SOG");
    Check(!stale.points[0].wind && stale.display == ForecastDisplay::Unavailable,
          "Unavailable forecast gives no values");
    std::vector<PassPoint> long_route(80, PassPoint{"P", {57, 16}, 1.0});
    auto lr = AssembleRouteForecast(long_route, false, s, T0, 6.0, "SOG");
    Check(lr.points.size() == kMaxRouteForecastRows && lr.truncated, "Rows bounded");
  } catch (const std::exception &error) {
    std::cerr << "weather forecast view: " << error.what() << '\n';
    return 1;
  }
  std::cout << "weather forecast view: ok\n";
  return 0;
}

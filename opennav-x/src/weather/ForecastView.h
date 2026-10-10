#pragma once
// Pure presentation helpers for provider-neutral forecast wind (SCRUM-327..331).
// Shared by the chart overlay (integration) and XNav pages (UI). No OpenCPN,
// wxWidgets, network or credential access; deterministic and unit-tested.
// Forecasts are advisory: they never replace measured onboard wind and never
// change a route, waypoint or navigation state.
#include "application/ChartDeclutter.h"
#include "weather/Weather.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace opennav::weather {
constexpr double kKnotsPerMps = 3600.0 / 1852.0;
// A forecast value represents its grid cell; beyond this the point has no
// coverage. GFS 0.25° cells are ~15 nm north-south.
constexpr double kCoverageRadiusNm = 12;
// Nearest valid time must be this close to the wanted time (hourly steps).
constexpr std::chrono::minutes kCoverageTimeGap{90};
constexpr std::size_t kMaxRouteForecastRows = 50;
constexpr std::size_t kMaxWindArrows = 48;  // SCRUM-317: bounded draw count.

inline double KnotsFromMps(double mps) { return mps * kKnotsPerMps; }

inline double NormalizeDegrees(double degrees) {
  if (!std::isfinite(degrees)) return degrees;
  degrees = std::fmod(degrees, 360.0);
  return degrees < 0 ? degrees + 360.0 : degrees;
}

// 16-point compass name for a TRUE direction.
inline const char *CompassPoint(double degrees) {
  static const char *const names[] = {"N","NNE","NE","ENE","E","ESE","SE","SSE",
                                      "S","SSW","SW","WSW","W","WNW","NW","NNW"};
  if (!std::isfinite(degrees)) return "";
  const int index = static_cast<int>(std::floor(NormalizeDegrees(degrees) / 22.5 + .5)) % 16;
  return names[index];
}

inline bool UsableWind(const ForecastWind &w) {
  return std::isfinite(w.speed_mps) && w.speed_mps >= 0 && w.speed_mps < 150 &&
         std::isfinite(w.direction_from_true_deg) &&
         std::isfinite(w.requested.latitude_deg) && std::isfinite(w.requested.longitude_deg) &&
         std::abs(w.requested.latitude_deg) <= 90 && std::abs(w.requested.longitude_deg) <= 180;
}

// Distinct valid times, ascending, bounded to kMaxForecastSteps.
inline std::vector<WallTime> ValidTimes(const ForecastSnapshot &snapshot) {
  std::vector<WallTime> times;
  for (const auto &w : snapshot.winds)
    if (UsableWind(w)) times.push_back(w.valid_time);
  std::sort(times.begin(), times.end());
  times.erase(std::unique(times.begin(), times.end()), times.end());
  if (times.size() > kMaxForecastSteps) times.resize(kMaxForecastSteps);
  return times;
}

// Nearest available valid time; an exact tie prefers the earlier time.
inline std::optional<WallTime> NearestValidTime(const std::vector<WallTime> &times,
                                                WallTime target) {
  std::optional<WallTime> best;
  for (const auto t : times) {
    if (!best) { best = t; continue; }
    const auto d = t > target ? t - target : target - t;
    const auto b = *best > target ? *best - target : target - *best;
    if (d < b || (d == b && t < *best)) best = t;
  }
  return best;
}

// The step shown: the user's choice when still present, else the nearest
// step to that choice, else the step nearest to now.
inline std::optional<WallTime> ResolveDisplayTime(const std::vector<WallTime> &times,
                                                  std::optional<WallTime> selected,
                                                  WallTime now) {
  return NearestValidTime(times, selected ? *selected : now);
}

// Move delta steps from current within the available steps (clamped).
inline std::optional<WallTime> StepDisplayTime(const std::vector<WallTime> &times,
                                               std::optional<WallTime> current,
                                               WallTime now, int delta) {
  const auto resolved = ResolveDisplayTime(times, current, now);
  if (!resolved) return {};
  const auto it = std::find(times.begin(), times.end(), *resolved);
  const long index = static_cast<long>(it - times.begin()) + delta;
  const long last = static_cast<long>(times.size()) - 1;
  return times[static_cast<std::size_t>(std::clamp(index, 0L, last))];
}

// "Now" within 30 min of now, else signed whole hours: "+3 h", "−2 h".
inline std::string StepLabel(WallTime valid, WallTime now) {
  const auto minutes = std::chrono::duration_cast<std::chrono::minutes>(valid - now).count();
  if (minutes > -30 && minutes < 30) return "Now";
  const long hours = std::lround(static_cast<double>(minutes) / 60.0);
  char text[32];
  std::snprintf(text, sizeof text, "%s%ld h", hours > 0 ? "+" : "\xE2\x88\x92",
                hours > 0 ? hours : -hours);
  return text;
}

// Compact age: "under 1 min", "12 min", "3 h 05 min", "2 d 4 h".
inline std::string AgeLabel(std::chrono::seconds age) {
  if (age.count() < 60) return "under 1 min";
  const long minutes = static_cast<long>(age.count() / 60);
  char text[48];
  if (minutes < 60) std::snprintf(text, sizeof text, "%ld min", minutes);
  else if (minutes < 24 * 60)
    std::snprintf(text, sizeof text, "%ld h %02ld min", minutes / 60, minutes % 60);
  else std::snprintf(text, sizeof text, "%ld d %ld h", minutes / 1440, minutes % 1440 / 60);
  return text;
}

enum class ForecastDisplay { Unavailable, Live, Stale };

// Live only when the provider says Live AND XNav fetched it within kLiveFor.
// Anything older (or without a fetch time) is Stale; past kCacheFor or for
// a disabled/credential-less/failed feature it is Unavailable.
inline ForecastDisplay DisplayState(const ForecastSnapshot &s, WallTime now) {
  if (s.state != ForecastState::Live && s.state != ForecastState::Stale)
    return ForecastDisplay::Unavailable;
  if (ValidTimes(s).empty()) return ForecastDisplay::Unavailable;
  if (s.fetched_at && now - *s.fetched_at > kCacheFor) return ForecastDisplay::Unavailable;
  if (s.state == ForecastState::Live && s.fetched_at && now >= *s.fetched_at &&
      now - *s.fetched_at <= kLiveFor)
    return ForecastDisplay::Live;
  return ForecastDisplay::Stale;
}

inline std::string Upper(std::string text) {
  for (auto &c : text) if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
  return text;
}

// "GRIBstream GFS" — never invents a model name.
inline std::string SourceLabel(const ForecastSnapshot &s) {
  std::string label = s.provider.empty() ? std::string("Forecast provider") : s.provider;
  if (!s.model.empty()) label += " " + Upper(s.model);
  return label;
}

inline std::chrono::seconds AgeSince(WallTime then, WallTime now) {
  return now > then ? std::chrono::duration_cast<std::chrono::seconds>(now - then)
                    : std::chrono::seconds{0};
}

// One-line provenance, always beginning with FORECAST so it cannot be read as
// measured wind: "FORECAST · GRIBstream GFS · run 3 h 05 min old".
inline std::string ProvenanceLabel(const ForecastSnapshot &s, WallTime now) {
  const auto display = DisplayState(s, now);
  if (display == ForecastDisplay::Unavailable) return "FORECAST UNAVAILABLE";
  std::string text = display == ForecastDisplay::Stale ? "STALE FORECAST" : "FORECAST";
  text += " \xC2\xB7 " + SourceLabel(s);
  if (s.model_run) text += " \xC2\xB7 run " + AgeLabel(AgeSince(*s.model_run, now)) + " old";
  if (display == ForecastDisplay::Stale)
    text += s.fetched_at ? " \xC2\xB7 fetched " + AgeLabel(AgeSince(*s.fetched_at, now)) + " ago"
                         : std::string(" \xC2\xB7 fetch time unknown");
  return text;
}

// Credential-free explanation when nothing can be shown.
inline std::string UnavailableReason(const ForecastSnapshot &s, WallTime now) {
  switch (s.state) {
  case ForecastState::Disabled: return "Weather forecasts are off. Enable them in Settings \xE2\x80\xBA Weather.";
  case ForecastState::NoCredential: return "No GRIBstream token stored. Add one in Settings \xE2\x80\xBA Weather.";
  case ForecastState::Loading: return "Loading forecast\xE2\x80\xA6";
  case ForecastState::Error: return s.status.empty() ? "Forecast request failed." : s.status;
  case ForecastState::Live: case ForecastState::Stale: break;
  }
  if (s.fetched_at && now - *s.fetched_at > kCacheFor)
    return "Cached forecast expired. Waiting for a new forecast.";
  return "No forecast values received yet.";
}

// Great-circle distance in nautical miles, for coverage matching only.
inline double DistanceNm(Coordinate a, Coordinate b) {
  constexpr double rad = 3.14159265358979323846 / 180.0;
  const double dlat = (b.latitude_deg - a.latitude_deg) * rad;
  const double dlon = (b.longitude_deg - a.longitude_deg) * rad;
  const double h = std::sin(dlat / 2) * std::sin(dlat / 2) +
      std::cos(a.latitude_deg * rad) * std::cos(b.latitude_deg * rad) *
      std::sin(dlon / 2) * std::sin(dlon / 2);
  return 2 * std::asin(std::min(1.0, std::sqrt(h))) * 3440.065;
}

// Where a value applies: provider grid point when given, else requested point.
inline Coordinate WindPosition(const ForecastWind &w) { return w.grid ? *w.grid : w.requested; }

// Forecast wind covering position at the step nearest to `at`, or nothing
// (no coverage) when no value is within kCoverageRadiusNm / kCoverageTimeGap.
inline std::optional<ForecastWind> WindNear(const ForecastSnapshot &s, Coordinate position,
                                            WallTime at,
                                            double max_distance_nm = kCoverageRadiusNm,
                                            std::chrono::minutes max_gap = kCoverageTimeGap) {
  if (!std::isfinite(position.latitude_deg) || !std::isfinite(position.longitude_deg))
    return {};
  const auto step = NearestValidTime(ValidTimes(s), at);
  if (!step) return {};
  const auto gap = *step > at ? *step - at : at - *step;
  if (gap > max_gap) return {};
  std::optional<ForecastWind> best;
  double best_nm = max_distance_nm;
  for (const auto &w : s.winds) {
    if (w.valid_time != *step || !UsableWind(w)) continue;
    const double d = std::min(DistanceNm(position, w.requested), DistanceNm(position, WindPosition(w)));
    if (d <= best_nm && (!best || d < best_nm)) { best = w; best_nm = d; }
  }
  return best;
}

// All usable winds at one step, in snapshot order (sorted by lat, lon).
inline std::vector<ForecastWind> WindsAt(const ForecastSnapshot &s, WallTime valid_time) {
  std::vector<ForecastWind> winds;
  for (const auto &w : s.winds)
    if (w.valid_time == valid_time && UsableWind(w)) winds.push_back(w);
  return winds;
}

// "14 kn from 230° SW"
inline std::string WindLabel(const ForecastWind &w) {
  char text[64];
  const double from = NormalizeDegrees(w.direction_from_true_deg);
  std::snprintf(text, sizeof text, "%.0f kn from %03.0f\xC2\xB0 %s", KnotsFromMps(w.speed_mps),
                std::fmod(std::round(from), 360.0), CompassPoint(from));
  return text;
}

// ---- Chart arrows (SCRUM-317 decluttering) --------------------------------
struct ScreenPoint { double x = 0, y = 0; };

// Minimum on-screen spacing between arrows (logical px at 100%).
inline double ArrowSpacingPx(application::ChartDetail detail) {
  switch (detail) {
  case application::ChartDetail::Full: return 64;
  case application::ChartDetail::Reduced: return 88;
  case application::ChartDetail::Overview: return 120;
  }
  return 120;
}

// Deterministic greedy decimation in input order: keeps on-screen points at
// least min_spacing apart, never more than max_count. Returns input indices.
inline std::vector<std::size_t> DecimateArrows(const std::vector<ScreenPoint> &points,
                                               double width, double height,
                                               double min_spacing,
                                               std::size_t max_count = kMaxWindArrows) {
  std::vector<std::size_t> kept;
  if (!(width > 0) || !(height > 0) || !(min_spacing > 0)) return kept;
  const double margin = 8, spacing2 = min_spacing * min_spacing;
  for (std::size_t i = 0; i < points.size() && kept.size() < max_count; ++i) {
    const auto &p = points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < -margin || p.y < -margin ||
        p.x > width + margin || p.y > height + margin)
      continue;
    bool clear = true;
    for (const auto k : kept) {
      const double dx = points[k].x - p.x, dy = points[k].y - p.y;
      if (dx * dx + dy * dy < spacing2) { clear = false; break; }
    }
    if (clear) kept.push_back(i);
  }
  return kept;
}

// SCRUM-355: arrows are laid on a screen grid, each taking the nearest model
// sample, rather than only at the sample points. GFS is a 0.25 degree model,
// so at harbour scale the nearest sample usually lies outside the view and
// point-only drawing showed no arrows at all. A sample is never stretched past
// about one model cell: where the fetched grid is coarser than that, cells stay
// empty instead of presenting wind the forecast did not give for that place.
constexpr double kMaxSampleDistanceDeg = 0.35;
inline std::optional<std::size_t> NearestWindSample(const std::vector<ForecastWind> &winds,
                                                    Coordinate at,
                                                    double max_deg = kMaxSampleDistanceDeg) {
  if (!std::isfinite(at.latitude_deg) || !std::isfinite(at.longitude_deg) ||
      std::abs(at.latitude_deg) > 90 || !(max_deg > 0))
    return std::nullopt;
  // Equirectangular: longitude shrinks with latitude.
  const double k = std::cos(at.latitude_deg * 3.14159265358979323846 / 180.0);
  std::optional<std::size_t> best;
  double best_d2 = max_deg * max_deg;
  for (std::size_t i = 0; i < winds.size(); ++i) {
    const auto c = WindPosition(winds[i]);
    const double dlat = c.latitude_deg - at.latitude_deg;
    const double dlon = std::remainder(c.longitude_deg - at.longitude_deg, 360.0) * k;
    const double d2 = dlat * dlat + dlon * dlon;
    if (d2 < best_d2 || (!best && d2 == best_d2)) { best_d2 = d2; best = i; }
  }
  return best;
}
// Screen-grid pitch: never tighter than the declutter spacing, and widened
// until the whole view fits under the arrow cap, so coverage stays even
// instead of the cap filling only the top rows.
inline double ArrowGridPitch(double width, double height, double min_spacing,
                             std::size_t max_count = kMaxWindArrows) {
  if (!(width > 0) || !(height > 0) || !(min_spacing > 0) || max_count == 0) return 0;
  double pitch = min_spacing;
  for (int guard = 0; guard < 64; ++guard) {
    if (std::ceil(width / pitch) * std::ceil(height / pitch) <= static_cast<double>(max_count))
      break;
    pitch *= 1.15;
  }
  return pitch;
}

// Geographic lattice step (SCRUM-360): the smallest "nice" degree value not
// tighter than the requested one, so arrows sit at fixed chart positions that
// pan with the chart and only re-space when the zoom changes.
inline double NiceDegreeStep(double raw_deg) {
  if (!std::isfinite(raw_deg) || raw_deg <= 0) return 0;
  static const double ladder[] = {1, 2, 2.5, 5};
  for (double decade = 1e-4; decade <= 100; decade *= 10)
    for (const double m : ladder)
      if (m * decade >= raw_deg) return m * decade;
  return 0;
}
// Prototype wind vector (#windLayer path "m x y 18-8-6 0m6 0-3 5"): an open
// chevron -- a shaft centred on the lattice point and two short barbs at the
// downwind tip. Segments as x1,y1,x2,y2; angle is clockwise from screen up.
inline std::vector<double> ChevronSegments(double x, double y, double angle_rad,
                                           double scale) {
  const double half = 10 * scale, barb = 6 * scale, spread = 0.52;  // ~30 deg
  const double dx = std::sin(angle_rad), dy = -std::cos(angle_rad);
  const double tx = x + dx * half, ty = y + dy * half;
  const auto barb_end = [&](double side) {
    const double a = angle_rad + 3.14159265358979 + side * spread;
    return std::pair<double, double>{tx + std::sin(a) * barb, ty - std::cos(a) * barb};
  };
  const auto [l1, l2] = barb_end(1);
  const auto [r1, r2] = barb_end(-1);
  return {x - dx * half, y - dy * half, tx, ty, tx, ty, l1, l2, tx, ty, r1, r2};
}
// Below this the wind is drawn as a calm ring, not an arrow with a direction.
constexpr double kCalmKn = 1.0;

// ---- Route forecast (SCRUM-331) -------------------------------------------
struct PassPoint {
  std::string name;
  Coordinate position;
  // Distance from the previous point (or from the vessel for the first point
  // when the list starts at the vessel). Missing breaks the ETA chain.
  std::optional<double> leg_nm;
};
struct RoutePointForecast {
  std::string name;
  std::optional<WallTime> eta;   // Estimated passing time, when a basis exists.
  WallTime lookup{};             // Time used for the forecast lookup.
  std::optional<ForecastWind> wind;  // Empty: no forecast coverage.
};
struct RouteForecast {
  ForecastDisplay display = ForecastDisplay::Unavailable;
  std::string assumption;
  std::vector<RoutePointForecast> points;
  bool truncated = false;
};

inline bool UsableSpeedKn(std::optional<double> speed_kn) {
  return speed_kn && std::isfinite(*speed_kn) && *speed_kn >= 0.5 && *speed_kn <= 60;
}

// Pure assembly; never modifies or activates a route. The first point is
// passed now unless first_leg_from_vessel (then its leg is from the vessel).
inline RouteForecast AssembleRouteForecast(const std::vector<PassPoint> &points,
                                           bool first_leg_from_vessel,
                                           const ForecastSnapshot &snapshot, WallTime now,
                                           std::optional<double> speed_kn,
                                           const std::string &speed_basis) {
  RouteForecast result;
  result.display = DisplayState(snapshot, now);
  const bool eta_basis = UsableSpeedKn(speed_kn);
  char text[240];
  if (eta_basis)
    std::snprintf(text, sizeof text,
        "Passing times assume %s now at a steady %.1f kn (%s). Advisory only.",
        first_leg_from_vessel ? "continuing from the current position" : "departure from the first waypoint",
        *speed_kn, speed_basis.c_str());
  else
    std::snprintf(text, sizeof text,
        "No speed basis for passing times; forecast shown for now at every point. Advisory only.");
  result.assumption = text;
  std::optional<WallTime> clock = now;
  const std::size_t count = std::min(points.size(), kMaxRouteForecastRows);
  result.truncated = points.size() > count;
  for (std::size_t i = 0; i < count; ++i) {
    const auto &p = points[i];
    RoutePointForecast row;
    row.name = p.name;
    if (eta_basis && clock) {
      if (i == 0 && !first_leg_from_vessel) {
        row.eta = clock;
      } else if (p.leg_nm && std::isfinite(*p.leg_nm) && *p.leg_nm >= 0 && *p.leg_nm < 20000) {
        clock = *clock + std::chrono::duration_cast<WallTime::duration>(
            std::chrono::duration<double, std::ratio<3600>>(*p.leg_nm / *speed_kn));
        row.eta = clock;
      } else {
        clock.reset();  // Unknown leg: later passing times are unknown too.
      }
    }
    row.lookup = row.eta ? *row.eta : now;
    if (result.display != ForecastDisplay::Unavailable)
      row.wind = WindNear(snapshot, p.position, row.lookup);
    result.points.push_back(std::move(row));
  }
  return result;
}
} // namespace opennav::weather

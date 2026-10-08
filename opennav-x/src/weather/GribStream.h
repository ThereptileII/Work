#pragma once
// Network-free GRIBstream codec (SCRUM-324/326). Builds the bounded timeseries
// request and validates the untrusted CSV response into XNav-owned values.
// Locale-independent: never uses printf/strtod on numbers (wx may set a
// decimal-comma locale on the boat PC).
#include "weather/Weather.h"
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace opennav::weather::gribstream {
constexpr std::string_view kDefaultModel = "gfs";
constexpr std::string_view kHost = "https://gribstream.com";
// Bounded response: 64 points x 49 steps of ~110 byte rows is ~350 KiB.
constexpr std::size_t kMaxResponseBytes = 2u * 1024u * 1024u;
constexpr std::size_t kMaxResponseRows =
    kMaxForecastPoints * kMaxForecastSteps * 2;

// Model identifiers are a closed lowercase token; anything else is refused
// so that configuration can never inject a path or query string.
bool ValidModel(std::string_view model);
std::string TimeseriesUrl(std::string_view model);

// Drops invalid/duplicate coordinates, normalizes longitude, rounds to 1e-4
// degrees, caps points at kMaxForecastPoints (first ones win: callers order
// vessel, route, grid) and the window at kMaxForecastSteps hourly steps.
// nullopt when nothing usable remains.
std::optional<ForecastQuery> Clamp(const ForecastQuery &query, WallTime now);
// JSON body for POST /api/v2/<model>/timeseries. Coordinates are named
// p0..pN in the clamped order; the parser maps rows back by that name.
std::string RequestBody(const ForecastQuery &clamped);

// Strict ISO-8601 UTC: YYYY-MM-DDTHH:MM:SS[.fraction](Z|+00:00).
std::optional<WallTime> ParseUtc(std::string_view text);
std::string FormatUtc(WallTime time);  // YYYY-MM-DDTHH:MM:SSZ

struct WindVector {
  double speed_mps = 0, direction_from_true_deg = 0;
};
// u/v in m/s, positive toward east/north. Returns meteorological speed and
// direction FROM (true, [0,360)). nullopt for non-finite or implausible input.
std::optional<WindVector> FromUV(double u_mps, double v_mps);

struct ParseResult {
  bool ok = false;
  std::string error;  // Credential- and payload-free.
  std::vector<ForecastWind> winds;  // Sorted: valid_time, latitude, longitude.
  std::size_t rejected_rows = 0;
  bool truncated = false;
};
ParseResult ParseCsv(std::string_view csv, const ForecastQuery &clamped);
} // namespace opennav::weather::gribstream

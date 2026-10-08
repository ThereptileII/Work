#pragma once
// Provider-neutral XNav weather contract (SCRUM-324/326/328/330).
// UI, SmartNav and chart code consume only these owned values; no
// GRIBstream payload, socket or credential crosses this boundary.
#include "ais/Credentials.h"  // Secret: non-copyable, never streamed/logged.
#include "application/NavigationObjects.h"  // CommandResult
#include "vessel/VesselState.h"
#include <functional>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace opennav::weather {
using WallTime = std::chrono::system_clock::time_point;

struct Coordinate {
  double latitude_deg = 0, longitude_deg = 0;
};

// One forecast wind value at one coordinate and valid time. Direction is
// meteorological: the TRUE direction the wind blows FROM, 0..360.
struct ForecastWind {
  Coordinate requested;                 // Coordinate XNav asked for.
  std::optional<Coordinate> grid;       // Provider native grid point, if given.
  double speed_mps = 0;
  double direction_from_true_deg = 0;
  WallTime model_run{};                 // GRIBstream forecasted_at.
  WallTime valid_time{};                // GRIBstream forecasted_time.
};

enum class ForecastState {
  Disabled,      // Feature off by the user.
  NoCredential,  // Enabled but no token stored.
  Loading,       // First request in flight, nothing cached yet.
  Live,          // Fetched from the provider within the freshness window.
  Stale,         // Only older cached data; must be shown as historical.
  Error          // Last request failed and nothing usable is cached.
};

// Owned snapshot. "Live" vs "Stale" is decided from fetched_at, never from
// valid_time: a forecast for 18:00 fetched 3 h ago is stale even at 17:00.
struct ForecastSnapshot {
  ForecastState state = ForecastState::Disabled;
  std::string provider = "GRIBstream", model;  // e.g. "gfs".
  std::optional<WallTime> fetched_at;           // When XNav received it.
  std::optional<WallTime> model_run;            // Newest run in the set.
  std::string status;                           // Human, credential-free.
  // Sorted by valid_time, then latitude, then longitude.
  std::vector<ForecastWind> winds;
};

// Bounded request description. Implementations clamp coordinates and steps.
struct ForecastQuery {
  std::vector<Coordinate> points;  // Vessel, route samples, chart grid.
  WallTime from{}, until{};        // Valid-time window.
};

// Limits shared by provider, cache and UI so no layer can overload the boat
// PC or the user's GRIBstream quota.
constexpr std::size_t kMaxForecastPoints = 64;
constexpr std::size_t kMaxForecastSteps = 49;  // e.g. 48 h hourly + now.
constexpr std::chrono::minutes kLiveFor{90};   // Then Stale.
constexpr std::chrono::hours kCacheFor{24};    // Then discarded entirely.

// Credential-free provider status for Settings → Test connection.
struct ConnectionTest {
  bool ok = false;
  std::string message;  // Never contains the token.
};
} // namespace opennav::weather

namespace opennav::application {
// UI-facing actions, mirroring OnlineAisActions. Callbacks return copies.
struct WeatherActions {
  std::function<weather::ForecastSnapshot(vessel::Time)> read;
  std::function<CommandResult(bool)> enable;
  std::function<CommandResult(const ais::Secret &)> store_token;
  std::function<CommandResult()> remove_token;
  std::function<bool()> token_present;
  // Asynchronous; the result arrives through read()/status, never blocks UI.
  std::function<CommandResult()> test_connection;
  // Replace the bounded query (vessel position, route samples, chart grid).
  std::function<void(const weather::ForecastQuery &)> request;
  // Latest user-requested Test connection outcome (credential-free) and
  // whether one is still running. Added for the Settings Test button.
  std::function<weather::ConnectionTest()> last_test;
  std::function<bool()> test_pending;
  // Route shown on the route page. Sampled into the next query when no route
  // is active, so an inactive planned route also gets forecast coverage.
  // Empty clears it. Never activates or modifies the route.
  std::function<void(std::vector<weather::Coordinate>)> focus_route;
};
} // namespace opennav::application

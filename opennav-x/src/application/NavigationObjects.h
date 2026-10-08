#pragma once
#include "vessel/AisState.h"
#include <chrono>
#include <functional>
#include <optional>
#include <vector>

namespace opennav::application {
struct Waypoint {
  std::string id, revision, name, description;
  double latitude_deg = 0, longitude_deg = 0;
  bool editable = false, removable = false, in_route = false;
  std::optional<double> incoming_nm, incoming_course_true_deg;
};
struct Route {
  std::string id, revision, name, description;
  std::vector<Waypoint> points;
  bool active = false, editable = false, visible = false;
};
struct Catalog {
  std::vector<Route> routes;
  std::vector<Waypoint> waypoints;
  bool truncated = false;
};
struct CommandResult {
  bool ok = false;
  std::string message, identity;
};
enum class ChartOrientation { NorthUp, CourseUp, HeadUp };
enum class ChartFormat { Unavailable, Vector, Raster };
struct ChartLayerState {
  // No value means this boundary cannot observe the layer; false means hidden.
  std::optional<bool> visible;
  bool editable = false;
  std::string reason;
};
struct ChartPresentationState {
  bool available = false;
  std::string reason;
  ChartFormat format = ChartFormat::Unavailable;
  // Format describes the current chart or quilt reference, never a style preset.
  std::string format_reason;
  std::optional<ChartOrientation> orientation;
  ChartLayerState ais_vessels, enc_text, depth_soundings;
  ChartLayerState chart_symbols{{}, false, "Chart symbols remain managed by OpenCPN"};
  ChartLayerState depth_contours{{}, false, "OpenCPN retains safety-contour presentation"};
  ChartLayerState route_corridor{{}, false, "Chart corridor integration unavailable"};
  ChartLayerState wind_vectors{{}, false, "Chart wind-vector provider unavailable"};
  ChartLayerState radar_overlay{{}, false, "No compatible radar adapter connected"};
};
struct ChartPresentationResult {
  CommandResult command;
  ChartPresentationState state;
};
struct Coordinate {
  double latitude_deg = 0, longitude_deg = 0;
};
struct WaypointContext {
  std::optional<Waypoint> waypoint;
  vessel::Sample range_nm, bearing_true_deg;
  vessel::Time observed_at{};
  std::string reason;
};
struct AnchorFix {
  Coordinate position;
  vessel::Time observed_at{};
  // Metres east/north from the watched mark, projected inside integration
  // using pinned OpenCPN bearing/range. UI must not recompute geodesy.
  std::optional<double> east_m, north_m;
  std::string position_source;
};
struct AnchorState {
  std::string waypoint_id, source, state;
  std::optional<Coordinate> anchor;
  std::optional<double> radius_m;
  vessel::Sample distance_m;
  bool alarm = false;
  vessel::Time observed_at{};
  std::vector<AnchorFix> recent_positions;
  std::optional<AnchorFix> vessel_position;
  // Owned presentation preference copied from OpenCPN. Geometry and watch
  // configuration remain in metres; no UI dependency on upstream globals.
  double distance_units_per_m = 1.;
  std::string distance_unit = "m";
};
// Exact owned selection for a human-confirmed transition. Both upstream watch
// slots are included; no route activation may silently leave a second watch.
struct AnchorWatchSelection {
  bool available = false;
  std::vector<Waypoint> watches;
  std::string reason;
};
struct NavigationNameSuggestion {
  std::string name;
  bool from_chart = false;
};
// UI receives owned values and explicit human-command callbacks. The service
// implementation remains inside the OpenCPN integration boundary.
struct NavigationActions {
  // Copies only; every action returns fresh readback and never caches preferences.
  std::function<ChartPresentationState()> chart_presentation;
  std::function<ChartPresentationResult(bool)> set_chart_ais, set_chart_enc_text,
      set_chart_soundings;
  std::function<ChartPresentationResult(ChartOrientation)> set_chart_orientation;
  // SCRUM-328/329: forecast wind chart layer (off by default) and the shared
  // forecast time step (nullopt = nearest to now). Presentation only.
  std::function<ChartPresentationResult(bool)> set_chart_wind;
  std::function<std::optional<std::chrono::system_clock::time_point>()> weather_time;
  std::function<void(std::optional<std::chrono::system_clock::time_point>)> set_weather_time;
  std::function<CommandResult(int)> view_ais;
  std::function<Catalog()> catalog;
  // One owned selection, or unavailable for a missing/ambiguous identity.
  std::function<std::optional<Route>(const std::string &)> route;
  std::function<WaypointContext(const std::string &, vessel::Time)> waypoint_context;
  std::function<vessel::AisState(vessel::Time)> ais;
  std::function<AnchorState(vessel::Time)> anchor;
  std::function<AnchorWatchSelection()> anchor_watches;
  // New-object defaults only. Existing names are never refreshed from charts.
  std::function<NavigationNameSuggestion(Coordinate)> suggest_waypoint_name;
  std::function<NavigationNameSuggestion()> suggest_route_name;
  std::function<std::optional<Coordinate>()> chart_position;
  std::function<CommandResult(const Route &)> activate, reverse, deactivate;
  std::function<CommandResult(const Route &, const AnchorWatchSelection &)>
      activate_after_anchor;
  std::function<CommandResult(const Route &, const std::string &,
                              const std::string &)>
      edit_route;
  std::function<CommandResult(const Waypoint &, const std::string &,
                              const std::string &)>
      edit_waypoint;
  std::function<CommandResult(const Waypoint &)> delete_waypoint;
  // Inactive, unprotected route only; shared/saved marks are kept by OpenCPN.
  std::function<CommandResult(const Route &)> delete_route;
  std::function<CommandResult(Coordinate, const std::string &,
                              const std::string &)>
      create_waypoint;
  std::function<CommandResult(Coordinate, const std::string &)> go_to;
  std::function<CommandResult(const Waypoint &)> go_to_waypoint;
  std::function<void(const std::string &)> view_route;
  std::function<CommandResult(const std::string &)> view_waypoint;
  std::function<void()> start_route, finish_route, measure, object_info,
      orientation, toggle_ais, fullscreen;
  std::function<CommandResult()> undo_route_point, cancel_route;
  // Inspect the native draft/undo stack without modifying either.
  std::function<bool()> can_undo_route_point;
  std::function<CommandResult(const std::string &, const std::string &)>
      finish_route_named;
  std::function<void(Coordinate)> object_info_at;
  std::function<void()> legacy_settings, legacy_route_manager, plugin_settings;
  std::function<CommandResult(double)> start_anchor;
  std::function<CommandResult(const std::string &)> clear_anchor;
};
// Restrict mutations while inspecting isolated recordings. Read-only catalog,
// chart interaction and existing native OpenCPN state remain independent.
NavigationActions GuardNavigationChanges(NavigationActions actions,
                                         std::function<bool()> allowed);
} // namespace opennav::application

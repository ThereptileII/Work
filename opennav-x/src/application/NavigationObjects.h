#pragma once
#include "vessel/AisState.h"
#include <functional>
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
struct Coordinate {
  double latitude_deg = 0, longitude_deg = 0;
};
struct AnchorFix {
  Coordinate position;
  vessel::Time observed_at{};
};
struct AnchorState {
  std::string waypoint_id, source, state;
  std::optional<Coordinate> anchor;
  std::optional<double> radius_m;
  vessel::Sample distance_m;
  bool alarm = false;
  vessel::Time observed_at{};
  std::vector<AnchorFix> recent_positions;
};
// UI receives owned values and explicit human-command callbacks. The service
// implementation remains inside the OpenCPN integration boundary.
struct NavigationActions {
  std::function<CommandResult(int)> view_ais;
  std::function<Catalog()> catalog;
  std::function<vessel::AisState(vessel::Time)> ais;
  std::function<AnchorState()> anchor;
  std::function<std::optional<Coordinate>()> chart_position;
  std::function<CommandResult(const Route &)> activate, reverse;
  std::function<CommandResult()> deactivate;
  std::function<CommandResult(const Route &, const std::string &,
                              const std::string &)>
      edit_route;
  std::function<CommandResult(const Waypoint &, const std::string &,
                              const std::string &)>
      edit_waypoint;
  std::function<CommandResult(const Waypoint &)> delete_waypoint;
  std::function<CommandResult(Coordinate, const std::string &,
                              const std::string &)>
      create_waypoint;
  std::function<CommandResult(Coordinate, const std::string &)> go_to;
  std::function<CommandResult(const Waypoint &)> go_to_waypoint;
  std::function<void(const std::string &)> view_route, view_waypoint;
  std::function<void()> start_route, finish_route, measure, object_info,
      orientation, toggle_ais, fullscreen;
  std::function<CommandResult()> undo_route_point, cancel_route;
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

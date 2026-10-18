#include "application/NavigationObjects.h"
namespace opennav::application {
namespace {
template <class... Args>
std::function<CommandResult(Args...)>
Guard(std::function<CommandResult(Args...)> action,
      std::function<bool()> allowed) {
  if (!action)
    return {};
  return
      [action = std::move(action), allowed = std::move(allowed)](Args... args) {
        return allowed()
                   ? action(args...)
                   : CommandResult{
                         false,
                         "Stop REPLAY before changing real OpenCPN navigation"};
      };
}
template <class... Args>
std::function<void(Args...)> Guard(std::function<void(Args...)> action,
                                   std::function<bool()> allowed) {
  if (!action)
    return {};
  return
      [action = std::move(action), allowed = std::move(allowed)](Args... args) {
        if (allowed())
          action(args...);
      };
}
} // namespace
NavigationActions GuardNavigationChanges(NavigationActions a,
                                         std::function<bool()> allowed) {
  a.start_track = Guard(a.start_track, allowed);
  a.stop_track = Guard(a.stop_track, allowed);
  a.rename_track = Guard(a.rename_track, allowed);
  a.set_track_visible = Guard(a.set_track_visible, allowed);
  a.view_track = Guard(a.view_track, allowed);
  if (a.track_recording)
    a.track_recording = [read = a.track_recording, allowed] {
      auto state = read();
      if (!allowed()) {
        state.can_start = state.can_stop = false;
        state.reason = "Stop REPLAY before changing real track recording";
      }
      return state;
    };
  if (a.tracks)
    a.tracks = [read = a.tracks, allowed] {
      auto catalog = read();
      if (!allowed())
        for (auto &track : catalog.tracks) track.editable = false;
      return catalog;
    };
  if (a.track)
    a.track = [read = a.track, allowed](const std::string &id) {
      auto track = read(id);
      if (track && !allowed()) track->editable = false;
      return track;
    };
  a.activate = Guard(a.activate, allowed);
  a.activate_after_anchor = Guard(a.activate_after_anchor, allowed);
  a.deactivate = Guard(a.deactivate, allowed);
  a.reverse = Guard(a.reverse, allowed);
  a.edit_route = Guard(a.edit_route, allowed);
  a.set_route_passage = Guard(a.set_route_passage, allowed);
  a.edit_waypoint = Guard(a.edit_waypoint, allowed);
  a.delete_waypoint = Guard(a.delete_waypoint, allowed);
  a.delete_route = Guard(a.delete_route, allowed);
  a.create_waypoint = Guard(a.create_waypoint, allowed);
  a.go_to = Guard(a.go_to, allowed);
  a.go_to_waypoint = Guard(a.go_to_waypoint, allowed);
  a.undo_route_point = Guard(a.undo_route_point, allowed);
  if (a.can_undo_route_point)
    a.can_undo_route_point = [check = a.can_undo_route_point, allowed] {
      return allowed() && check();
    };
  if (a.can_finish_route)
    a.can_finish_route = [check = a.can_finish_route, allowed] {
      return allowed() && check();
    };
  a.cancel_route = Guard(a.cancel_route, allowed);
  a.start_route = Guard(a.start_route, allowed);
  a.finish_route = Guard(a.finish_route, allowed);
  a.finish_route_named = Guard(a.finish_route_named, allowed);
  a.view_route = Guard(a.view_route, allowed);
  a.start_anchor = Guard(a.start_anchor, allowed);
  a.clear_anchor = Guard(a.clear_anchor, allowed);
  a.legacy_settings = Guard(a.legacy_settings, allowed);
  a.connections_settings = Guard(a.connections_settings, allowed);
  a.plugin_settings = Guard(a.plugin_settings, allowed);
  a.ais_alarm_settings = Guard(a.ais_alarm_settings, allowed);
  a.alarm_sound_settings = Guard(a.alarm_sound_settings, allowed);
  a.legacy_route_manager = Guard(a.legacy_route_manager, allowed);
  return a;
}
} // namespace opennav::application

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
  a.activate = Guard(a.activate, allowed);
  a.deactivate = Guard(a.deactivate, allowed);
  a.reverse = Guard(a.reverse, allowed);
  a.edit_route = Guard(a.edit_route, allowed);
  a.edit_waypoint = Guard(a.edit_waypoint, allowed);
  a.delete_waypoint = Guard(a.delete_waypoint, allowed);
  a.create_waypoint = Guard(a.create_waypoint, allowed);
  a.go_to = Guard(a.go_to, allowed);
  a.go_to_waypoint = Guard(a.go_to_waypoint, allowed);
  a.undo_route_point = Guard(a.undo_route_point, allowed);
  a.cancel_route = Guard(a.cancel_route, allowed);
  a.start_route = Guard(a.start_route, allowed);
  a.finish_route = Guard(a.finish_route, allowed);
  a.finish_route_named = Guard(a.finish_route_named, allowed);
  a.view_route = Guard(a.view_route, allowed);
  a.start_anchor = Guard(a.start_anchor, allowed);
  a.clear_anchor = Guard(a.clear_anchor, allowed);
  a.legacy_settings = Guard(a.legacy_settings, allowed);
  a.plugin_settings = Guard(a.plugin_settings, allowed);
  a.legacy_route_manager = Guard(a.legacy_route_manager, allowed);
  return a;
}
} // namespace opennav::application

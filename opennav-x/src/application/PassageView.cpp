#include "application/PassageView.h"
#include <cmath>

namespace opennav::application {
namespace {
std::optional<double> Nonnegative(std::optional<double> value) {
  return value && std::isfinite(*value) && *value >= 0 ? value : std::nullopt;
}
} // namespace
PassageView PresentPassage(const vessel::VesselState &state,
                           const smartnav::NavigationAdvice &advice,
                           const smartnav::EnergyPrediction &energy,
                           vessel::Time now) {
  PassageView view;
  view.reason = "Choose a saved passage or plot a new one.";
  const auto &route = state.navigation.route;
  if (!route || route->state == vessel::RouteState::NoActiveRoute ||
      route->route_id.empty())
    return view;
  view.active = true;
  view.route_id = route->route_id;
  view.destination = route->remaining_steps.empty()
                         ? route->route_name
                         : route->remaining_steps.back().name;
  view.waypoint_count = route->waypoint_count;
  view.distance_nm = vessel::AssessRoute(*route, now).remaining_distance_nm;
  view.current = view.distance_nm.has_value();
  view.reason = "Current position and coherent route progress are required.";
  const bool matching_advice = view.current && advice.route_valid &&
                               advice.calculated_at == now &&
                               advice.route_id == route->route_id &&
                               advice.route_revision == route->route_revision &&
                               advice.revision_scope == route->revision_scope;
  if (!matching_advice) {
    // SmartNav also validates the currently selected position against the
    // route's source/time. Do not display a retained distance after GPS loss
    // or source change simply because the older publication has not expired.
    view.current = false;
    view.distance_nm.reset();
    return view;
  }
  view.reason = advice.reason;
  // SmartNav already validates step identity/order and produces the cumulative
  // distances. Do not sum stored legs again or substitute total route length.
  if (route->active_waypoint_index && route->remaining_steps.size() <= 10000)
    for (std::size_t i = 0; i < route->remaining_steps.size() && i < 128; ++i) {
      const auto &step = route->remaining_steps[i];
      PassagePointView point;
      point.id = step.waypoint_id;
      point.name = step.name;
      point.ordinal = *route->active_waypoint_index + i + 1;
      for (const auto &event : advice.events) {
        if (event.identity != point.id)
          continue;
        if (event.kind == smartnav::EventKind::Waypoint ||
            event.kind == smartnav::EventKind::Destination) {
          point.distance_nm = Nonnegative(event.distance_nm);
          point.seconds = Nonnegative(event.seconds_from_now);
          if (event.kind == smartnav::EventKind::Destination)
            view.seconds = point.seconds;
        } else if (event.kind == smartnav::EventKind::Turn) {
          if (event.course_change_deg &&
              std::isfinite(*event.course_change_deg) &&
              std::abs(*event.course_change_deg) <= 180)
            point.turn_deg = event.course_change_deg;
          if (event.course_true_deg && std::isfinite(*event.course_true_deg) &&
              *event.course_true_deg >= 0 && *event.course_true_deg < 360)
            point.course_true_deg = event.course_true_deg;
        }
      }
      view.points.push_back(std::move(point));
    }
  // The energy wrapper carries the exact immutable input publication. A value
  // from an earlier route/revision/observation must never leak onto this sheet.
  if (energy.input_route == route && energy.calculated_at == now &&
      energy.arrival.estimate) {
    view.arrival_soc = Nonnegative(energy.arrival.estimate->soc_percent);
    if (view.arrival_soc && *view.arrival_soc > 100)
      view.arrival_soc.reset();
    view.below_reserve = energy.arrival.estimate->below_reserve;
  }
  return view;
}
} // namespace opennav::application

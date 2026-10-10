#include "application/PassageView.h"
#include <cctype>
#include <cmath>
#include <cstdio>

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
NextTurnView PresentNextTurn(const PassageView &passage) {
  NextTurnView view;
  if (!passage.active || !passage.current || passage.points.empty()) return view;
  const auto &next = passage.points.front();
  if (next.name.empty() || !next.distance_nm) return view;
  view.visible = true;
  std::string upper;
  for (std::size_t i = 0; i < next.name.size(); ++i) {
    const unsigned char c = next.name[i];
    upper += c < 0x80 ? static_cast<char>(std::toupper(c)) : static_cast<char>(c);
  }
  // Common Swedish lower-case letters in UTF-8 (å ä ö -> Å Ä Ö).
  for (const auto &[from, to] : {std::pair<const char *, const char *>{"\xC3\xA5", "\xC3\x85"},
                                 {"\xC3\xA4", "\xC3\x84"}, {"\xC3\xB6", "\xC3\x96"}})
    for (std::size_t at = upper.find(from); at != std::string::npos; at = upper.find(from, at + 2))
      upper.replace(at, 2, to);
  view.eyebrow = "NEXT \xC2\xB7 " + upper;
  const bool last = passage.points.size() == 1;
  char buffer[64];
  if (last) {
    view.headline = "Arrival";
  } else if (next.turn_deg && std::abs(*next.turn_deg) >= 5) {
    view.direction = *next.turn_deg > 0 ? 1 : -1;
    view.turn_deg = static_cast<int>(std::lround(std::abs(*next.turn_deg)));
    view.headline = view.direction > 0 ? "Starboard" : "Port";
  } else {
    view.headline = "Straight on";
  }
  std::snprintf(buffer, sizeof buffer, "%.1f nm", *next.distance_nm);
  view.detail = buffer;
  if (next.seconds && *next.seconds < 99 * 3600) {
    const auto minutes = static_cast<long>(std::lround(*next.seconds / 60));
    if (minutes < 60) std::snprintf(buffer, sizeof buffer, " \xC2\xB7 in %ld min", minutes);
    else std::snprintf(buffer, sizeof buffer, " \xC2\xB7 in %ld h %02ld min", minutes / 60, minutes % 60);
    view.detail += buffer;
  }
  if (!last && next.course_true_deg) {
    std::snprintf(buffer, sizeof buffer, " \xC2\xB7 new course %03d\xC2\xB0",
                  static_cast<int>(std::lround(*next.course_true_deg)) % 360);
    view.detail += buffer;
  }
  return view;
}
} // namespace opennav::application

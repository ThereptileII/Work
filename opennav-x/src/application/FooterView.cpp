#include "application/FooterView.h"
#include "vessel/RouteProgress.h"
#include <cmath>
#include <iomanip>
#include <locale>
#include <set>
#include <sstream>

namespace opennav::application {
namespace {
bool Current(SignalState state) {
  return state == SignalState::Current || state == SignalState::Aging;
}
std::string FormatCoordinate(double value, bool latitude) {
  // Round in integer thousandths of a minute before splitting, so 59.9996'
  // carries into the next degree rather than producing an impossible 60'.
  const auto minutes = std::llround(std::abs(value) * 60000.0);
  std::ostringstream out; out.imbue(std::locale::classic());
  out << std::setfill('0') << std::setw(latitude ? 2 : 3) << minutes / 60000
      << "° " << std::setw(2) << (minutes % 60000) / 1000 << '.'
      << std::setw(3) << minutes % 1000 << "′ "
      << (latitude ? (value < 0 ? 'S' : 'N') : (value < 0 ? 'W' : 'E'));
  return out.str();
}
}
FooterView PresentFooter(const vessel::VesselState &vessel, const AnchorState &anchor,
                         const SourceHealthView &health, vessel::Time now) {
  FooterView v;
  const auto &nav = vessel.navigation;
  v.position_state = PresentPositionHealth(nav, now).state;
  if (Current(v.position_state)) {
    v.position = FormatCoordinate(*nav.latitude_deg.value, true) + "   " +
                 FormatCoordinate(*nav.longitude_deg.value, false);
    if (v.position_state == SignalState::Aging) v.position += " / AGING";
    v.navigation_state = "EXPLORING";
    if (!anchor.waypoint_id.empty()) v.navigation_state = "ANCHOR WATCH";
    else if (nav.route) {
      const auto route = vessel::AssessRoute(*nav.route, now);
      if (route.state == vessel::RouteState::Valid) v.navigation_state = "ROUTE ACTIVE";
      else if (!nav.route->route_id.empty() && route.state != vessel::RouteState::NoActiveRoute)
        v.navigation_state = "ROUTE WAITING";
    }
  } else {
    if (v.position_state == SignalState::Stale) v.position = "GPS POSITION STALE";
    if (v.position_state == SignalState::Invalid) v.position = "GPS POSITION INVALID";
    if (v.position_state == SignalState::Estimated) v.position = "GPS POSITION ESTIMATED";
    if (v.position_state == SignalState::Uncertain) v.position = "GPS POSITION UNCERTAIN";
  }
  v.cog_state = PresentSignalHealth(nav.cog_deg, now).state;
  if (nav.cog_deg.value && (!std::isfinite(*nav.cog_deg.value) ||
      *nav.cog_deg.value < 0 || *nav.cog_deg.value > 360)) v.cog_state = SignalState::Invalid;
  if (Current(v.cog_state)) {
    std::ostringstream course; course.imbue(std::locale::classic());
    course << std::setfill('0') << std::setw(3)
           << (static_cast<int>(std::lround(*nav.cog_deg.value)) % 360) << "°";
    v.cog = course.str();
    if (v.cog_state == SignalState::Aging) v.cog += " AGING";
  } else if (v.cog_state == SignalState::Stale) v.cog = "STALE";
  // No owned cross-track error contract exists yet. Do not derive it from
  // route coordinates or borrow a retained OpenCPN value as a live reading.
  bool attention = false;
  std::set<std::string> seen;
  const std::set<std::string> onboard{"gps","heading","depth","wind","motor",
                                     "battery","rudder","ais","pilot","water"};
  for (const auto &signal : health.signals) {
    if (!onboard.count(signal.id) || !seen.insert(signal.id).second) continue;
    if (signal.state == SignalState::Current) ++v.live_signals;
    else if (signal.state == SignalState::Aging) ++v.aging_signals;
    else if (signal.state == SignalState::Stale) ++v.stale_signals;
    attention = attention || (signal.state != SignalState::Current && signal.state != SignalState::Unavailable);
  }
  v.health_state = attention ? SignalState::Aging : v.live_signals ? SignalState::Current : SignalState::Unavailable;
  v.health_summary = std::to_string(v.live_signals) + (v.live_signals == 1 ? " live signal" : " live signals");
  if (v.stale_signals) v.health_summary += ", " + std::to_string(v.stale_signals) + " stale";
  else if (v.aging_signals) v.health_summary += ", " + std::to_string(v.aging_signals) + " aging";
  v.historical = vessel.replayed || vessel.simulated || health.historical;
  if (v.historical) {
    v.navigation_state = vessel.replayed ? "REPLAY" : vessel.simulated ? "TEST DATA" : "HISTORICAL";
    v.health_source = vessel.replayed ? "Recorded data" : vessel.simulated ? "Test data" : "Historical data";
    v.health_summary = "Inspect quality";
    v.health_state = SignalState::Uncertain;
    v.live_signals = v.aging_signals = v.stale_signals = 0;
  }
  return v;
}
} // namespace opennav::application

#include "application/Alerts.h"
#include "vessel/RouteProgress.h"
#include <algorithm>
#include <cmath>

namespace opennav::application {
const char *AlertLevelName(AlertLevel l) {
  switch (l) {
  case AlertLevel::Info:
    return "INFO";
  case AlertLevel::Advisory:
    return "ADVISORY";
  case AlertLevel::Warning:
    return "WARNING";
  case AlertLevel::Critical:
    return "CRITICAL";
  }
  return "UNKNOWN";
}
void AlertCenter::Observe(const AlertInput &s) {
  const std::string mode = s.vessel.replayed    ? "REPLAY"
                           : s.vessel.simulated ? "DEMO"
                                                : "LIVE";
  if (mode != mode_ || (clock_started_ && s.now < previous_)) {
    current_.clear();
    seen_.clear();
    pilot_engaged_ = false;
  }
  mode_ = mode;
  previous_ = s.now;
  clock_started_ = true;
  std::vector<Alert> next;
  auto add = [&](const char *id, const char *title, const char *action,
                 const char *source, AlertLevel level, AlertArea area) {
    Alert a{id, title, action, source, level, area, s.now, 0, false};
    for (const auto &old : current_)
      if (old.id == id && old.level == level) {
        a.first_observed = old.first_observed;
        a.episode = old.episode;
        a.acknowledged = old.acknowledged;
        break;
      }
    if (!a.episode)
      a.episode = next_episode_++;
    next.push_back(std::move(a));
  };
  auto sensor = [&](const char *id, const char *title, const vessel::Sample &v,
                    AlertLevel level) {
    const auto a = vessel::Assess(v, s.now);
    const bool usable = a.value && (a.quality == vessel::Quality::Live ||
                                    a.quality == vessel::Quality::Aging ||
                                    a.quality == vessel::Quality::Estimated);
    if (usable)
      seen_.insert(id);
    // Never-installed sensors do not generate an alert storm. A source with an
    // explicit invalid observation is actionable even before its first good
    // fix.
    if (!usable &&
        (seen_.count(id) ||
         (!v.source.empty() && v.validity == vessel::Validity::Invalid)))
      add(id, title,
          "Check source health and the corresponding vessel instrument. "
          "Dependent advice is withheld.",
          "Selected Vessel Data / validity and original observation age", level,
          AlertArea::Sources);
  };
  sensor("position-lat", "Position unavailable or stale",
         s.vessel.navigation.latitude_deg, AlertLevel::Critical);
  sensor("position-lon", "Position unavailable or stale",
         s.vessel.navigation.longitude_deg, AlertLevel::Critical);
  // Present one GPS alert, while monitoring both coordinates independently.
  if (next.size() == 2 && next[0].id == "position-lat" &&
      next[1].id == "position-lon")
    next.pop_back();
  sensor("heading", "Heading unavailable or stale",
         s.vessel.navigation.heading_true_deg, AlertLevel::Warning);
  sensor("depth", "Measured depth unavailable or stale",
         s.vessel.environment.depth_below_transducer_m, AlertLevel::Warning);
  sensor("wind-speed", "Apparent wind speed unavailable or stale",
         s.vessel.wind.apparent_speed_kn, AlertLevel::Advisory);
  sensor("wind-angle", "Apparent wind angle unavailable or stale",
         s.vessel.wind.apparent_angle_deg, AlertLevel::Advisory);
  sensor("rudder", "Rudder feedback unavailable or stale",
         s.vessel.rudder.angle_deg, AlertLevel::Advisory);
  sensor("motor-rpm", "Motor RPM unavailable or stale",
         s.vessel.propulsion.motor_rpm, AlertLevel::Warning);
  sensor("motor-temperature", "Motor temperature unavailable or stale",
         s.vessel.propulsion.motor_temperature_c, AlertLevel::Warning);
  sensor("battery-soc", "Battery SOC unavailable or stale",
         s.vessel.battery.soc_percent, AlertLevel::Warning);
  sensor("battery-voltage", "Battery voltage unavailable or stale",
         s.vessel.battery.voltage_v, AlertLevel::Warning);
  sensor("battery-current", "Battery current unavailable or stale",
         s.vessel.battery.current_native_a, AlertLevel::Warning);
  bool ais_alarm = false;
  // Upstream owns the alarm state. Missing traffic does not prove receiver
  // loss.
  for (const auto &t : s.ais.targets)
    ais_alarm |= t.upstream_alarm;
  if (ais_alarm)
    add("ais-alarm", "OpenCPN AIS alarm",
        "Inspect AIS targets and the existing OpenCPN alarm. XNav "
        "acknowledgement does not acknowledge OpenCPN.",
        "OpenCPN AIS alarm state", AlertLevel::Critical, AlertArea::Ais);
  if (s.anchor.alarm)
    add("anchor-alarm", "OpenCPN anchor watch alarm",
        "Check vessel position and anchor watch. XNav acknowledgement does not "
        "clear the anchor alarm.",
        "OpenCPN anchor watch", AlertLevel::Critical, AlertArea::Anchor);
  const auto route = s.vessel.navigation.route;
  const auto &arrival = s.energy.arrival;
  if (arrival.estimate && arrival.reason == smartnav::EnergyReason::None &&
      route && route == s.energy.input_route &&
      vessel::AssessRoute(*route, s.now).remaining_distance_nm &&
      s.energy.calculated_at == s.now) {
    if (std::isfinite(arrival.estimate->energy_shortfall_kwh) &&
        arrival.estimate->energy_shortfall_kwh > 0)
      add("energy-shortfall", "Estimated route energy insufficient",
          "Review destination, available energy and consumption assumptions. "
          "This is advisory, not a guarantee.",
          "Validated route and battery energy model", AlertLevel::Warning,
          AlertArea::Energy);
    else if (arrival.estimate->below_reserve)
      add("energy-reserve", "Estimated destination energy below reserve",
          "Review the advisory arrival estimate and configured reserve.",
          "Validated route and battery energy model", AlertLevel::Advisory,
          AlertArea::Energy);
  }
  if (!s.vessel.replayed) {
    if (s.pilot.fresh)
      pilot_engaged_ =
          s.pilot.feedback.mode != adapters::PilotMode::Standby &&
          s.pilot.feedback.mode != adapters::PilotMode::Unavailable;
    if (!s.pilot.fresh && pilot_engaged_)
      add("pilot-feedback", "Autopilot feedback lost / mode unknown",
          "Check the physical pilot. Use physical STANDBY if needed; loss of "
          "communication does not disengage the pilot.",
          "Pilot adapter feedback age", AlertLevel::Critical, AlertArea::Pilot);
    if (s.pilot.command.state == adapters::CommandState::TimedOut)
      add("pilot-timeout", "Autopilot command outcome unconfirmed",
          "Check the physical pilot before any further command. The request is "
          "not retried automatically.",
          "Manual command feedback timeout", AlertLevel::Critical,
          AlertArea::Pilot);
    else if (s.pilot.command.state == adapters::CommandState::Rejected ||
             s.pilot.command.state == adapters::CommandState::StaleFeedback)
      add("pilot-rejected", "Autopilot request not accepted",
          "Inspect manual pilot command status and current physical feedback.",
          "Manual command result", AlertLevel::Warning, AlertArea::Pilot);
  }
  std::stable_sort(next.begin(), next.end(),
                   [](const Alert &a, const Alert &b) {
                     if (a.level != b.level)
                       return a.level > b.level;
                     return a.id < b.id;
                   });
  current_ = std::move(next);
}
bool AlertCenter::Acknowledge(const std::string &id, std::uint64_t episode) {
  for (auto &a : current_)
    if (a.id == id && a.episode == episode) {
      a.acknowledged = true;
      return true;
    }
  return false;
}
} // namespace opennav::application

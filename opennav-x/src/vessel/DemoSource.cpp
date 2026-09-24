#include "vessel/DemoSource.h"
#include <algorithm>
#include <cmath>

namespace opennav::vessel {
const char *ScenarioName(DemoScenario s) {
  switch (s) {
  case DemoScenario::Cruise:
    return "Cruising";
  case DemoScenario::Stale:
    return "Sensors stale";
  case DemoScenario::Unavailable:
    return "Sensors unavailable";
  case DemoScenario::RouteInactive:
    return "Route inactive";
  case DemoScenario::RouteEnding:
    return "Route ending";
  case DemoScenario::LowSoc:
    return "Low battery";
  case DemoScenario::HighPower:
    return "High power";
  case DemoScenario::Insufficient:
    return "Energy shortfall";
  }
  return "Cruising";
}
VesselState DemoFixture(DemoScenario scenario, unsigned seconds, Time at) {
  VesselState s;
  s.simulated = true;
  const double t = seconds, wave = std::sin(t / 18.0);
  const auto measured = [at](double v) {
    return Sample{v, "DEMO deterministic trip v0.1", at, Validity::Measured};
  };
  const auto estimated = [at](double v) {
    return Sample{v, "DEMO configured assumption v0.1", at,
                  Validity::Estimated};
  };
  const double speed = 6.3, motor = scenario == DemoScenario::HighPower
                                        ? 18.0
                                        : 7.2 + wave * 0.5;
  const double net =
      motor +
      0.4; // Explicit synthetic hotel load, never assumed for live data.
  double distance = std::max(0.0, 18.2 - t * speed / 60.0);
  if (scenario == DemoScenario::RouteEnding)
    distance = std::max(0.0, 0.5 - t * speed / 60.0);
  const double consumed = (scenario == DemoScenario::HighPower
                               ? 18.4 * t
                               : 7.6 * t + 9.0 * (1 - std::cos(t / 18.0))) /
                          60.0;
  double soc = std::max(0.0, 78.0 - consumed / 48.0 * 100.0);
  if (scenario == DemoScenario::LowSoc)
    soc = 12.0;
  if (scenario == DemoScenario::Insufficient)
    soc = 24.0;
  s.navigation.latitude_deg = measured(59.08 + std::min(t, 173.0) * 0.0003);
  s.navigation.longitude_deg = measured(18.5 + std::min(t, 173.0) * 0.0032);
  s.navigation.sog_kn = measured(speed);
  s.navigation.cog_deg = measured(79.7 + wave);
  s.navigation.heading_true_deg = measured(81.0 + wave * 2);
  s.navigation.stw_kn = measured(6.0 + wave * 0.1);
  s.wind.apparent_speed_kn = measured(16.2 + wave * 1.5);
  s.wind.apparent_angle_deg = measured(72 + wave * 4);
  s.wind.true_speed_kn = measured(12.8 + wave);
  s.wind.true_angle_deg = measured(94 + wave * 3);
  s.environment.depth_below_transducer_m = measured(8.4 + wave * 2.0);
  s.environment.water_temperature_c = measured(15.4 + wave * 0.2);
  s.rudder.angle_deg = measured(wave * 4);
  s.propulsion.electrical_power_kw = measured(motor);
  s.propulsion.motor_rpm = measured((motor > 10 ? 1220 : 820) + wave * 30);
  s.propulsion.motor_temperature_c = measured(62 + wave * 2);
  s.propulsion.shaft_power_kw = estimated(motor * 0.88);
  s.propulsion.gear = {"Forward", "DEMO motor controller", at,
                       Validity::Measured};
  s.propulsion.regeneration = {"Inactive", "DEMO motor controller", at,
                               Validity::Measured};
  s.battery.soc_percent = measured(soc);
  s.battery.voltage_v = measured(343 + wave * 2);
  s.battery.current_a = measured(net * 1000 / (*s.battery.voltage_v.value));
  s.battery.net_discharge_kw = measured(net);
  s.battery.soh_percent = measured(94);
  s.battery.usable_capacity_kwh = estimated(48);
  s.tanks.fresh_water_percent = measured(72);
  s.tanks.waste_percent = measured(18);
  s.connectivity.status = {"Isolated demo source", "DEMO internal generator",
                           at, Validity::Measured};
  // No claimed CAN/NMEA connection or fuel tank on this synthetic electric
  // boat.
  auto route = std::make_shared<RouteProgressSnapshot>();
  route->source = "DEMO synthetic remaining-distance timeline v0.1 (NM), not "
                  "OpenCPN live progress";
  route->observed_at = at;
  route->position_observed_at = at;
  route->position_source = s.navigation.latitude_deg.source;
  route->revision_scope = "DEMO-v0.1";
  route->route_revision = 1;
  if (scenario == DemoScenario::RouteInactive || distance == 0)
    route->state = RouteState::NoActiveRoute;
  else {
    route->route_id = "DEMO-coastal-passage";
    route->waypoint_count = 3;
    const unsigned index = distance > 12.2 ? 0 : distance > 6.2 ? 1 : 2;
    route->active_waypoint_index = index;
    route->active_waypoint_id = index == 0   ? "DEMO-Harbour-mouth"
                                : index == 1 ? "DEMO-Outer-channel"
                                             : "DEMO-Sheltered-bay";
    route->state = RouteState::Valid;
    route->remaining_distance_nm = distance;
    route->route_name = "DEMO coastal passage";
    const double courses[] = {80, 112, 85};
    const char* ids[] = {"DEMO-Harbour-mouth", "DEMO-Outer-channel", "DEMO-Sheltered-bay"};
    const char* names[] = {"Harbour mouth", "Outer channel", "Sheltered bay"};
    for (unsigned i = index; i < 3; ++i) {
      const double leg = i == index ? distance - (index == 0 ? 12.2 : index == 1 ? 6.2 : 0)
                                    : i == 1 ? 6.0 : 6.2;
      route->remaining_steps.push_back({ids[i], names[i], 59.1 + i * 0.02,
                                       18.6 + i * 0.1, leg, courses[i]});
    }
    const double previous = distance + speed / 60.0;
    if (seconds && scenario != DemoScenario::RouteEnding &&
        ((distance <= 12.2 && previous > 12.2) ||
         (distance <= 6.2 && previous > 6.2))) {
      route->state = RouteState::ActivePointChanged;
      route->remaining_distance_nm.reset();
      route->remaining_steps.clear();
    }
  }
  s.navigation.route = std::move(route);
  if (scenario == DemoScenario::Unavailable) {
    s.environment.depth_below_transducer_m = {};
    s.wind = {};
    s.propulsion.motor_rpm = {};
    s.battery.net_discharge_kw = {};
    s.battery.current_a = {};
  }
  return s;
}
void DemoSource::Pause(bool value, Time now) {
  if (value == paused_)
    return;
  if (value)
    paused_at_ = now;
  else
    start_ += now - paused_at_;
  paused_ = value;
}
VesselState DemoSource::Read(Time now) const {
  const auto observation = paused_                            ? paused_at_
                           : scenario_ == DemoScenario::Stale ? start_
                                                              : now;
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::seconds>(observation - start_)
          .count();
  return DemoFixture(
      scenario_,
      static_cast<unsigned>(std::clamp<long long>(elapsed, 0, 86400)),
      observation);
}
} // namespace opennav::vessel

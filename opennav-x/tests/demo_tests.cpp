#include "smartnav/VesselEnergy.h"
#include "vessel/DataItems.h"
#include "vessel/DemoSource.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace vessel;
using namespace std::chrono_literals;
void Check(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
int main() {
  try {
    const Time t{100s};
    const auto model = smartnav::PreviewEnergyModel(true);
    auto a = DemoFixture(DemoScenario::Cruise, 0, t),
         b = DemoFixture(DemoScenario::Cruise, 0, t);
    for (const auto &i : DataItems(a)) {
      if (i.sample->value)
        Check(i.sample->source.find("DEMO") == 0,
              "Every demo datum identifies its source");
      else
        Check(Assess(*i.sample, t).quality == Quality::Unavailable,
              "Missing is not zero");
    }
    Check(a.simulated && a.navigation.latitude_deg.value ==
                             b.navigation.latitude_deg.value,
          "Deterministic position");
    Check(a.battery.soc_percent.value == b.battery.soc_percent.value,
          "Deterministic SOC");
    auto traffic = DemoAis(a);
    Check(traffic.simulated && traffic.targets.size() == 2,
          "Labelled AIS fixture");
    Check(DemoAis(VesselState{}).targets.empty(),
          "Live state never manufactures AIS");
    Check(traffic.targets.front().cpa_nm.source.find("DEMO") == 0,
          "Synthetic CPA identifies source");
    Check(Assess(traffic.targets.front().cpa_nm, t + 6s).quality ==
              Quality::Stale,
          "Demo traffic does not freshen on read");
    auto later = DemoFixture(DemoScenario::Cruise, 30, t + 30s);
    Check(later.navigation.latitude_deg.value !=
              a.navigation.latitude_deg.value,
          "GPS moves");
    Check(later.environment.depth_below_transducer_m.value !=
              a.environment.depth_below_transducer_m.value,
          "Depth changes");
    Check(later.wind.apparent_speed_kn.value != a.wind.apparent_speed_kn.value,
          "Wind changes");
    Check(later.propulsion.motor_rpm.value != a.propulsion.motor_rpm.value,
          "Motor changes");
    Check(later.battery.soc_percent.value < a.battery.soc_percent.value,
          "SOC decreases");
    Check(later.navigation.route->remaining_distance_nm <
              a.navigation.route->remaining_distance_nm,
          "Remaining distance decreases");
    auto energy = smartnav::PredictVesselEnergy(model, a, t);
    Check(energy.range.estimate && energy.arrival.estimate &&
              energy.arrival.estimate->soc_percent,
          "Cruise predictions");
    Check(*energy.arrival.estimate->soc_percent > model.reserve_soc_percent,
          "Cruise above reserve");
    Check(a.battery.net_discharge_kw.value >
              a.propulsion.electrical_power_kw.value,
          "Net load includes explicit hotel load");
    Check(AssessText(a.propulsion.gear, t + 5s).quality == Quality::Stale,
          "Discrete telemetry ages");
    DemoSource demo(t);
    demo.Select(DemoScenario::Stale, t);
    auto stale = demo.Read(t + 6s);
    Check(stale.battery.soc_percent.observed_at == t,
          "Read cannot refresh stopped sensors");
    Check(!smartnav::PredictVesselEnergy(model, stale, t + 6s).arrival.estimate,
          "Stale arrival withheld");
    Check(!AssessRoute(*stale.navigation.route, t + 6s).remaining_distance_nm,
          "Stale route withheld");
    demo.Select(DemoScenario::Cruise, t + 7s);
    demo.Pause(true, t + 8s);
    Check(!smartnav::PredictVesselEnergy(model, demo.Read(t + 14s), t + 14s)
               .arrival.estimate,
          "Paused demo expires");
    demo.Pause(false, t + 15s);
    Check(smartnav::PredictVesselEnergy(model, demo.Read(t + 15s), t + 15s)
              .arrival.estimate.has_value(),
          "Resume supplies new observation");
    auto missing = DemoFixture(DemoScenario::Unavailable, 0, t);
    Check(!missing.battery.net_discharge_kw.value &&
              !missing.wind.apparent_speed_kn.value,
          "Explicit sensor loss");
    Check(!smartnav::PredictVesselEnergy(model, missing, t).arrival.estimate,
          "Missing net power withholds arrival");
    auto inactive = DemoFixture(DemoScenario::RouteInactive, 0, t);
    auto no_route = smartnav::PredictVesselEnergy(model, inactive, t);
    Check(no_route.range.estimate && !no_route.arrival.estimate,
          "No route permits range only");
    auto ending = DemoFixture(DemoScenario::RouteEnding, 5, t);
    Check(ending.navigation.route->state == RouteState::NoActiveRoute &&
              !ending.navigation.route->remaining_distance_nm,
          "Route ends unavailable, not zero arrival");
    auto low = smartnav::PredictVesselEnergy(
        model, DemoFixture(DemoScenario::LowSoc, 0, t), t);
    Check(low.range.estimate && low.range.estimate->range_nm == 0 &&
              low.arrival.estimate->below_reserve,
          "Low SOC reserve behavior");
    auto high = smartnav::PredictVesselEnergy(
        model, DemoFixture(DemoScenario::HighPower, 0, t), t);
    Check(high.range.estimate->range_nm < energy.range.estimate->range_nm,
          "Higher power lowers range");
    auto shortfall = smartnav::PredictVesselEnergy(
        model, DemoFixture(DemoScenario::Insufficient, 0, t), t);
    Check(shortfall.arrival.estimate &&
              !shortfall.arrival.estimate->soc_percent &&
              shortfall.arrival.estimate->energy_shortfall_kwh > 0,
          "Shortfall never fabricates arrival SOC");
    bool transition = false;
    unsigned last = 0;
    for (unsigned sec = 0; sec < 174; ++sec) {
      auto s =
          DemoFixture(DemoScenario::Cruise, sec, t + std::chrono::seconds(sec));
      if (s.navigation.route->state == RouteState::ActivePointChanged) {
        transition = true;
        Check(!s.navigation.route->remaining_distance_nm,
              "Transition suppresses range");
      }
      if (s.navigation.route->active_waypoint_index)
        last =
            static_cast<unsigned>(*s.navigation.route->active_waypoint_index);
    }
    Check(transition && last == 2, "Trip advances waypoints");
    const auto retained = a.navigation.route;
    a = {};
    Check(retained->remaining_distance_nm == 18.2,
          "Owned route survives source reset");
    Check(!smartnav::PredictVesselEnergy(smartnav::PreviewEnergyModel(false), b,
                                         t)
               .arrival.estimate,
          "Live has no demo capacity default");
    std::cout << "Demo provenance, all eight scenarios, transitions, lifetime "
                 "and energy validity passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

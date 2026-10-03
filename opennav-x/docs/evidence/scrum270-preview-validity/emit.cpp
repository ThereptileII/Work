// Evidence adapter only: all trip, route validity and energy calculations are
// the unchanged application/fixture implementations linked by prove.py.
#include "vessel/DemoSource.h"
#include "smartnav/VesselEnergy.h"
#include <iomanip>
#include <iostream>
int main() {
  using namespace opennav;
  const vessel::Time start{std::chrono::seconds(100)};
  std::cout << std::setprecision(17) << "[";
  bool comma = false;
  for (unsigned second : {0u, 114u, 115u, 116u}) {
    const auto now = start + std::chrono::seconds(second);
    const auto state = vessel::DemoFixture(vessel::DemoScenario::Cruise, second, now);
    const auto route = vessel::AssessRoute(*state.navigation.route, now);
    const auto energy = smartnav::PredictVesselEnergy(smartnav::PreviewEnergyModel(true), state, now);
    if (comma) std::cout << ',';
    comma = true;
    std::cout << "{\"second\":" << second << ",\"route\":{\"state\":\""
              << vessel::RouteStateName(route.state) << "\"";
    if (route.remaining_distance_nm)
      std::cout << ",\"remaining_nm\":" << *route.remaining_distance_nm;
    std::cout << "},\"energy\":{";
    if (energy.arrival.estimate && energy.arrival.estimate->soc_percent)
      std::cout << "\"arrival_soc\":" << *energy.arrival.estimate->soc_percent;
    std::cout << "},\"data\":[{\"name\":\"Battery SOC\",\"value\":"
              << *state.battery.soc_percent.value
              << "},{\"name\":\"Latitude\",\"value\":"
              << *state.navigation.latitude_deg.value << "}]}";
  }
  std::cout << "]\n";
}

#include "smartnav/EnergyConfiguration.h"
#include "vessel/RouteProgress.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace smartnav;
using namespace std::chrono_literals;
void Check(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
const std::string csv = "OpenNavXPowerCurve,1\nreference,STW\nbasis,whole-"
                        "pack\nspeed_kn,power_kw\n2,1\n4,3\n6,9\n";
void Import() {
  const auto c = ImportPowerCurve(csv, "Test calibration only");
  Check(InterpolatePower(c, 3) == 2,
        "Linear interpolation between calibration observations");
  Check(InterpolatePower(c, 2) == 1 && InterpolatePower(c, 6) == 9,
        "Exact boundary samples");
  Check(!InterpolatePower(c, 1.99) && !InterpolatePower(c, 6.01),
        "Never extrapolate beyond measured speed domain");
  auto copy = ImportPowerCurve(ExportPowerCurve(c), c.source);
  Check(copy.points.size() == 3 && InterpolatePower(copy, 5) == 6,
        "Export/import roundtrip");
  for (auto bad :
       {std::string{}, csv + "4,3\n", csv + "7,nan\n", csv + "7,inf\n",
        csv + "7,0\n", csv + "7,2,3\n", csv + "\n"}) {
    bool caught = false;
    try {
      ImportPowerCurve(bad, "test");
    } catch (const std::invalid_argument &) {
      caught = true;
    }
    Check(caught,
          "Malformed, duplicate, unsorted, nonfinite or zero power rejected");
  }
  auto wrong = csv;
  wrong.replace(wrong.find("speed_kn"), 8, "speed_km");
  bool caught = false;
  try {
    ImportPowerCurve(wrong, "test");
  } catch (const std::invalid_argument &) {
    caught = true;
  }
  Check(caught, "Unit contract cannot change silently");
  wrong = csv;
  wrong.replace(wrong.find("reference,STW"), 13, "reference,???");
  caught = false;
  try {
    ImportPowerCurve(wrong, "test");
  } catch (const std::invalid_argument &) {
    caught = true;
  }
  Check(caught, "Speed reference mandatory");
  caught = false;
  try {
    ImportPowerCurve(csv, "");
  } catch (const std::invalid_argument &) {
    caught = true;
  }
  Check(caught, "Calibration provenance mandatory");
}
void Predictions() {
  const vessel::Time t{100s};
  auto observed = [&](double v, const std::string &device = "") {
    return vessel::Sample{
        v, "Test observation", t, vessel::Validity::Measured, {}, device};
  };
  vessel::VesselState s;
  s.navigation.sog_kn = observed(5);
  s.navigation.stw_kn = observed(4);
  s.battery.soc_percent = observed(80, "pack-1");
  s.battery.net_discharge_kw = observed(3, "pack-1");
  auto r = std::make_shared<vessel::RouteProgressSnapshot>();
  r->state = vessel::RouteState::Valid;
  r->route_id = "route";
  r->route_revision = 1;
  r->revision_scope = "test";
  r->active_waypoint_id = "wp";
  r->active_waypoint_index = 0;
  r->waypoint_count = 1;
  r->remaining_distance_nm = 10;
  r->observed_at = t;
  r->position_observed_at = t;
  r->source = "Upstream contract fixture";
  r->position_source = "Selected GPS";
  s.navigation.route = r;
  EnergyConfiguration c;
  c.battery = {20, 20, 0.5, "Explicit test battery configuration"};
  Check(!PredictConfiguredEnergy(c, s, t).arrival.estimate,
        "Live identity must be configured");
  c.battery_device_id = "pack-1";
  auto e = PredictConfiguredEnergy(c, s, t);
  Check(e.arrival.estimate &&
            std::abs(*e.arrival.estimate->soc_percent - 50) < 1e-9,
        "Live configured pack consumes remaining route contract");
  s.battery.net_discharge_kw.device_id = "pack-2";
  Check(!PredictConfiguredEnergy(c, s, t).arrival.estimate,
        "Cross-pack arrival suppressed");
  c.consumption = ConsumptionModel::CalibratedCurve;
  c.curve = ImportPowerCurve(csv, "Synthetic calibration");
  e = PredictConfiguredEnergy(c, s, t);
  Check(e.arrival.estimate &&
            std::abs(*e.arrival.estimate->soc_percent - 50) < 1e-9,
        "STW curve uses water speed, while passage time uses SOG");
  c.curve.basis = PowerBasis::Shaft;
  Check(!PredictConfiguredEnergy(c, s, t).arrival.estimate,
        "No guessed efficiency or hotel load");
  c.shaft_efficiency = .75;
  c.hotel_kw = .5;
  e = PredictConfiguredEnergy(c, s, t);
  Check(e.arrival.estimate &&
            std::abs(*e.arrival.estimate->soc_percent - 35) < 1e-9,
        "Configured shaft efficiency and hotel load become whole-pack "
        "consumption");
  s.navigation.stw_kn.observed_at = t - 5s;
  Check(!PredictConfiguredEnergy(c, s, t).arrival.estimate,
        "Stale curve speed blocks prediction");
  s.navigation.stw_kn = observed(8);
  Check(!PredictConfiguredEnergy(c, s, t).arrival.estimate,
        "No prediction outside calibration domain");
  s.navigation.stw_kn = observed(4);
  s.battery.soc_percent.observed_at = t - 5s;
  Check(!PredictConfiguredEnergy(c, s, t).arrival.estimate,
        "Stale SOC blocks curve prediction");
  s.battery.soc_percent.observed_at = t;
  r->state = vessel::RouteState::ActivePointChanged;
  e = PredictConfiguredEnergy(c, s, t);
  Check(e.range.estimate && !e.arrival.estimate,
        "Route transition suppresses destination, retains independent range");
  r->state = vessel::RouteState::Valid;
  r->remaining_distance_nm = 100;
  e = PredictConfiguredEnergy(c, s, t);
  Check(e.arrival.estimate && !e.arrival.estimate->soc_percent &&
            e.arrival.estimate->energy_shortfall_kwh > 0,
        "Insufficient energy is not a valid zero SOC arrival");
}
int main(int argc, char **argv) {
  try {
    Check(argc == 2, "Choose test group");
    std::string s = argv[1];
    if (s == "import")
      Import();
    else if (s == "prediction")
      Predictions();
    else
      throw std::runtime_error("Unknown group");
    std::cout << s << " passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

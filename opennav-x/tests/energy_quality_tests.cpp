#include "smartnav/EnergyConfiguration.h"
#include "smartnav/VesselEnergy.h"
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace std::chrono_literals;
void Check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
int main() {
  try {
    const vessel::Time now{100s};
    auto sample = [&](double n) {
      return vessel::Sample{n, "fixture", now, vessel::Validity::Measured};
    };
    smartnav::EnergyModel model{24, 20, .5, "fixture assumptions"};
    smartnav::EnergyInputs input{sample(68), sample(5), sample(4), sample(10)};
    auto predict = [&] { return smartnav::PredictEnergy(model, input, now); };
    auto p = predict();
    Check(p.arrival.quality == smartnav::EnergyQuality::Current,
          "Current measured inputs");
    input.soc_percent.observed_at -= 2500ms;
    p = predict();
    Check(p.arrival.estimate &&
              p.arrival.quality == smartnav::EnergyQuality::Aging,
          "Aging explicit while admissible");
    input.soc_percent.observed_at -= 3s;
    p = predict();
    Check(!p.arrival.estimate &&
              p.arrival.input == smartnav::EnergyInput::Soc &&
              p.arrival.quality == smartnav::EnergyQuality::Unavailable,
          "Stale SOC blocker");
    Check(smartnav::EnergyStatus(p.arrival.reason, p.arrival.input) ==
              "Battery SOC stale",
          "Actionable reason");
    input.soc_percent = sample(68);
    input.total_discharge_kw.validity = vessel::Validity::Estimated;
    p = predict();
    Check(p.arrival.quality == smartnav::EnergyQuality::Modeled,
          "Derived/modelled input is limited");
    input.total_discharge_kw.observed_at -= 6s;
    p = predict();
    Check(p.arrival.input == smartnav::EnergyInput::Consumption &&
              !p.range.estimate,
          "Power loss suppresses estimates");
    input.total_discharge_kw = sample(4);
    input.distance_remaining_nm = {};
    p = predict();
    Check(p.range.estimate &&
              p.arrival.input == smartnav::EnergyInput::RouteDistance,
          "Missing route not zero arrival");
    input.distance_remaining_nm = sample(10);
    input.soc_percent.validity = vessel::Validity::Invalid;
    p = predict();
    Check(p.arrival.reason == smartnav::EnergyReason::InvalidInput,
          "Invalid distinct from missing");
    input.soc_percent = {};
    p = predict();
    Check(p.arrival.reason == smartnav::EnergyReason::MissingInput,
          "Unobserved distinct from invalid");
    input.soc_percent = sample(68);
    input.sog_kn = sample(0);
    p = predict();
    Check(p.arrival.input == smartnav::EnergyInput::Speed,
          "Stopped speed blocker");
    input.distance_remaining_nm = sample(0);
    p = predict();
    Check(p.arrival.estimate &&
              p.arrival.quality == smartnav::EnergyQuality::Current &&
              !p.range.estimate,
          "Coherent arrival needs no power forecast");
    vessel::VesselState s;
    s.battery.soc_percent = sample(68);
    s.battery.soc_percent.device_id = "pack";
    smartnav::EnergyConfiguration c;
    c.battery = model;
    c.battery_device_id = "wrong";
    p = smartnav::PredictConfiguredEnergy(c, s, now);
    Check(p.arrival.input == smartnav::EnergyInput::BatteryIdentity,
          "Wrong pack explanation");
    c.battery_device_id = "pack";
    c.consumption = smartnav::ConsumptionModel::CalibratedCurve;
    p = smartnav::PredictConfiguredEnergy(c, s, now);
    Check(p.arrival.input == smartnav::EnergyInput::Curve,
          "Missing curve explanation");
    c.curve =
        smartnav::ImportPowerCurve("OpenNavXPowerCurve,1\nreference,STW\nbasis,"
                                   "whole-pack\nspeed_kn,power_kw\n3,2\n5,4\n",
                                   "fixture");
    p = smartnav::PredictConfiguredEnergy(c, s, now);
    Check(p.arrival.input == smartnav::EnergyInput::CurveSpeed,
          "Missing STW explanation");
    s.navigation.stw_kn = sample(8);
    p = smartnav::PredictConfiguredEnergy(c, s, now);
    Check(!p.arrival.estimate &&
              p.arrival.reason == smartnav::EnergyReason::InvalidInput &&
              p.arrival.input == smartnav::EnergyInput::CurveSpeed,
          "No extrapolated confidence");
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}

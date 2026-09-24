#include "smartnav/VesselEnergy.h"
namespace opennav::smartnav {
EnergyModel PreviewEnergyModel(bool demo) {
  return demo ? EnergyModel{48, 15, 0.5,
                            "DEMO: 48 kWh usable, 15% reserve; constant speed "
                            "and whole-pack discharge"}
              : EnergyModel{};
}
EnergyPrediction PredictVesselEnergy(const EnergyModel &m,
                                     const vessel::VesselState &s,
                                     vessel::Time now) {
  EnergyInputs inputs{s.battery.soc_percent,
                      s.navigation.sog_kn,
                      s.battery.net_discharge_kw,
                      {}};
  if (s.navigation.route)
    inputs.distance_remaining_nm =
        vessel::RouteDistanceSample(*s.navigation.route, now);
  auto prediction = PredictEnergy(m, inputs, now);
  prediction.input_route = s.navigation.route;
  return prediction;
}
const char *EnergyReasonName(EnergyReason r) {
  switch (r) {
  case EnergyReason::None:
    return "Valid advisory estimate";
  case EnergyReason::InvalidModel:
    return "Battery capacity / reserve not configured";
  case EnergyReason::MissingInput:
    return "Required input unavailable";
  case EnergyReason::InvalidInput:
    return "Invalid input";
  case EnergyReason::StaleInput:
    return "Required input stale";
  case EnergyReason::UncertainInput:
    return "Input uncertain";
  case EnergyReason::NotUnderway:
    return "Below minimum speed";
  case EnergyReason::NotDischarging:
    return "Battery not discharging";
  case EnergyReason::ArithmeticLimit:
    return "Calculation outside supported range";
  }
  return "Unavailable";
}
} // namespace opennav::smartnav

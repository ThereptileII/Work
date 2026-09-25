#include "smartnav/VesselEnergy.h"
namespace opennav::smartnav {
const char *EnergyInputName(EnergyInput i) {
  switch (i) {
  case EnergyInput::None:
    return "";
  case EnergyInput::Model:
    return "Battery capacity / reserve";
  case EnergyInput::BatteryIdentity:
    return "Selected battery identity";
  case EnergyInput::Soc:
    return "Battery SOC";
  case EnergyInput::RouteDistance:
    return "Active route distance";
  case EnergyInput::Speed:
    return "Speed over ground";
  case EnergyInput::Consumption:
    return "Whole-pack consumption";
  case EnergyInput::Curve:
    return "Propulsion curve";
  case EnergyInput::CurveSpeed:
    return "Curve reference speed";
  case EnergyInput::HotelLoad:
    return "Auxiliary load";
  case EnergyInput::Efficiency:
    return "Shaft efficiency";
  }
  return "Unknown input";
}
const char *EnergyQualityName(EnergyQuality q) {
  switch (q) {
  case EnergyQuality::Unavailable:
    return "UNAVAILABLE";
  case EnergyQuality::Current:
    return "CURRENT INPUTS";
  case EnergyQuality::Aging:
    return "LIMITED / AGING INPUTS";
  case EnergyQuality::Modeled:
    return "LIMITED / ESTIMATED INPUTS";
  }
  return "UNAVAILABLE";
}
std::string EnergyStatus(EnergyReason r, EnergyInput i) {
  const auto input = std::string(EnergyInputName(i));
  if (r == EnergyReason::None)
    return "Valid advisory estimate";
  if (input.empty())
    return EnergyReasonName(r);
  switch (r) {
  case EnergyReason::MissingInput:
    return input + " unavailable";
  case EnergyReason::StaleInput:
    return input + " stale";
  case EnergyReason::UncertainInput:
    return input + " uncertain";
  case EnergyReason::InvalidInput:
    return input + " invalid / outside model domain";
  case EnergyReason::InvalidModel:
    return input + " not configured";
  default:
    return input + ": " + EnergyReasonName(r);
  }
}
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

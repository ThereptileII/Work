#pragma once

#include "vessel/VesselState.h"

#include <limits>
#include <optional>
#include <string>

namespace opennav::smartnav {

// Explicit configuration; no boat capacity or reserve is guessed.
// Capacity is deliverable energy across the BMS's reported 0..100% SOC range.
struct EnergyModel {
  double capacity_kwh = std::numeric_limits<double>::quiet_NaN();
  double reserve_soc_percent = std::numeric_limits<double>::quiet_NaN();
  double minimum_speed_kn = 0.5;
  std::string source;
};

struct EnergyInputs {
  vessel::Sample soc_percent;
  vessel::Sample sog_kn;
  // Positive NET discharge from the whole battery, including hotel loads.
  // Motor-only power cannot be substituted for this quantity.
  vessel::Sample total_discharge_kw;
  vessel::Sample distance_remaining_nm;
};

enum class EnergyReason {
  None, InvalidModel, MissingInput, InvalidInput, StaleInput,
  UncertainInput, NotUnderway, NotDischarging, ArithmeticLimit
};

template <class T> struct Prediction {
  std::optional<T> estimate;
  EnergyReason reason = EnergyReason::MissingInput;
};

struct RangeEstimate {
  double range_nm, endurance_hours, energy_above_reserve_kwh;
};
struct ArrivalEstimate {
  double passage_hours, energy_required_kwh;
  // Empty when the constant-condition model exhausts the pack before arrival.
  // Never report a negative physical SOC or a fabricated successful arrival.
  std::optional<double> soc_percent;
  double energy_shortfall_kwh;
  bool below_reserve;
};
struct EnergyPrediction {
  Prediction<RangeEstimate> range;
  Prediction<ArrivalEstimate> arrival;
  std::string model_source;
  vessel::Time calculated_at{};
};

// Every output is an advisory ESTIMATE: constant present speed/net discharge,
// linear SOC-energy relationship, no weather/current/route-leg forecast.
// No widgets, OpenCPN globals, device commands or automatic steering.
EnergyPrediction PredictEnergy(const EnergyModel& model, const EnergyInputs& inputs,
                               vessel::Time now, vessel::Freshness freshness = {});

}  // namespace opennav::smartnav

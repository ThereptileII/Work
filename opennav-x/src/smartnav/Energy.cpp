#include "smartnav/Energy.h"

#include <algorithm>
#include <cmath>

namespace opennav::smartnav {
namespace {
EnergyReason Check(const vessel::Sample& sample, vessel::Time now, vessel::Freshness freshness) {
  if (sample.value && !std::isfinite(*sample.value)) return EnergyReason::InvalidInput;
  const auto assessed = vessel::Assess(sample, now, freshness);
  switch (assessed.quality) {
    case vessel::Quality::Live:
    case vessel::Quality::Aging:
    case vessel::Quality::Estimated: return EnergyReason::None;
    case vessel::Quality::Stale: return EnergyReason::StaleInput;
    case vessel::Quality::Uncertain: return EnergyReason::UncertainInput;
    case vessel::Quality::Unavailable: return EnergyReason::MissingInput;
  }
  return EnergyReason::InvalidInput;
}

bool ValidModel(const EnergyModel& model) {
  return std::isfinite(model.capacity_kwh) && model.capacity_kwh > 0 &&
         std::isfinite(model.reserve_soc_percent) && model.reserve_soc_percent >= 0 &&
         model.reserve_soc_percent <= 100 && std::isfinite(model.minimum_speed_kn) &&
         model.minimum_speed_kn > 0 && !model.source.empty();
}
}  // namespace

EnergyPrediction PredictEnergy(const EnergyModel& model, const EnergyInputs& inputs,
                               vessel::Time now, vessel::Freshness freshness) {
  EnergyPrediction result;
  result.model_source = model.source;
  result.calculated_at = now;
  const auto fail_both = [&result](EnergyReason reason) {
    result.range.reason = result.arrival.reason = reason;
    return result;
  };
  if (!ValidModel(model)) return fail_both(EnergyReason::InvalidModel);
  auto reason = Check(inputs.soc_percent, now, freshness);
  if (reason != EnergyReason::None) return fail_both(reason);
  const double soc = *inputs.soc_percent.value;
  if (soc < 0 || soc > 100) return fail_both(EnergyReason::InvalidInput);
  const double remaining = model.capacity_kwh * (soc / 100);
  const double usable = model.capacity_kwh * (std::max(0.0, soc - model.reserve_soc_percent) / 100);
  if ((soc > 0 && remaining == 0) || (soc > model.reserve_soc_percent && usable == 0))
    return fail_both(EnergyReason::ArithmeticLimit);

  // Range does not require a route. Arrival does not pretend that a missing
  // distance is zero; the caller must supply a current route-distance sample.
  const auto distance_reason = Check(inputs.distance_remaining_nm, now, freshness);
  if (distance_reason != EnergyReason::None) result.arrival.reason = distance_reason;
  else if (*inputs.distance_remaining_nm.value < 0) result.arrival.reason = EnergyReason::InvalidInput;
  else result.arrival.reason = EnergyReason::None;

  EnergyReason underway = Check(inputs.sog_kn, now, freshness);
  if (underway == EnergyReason::None) {
    if (*inputs.sog_kn.value < 0) underway = EnergyReason::InvalidInput;
    else if (*inputs.sog_kn.value < model.minimum_speed_kn) underway = EnergyReason::NotUnderway;
  }
  if (underway == EnergyReason::None) {
    underway = Check(inputs.total_discharge_kw, now, freshness);
    if (underway == EnergyReason::None && *inputs.total_discharge_kw.value <= 0)
      underway = EnergyReason::NotDischarging;
  }
  result.range.reason = underway;
  if (underway == EnergyReason::None) {
    const double hours = usable / *inputs.total_discharge_kw.value;
    const double distance = hours * *inputs.sog_kn.value;
    if (std::isfinite(hours) && std::isfinite(distance) &&
        !(usable > 0 && (hours == 0 || distance == 0)))
      result.range.estimate = RangeEstimate{distance, hours, usable};
    else result.range.reason = EnergyReason::ArithmeticLimit;
  }

  if (result.arrival.reason != EnergyReason::None) return result;
  const double distance = *inputs.distance_remaining_nm.value;
  if (distance == 0) {
    // Already at the supplied destination: no motion or power assumption needed.
    result.arrival.estimate = ArrivalEstimate{0, 0, soc, 0, soc < model.reserve_soc_percent};
    return result;
  }
  if (underway != EnergyReason::None) {
    result.arrival.reason = underway;
    return result;
  }
  const double hours = distance / *inputs.sog_kn.value;
  const double required = hours * *inputs.total_discharge_kw.value;
  if (!std::isfinite(hours) || !std::isfinite(required) || hours == 0 || required == 0) {
    result.arrival.reason = EnergyReason::ArithmeticLimit;
    return result;
  }
  const bool depleted = required > remaining;
  const auto arrival_soc = depleted ? std::optional<double>{}
      : std::optional<double>{std::clamp((remaining - required) / model.capacity_kwh * 100, 0.0, 100.0)};
  result.arrival.estimate = ArrivalEstimate{hours, required, arrival_soc,
      std::max(0.0, required - remaining), required > usable || soc < model.reserve_soc_percent};
  return result;
}

}  // namespace opennav::smartnav

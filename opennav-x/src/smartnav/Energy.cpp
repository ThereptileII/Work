#include "smartnav/Energy.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>

namespace opennav::smartnav {
namespace {
EnergyReason Check(const vessel::Sample &sample, vessel::Time now,
                   vessel::Freshness freshness) {
  if (sample.value && !std::isfinite(*sample.value))
    return EnergyReason::InvalidInput;
  if (sample.validity == vessel::Validity::Invalid &&
      (sample.value || !sample.source.empty()))
    return EnergyReason::InvalidInput;
  // The consumer ceiling cannot relax a stricter per-source policy, and a
  // long source threshold cannot relax the established energy freshness gate.
  freshness.aging_after =
      std::min(freshness.aging_after, sample.freshness.aging_after);
  freshness.stale_after =
      std::min(freshness.stale_after, sample.freshness.stale_after);
  const auto assessed = vessel::Assess(sample, now, freshness);
  switch (assessed.quality) {
  case vessel::Quality::Live:
  case vessel::Quality::Aging:
  case vessel::Quality::Estimated:
    return EnergyReason::None;
  case vessel::Quality::Stale:
    return EnergyReason::StaleInput;
  case vessel::Quality::Uncertain:
    return EnergyReason::UncertainInput;
  case vessel::Quality::Unavailable:
    return EnergyReason::MissingInput;
  }
  return EnergyReason::InvalidInput;
}

bool ValidModel(const EnergyModel &model) {
  return std::isfinite(model.capacity_kwh) && model.capacity_kwh > 0 &&
         std::isfinite(model.reserve_soc_percent) &&
         model.reserve_soc_percent >= 0 && model.reserve_soc_percent <= 100 &&
         std::isfinite(model.minimum_speed_kn) && model.minimum_speed_kn > 0 &&
         !model.source.empty();
}
EnergyQuality InputQuality(std::initializer_list<const vessel::Sample *> inputs,
                           vessel::Time now, vessel::Freshness policy) {
  auto quality = EnergyQuality::Current;
  for (const auto *s : inputs) {
    if (now - s->observed_at >=
        std::min(policy.aging_after, s->freshness.aging_after))
      return EnergyQuality::Aging;
    if (s->validity == vessel::Validity::Estimated)
      quality = EnergyQuality::Modeled;
  }
  return quality;
}
} // namespace

EnergyPrediction PredictEnergy(const EnergyModel &model,
                               const EnergyInputs &inputs, vessel::Time now,
                               vessel::Freshness freshness) {
  EnergyPrediction result;
  result.model_source = model.source;
  result.calculated_at = now;
  const auto fail_both = [&result](EnergyReason reason, EnergyInput input) {
    result.range.reason = result.arrival.reason = reason;
    result.range.input = result.arrival.input = input;
    return result;
  };
  if (!ValidModel(model))
    return fail_both(EnergyReason::InvalidModel, EnergyInput::Model);
  auto reason = Check(inputs.soc_percent, now, freshness);
  if (reason != EnergyReason::None)
    return fail_both(reason, EnergyInput::Soc);
  const double soc = *inputs.soc_percent.value;
  if (soc < 0 || soc > 100)
    return fail_both(EnergyReason::InvalidInput, EnergyInput::Soc);
  const double remaining = model.capacity_kwh * (soc / 100);
  const double usable = model.capacity_kwh *
                        (std::max(0.0, soc - model.reserve_soc_percent) / 100);
  if ((soc > 0 && remaining == 0) ||
      (soc > model.reserve_soc_percent && usable == 0))
    return fail_both(EnergyReason::ArithmeticLimit, EnergyInput::Model);

  // Range does not require a route. Arrival does not pretend that a missing
  // distance is zero; the caller must supply a current route-distance sample.
  const auto distance_reason =
      Check(inputs.distance_remaining_nm, now, freshness);
  if (distance_reason != EnergyReason::None)
    result.arrival.reason = distance_reason;
  else if (*inputs.distance_remaining_nm.value < 0)
    result.arrival.reason = EnergyReason::InvalidInput;
  else
    result.arrival.reason = EnergyReason::None;
  if (result.arrival.reason != EnergyReason::None)
    result.arrival.input = EnergyInput::RouteDistance;

  EnergyReason underway = Check(inputs.sog_kn, now, freshness);
  auto underway_input = EnergyInput::Speed;
  if (underway == EnergyReason::None) {
    if (*inputs.sog_kn.value < 0)
      underway = EnergyReason::InvalidInput;
    else if (*inputs.sog_kn.value < model.minimum_speed_kn)
      underway = EnergyReason::NotUnderway;
  }
  if (underway == EnergyReason::None) {
    underway_input = EnergyInput::Consumption;
    underway = Check(inputs.total_discharge_kw, now, freshness);
    if (underway == EnergyReason::None && *inputs.total_discharge_kw.value <= 0)
      underway = EnergyReason::NotDischarging;
  }
  result.range.reason = underway;
  if (underway != EnergyReason::None)
    result.range.input = underway_input;
  if (underway == EnergyReason::None) {
    const double hours = usable / *inputs.total_discharge_kw.value;
    const double distance = hours * *inputs.sog_kn.value;
    if (std::isfinite(hours) && std::isfinite(distance) &&
        !(usable > 0 && (hours == 0 || distance == 0)))
      result.range.estimate = RangeEstimate{distance, hours, usable};
    else {
      result.range.reason = EnergyReason::ArithmeticLimit;
      result.range.input = EnergyInput::Consumption;
    }
  }
  if (result.range.estimate)
    result.range.quality = InputQuality(
        {&inputs.soc_percent, &inputs.sog_kn, &inputs.total_discharge_kw}, now,
        freshness);

  if (result.arrival.reason != EnergyReason::None)
    return result;
  const double distance = *inputs.distance_remaining_nm.value;
  if (distance == 0) {
    // Already at the supplied destination: no motion or power assumption
    // needed.
    result.arrival.estimate =
        ArrivalEstimate{0, 0, soc, 0, soc < model.reserve_soc_percent};
    result.arrival.quality = InputQuality(
        {&inputs.soc_percent, &inputs.distance_remaining_nm}, now, freshness);
    return result;
  }
  if (underway != EnergyReason::None) {
    result.arrival.reason = underway;
    result.arrival.input = underway_input;
    return result;
  }
  const double hours = distance / *inputs.sog_kn.value;
  const double required = hours * *inputs.total_discharge_kw.value;
  if (!std::isfinite(hours) || !std::isfinite(required) || hours == 0 ||
      required == 0) {
    result.arrival.reason = EnergyReason::ArithmeticLimit;
    result.arrival.input = EnergyInput::Consumption;
    return result;
  }
  const bool depleted = required > remaining;
  const auto arrival_soc =
      depleted
          ? std::optional<double>{}
          : std::optional<double>{std::clamp(
                (remaining - required) / model.capacity_kwh * 100, 0.0, 100.0)};
  result.arrival.estimate = ArrivalEstimate{
      hours, required, arrival_soc, std::max(0.0, required - remaining),
      required > usable || soc < model.reserve_soc_percent};
  result.arrival.quality =
      InputQuality({&inputs.soc_percent, &inputs.sog_kn,
                    &inputs.total_discharge_kw, &inputs.distance_remaining_nm},
                   now, freshness);
  return result;
}

} // namespace opennav::smartnav

#include "application/EnergyView.h"
#include <cmath>

namespace opennav::application {
namespace {
vessel::Assessment Current(const vessel::Sample &sample, vessel::Time now) {
  auto value = vessel::Assess(sample, now);
  if (value.quality != vessel::Quality::Live &&
      value.quality != vessel::Quality::Aging &&
      value.quality != vessel::Quality::Estimated)
    value.value.reset();
  return value;
}
template <class T>
void Withhold(smartnav::Prediction<T> &p, smartnav::EnergyReason reason,
               smartnav::EnergyInput input) {
  p.estimate.reset();
  p.reason = reason;
  p.input = input;
  p.quality = smartnav::EnergyQuality::Unavailable;
}
} // namespace
EnergyView PresentEnergy(const vessel::VesselState &state,
                         const smartnav::EnergyModel &model,
                         const smartnav::EnergyPrediction &prediction,
                         const smartnav::NavigationAdvice &advice,
                         vessel::Time now) {
  EnergyView view;
  view.passage = PresentPassage(state, advice, prediction, now);
  view.prediction = prediction;
  view.soc = Current(state.battery.soc_percent, now);
  if (view.soc.value && (*view.soc.value < 0 || *view.soc.value > 100)) {
    view.soc.value.reset();
    view.soc.quality = vessel::Quality::Unavailable;
  }
  view.capacity = Current(state.battery.usable_capacity_kwh, now);
  view.voltage = Current(state.battery.voltage_v, now);
  view.current = Current(state.battery.current_a, now);
  view.power = Current(state.propulsion.electrical_power_kw, now);
  view.rpm = Current(state.propulsion.motor_rpm, now);
  view.temperature = Current(state.propulsion.motor_temperature_c, now);
  // Capacity is configuration or an explicitly assessed estimate, never a
  // measured energy counter. The view labels this quantity as estimated.
  const auto capacity = view.capacity.value.value_or(model.capacity_kwh);
  if (view.soc.value && std::isfinite(capacity) && capacity > 0 && capacity <= 100000)
    view.remaining_kwh = capacity * *view.soc.value / 100.;
  if (prediction.calculated_at != now || !view.soc.value) {
    const auto input = view.soc.value ? smartnav::EnergyInput::None
                                      : smartnav::EnergyInput::Soc;
    const auto reason = prediction.calculated_at != now ||
                                view.soc.quality == vessel::Quality::Stale
                            ? smartnav::EnergyReason::StaleInput
                            : smartnav::EnergyReason::MissingInput;
    // Preserve the model's specific failure (identity, calibration, etc.) when
    // it has already withheld an estimate in this same observation batch.
    if (prediction.calculated_at != now || view.prediction.range.estimate)
      Withhold(view.prediction.range, reason, input);
    if (prediction.calculated_at != now || view.prediction.arrival.estimate)
      Withhold(view.prediction.arrival, reason, input);
  } else if (!view.passage.current || prediction.input_route != state.navigation.route) {
    if (view.prediction.arrival.estimate)
      Withhold(view.prediction.arrival, smartnav::EnergyReason::MissingInput,
                 smartnav::EnergyInput::RouteDistance);
  }
  return view;
}
} // namespace opennav::application

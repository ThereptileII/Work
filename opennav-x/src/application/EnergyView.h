#pragma once
#include "application/PassageView.h"

namespace opennav::application {
// One copied presentation batch. No retained OpenCPN objects or independent
// range/route calculation; the tested energy model owns all forecasts.
struct EnergyView {
  PassageView passage;
  smartnav::EnergyPrediction prediction;
  vessel::Assessment soc, capacity, voltage, current, power, rpm, temperature;
  std::optional<double> remaining_kwh;
};
EnergyView PresentEnergy(const vessel::VesselState &state,
                         const smartnav::EnergyModel &model,
                         const smartnav::EnergyPrediction &prediction,
                         const smartnav::NavigationAdvice &advice,
                         vessel::Time now);
} // namespace opennav::application

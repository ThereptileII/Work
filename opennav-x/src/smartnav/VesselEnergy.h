#pragma once
#include "smartnav/Energy.h"
#include "vessel/RouteProgress.h"

namespace opennav::smartnav {
// Presentation bridge owns only values. Live capacity/reserve are deliberately
// unconfigured until a separate source/configuration contract is implemented.
EnergyModel PreviewEnergyModel(bool demo);
EnergyPrediction PredictVesselEnergy(const EnergyModel &model,
                                     const vessel::VesselState &state,
                                     vessel::Time now);
const char *EnergyReasonName(EnergyReason reason);
} // namespace opennav::smartnav

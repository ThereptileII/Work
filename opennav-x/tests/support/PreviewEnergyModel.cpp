#include "smartnav/VesselEnergy.h"

// Synthetic battery configuration is never linked into the installed product.
namespace opennav::smartnav {
EnergyModel PreviewEnergyModel(bool demo) {
  return demo ? EnergyModel{48, 15, 0.5,
                            "DEMO: 48 kWh usable, 15% reserve; constant speed "
                            "and whole-pack discharge"}
              : EnergyModel{};
}
} // namespace opennav::smartnav

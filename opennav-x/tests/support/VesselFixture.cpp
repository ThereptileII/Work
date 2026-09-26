#include "vessel/VesselState.h"

// Linked only into dedicated tests or explicitly enabled developer builds.
namespace opennav::vessel {
VesselState SimulatorFixture(Time observed_at) {
  const auto measured = [observed_at](double value) {
    return Sample{value, "OpenNav simulator", observed_at, Validity::Measured};
  };
  VesselState state;
  state.simulated = true;
  state.navigation.sog_kn = measured(6.3);
  state.navigation.cog_deg = measured(147.0);
  state.environment.depth_below_transducer_m = measured(8.4);
  state.wind.apparent_speed_kn = measured(16.2);
  state.wind.apparent_angle_deg = measured(72.0);
  // Deliberately no position, heading or control-device state: this fixture
  // must not move the real chart, imply an autopilot mode or emit commands.
  return state;
}

} // namespace opennav::vessel

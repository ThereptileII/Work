#pragma once
#include "adapters/Autopilot.h"

namespace opennav::application {
// Owned display values and manual-button availability only. This model has no
// transport or SmartNav dependency and cannot issue a command.
struct PilotPresentation {
  adapters::PilotMode mode = adapters::PilotMode::Unavailable;
  std::optional<double> heading_magnetic_deg, actual_heading_magnetic_deg;
  bool commanded = false, pending = false, enabled = false;
  bool available = false, degraded = false;
  bool output_unavailable = false;
  bool standby = false, auto_mode = false, track = false, wind = false;
  bool alter_course = false, can_toggle = false;
  std::string state, connection, note;
};
PilotPresentation PresentPilot(const adapters::PilotView &, vessel::Time now,
                               bool permit_control, bool replayed);
} // namespace opennav::application

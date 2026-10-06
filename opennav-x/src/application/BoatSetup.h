#pragma once
#include "application/DisplayPreferences.h"
#include "application/Settings.h"
#include "application/SourceHealthView.h"
namespace opennav::application {
enum class BoatSetupState { Pending, Complete, ExistingProfile, Invalid };
BoatSetupState ReadBoatSetupState(const std::optional<std::string>& record,
                                 bool fresh_profile);
struct BoatSetupDraft {
  Settings settings;
  DisplayPreferences display;
  std::string vessel_name;
  double safety_depth_m = std::numeric_limits<double>::quiet_NaN();
};
// Only configuration assumptions; no telemetry or actuator permission changes.
void ValidateBoatSetup(const BoatSetupDraft&);
std::vector<std::string> BoatSetupSensorSummary(const vessel::VesselState&,
    const std::vector<vessel::SourceHealth>&, vessel::Time now);
std::vector<std::string> BoatSetupSummary(const BoatSetupDraft&);
} // namespace opennav::application

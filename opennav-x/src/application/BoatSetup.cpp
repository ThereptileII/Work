#include "application/BoatSetup.h"
#include <cmath>
#include <stdexcept>
namespace opennav::application {
BoatSetupState ReadBoatSetupState(const std::optional<std::string>& record,
                                 bool fresh_profile) {
  if (!record) return fresh_profile ? BoatSetupState::Pending : BoatSetupState::ExistingProfile;
  if (*record == "v1|pending") return BoatSetupState::Pending;
  if (*record == "v1|complete") return BoatSetupState::Complete;
  if (*record == "v1|existing") return BoatSetupState::ExistingProfile;
  return BoatSetupState::Invalid;
}
void ValidateBoatSetup(const BoatSetupDraft& d) {
  ValidateSettings(d.settings);
  if (!ValidDisplayPreferences(d.display)) throw std::invalid_argument("Choose supported display preferences");
  if (d.vessel_name.size() > 128 ||
      (!d.vessel_name.empty() && d.vessel_name.find_first_not_of(" \t\r\n") == std::string::npos))
    throw std::invalid_argument("Enter a vessel name, or leave it unconfigured");
  for (unsigned char c : d.vessel_name)
    if (c < 32 || c == 127) throw std::invalid_argument("Vessel name contains a control character");
  if (!std::isnan(d.safety_depth_m) && (!std::isfinite(d.safety_depth_m) || d.safety_depth_m < 0 || d.safety_depth_m > 1000000))
    throw std::invalid_argument("Safety depth must be a nonnegative number in metres");
  if (std::isfinite(d.settings.hazard.draft_m) && std::isfinite(d.safety_depth_m) &&
      d.safety_depth_m < d.settings.hazard.draft_m)
    throw std::invalid_argument("Safety depth must be at least the vessel draft");
}
std::vector<std::string> BoatSetupSensorSummary(const vessel::VesselState& state,
    const std::vector<vessel::SourceHealth>& sources, vessel::Time now) {
  if (state.simulated || state.replayed)
    return {"Live sensor check unavailable during Demo or Replay. Return to Live to inspect your boat."};
  const auto gps = PresentPositionHealth(state.navigation, now);
  std::vector<std::string> result{"GPS: " + gps.status + (gps.source.empty() ? "" : " — " + gps.source)};
  for (const auto& source : sources) {
    const auto health = PresentSignalHealth(source.sample, now);
    result.push_back(std::string(vessel::Describe(source.quantity).name) + ": " + health.status +
        " — " + source.source_id + (source.selected ? " (selected)" : ""));
  }
  if (sources.empty()) result.push_back("No instrument sources detected. No readings have been substituted.");
  return result;
}
std::vector<std::string> BoatSetupSummary(const BoatSetupDraft& d) {
  auto number = [](double n, const char* unit) { return std::isfinite(n) ? SettingNumber(n) + unit : "Unconfigured"; };
  return {"Vessel: " + (d.vessel_name.empty() ? "Unconfigured" : d.vessel_name),
    "Draft: " + number(d.settings.hazard.draft_m," m"),
    "Chart safety depth: " + number(d.safety_depth_m," m"),
    "Display: " + std::to_string(d.display.scale_percent) + "% / " +
        (d.display.layout == ChartLayout::Balanced ? "Balanced" : d.display.layout == ChartLayout::ChartFocus ? "Chart focus" : "Instrument focus"),
    "Usable capacity assumption: " + number(d.settings.energy.battery.capacity_kwh," kWh"),
    "Reserve assumption: " + number(d.settings.energy.battery.reserve_soc_percent,"%"),
    "Energy predictions require live battery data and a configured source/consumption model.",
    "Autopilot setup is status only. It cannot enable control or send commands."};
}
} // namespace opennav::application

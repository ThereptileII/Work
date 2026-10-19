#pragma once
#include "adapters/St4000Pilot.h"
#include "adapters/BoatN2k.h"
#include "application/MarineMapping.h"
#include "smartnav/EnergyConfiguration.h"
#include "smartnav/HazardLookAhead.h"
#include "vessel/SensorRegistry.h"
#include <map>

namespace opennav::application {
struct RailPreset {
  std::string name;
  std::vector<std::string> values;
  bool operator==(const RailPreset &other) const {
    return name == other.name && values == other.values;
  }
};
// Separately versioned profile storage keeps old AlphaSettings readers usable.
// The complete Settings codec still includes presets in portable backups.
std::string EncodeRailPresets(const std::vector<RailPreset> &presets);
std::vector<RailPreset> DecodeRailPresets(const std::string &record);
// Configuration is not sensor telemetry. Missing numeric configuration is NaN;
// it never becomes a freshly observed battery/safety datum on a UI read.
struct Settings {
  std::vector<std::string> data_rail{"sog", "depth", "aws", "heading"};
  // Named presets describe four display quantities, never sensor settings.
  // SettingsStore keeps these outside the downgrade-compatible core record.
  std::vector<RailPreset> rail_presets;
  std::vector<std::string> instruments{
      "sog", "cog",   "heading",    "stw",      "aws",    "awa", "tws",
      "twa", "depth", "water_temp", "pressure", "rudder", "heel"};
  smartnav::EnergyConfiguration energy;
  vessel::CurrentConvention current = vessel::CurrentConvention::Unconfigured;
  smartnav::HazardConfiguration hazard;
  std::map<vessel::Quantity, vessel::SourcePolicy> sources;
  std::vector<SignalKMapping> signal_k_mappings;
  adapters::St4000Binding pilot;
  adapters::BoatN2kBinding boat_bridge;
};
// Bounded, complete serialization for backups and prior inline-preset records.
// Profile storage strips presets into their own key for older binary readers.
// Rejects unknown fields, duplicates, malformed numbers and unsupported versions.
std::string EncodeSettings(const Settings &settings);
Settings DecodeSettings(const std::string &record);
void ValidateSettings(const Settings &settings);
// UI/config numeric parsing uses canonical decimal units, never locale guesses.
// Empty means unconfigured. Nonfinite spellings and trailing text are rejected.
double ParseSettingNumber(const std::string &text);
std::string SettingNumber(double value);
} // namespace opennav::application

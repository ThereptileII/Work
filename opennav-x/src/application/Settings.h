#pragma once
#include "smartnav/EnergyConfiguration.h"
#include "smartnav/HazardLookAhead.h"
#include "vessel/SensorRegistry.h"
#include <map>

namespace opennav::application {
// Configuration is not sensor telemetry. Missing numeric configuration is NaN;
// it never becomes a freshly observed battery/safety datum on a UI read.
struct Settings {
  smartnav::EnergyConfiguration energy;
  vessel::CurrentConvention current = vessel::CurrentConvention::Unconfigured;
  smartnav::HazardConfiguration hazard;
  std::map<vessel::Quantity, vessel::SourcePolicy> sources;
};
// Bounded, versioned serialization for one OpenCPN profile entry. Rejects
// unknown fields, duplicates, malformed numbers and unsupported versions.
std::string EncodeSettings(const Settings &settings);
Settings DecodeSettings(const std::string &record);
void ValidateSettings(const Settings &settings);
// UI/config numeric parsing uses canonical decimal units, never locale guesses.
// Empty means unconfigured. Nonfinite spellings and trailing text are rejected.
double ParseSettingNumber(const std::string &text);
std::string SettingNumber(double value);
} // namespace opennav::application

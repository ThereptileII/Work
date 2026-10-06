#pragma once
#include "application/Settings.h"
#include "application/DisplayPreferences.h"
#include <cstddef>
namespace opennav::application {
constexpr std::size_t SettingsBackupLimit = 128 * 1024;
struct SettingsBackup {
  Settings settings;
  DisplayPreferences display;
  std::string vessel_name;
  double chart_safety_depth_m = std::numeric_limits<double>::quiet_NaN();
};
// Version 1 exports only explicitly supported SKAGER configuration. Pilot
// binding/permissions and all host files, credentials and navigation objects
// are excluded. Import rejects them, rather than silently accepting extra data.
std::string EncodeSettingsBackup(SettingsBackup backup);
SettingsBackup DecodeSettingsBackup(const std::string &record);
void ValidateSettingsBackup(const SettingsBackup &backup);
std::string SettingsBackupPreview(const SettingsBackup &backup);
} // namespace opennav::application

#pragma once
#include "application/NavigationObjects.h"
#include "application/Settings.h"
#include "application/SettingsBackup.h"
#include "application/BoatSetup.h"
#include "application/DisplayPreferences.h"
#include <wx/fileconf.h>
namespace opennav::integration {
class SettingsStore {
public:
  explicit SettingsStore(wxFileConfig &config, bool fresh_profile = false);
  application::BoatSetupState SetupState() const { return setup_state_; }
  application::CommandResult RequestBoatSetup();
  application::CommandResult SaveBoatSetup(const application::BoatSetupDraft &draft);
  const application::Settings &Read() const { return settings_; }
  const std::string &VesselName() const { return vessel_name_; }
  const std::string &Status() const { return status_; }
  const application::DisplayPreferences &Display() const { return display_; }
  const std::string &DisplayStatus() const { return display_status_; }
  application::CommandResult RestoreBackup(const application::SettingsBackup &backup);
  application::CommandResult SaveDisplay(const application::DisplayPreferences &display);
  application::CommandResult Save(const application::Settings &settings);
  application::CommandResult SaveVessel(const application::Settings &settings,
                                        const std::string &name,
                                        double chart_safety_depth_m);

private:
  application::BoatSetupState setup_state_ = application::BoatSetupState::ExistingProfile;
  wxFileConfig &config_;
  application::Settings settings_;
  std::string vessel_name_;
  std::string status_;
  application::DisplayPreferences display_;
  std::string display_status_;
};
} // namespace opennav::integration

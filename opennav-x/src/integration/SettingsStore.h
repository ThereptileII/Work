#pragma once
#include "application/NavigationObjects.h"
#include "application/Settings.h"
#include <wx/fileconf.h>
namespace opennav::integration {
class SettingsStore {
public:
  explicit SettingsStore(wxFileConfig &config);
  const application::Settings &Read() const { return settings_; }
  const std::string &VesselName() const { return vessel_name_; }
  const std::string &Status() const { return status_; }
  application::CommandResult Save(const application::Settings &settings);
  application::CommandResult SaveVessel(const application::Settings &settings,
                                        const std::string &name,
                                        double chart_safety_depth_m);

private:
  wxFileConfig &config_;
  application::Settings settings_;
  std::string vessel_name_;
  std::string status_;
};
} // namespace opennav::integration

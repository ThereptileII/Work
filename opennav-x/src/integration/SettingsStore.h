#pragma once
#include "application/NavigationObjects.h"
#include "application/Settings.h"
#include <wx/fileconf.h>
namespace opennav::integration {
class SettingsStore {
public:
  explicit SettingsStore(wxFileConfig &config);
  const application::Settings &Read() const { return settings_; }
  const std::string &Status() const { return status_; }
  application::CommandResult Save(const application::Settings &settings);

private:
  wxFileConfig &config_;
  application::Settings settings_;
  std::string status_;
};
} // namespace opennav::integration

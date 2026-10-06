#pragma once
#include "application/SettingsBackup.h"
#include "application/NavigationObjects.h"
#include "ui/Controls.h"
namespace opennav::ui {
void ExportSettingsBackup(wxWindow &parent, LightMode light, int scale,
    const std::function<application::SettingsBackup()> &read);
void ImportSettingsBackup(wxWindow &parent, LightMode light, int scale,
    const std::function<application::CommandResult(const application::SettingsBackup &)> &restore);
}

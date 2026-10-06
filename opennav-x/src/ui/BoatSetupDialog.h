#pragma once
#include "application/BoatSetup.h"
#include "application/NavigationObjects.h"
#include "ui/Theme.h"
#include <wx/gdicmn.h>
#include <functional>
class wxDialog;
class wxWindow;
namespace opennav::ui {
struct BoatSetupActions {
  std::function<application::CommandResult(const application::BoatSetupDraft&)> save;
  std::function<std::vector<std::string>()> sensors;
};
// Physical outer bounds, including window chrome, kept on the parent's display.
wxRect BoatSetupDialogBounds(const wxRect& work_area, const wxRect& parent,
                             const wxSize& preferred, int margin);
// Modeless: startup health and the live source reducer keep running. The only
// write callback receives configuration; no hardware control API is present.
wxDialog* ShowBoatSetupDialog(wxWindow&, application::BoatSetupDraft,
                             BoatSetupActions, LightMode);
} // namespace opennav::ui

#include "integration/OpenCPNIntegration.h"
#include "integration/StartupMode.h"
#include "platform/PlatformIntegration.h"
#include "ui/Shell.h"

#include "chcanv.h"
#include "ocpn_frame.h"
#include "toolbar.h"
#include "viewport.h"

#include <wx/cmdline.h>
#include <wx/fileconf.h>
#include <wx/log.h>
#include <wx/menu.h>
#include <wx/msgdlg.h>
#include <wx/stdpaths.h>

#include <iostream>
#include <memory>
#include <optional>
#include <vector>

extern ColorScheme global_color_scheme;
extern ocpnFloatingToolbarDialog* g_MainToolbar;
extern bool g_bDeferredInitDone;

namespace opennav {
namespace {
using integration::InterfaceMode;
using integration::StartupMode;
integration::StartupFlags flags;
StartupMode selected = StartupMode::Legacy;
bool demo = false;
std::unique_ptr<ui::Shell> shell;
MyFrame* host = nullptr;
std::optional<InterfaceMode> restart;
std::vector<std::string> profile_arguments;
std::string executable;

void RequestMode(InterfaceMode mode) {
  if (!host || !g_bDeferredInitDone || restart) return;
  restart = mode;
  // OpenCPN may refuse close while initialising, compressing or updating charts.
  // Only PrepareClose commits the request and releases the shell.
  // A canvas popup still unwinds and unbinds handlers after its menu callback.
  // Closing there would delete the canvas while that stack is still active.
  host->CallAfter([] {
    if (!host) { restart.reset(); return; }
    host->Close();
    if (host) restart.reset();  // Upstream vetoed the close request.
  });
}

ui::LightMode Light() {
  if (global_color_scheme == GLOBAL_COLOR_SCHEME_NIGHT) return ui::LightMode::Night;
  if (global_color_scheme == GLOBAL_COLOR_SCHEME_DUSK) return ui::LightMode::Dusk;
  return ui::LightMode::Day;
}
}  // namespace

void AddCommandLine(wxCmdLineParser& parser) {
  parser.AddSwitch("", "xnav", "OpenNav X interface");
  parser.AddSwitch("", "legacy", "Original OpenCPN interface");
  parser.AddSwitch("", "safe-mode", "Legacy recovery; OpenNav modules disabled");
  parser.AddSwitch("", "xnav-demo", "Explicit simulated XNav telemetry; no device commands");
}

bool ParseCommandLine(wxCmdLineParser& parser) {
  flags = {parser.Found("xnav"), parser.Found("legacy"),
           parser.Found("safe-mode") || parser.Found("safe_mode")};
  demo = parser.Found("xnav-demo");
  try { (void)integration::ResolveStartup(flags); }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return false; }
  if (parser.Found("remote") && (flags.xnav || flags.legacy || flags.safe || demo)) {
    std::cerr << "OpenNav startup options cannot be combined with --remote\n";
    return false;
  }
  wxString configdir;
  if (parser.Found("configdir", &configdir)) {
    profile_arguments.push_back("--configdir");
    profile_arguments.push_back(configdir.ToStdString(wxConvUTF8));
  }
  if (parser.Found("portable")) profile_arguments.push_back("--portable");
  if (parser.Found("no_opengl")) profile_arguments.push_back("--no_opengl");
  if (parser.Found("fullscreen")) profile_arguments.push_back("--fullscreen");
  return true;
}

bool SafeRequested() { return flags.safe; }
bool IsXNav() { return selected == StartupMode::XNav; }
bool HideLegacyToolbar(const void* toolbar) { return IsXNav() && toolbar == g_MainToolbar; }

void SelectMode(wxFileConfig& config, bool upstream_safe) {
  wxString value;
  std::optional<InterfaceMode> persisted;
  if (config.Read("/OpenNav/InterfaceMode", &value)) {
    persisted = integration::ParseInterfaceMode(value.ToStdString());
    if (!persisted) {
      wxLogWarning("Invalid OpenNav InterfaceMode; using Legacy recovery");
      persisted = InterfaceMode::Legacy;
    }
  }
  auto effective_flags = flags;
  effective_flags.safe = effective_flags.safe || upstream_safe;
  selected = integration::ResolveStartup(effective_flags, persisted).mode;
  executable = wxStandardPaths::Get().GetExecutablePath().ToStdString(wxConvUTF8);
  wxLogMessage("OpenNav startup: %s", selected == StartupMode::Safe ? "safe" : IsXNav() ? "xnav" : "legacy");
}

void Attach(MyFrame& frame, wxAuiManager& manager, wxFileConfig&) {
  host = &frame;
  if (!IsXNav()) {
    frame.SetTitle(selected == StartupMode::Safe ? "OpenNav Safe Mode / OpenCPN" : "OpenCPN / Legacy");
    return;
  }
  frame.SetTitle("OpenNav X / OpenCPN");
  ui::ShellActions actions;
  actions.zoom_in = [&frame] { frame.GetPrimaryCanvas()->ZoomCanvas(2.0, false); };
  actions.zoom_out = [&frame] { frame.GetPrimaryCanvas()->ZoomCanvas(0.5, false); };
  actions.follow = [&frame] { frame.TogglebFollow(frame.GetPrimaryCanvas()); };
  actions.legacy = [] { RequestMode(InterfaceMode::Legacy); };
  actions.theme = [&frame](ui::LightMode mode) {
    frame.SetAndApplyColorScheme(mode == ui::LightMode::Night ? GLOBAL_COLOR_SCHEME_NIGHT
                                : mode == ui::LightMode::Dusk ? GLOBAL_COLOR_SCHEME_DUSK
                                                             : GLOBAL_COLOR_SCHEME_DAY);
  };
  shell = std::make_unique<ui::Shell>(frame, manager, std::move(actions), Light(), demo);
}

void AppendModeMenu(wxMenu& menu) {
  const auto id = wxWindow::NewControlId();
  menu.AppendSeparator();
  menu.Append(id, IsXNav() ? "Open Legacy OpenCPN" : "Switch to XNav");
  menu.Bind(wxEVT_MENU, [](wxCommandEvent&) {
    RequestMode(IsXNav() ? InterfaceMode::Legacy : InterfaceMode::XNav);
  }, id);
}

bool PrepareClose(wxFileConfig& config) {
  if (restart) {
    wxString previous;
    const bool had_value = config.Read("/OpenNav/InterfaceMode", &previous);
    config.Write("/OpenNav/InterfaceMode",
                 wxString::FromUTF8(integration::ToConfigValue(*restart).data()));
    if (!config.Flush()) {
      if (had_value) config.Write("/OpenNav/InterfaceMode", previous);
      else config.DeleteEntry("/OpenNav/InterfaceMode");
      restart.reset();
      wxMessageBox("Cannot save interface preference. OpenCPN remains open.",
                   "OpenNav restart", wxOK | wxICON_ERROR, host);
      return false;
    }
  }
  // Remove OpenNav AUI panes before upstream persists its stock perspective.
  shell.reset();
  host = nullptr;
  return true;
}

void CompleteRestart() {
  if (!restart) return;
  auto args = profile_arguments;
  args.push_back(*restart == InterfaceMode::XNav ? "--xnav" : "--legacy");
  if (!platform::RestartAfterExit(executable, args)) {
    wxLogError("OpenNav restart could not launch. Reopen OpenCPN to use the saved interface mode.");
  }
}

}  // namespace opennav

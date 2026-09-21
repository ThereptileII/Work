#include "integration/OpenCPNIntegration.h"
#include "integration/NavigationBridge.h"
#include "integration/OpenCPNRouteReader.h"
#include "integration/RoutePassWatch.h"
#include "integration/StartupMode.h"
#include "platform/PlatformIntegration.h"
#include "platform/PortableProfile.h"
#include "integration/PreviewDiagnostics.h"
#include "ui/Shell.h"
#ifdef OPENNAV_ROUTE_TESTS
#include "RouteProgressScenario.h"
#include <wx/filefn.h>
#endif

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
#include <wx/filefn.h>
#include <wx/filename.h>
#include <wx/utils.h>
#include "ocpn_plugin.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

extern ColorScheme global_color_scheme;
extern ocpnFloatingToolbarDialog* g_MainToolbar;
extern bool g_bDeferredInitDone;
extern bool g_bportable;
extern std::string g_configdir;

namespace opennav {
namespace integration {
struct ObservedRoutePass {
  explicit ObservedRoutePass(RouteRead r) : read(std::move(r)) {}
  RouteRead read;
  mutable RoutePassWatch watch;
};
}
namespace {
using integration::InterfaceMode;
using integration::StartupMode;
integration::StartupFlags flags;
StartupMode selected = StartupMode::Legacy;
bool demo = false;
#ifdef OPENNAV_ROUTE_TESTS
std::string route_test_profile;
#endif
std::unique_ptr<ui::Shell> shell;
std::unique_ptr<NavigationBridge> navigation;
std::unique_ptr<integration::RouteProgressInput> route_progress;
MyFrame* host = nullptr;
std::optional<InterfaceMode> restart;
std::vector<std::string> profile_arguments;
std::string executable;
bool restart_safe=false;
std::string diagnostic_directory;
std::optional<platform::PreviewPaths> preview_paths;

void RequestMode(InterfaceMode mode,bool safe=false) {
  if (!host || !g_bDeferredInitDone || restart) return;
  restart = mode;
  restart_safe=safe;
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
#ifdef OPENNAV_ROUTE_TESTS
  parser.AddSwitch("", "xnav-route-fixture", "TEST BUILD ONLY: isolated route contract scenario");
#endif
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
  parser.Found("configdir", &configdir);
  try {
    preview_paths=platform::PreviewProfile(std::filesystem::u8path(wxStandardPaths::Get().GetExecutablePath().ToStdString(wxConvUTF8)),configdir.ToStdString(wxConvUTF8));
    if(preview_paths) {
      g_bportable=true;g_configdir=platform::PathUtf8(preview_paths->profile);configdir=wxString::FromUTF8(g_configdir);
      diagnostic_directory=platform::PathUtf8(preview_paths->logs);
      // OpenCPN normalizes portable resources relative to PrivateDataDir.
      // Match that base for direct launch and restart as well as the launchers.
      if(!wxSetWorkingDirectory(configdir))
        throw std::runtime_error("Cannot use the Developer Preview profile as its working directory");
      if(parser.Found("remote")) throw std::runtime_error("Developer Preview does not send remote commands to another OpenCPN instance");
    } else if(!configdir.empty()) diagnostic_directory=configdir.ToStdString(wxConvUTF8);
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return false;}
  if (!configdir.empty()) {
    profile_arguments.push_back("--configdir");
    profile_arguments.push_back(configdir.ToStdString(wxConvUTF8));
  }
#ifdef OPENNAV_ROUTE_TESTS
  if (parser.Found("xnav-route-fixture")) {
    if (!flags.xnav || flags.safe || flags.legacy || demo || configdir.empty() ||
        !wxFileExists(configdir + "/OPENNAV_ROUTE_FIXTURE")) {
      std::cerr << "Route fixture requires explicit XNav and a marked disposable profile\n";
      return false;
    }
    route_test_profile = configdir.ToStdString(wxConvUTF8);
  }
#endif
  if (parser.Found("portable") || preview_paths) profile_arguments.push_back("--portable");
  if (parser.Found("no_opengl")) profile_arguments.push_back("--no_opengl");
  if (parser.Found("fullscreen")) profile_arguments.push_back("--fullscreen");
  return true;
}

bool SafeRequested() { return flags.safe; }
bool IsPortablePreview() { return preview_paths.has_value(); }
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

void Attach(MyFrame& frame, wxAuiManager& manager, wxFileConfig& config) {
  host = &frame;
  if(preview_paths && !config.ReadBool("/OpenNav/PreviewWindowPlaced",false)) {
    const auto desktop=wxGetClientDisplayRect();
    const auto preferred=frame.FromDIP(wxSize(1280,800));
    frame.SetSize(wxRect(desktop.GetPosition(),wxSize(std::min(desktop.width,preferred.x),
                                                    std::min(desktop.height,preferred.y))));
    config.Write("/OpenNav/PreviewWindowPlaced",true);
  }
  if (!IsXNav()) {
    frame.SetTitle(selected == StartupMode::Safe ? "OpenNav Safe Mode / OpenCPN" : "OpenCPN / Legacy");
    return;
  }
  frame.SetTitle("OpenNav X / OpenCPN");
  ui::ShellActions actions;
  // Names assigned by MyFrame::CreateCanvasLayout in the pinned OpenCPN.
  // The UI only toggles pane visibility; it never owns/reparents a canvas.
  actions.navigation_panes = {"ChartCanvas", "ChartCanvas2"};
  actions.zoom_in = [&frame] { frame.GetPrimaryCanvas()->ZoomCanvas(2.0, false); };
  actions.zoom_out = [&frame] { frame.GetPrimaryCanvas()->ZoomCanvas(0.5, false); };
  actions.follow = [&frame] { frame.TogglebFollow(frame.GetPrimaryCanvas()); };
  actions.legacy = [] { RequestMode(InterfaceMode::Legacy); };
  actions.restart_xnav=[] {RequestMode(InterfaceMode::XNav);};
  actions.safe=[] {RequestMode(InterfaceMode::Legacy,true);};
  actions.route=[] {return CurrentRouteProgress();};
  actions.build_info=[&frame] {return integration::PreviewBuildInfo(frame.GetDPI().x,g_configdir);};
  actions.diagnostics_folder=[] {
    if(!diagnostic_directory.empty()) wxLaunchDefaultApplication(wxString::FromUTF8(diagnostic_directory));
  };
  actions.diagnostic_snapshot=[&frame,last=vessel::Time{}](const vessel::VesselState& state) mutable {
    const auto now=vessel::Clock::now();
    if(diagnostic_directory.empty() || now-last<std::chrono::seconds(1))return;
    last=now;
    integration::WritePreviewDiagnostics(diagnostic_directory+"/opennav-diagnostics.json",state,
                                         integration::PreviewBuildInfo(frame.GetDPI().x,g_configdir));
  };
  actions.demo_chart=[] { if(g_bDeferredInitDone) JumpToPosition(59.08,18.5,0.003); };
  actions.theme = [&frame](ui::LightMode mode) {
    frame.SetAndApplyColorScheme(mode == ui::LightMode::Night ? GLOBAL_COLOR_SCHEME_NIGHT
                                : mode == ui::LightMode::Dusk ? GLOBAL_COLOR_SCHEME_DUSK
                                                             : GLOBAL_COLOR_SCHEME_DAY);
  };
  shell = std::make_unique<ui::Shell>(frame, manager, std::move(actions), Light(), demo);
  navigation = std::make_unique<NavigationBridge>([](const vessel::VesselState& state) {
    if (shell) shell->UpdateState(state);
  });
  route_progress = std::make_unique<integration::RouteProgressInput>(
      "OpenNav session " + std::to_string(vessel::Clock::now().time_since_epoch().count()));
#ifdef OPENNAV_ROUTE_TESTS
  if (!route_test_profile.empty()) test::EnableRouteScenario(route_test_profile);
#endif
}

RouteObservation BeforeRouteProgress() {
  if (!navigation || !route_progress) return {};
  return std::make_shared<const integration::ObservedRoutePass>(
      integration::ReadRouteProgress(navigation->PositionState()));
}

void AfterRouteProgress(const RouteObservation& before) {
  if (!before || !navigation || !route_progress) return;
  auto after = integration::ReadRouteProgress(navigation->PositionState());
  after.interrupted = before->watch.Finish();
  route_progress->Complete(before->read, after, vessel::Clock::now());
#ifdef OPENNAV_ROUTE_TESTS
  test::RouteScenarioStep(route_progress->Current());
#endif
}

vessel::RouteProgress CurrentRouteProgress() {
  if (!navigation || !route_progress) return {};
  const auto read = integration::ReadRouteProgress(navigation->PositionState());
  route_progress->CheckCurrent(read.route, vessel::Clock::now());
  return route_progress->Current();
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
  if (restart && !restart_safe) {
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
  navigation.reset();
  route_progress.reset();
  shell.reset();
  host = nullptr;
  return true;
}

void CompleteRestart() {
  if (!restart) return;
  auto args = profile_arguments;
  args.push_back(restart_safe ? "--safe-mode" : *restart == InterfaceMode::XNav ? "--xnav" : "--legacy");
  if (!platform::RestartAfterExit(executable, args)) {
    wxLogError("OpenNav restart could not launch. Reopen OpenCPN to use the saved interface mode.");
  }
}

}  // namespace opennav

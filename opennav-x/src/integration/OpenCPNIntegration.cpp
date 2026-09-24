#include "integration/OpenCPNIntegration.h"
#include "adapters/Autopilot.h"
#include "adapters/Radar.h"
#include "integration/MarineBridge.h"
#include "integration/NavigationActions.h"
#include "integration/NavigationBridge.h"
#include "integration/NavigationObjects.h"
#include "integration/OpenCPNRouteReader.h"
#include "integration/PreviewDiagnostics.h"
#include "integration/PreviewResources.h"
#include "integration/RecoveryStore.h"
#include "integration/RoutePassWatch.h"
#include "integration/RuntimeDiagnostics.h"
#include "integration/SettingsStore.h"
#include "integration/StartupMode.h"
#include "model/base_platform.h"
#include "model/safe_mode.h"
#include "platform/PlatformIntegration.h"
#include "platform/PortableProfile.h"
#include "ui/Shell.h"
#ifdef OPENNAV_ROUTE_TESTS
#include "RouteProgressScenario.h"
#include "NavigationObjectScenario.h"
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
extern wxString gWorldShapefileLocation;
extern wxString g_AW1GUID,g_AW2GUID;

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
std::string route_test_profile,object_test_profile;
#endif
std::unique_ptr<ui::Shell> shell;
std::unique_ptr<NavigationBridge> navigation;
std::unique_ptr<integration::MarineBridge> marine;
std::unique_ptr<integration::SettingsStore> settings;
std::unique_ptr<integration::RecoveryStore> recovery;
bool recovery_safe = false;
adapters::UnavailableRadar radar;
application::AnchorState anchor_state;
struct PilotServices {
  adapters::UnavailableAutopilot hardware;
  adapters::SimulatedAutopilot simulator{vessel::Clock::now()};
  adapters::ManualAutopilot live{hardware},demo{simulator};
  bool was_demo=false;
  adapters::ManualAutopilot& Select(bool simulated){return simulated?demo:live;}
};
std::unique_ptr<PilotServices> pilots;
vessel::VesselState selected_navigation;
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
  if (mode == InterfaceMode::XNav && recovery && recovery->RequiresSafe() &&
      !recovery->Retry()) {
    wxMessageBox("Cannot reset the startup recovery record. Inspect the "
                 "profile storage and diagnostics before retrying XNav.",
                 "OpenNav recovery", wxOK | wxICON_ERROR, host);
    return;
  }
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
  parser.AddSwitch("", "xnav-object-fixture", "TEST BUILD ONLY: isolated navigation object scenario");
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
  if(parser.Found("xnav-object-fixture")){
    if(!flags.xnav||flags.safe||flags.legacy||demo||configdir.empty()||parser.Found("xnav-route-fixture")||!wxFileExists(configdir+"/OPENNAV_OBJECT_FIXTURE")){
      std::cerr<<"Object fixture requires explicit XNav and a marked disposable profile\n";return false;
    }
    object_test_profile=configdir.ToStdString(wxConvUTF8);
  }
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

bool CheckStartupRecovery() {
  recovery = std::make_unique<integration::RecoveryStore>(
      g_BasePlatform->GetPrivateDataDir());
  recovery_safe = recovery->RequiresSafe() && !flags.legacy;
  if (recovery_safe)
    wxLogWarning("OpenNav automatic Safe Mode: %s",
                 wxString::FromUTF8(recovery->Reason()));
  return recovery_safe;
}

void SelectMode(wxFileConfig& config, bool upstream_safe) {
  if (preview_paths) {
    const auto basemap = integration::PreviewBasemapDefault(
        preview_paths->root, gWorldShapefileLocation.ToStdString(wxConvUTF8));
    if (basemap) {
      gWorldShapefileLocation = wxString::FromUTF8(platform::PathUtf8(*basemap));
      wxLogMessage("OpenNav portable basemap: using bundled coastline at %s",
                   gWorldShapefileLocation);
    }
  }
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
  effective_flags.safe = effective_flags.safe || recovery_safe;
  selected = integration::ResolveStartup(effective_flags, persisted).mode;
  if (IsXNav() && recovery && !recovery->BeginXNav()) {
    selected = StartupMode::Safe;
    recovery_safe = true;
    safe_mode::set_mode(true);
  }
  if (diagnostic_directory.empty() && IsXNav()) {
    const auto folder =
        wxFileName(g_BasePlatform->GetPrivateDataDir(), "opennav-logs")
            .GetFullPath();
    if (wxDirExists(folder) ||
        wxFileName::Mkdir(folder, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL))
      diagnostic_directory = folder.ToStdString(wxConvUTF8);
    else
      wxLogWarning("OpenNav diagnostic directory unavailable: %s", folder);
  }
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
    if (recovery_safe)
      frame.CallAfter([&frame] {
        wxMessageBox(
            "XNav did not complete startup reliably. OpenCPN is running in "
            "Safe Mode with OpenNav modules, plugins and OpenGL disabled. "
            "Navigation data has not been reset. Inspect the OpenCPN log; use "
            "Switch to XNav only when ready to retry.",
            "OpenNav startup recovery", wxOK | wxICON_INFORMATION, &frame);
      });
    return;
  }
  frame.SetTitle("OpenNav X / OpenCPN");
  settings = std::make_unique<integration::SettingsStore>(config);
  marine = std::make_unique<integration::MarineBridge>();
  auto configure_sources = [] {
    marine->SetBindings(settings->Read().signal_k_mappings);
    for (const auto &q : vessel::Quantities()) {
      auto p = settings->Read().sources.find(q.quantity);
      marine->Sources().Configure(q.quantity,
                                  p == settings->Read().sources.end()
                                      ? vessel::SourcePolicy{}
                                      : p->second);
    }
  };
  configure_sources();
  ui::ShellActions actions;
  actions.settings = [] { return settings->Read(); };
  actions.settings_status = [] { return settings->Status(); };
  actions.save_settings = [configure_sources](const auto &s) {
    auto result = settings->Save(s);
    if (result.ok)
      configure_sources();
    return result;
  };
  actions.source_health = [] {
    return marine->Sources().Health(vessel::Clock::now());
  };
  actions.radar = [] { return radar.GetState(); };
  // Names assigned by MyFrame::CreateCanvasLayout in the pinned OpenCPN.
  // The UI only toggles pane visibility; it never owns/reparents a canvas.
  actions.navigation_panes = {"ChartCanvas", "ChartCanvas2"};
  actions.navigation=integration::MakeNavigationActions(frame,[]{return selected_navigation.navigation;},[]{
    auto copy=anchor_state;
    if(!copy.waypoint_id.empty() && copy.waypoint_id!=g_AW1GUID.ToStdString(wxConvUTF8) && copy.waypoint_id!=g_AW2GUID.ToStdString(wxConvUTF8)){copy={};copy.state="Anchor watch changed; waiting for normal observation";}
    return copy;
  });
  actions.route_creating=[&frame]{return frame.GetPrimaryCanvas()->m_routeState>0;};
  pilots=std::make_unique<PilotServices>();
  actions.pilot_tick=[](bool simulated){const auto now=vessel::Clock::now();if(pilots->was_demo!=simulated){pilots->Select(pilots->was_demo).Enable(false,now);pilots->was_demo=simulated;}auto& p=pilots->Select(simulated);p.Tick(now);return p.GetState(now);};
  actions.pilot_log=[](bool simulated){return pilots->Select(simulated).Log();};
  actions.pilot_command=[](bool simulated,auto action,double delta){auto c=pilots->Select(simulated).Request(action,delta,vessel::Clock::now());wxLogMessage("OpenNav manual autopilot [%s] request %llu: %s / %s",simulated?"DEMO":"unavailable hardware",static_cast<unsigned long long>(c.request.id),wxString::FromUTF8(adapters::CommandStateName(c.state)),wxString::FromUTF8(c.detail));};
  actions.pilot_enable=[](bool simulated,bool enabled){pilots->Select(simulated).Enable(enabled,vessel::Clock::now());wxLogMessage("OpenNav manual autopilot %s: %s",simulated?"DEMO":"unavailable hardware",enabled?"enable requested":"disabled");};
  actions.zoom_in = [&frame] { frame.GetPrimaryCanvas()->ZoomCanvas(2.0, false); };
  actions.zoom_out = [&frame] { frame.GetPrimaryCanvas()->ZoomCanvas(0.5, false); };
  actions.follow = [&frame] { frame.TogglebFollow(frame.GetPrimaryCanvas()); };
  actions.legacy = [] { RequestMode(InterfaceMode::Legacy); };
  actions.restart_xnav=[] {RequestMode(InterfaceMode::XNav);};
  actions.safe=[] {RequestMode(InterfaceMode::Legacy,true);};
  actions.route=[] {return CurrentRouteProgress();};
  actions.live_state = [] {
    const auto now = vessel::Clock::now();
    auto state =
        marine ? marine->Merge(selected_navigation, now) : selected_navigation;
    if (settings)
      vessel::NormalizeBatteryPower(state,
                                    settings->Read().energy.battery_device_id,
                                    settings->Read().current, now);
    return state;
  };
  actions.build_info = [&frame] {
    return integration::PreviewBuildInfo(
        frame.GetDPI().x,
        g_BasePlatform->GetPrivateDataDir().ToStdString(wxConvUTF8));
  };
  actions.diagnostics_folder=[] {
    if(!diagnostic_directory.empty()) wxLaunchDefaultApplication(wxString::FromUTF8(diagnostic_directory));
  };
  actions.diagnostic_snapshot = [&frame, last = vessel::Time{}](
                                    const vessel::VesselState &state,
                                    const smartnav::EnergyPrediction &energy,
                                    const std::string &page) mutable {
    const auto now=vessel::Clock::now();
    if (diagnostic_directory.empty() || now - last < std::chrono::seconds(1))
      return;
    last=now;
    auto runtime =
        integration::ReadRuntimeDiagnostics(*frame.GetPrimaryCanvas());
    if (shell) {
      const auto metrics = shell->Metrics();
      runtime["ui_update"]["ticks"] = wxString::Format(
          "%llu", static_cast<unsigned long long>(metrics.ticks));
      runtime["ui_update"]["last_ms"] = metrics.last_ms;
      runtime["ui_update"]["mean_ms"] = metrics.mean_ms;
      runtime["ui_update"]["maximum_ms"] = metrics.maximum_ms;
      runtime["ui_update"]["timer_period_ms"] = 250;
      runtime["ui_update"]["scope"] =
          wxString("Shell update callback including diagnostics; excludes "
                   "asynchronous chart painting");
    }
    integration::WritePreviewDiagnostics(
        diagnostic_directory + "/opennav-diagnostics.json", state,
        integration::PreviewBuildInfo(
            frame.GetDPI().x,
            g_BasePlatform->GetPrivateDataDir().ToStdString(wxConvUTF8)),
        energy, settings->Read(), marine->Sources().Health(now), page, runtime);
  };
  actions.demo_chart=[] { if(g_bDeferredInitDone) JumpToPosition(59.08,18.5,0.003); };
  actions.theme = [&frame](ui::LightMode mode) {
    frame.SetAndApplyColorScheme(mode == ui::LightMode::Night ? GLOBAL_COLOR_SCHEME_NIGHT
                                : mode == ui::LightMode::Dusk ? GLOBAL_COLOR_SCHEME_DUSK
                                                             : GLOBAL_COLOR_SCHEME_DAY);
  };
  shell = std::make_unique<ui::Shell>(frame, manager, std::move(actions), Light(), demo);
  navigation = std::make_unique<NavigationBridge>(
      [](const vessel::VesselState &state) { selected_navigation = state; });
  route_progress = std::make_unique<integration::RouteProgressInput>(
      "OpenNav session " + std::to_string(vessel::Clock::now().time_since_epoch().count()));
#ifdef OPENNAV_ROUTE_TESTS
  if (!route_test_profile.empty()) test::EnableRouteScenario(route_test_profile);
  if (!object_test_profile.empty()) test::EnableObjectScenario(object_test_profile);
#endif
}

void AfterAnchorWatch(){
  if (recovery && IsXNav())
    recovery->ObserveHealthy(g_bDeferredInitDone, vessel::Clock::now());
  if(!navigation)return;
  auto current=integration::ObserveAnchor(selected_navigation.navigation,vessel::Clock::now());
  if(current.waypoint_id==anchor_state.waypoint_id)current.recent_positions=anchor_state.recent_positions;
  if(current.distance_m.value && selected_navigation.navigation.latitude_deg.observed_at>anchor_state.distance_m.observed_at){
    current.recent_positions.push_back({{*selected_navigation.navigation.latitude_deg.value,*selected_navigation.navigation.longitude_deg.value},selected_navigation.navigation.latitude_deg.observed_at});
    if(current.recent_positions.size()>300)current.recent_positions.erase(current.recent_positions.begin());
  }
  anchor_state=std::move(current);
}
bool ShowNavigationObjectCard(const std::string& id,bool route){if(!IsXNav()||!shell||!host)return false;host->CallAfter([id,route]{if(shell)shell->ShowObject(id,route);});return true;}
bool ShowAisCard(int mmsi){if(!IsXNav()||!shell||!host)return false;host->CallAfter([mmsi]{if(shell)shell->ShowAis(mmsi);});return true;}

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
  test::ObjectScenarioStep(selected_navigation.navigation);
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
  if (recovery && IsXNav())
    recovery->CleanClose();
  // Remove OpenNav AUI panes before upstream persists its stock perspective.
  navigation.reset();
  marine.reset();
  selected_navigation = {};
  route_progress.reset();
  shell.reset();
  settings.reset();
  pilots.reset();anchor_state={};
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

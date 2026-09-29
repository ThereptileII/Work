#include "integration/OpenCPNIntegration.h"
#include "application/AnchorView.h"
#include "integration/BuildFeatures.h"
#include "integration/DashboardPresentation.h"
#include "integration/DashboardPresentationApi.h"
#include "adapters/Autopilot.h"
#include "integration/OpenCPNPilot.h"
#include "adapters/Radar.h"
#include "integration/InstalledResources.h"
#include "integration/InstallerSelfTest.h"
#include "integration/MarineBridge.h"
#include "integration/NavigationActions.h"
#include "integration/NavigationBridge.h"
#include "integration/NavigationObjects.h"
#include "integration/OpenCPNRouteReader.h"
#include "integration/PreviewDiagnostics.h"
#include "diagnostics/TestUiTrace.h"
#include "integration/PreviewResources.h"
#include "integration/RecoveryStore.h"
#include "integration/RoutePassWatch.h"
#include "integration/RuntimeDiagnostics.h"
#include "integration/SettingsStore.h"
#include "integration/ChartPresentation.h"
#include "integration/OnlineAis.h"
#include "integration/OnlineAisOverlay.h"
#include "integration/AisViewport.h"
#include "integration/StartupMode.h"
#if XNAV_ENABLE_TEST_FIXTURES
#include "adapters/SimulatedAutopilot.h"
#endif
#include "model/base_platform.h"
#include "model/comm_drv_registry.h"
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
#include "gshhs.h"
#include "shapefile_basemap.h"
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
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

extern ColorScheme global_color_scheme;
extern ShapeBaseChartSet gShapeBasemap;
extern ocpnFloatingToolbarDialog* g_MainToolbar;
extern bool g_bDeferredInitDone;
extern bool g_bportable;
extern std::string g_configdir;
extern wxString gWorldShapefileLocation;
extern wxString gWorldMapLocation, g_sAIS_Alert_Sound_File;
extern std::vector<std::string> TideCurrentDataSet;
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
std::unique_ptr<integration::OnlineAis> online_ais;
integration::OnlineAisOverlay online_chart;
std::shared_ptr<diagnostics::Commissioning> commissioning;
std::unique_ptr<NavigationBridge> navigation;
std::unique_ptr<integration::MarineBridge> marine;
std::unique_ptr<integration::SettingsStore> settings;
std::unique_ptr<integration::RecoveryStore> recovery;
bool recovery_safe = false;
bool recovery_notice_scheduled = false;
adapters::UnavailableRadar radar;
application::AnchorState anchor_state;
struct PilotServices {
  explicit PilotServices(std::function<bool()> allowed)
      : hardware(std::move(allowed)) {}
  integration::OpenCPNPilot hardware;
#if XNAV_ENABLE_TEST_FIXTURES
  adapters::SimulatedAutopilot simulator{vessel::Clock::now()};
  adapters::ManualAutopilot live{hardware},demo{simulator};
#else
  adapters::UnavailableAutopilot isolated;
  adapters::ManualAutopilot live{hardware},demo{isolated};
#endif
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
  wxLogMessage("OpenNav human mode request: %s", safe ? "safe"
                                                 : mode == InterfaceMode::XNav
                                                     ? "xnav"
                                                     : "legacy");
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
    wxLogMessage("OpenNav mode request: asking OpenCPN to close");
    host->Close();
    if (host) {
      wxLogMessage(
          "OpenNav mode request: OpenCPN kept the current window open");
      restart.reset(); // Upstream vetoed the close request.
    }
  });
}

ui::LightMode Light() {
  if (global_color_scheme == GLOBAL_COLOR_SCHEME_NIGHT) return ui::LightMode::Night;
  if (global_color_scheme == GLOBAL_COLOR_SCHEME_DUSK) return ui::LightMode::Dusk;
  return ui::LightMode::Day;
}
}  // namespace

void AddCommandLine(wxCmdLineParser& parser) {
  integration::AddInstallerSelfTest(parser);
  parser.AddSwitch("", "xnav", "OpenNav X interface");
  parser.AddSwitch("", "legacy", "Original OpenCPN interface");
  parser.AddSwitch("", "safe-mode", "Legacy recovery; OpenNav modules disabled");
#if XNAV_ENABLE_TEST_FIXTURES
  parser.AddSwitch("", "xnav-demo", "Explicit simulated XNav telemetry; no device commands");
#endif
#ifdef OPENNAV_ROUTE_TESTS
  parser.AddSwitch("", "xnav-route-fixture", "TEST BUILD ONLY: isolated route contract scenario");
  parser.AddSwitch("", "xnav-object-fixture", "TEST BUILD ONLY: isolated navigation object scenario");
#endif
}

bool ParseCommandLine(wxCmdLineParser& parser) {
  if (integration::ParseInstallerSelfTest(parser)) return true;
  flags = {parser.Found("xnav"), parser.Found("legacy"),
           parser.Found("safe-mode") || parser.Found("safe_mode")};
#if XNAV_ENABLE_TEST_FIXTURES
  demo = parser.Found("xnav-demo");
#endif
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
        throw std::runtime_error(
            "Cannot use the portable OpenNav profile as its working directory");
      if (parser.Found("remote"))
        throw std::runtime_error("portable OpenNav does not send remote "
                                 "commands to another OpenCPN instance");
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
  try {
    integration::TestStartupFlags requested;
    requested.demo = demo;
#ifdef OPENNAV_ROUTE_TESTS
    requested.route = !route_test_profile.empty();
    requested.objects = !object_test_profile.empty();
#endif
    integration::ValidateTestStartup(requested, flags);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return false;
  }
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

void InitializeResourceDefaults(wxFileConfig& config) {
  // Installed generations can be removed by rollback/uninstall. Persist the
  // original stock defaults, never a disposable generation's resource paths.
  // Locale is initialized; upstream has not yet filled defaults or loaded tides.
  if (!preview_paths && !g_bportable) {
    const auto app = platform::PathFromUtf8(
        wxFileName(wxStandardPaths::Get().GetExecutablePath()).GetPath().ToStdString(wxConvUTF8));
    if (const auto defaults = integration::InstalledResourceDefaults(app)) {
      // LoadMyConfig reads these strings before ChangeLocale. Re-read only
      // this resource list after locale setup so Unicode paths survive the
      // same ToStdString conversion used by the pinned loader. Preserve its
      // entry order and duplicate removal; do not discard missing selections.
      {
        wxConfigPathChanger section(&config, "/TideCurrentDataSources/");
        if (config.GetNumberOfEntries()) {
          std::vector<std::string> configured;
          wxString key, value;
          long index;
          for (bool more = config.GetFirstEntry(key, index); more;
               more = config.GetNextEntry(key, index)) {
            config.Read(key, &value);
            const auto source = value.ToStdString();
            if (std::find(configured.begin(), configured.end(), source) == configured.end())
              configured.push_back(source);
          }
          TideCurrentDataSet = std::move(configured);
        }
      }
      integration::ResourceSelection selected_resources{
          TideCurrentDataSet, gWorldMapLocation.ToStdString(wxConvUTF8),
          gWorldShapefileLocation.ToStdString(wxConvUTF8),
          g_sAIS_Alert_Sound_File.ToStdString(wxConvUTF8)};
      integration::ApplyResourceDefaults(selected_resources, *defaults);
      if (TideCurrentDataSet.empty())
        for (const auto& source : selected_resources.tides)
          TideCurrentDataSet.push_back(wxString::FromUTF8(source).ToStdString());
      if (gWorldMapLocation.empty())
        gWorldMapLocation = wxString::FromUTF8(selected_resources.coastline) + wxFileName::GetPathSeparator();
      if (gWorldShapefileLocation.empty())
        gWorldShapefileLocation = wxString::FromUTF8(selected_resources.basemap);
      if (g_sAIS_Alert_Sound_File.empty())
        g_sAIS_Alert_Sound_File = wxString::FromUTF8(selected_resources.ais_alarm);
      wxLogMessage("OpenNav installed resource defaults: original supported OpenCPN; configured selections preserved");
    }
  }
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
  integration::ConfigureChartPresentation(config, IsXNav());
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
  settings = std::make_unique<integration::SettingsStore>(config);
  marine = std::make_unique<integration::MarineBridge>();
  auto configure_sources = [] {
    marine->SetBindings(settings->Read().signal_k_mappings);
    marine->SetBoatBridge(settings->Read().boat_bridge);
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
  commissioning = std::make_shared<diagnostics::Commissioning>(
      std::filesystem::u8path(diagnostic_directory) / "recordings", [] {
        for (const auto &driver :
             CommDriverRegistry::GetInstance().GetDrivers()) {
          const auto attributes = driver->GetAttributes();
          const auto direction = attributes.find("ioDirection");
          if ((direction != attributes.end() && direction->second != "IN") ||
              (direction == attributes.end() &&
               (driver->bus == NavAddr::Bus::N2000 ||
                driver->bus == NavAddr::Bus::N0183 ||
                driver->bus == NavAddr::Bus::Signalk)))
            return std::string("Replay requires output-capable OpenCPN "
                               "connections to be disabled. Use the isolated "
                               "portable profile for offline review.");
        }
        return std::string{};
      });
  actions.commissioning = commissioning;
  // Construct only after the XNav gate. Legacy and Safe never start this
  // optional internet client. Its default and first-install preference are OFF.
  online_ais = std::make_unique<integration::OnlineAis>(config,
      ais::CreateAisCredentials(), std::make_unique<ais::AisStreamProvider>());
  actions.online_ais.read = [](vessel::Time now) {
    application::OnlineAisState value;
    if (!online_ais) return value;
    value.enabled = online_ais->Enabled();
    value.credential_present = online_ais->CredentialPresent();
#ifdef __WXMSW__
    value.credential_writable = true;
#endif
    value.feed = online_ais->Read(now);
    return value;
  };
  actions.online_ais.enable = [](bool enabled) {
    return online_ais ? online_ais->Enable(enabled)
        : application::CommandResult{false, "Online AIS unavailable during shutdown"};
  };
  actions.online_ais.store_key = [](const ais::Secret &key) {
    return online_ais ? online_ais->StoreKey(key)
        : application::CommandResult{false, "Credential storage unavailable during shutdown"};
  };
  actions.online_ais.remove_key = [] {
    return online_ais ? online_ais->RemoveKey()
        : application::CommandResult{false, "Credential storage unavailable during shutdown"};
  };
  actions.online_ais_tick = [&frame, last = std::chrono::steady_clock::time_point{}, previous_selection = 0](bool live_allowed) mutable {
    if (!online_ais) return;
    auto *canvas = frame.GetPrimaryCanvas();
    std::optional<ais::Viewport> copied;
    if (canvas) {
      // ViewPort::SetBoxes already ran in normal OpenCPN processing. Read its
      // ordered, possibly unwrapped bbox without invoking navigation getters.
      auto &vp = canvas->GetVP();
      const auto &box = vp.GetBBox();
      copied = integration::AisViewport(vp.IsValid() && box.GetValid(),
          box.GetMinLat(), box.GetMaxLat(), box.GetMinLon(), box.GetMaxLon());
    }
    online_ais->ObserveViewport(copied, live_allowed && !restart &&
        (!commissioning || !commissioning->Replaying()));
    const auto steady = std::chrono::steady_clock::now();
    const int selected = shell ? shell->SelectedAis() : 0;
    const bool permitted = live_allowed && online_ais->Enabled() && !restart &&
        (!commissioning || !commissioning->Replaying());
    if (!permitted || steady-last >= std::chrono::seconds(1) || selected != previous_selection) {
      const auto now = vessel::Clock::now();
      const auto display = permitted ? ais::Aggregate(
          integration::CopyAisState(selected_navigation.navigation, now),
          online_ais->Read(now)).display : vessel::AisState{};
      if (online_chart.Update(display, now, selected)) frame.RefreshAllCanvas(false);
      last = steady; previous_selection = selected;
    }
  };
  actions.view_online_ais = [&frame](int mmsi) {
    if (!online_ais || !host || restart || (commissioning && commissioning->Replaying()))
      return application::CommandResult{false, "Live AIS chart selection unavailable"};
    const auto now = vessel::Clock::now();
    const auto feeds = ais::Aggregate(
        integration::CopyAisState(selected_navigation.navigation, now), online_ais->Read(now));
    vessel::AisSelection selection;
    if (!selection.Select(mmsi, feeds.display, now))
      return application::CommandResult{false, "Target position unavailable, ambiguous or stale"};
    auto *canvas = frame.GetPrimaryCanvas();
    if (canvas) for (const auto &t : feeds.display.targets)
      if (t.mmsi == mmsi && t.origin == vessel::AisOrigin::AisStreamOnline) {
        frame.JumpToPosition(canvas, *t.latitude_deg.value, *t.longitude_deg.value, canvas->GetVPScale());
        frame.InvalidateAllGL(); frame.RefreshAllCanvas(false);
        return application::CommandResult{true, "Selected supplemental internet position"};
      }
    return application::CommandResult{false, "Target or chart changed; select it again"};
  };
  actions.settings = [] { return settings->Read(); };
  actions.settings_status = [] { return settings->Status(); };
  actions.chart_style_status = integration::ChartPresentationStatus;
  actions.chart_style_requested = integration::XNavChartRequested;
  actions.set_chart_style = integration::SetXNavChartRequested;
  actions.boat_bridge_status = [] { return marine->BoatBridgeStatus(vessel::Clock::now()); };
  actions.save_settings = [configure_sources](const auto &s) {
    if (commissioning && commissioning->Replaying())
      return application::CommandResult{
          false, "Stop REPLAY before changing live settings"};
    const auto old_pilot = settings->Read().pilot;
    auto result = settings->Save(s);
    if (result.ok)
      configure_sources();
    if (result.ok && pilots &&
        (old_pilot.interface_id != s.pilot.interface_id || old_pilot.name != s.pilot.name ||
         old_pilot.permit_control != s.pilot.permit_control)) {
      pilots->live.Enable(false, vessel::Clock::now());
      pilots->hardware.Configure(s.pilot);
    }
    return result;
  };
  actions.source_health = [] {
    return marine->Health(vessel::Clock::now());
  };
  actions.radar = [] { return radar.GetState(); };
  // Names assigned by MyFrame::CreateCanvasLayout in the pinned OpenCPN.
  // The UI only toggles pane visibility; it never owns/reparents a canvas.
  actions.navigation_panes = {"ChartCanvas", "ChartCanvas2"};
  actions.chart_orientation = [&frame] {
    auto *canvas = frame.GetPrimaryCanvas();
    if (!canvas) return std::string("Unavailable");
    return std::string(canvas->GetUpMode() == COURSE_UP_MODE ? "Course"
                      : canvas->GetUpMode() == HEAD_UP_MODE ? "Head" : "North");
  };
  actions.chart_rotation = [&frame] {
    auto *canvas = frame.GetPrimaryCanvas();
    return canvas ? canvas->GetVP().rotation : std::numeric_limits<double>::quiet_NaN();
  };
  // XNav supplies orientation and data health itself. This per-canvas flag
  // does not change the persisted global Legacy compass preference.
  for (auto *window : frame.GetChildren())
    if (auto *canvas = dynamic_cast<ChartCanvas *>(window))
      canvas->SetShowGPSCompassWindow(false);
  actions.navigation=integration::MakeNavigationActions(frame,[]{return selected_navigation.navigation;},[]{
    auto copy=anchor_state;
    if(!copy.waypoint_id.empty() && copy.waypoint_id!=g_AW1GUID.ToStdString(wxConvUTF8) && copy.waypoint_id!=g_AW2GUID.ToStdString(wxConvUTF8)){copy={};copy.state="Anchor watch changed; waiting for normal observation";}
    return copy;
  });
  actions.navigation =
      application::GuardNavigationChanges(std::move(actions.navigation), [] {
        return !commissioning || !commissioning->Replaying();
      });
  actions.route_creating=[&frame]{return frame.GetPrimaryCanvas()->m_routeState>0;};
  pilots = std::make_unique<PilotServices>([] {
    return pilots && !pilots->was_demo && !restart &&
           (!commissioning || commissioning->AllowsHardwareControl());
  });
  pilots->was_demo = demo;
  pilots->hardware.Configure(settings->Read().pilot);
  actions.pilot_identity = [] {
    const bool sent = pilots && pilots->hardware.RequestIdentity(vessel::Clock::now());
    return application::CommandResult{
        sent, sent ? "Identity request attempted; waiting for an observed address claim"
                   : "Identity request withheld: check connection, format, isolation or five-second limit"};
  };
  actions.pilot_sources = [] {
    std::vector<std::string> result;
    for (const auto &identity : marine->Identities()) {
      try { adapters::ParsePilotName(identity.name); }
      catch (const std::invalid_argument &) { continue; }
      result.push_back(identity.interface_id + " / NAME " + identity.name +
                       " / address " + std::to_string(identity.address));
      if (result.size() >= 8) break;
    }
    return result;
  };
  actions.pilot_tick = [](bool simulated, vessel::Time now) {
    if (pilots->was_demo != simulated) {
      pilots->Select(pilots->was_demo).Enable(false, now);
      pilots->was_demo = simulated;
    }
    auto &p = pilots->Select(simulated);
    if (commissioning && !commissioning->AllowsHardwareControl()) {
      pilots->live.Enable(false, now);
      pilots->demo.Enable(false, now);
    }
    const auto before = p.GetState(now).command.state;
    p.Tick(now);
    auto view = p.GetState(now);
    view.adapter_status = simulated ? "DEMO / simulated feedback" : pilots->hardware.Description();
    if (before != view.command.state)
      wxLogMessage("OpenNav manual pilot %llu: %s / %s",
                   static_cast<unsigned long long>(view.command.request.id),
                   wxString::FromUTF8(adapters::CommandStateName(view.command.state)),
                   wxString::FromUTF8(view.command.detail));
    return view;
  };
  actions.pilot_log=[](bool simulated){return pilots->Select(simulated).Log();};
  actions.pilot_command = [](bool simulated, auto action, double delta) {
    if (commissioning && !commissioning->AllowsHardwareControl())
      return;
    auto c =
        pilots->Select(simulated).Request(action, delta, vessel::Clock::now());
    wxLogMessage("OpenNav manual autopilot [%s] request %llu: %s / %s",
                 simulated ? "DEMO" : "ST4000 live adapter",
                 static_cast<unsigned long long>(c.request.id),
                 wxString::FromUTF8(adapters::CommandStateName(c.state)),
                 wxString::FromUTF8(c.detail));
  };
  actions.pilot_enable = [](bool simulated, bool enabled) {
    if (enabled && commissioning && !commissioning->AllowsHardwareControl())
      return;
    pilots->Select(simulated).Enable(enabled, vessel::Clock::now());
    wxLogMessage("OpenNav manual autopilot %s: %s",
                 simulated ? "DEMO" : "ST4000 live adapter",
                 enabled ? "enable requested" : "disabled");
  };
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
  actions.field_environment = [&frame] {
    diagnostics::FieldEnvironment e;
    e.build = integration::PreviewBuildInfo(frame.GetDPI().x, "withheld");
    e.startup_recovery_required = recovery && recovery->RequiresSafe();
    if (recovery) {
      e.startup_failures_observed = recovery->FailuresObservedAtLaunch();
      e.previous_launch_unfinished = recovery->PreviousLaunchUnfinished();
    }
    auto runtime = integration::ReadRuntimeDiagnostics(*frame.GetPrimaryCanvas());
    auto &plugins = runtime["plugins"];
    for (int i=0; i<plugins.Size() && i<128; ++i)
      e.plugins.push_back(plugins[i]["name"].AsString().ToStdString(wxConvUTF8)+" / "+
          plugins[i]["version"].AsString().ToStdString(wxConvUTF8)+" / initialized "+
          (plugins[i]["initialized"].AsBool() ? "yes" : "no"));
    return e;
  };
  actions.diagnostics_folder=[] {
    if(!diagnostic_directory.empty()) wxLaunchDefaultApplication(wxString::FromUTF8(diagnostic_directory));
  };
  actions.diagnostic_snapshot = [&frame, &manager, last = vessel::Time{}](
                                    const vessel::VesselState &state,
                                    const smartnav::EnergyPrediction &energy,
                                    const std::string &page) mutable {
    const auto now=vessel::Clock::now();
    if (diagnostic_directory.empty() || now - last < std::chrono::seconds(1))
      return;
    last=now;
    XNAV_TEST_UI_TRACE("snapshot.begin");
    auto runtime =
        integration::ReadRuntimeDiagnostics(*frame.GetPrimaryCanvas());
    runtime["chart_presentation"]["status"] = wxString::FromUTF8(integration::ChartPresentationStatus());
    runtime["chart_presentation"]["requested"] = wxString(integration::XNavChartRequested() ? "XNav" : "Standard");
    runtime["test_fixtures"] = integration::TestFixturesEnabled();
    runtime["build_purpose"] = wxString::FromUTF8(integration::BuildPurpose().data());
    if (online_ais) {
      const auto copy = online_ais->Read(now);
      auto &health = runtime["online_ais"];
      health["enabled"] = online_ais->Enabled();
      health["credential_present"] = online_ais->CredentialPresent();
      health["connection_state"] = static_cast<int>(copy.health.connection);
      health["subscription_confirmed"] = copy.health.subscription_confirmed;
      health["targets"] = static_cast<int>(copy.targets.targets.size());
      health["accepted"] = wxString::Format("%llu", static_cast<unsigned long long>(copy.health.accepted));
      health["rejected"] = wxString::Format("%llu", static_cast<unsigned long long>(copy.health.rejected));
      health["reconnects"] = wxString::Format("%llu", static_cast<unsigned long long>(copy.health.reconnects));
    }
#if XNAV_ENABLE_TEST_FIXTURES
    // Copied wxAUI state for the isolated plugin-workspace regression.
    runtime["test_workspace_perspective"] = manager.SavePerspective();
#endif
    if (shell) {
      runtime["display"]["light"] = wxString::FromUTF8(shell->LightName());
      runtime["display"]["native_caption_themed"] = shell->NativeCaptionThemed();
      runtime["display"]["minimum_value_height_dip"] = shell->MinimumValueHeight();
      auto geometry = [](const std::vector<ui::ProductGeometry> &items) {
        wxJSONValue result(wxJSONTYPE_ARRAY);
        for(const auto &item:items) {
          wxJSONValue row;
          row["label"]=wxString::FromUTF8(item.label);
          if(!item.accessible_name.empty()) row["accessible_name"]=wxString::FromUTF8(item.accessible_name);
          row["x"]=item.screen.x; row["y"]=item.screen.y;
          row["width"]=item.screen.width; row["height"]=item.screen.height;
          row["enabled"]=item.enabled; row["visible"]=item.visible;
          result.Append(row);
        }
        return result;
      };
      runtime["display"]["product_controls"]=geometry(shell->ProductControls());
      runtime["display"]["product_regions"]=geometry(shell->ProductRegions());
      runtime["display"]["rail_regions"]=geometry(shell->RailRegions());
      runtime["display"]["interaction_controls"]=geometry(shell->InteractionControls());
      if (const auto drawer = shell->DrawerRegion()) {
        runtime["display"]["drawer"]["x"] = drawer->x;
        runtime["display"]["drawer"]["y"] = drawer->y;
        runtime["display"]["drawer"]["width"] = drawer->width;
        runtime["display"]["drawer"]["height"] = drawer->height;
      }
      runtime["display"]["route_creation_active"]=shell->RouteCreationActive();
      const auto chart_bounds=frame.GetPrimaryCanvas()->GetScreenRect();
      runtime["display"]["chart_region"]["x"]=chart_bounds.x;
      runtime["display"]["chart_region"]["y"]=chart_bounds.y;
      runtime["display"]["chart_region"]["width"]=chart_bounds.width;
      runtime["display"]["chart_region"]["height"]=chart_bounds.height;
      runtime["display"]["page_scroll_px"] = shell->PageScrollPosition();
      runtime["display"]["can_scroll_up"] = shell->CanScrollPage(-1);
      runtime["display"]["can_scroll_down"] = shell->CanScrollPage(1);
      runtime["smartnav"]["route_valid"] = shell->Advice().route_valid;
      runtime["smartnav"]["reason"] = wxString::FromUTF8(shell->Advice().reason);
      runtime["smartnav"]["event_count"] = static_cast<int>(shell->Advice().events.size());
      int ais_events=0;
      for(const auto &event:shell->Advice().events) if(event.kind==smartnav::EventKind::AisEncounter) ++ais_events;
      runtime["smartnav"]["ais_event_count"] = ais_events;
      runtime["ais_selected_mmsi"] = shell->SelectedAis();
      runtime["online_ais"]["chart_marks"] = static_cast<int>(online_chart.Size());
      runtime["alerts"] = wxJSONValue(wxJSONTYPE_ARRAY);
      for (const auto &a : shell->Alerts()) {
        wxJSONValue alert;
        alert["id"] = wxString::FromUTF8(a.id);
        alert["level"] = wxString::FromUTF8(application::AlertLevelName(a.level));
        alert["acknowledged"] = a.acknowledged;
        alert["episode"] = wxString::Format("%llu", static_cast<unsigned long long>(a.episode));
        runtime["alerts"].Append(alert);
      }
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
    if (commissioning) {
      const auto r = commissioning->RecordingStatus();
      runtime["recording"]["active"] = r.active;
      runtime["recording"]["captured"] = static_cast<int>(r.captured);
      runtime["recording"]["published"] = static_cast<int>(r.published);
      runtime["recording"]["error"] = wxString::FromUTF8(r.error);
      runtime["replay"]["active"] = commissioning->Replaying();
      runtime["replay"]["allows_hardware_control"] =
          commissioning->AllowsHardwareControl();
      if (const auto view = commissioning->ReadReplay(now)) {
        runtime["replay"]["paused"] = view->paused;
        runtime["replay"]["ended"] = view->ended;
        runtime["replay"]["elapsed_ms"] =
            static_cast<int>(view->elapsed.count());
      }
    }
    if (marine && !state.replayed && !state.simulated)
      runtime["boat_bridge"]["status"] = wxString::FromUTF8(marine->BoatBridgeStatus(now));
    if (pilots && !state.replayed) {
      const auto view = pilots->Select(state.simulated).GetState(now);
      auto &pilot = runtime["pilot"];
      pilot["simulated"] = state.simulated;
      pilot["enabled"] = view.enabled;
      pilot["fresh"] = view.fresh;
      pilot["mode"] = wxString::FromUTF8(adapters::PilotModeName(view.feedback.mode));
      pilot["source"] = wxString::FromUTF8(view.feedback.source);
      pilot["feedback_sequence"] = wxString::Format("%llu", static_cast<unsigned long long>(view.feedback.sequence));
      pilot["connection_epoch"] = wxString::Format("%llu", static_cast<unsigned long long>(view.feedback.connection_epoch));
      pilot["control_capability"] = view.capabilities.manual_control;
      pilot["track_capability"] = view.capabilities.track;
      pilot["wind_capability"] = view.capabilities.wind;
      pilot["command_state"] = wxString::FromUTF8(adapters::CommandStateName(view.command.state));
      pilot["command_detail"] = wxString::FromUTF8(view.command.detail);
      pilot["command_id"] = wxString::Format("%llu", static_cast<unsigned long long>(view.command.request.id));
      pilot["adapter"] = wxString::FromUTF8(state.simulated ? "DEMO" : pilots->hardware.Description());
      const auto target = vessel::Assess(view.feedback.locked_heading_magnetic_deg, now);
      const auto heading = vessel::Assess(view.feedback.heading_magnetic_deg, now);
      pilot["locked_heading_quality"] = wxString::FromUTF8(vessel::QualityName(target.quality));
      pilot["actual_heading_quality"] = wxString::FromUTF8(vessel::QualityName(heading.quality));
      if (target.value) pilot["locked_heading_magnetic_deg"] = *target.value;
      if (heading.value) pilot["actual_heading_magnetic_deg"] = *heading.value;
      if (view.feedback.sequence && view.feedback.observed_at <= now)
        pilot["mode_age_ms"] = static_cast<int>(std::min<long long>(2147483647,
            std::chrono::duration_cast<vessel::Duration>(now-view.feedback.observed_at).count()));
    }
    XNAV_TEST_UI_TRACE("snapshot.write");
    integration::WritePreviewDiagnostics(
        diagnostic_directory + "/opennav-diagnostics.json", state,
        integration::PreviewBuildInfo(
            frame.GetDPI().x,
            g_BasePlatform->GetPrivateDataDir().ToStdString(wxConvUTF8)),
        energy,
        state.replayed ? commissioning->ReplayAssumptions() : settings->Read(),
        state.replayed ? std::vector<vessel::SourceHealth>{}
                       : marine->Health(now),
        page, runtime);
    XNAV_TEST_UI_TRACE("snapshot.end");
  };
#if XNAV_ENABLE_TEST_FIXTURES
  actions.demo_chart=[] { if(g_bDeferredInitDone) JumpToPosition(59.08,18.5,0.003); };
#endif
  actions.theme = [&frame](ui::LightMode mode) {
    const auto scheme = mode == ui::LightMode::Night ? GLOBAL_COLOR_SCHEME_NIGHT
                        : mode == ui::LightMode::Dusk ? GLOBAL_COLOR_SCHEME_DUSK
                                                     : GLOBAL_COLOR_SCHEME_DAY;
    // The software shapefile renderer otherwise retains its constructor's day
    // land colour. Reuse the pinned upstream world-chart palette exactly.
    GSHHSChart palette;
    palette.SetColorScheme(scheme);
    gShapeBasemap.SetBasemapLandColor(palette.land);
    frame.SetAndApplyColorScheme(scheme);
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

void AfterDeferredInitialization() {
  integration::EnableDashboardPresentation();
  if (shell && host) {
    for (auto *window : host->GetChildren())
      if (auto *canvas = dynamic_cast<ChartCanvas *>(window))
        canvas->SetShowGPSCompassWindow(false);
    GSHHSChart palette;
    palette.SetColorScheme(global_color_scheme);
    gShapeBasemap.SetBasemapLandColor(palette.land);
    host->GetPrimaryCanvas()->ReloadVP();
  }
  if (!recovery_safe || recovery_notice_scheduled || !host) return;
  recovery_notice_scheduled = true;
  host->CallAfter([] {
    if (!host || restart) return;
    wxLogMessage("OpenNav recovery notice after deferred startup");
    wxMessageBox(
        "XNav did not complete startup reliably. OpenCPN is running in "
        "Safe Mode with OpenNav modules, plugins and OpenGL disabled. "
        "Navigation data has not been reset. Inspect the OpenCPN log; use "
        "Switch to XNav only when ready to retry.",
        "OpenNav startup recovery", wxOK | wxICON_INFORMATION, host);
  });
}

void AfterSettingsReconfigured() {
  // Options may detach/recreate AUI canvas panes. Reconcile only after upstream
  // has completed its normal chart/configuration work; never retain old panes.
  if (!shell || !host || !IsXNav()) return;
  for (auto *window : host->GetChildren())
    if (auto *canvas = dynamic_cast<ChartCanvas *>(window))
      canvas->SetShowGPSCompassWindow(false);
  shell->AfterCanvasLayoutChanged();
  host->InvalidateAllGL();
  host->ReloadAllVP();
  host->RefreshAllCanvas(false);
}

bool IsTransientXNavPane(const wxWindow *window) {
  return shell && IsXNav() && shell->OwnsPane(window);
}

bool LoadPersistentPerspective(wxAuiManager &manager, const wxString &perspective) {
  if (shell && IsXNav() && shell->OwnsManager(manager)) {
    OpenNavDashboardLayoutScope dashboard_layout;
    return shell->LoadPersistentPerspective(perspective);
  }
  return manager.LoadPerspective(perspective, false);
}

void AfterAnchorWatch(){
  if (recovery && IsXNav())
    recovery->ObserveHealthy(g_bDeferredInitDone, vessel::Clock::now());
  if(!navigation)return;
  auto current=integration::ObserveAnchor(selected_navigation.navigation,vessel::Clock::now());
  application::RetainAnchorHistory(current,anchor_state);
  anchor_state=std::move(current);
}
bool ShowNavigationObjectCard(const std::string& id,bool route){if(!IsXNav()||!shell||!host)return false;host->CallAfter([id,route]{if(shell)shell->ShowObject(id,route);});return true;}
bool ShowChartContext(double latitude, double longitude) {
  if (!IsXNav() || !shell || !host || !std::isfinite(latitude) ||
      !std::isfinite(longitude) || std::abs(latitude) > 90) return false;
  longitude = std::remainder(longitude, 360.0);
  host->CallAfter([latitude, longitude] {
    if (shell) shell->ShowChartContext({latitude, longitude});
  });
  return true;
}
bool IsAisSelected(int mmsi) { return IsXNav() && shell && mmsi > 0 && shell->SelectedAis() == mmsi; }
void DrawOnlineAis(ocpnDC &dc, ViewPort &vp, ChartCanvas *canvas) {
  if (IsXNav() && shell && canvas && !restart)
    online_chart.Draw(dc, vp, *canvas);
}
bool ShowOnlineAisAt(ChartCanvas &canvas, int x, int y) {
  if (!IsXNav() || !shell || restart) return false;
  const int id=online_chart.HitTest(canvas.GetVP(), canvas, x, y);
  return id>0 && ShowAisCard(id);
}
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
  wxLogMessage("OpenNav close preparation: preserving shared configuration");
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
#ifdef OPENNAV_ROUTE_TESTS
  test::StopObjectScenario();
#endif
  navigation.reset();
  marine.reset();
  selected_navigation = {};
  route_progress.reset();
  integration::FinishDashboardPresentation();
  shell.reset();
  online_chart.Clear();
  online_ais.reset(); // Stop/join worker before configuration/host teardown.
  commissioning.reset();
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

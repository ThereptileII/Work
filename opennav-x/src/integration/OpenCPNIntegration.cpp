#include "application/Brand.h"
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
#include "integration/UpdateStartupReceipt.h"
#include "integration/RoutePassWatch.h"
#include "integration/RuntimeDiagnostics.h"
#include "integration/SettingsStore.h"
#include "integration/ChartPresentation.h"
#include "integration/OChartsPresentation.h"
#include "integration/OnlineAis.h"
#include "integration/OnlineAisOverlay.h"
#include "integration/OnlineWeather.h"
#include "integration/WeatherOverlay.h"
#include "weather/GribStream.h"
#include "weather/IxForecastTransport.h"
#include "weather/QueryBuilder.h"
#include "integration/AisViewport.h"
#include "integration/StartupMode.h"
#if XNAV_ENABLE_TEST_FIXTURES
#include "adapters/SimulatedAutopilot.h"
#endif
#include "model/base_platform.h"
#include "model/comm_drv_registry.h"
#include "model/routeman.h"
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
#include "s52plib.h"
#include "s52utils.h"
#ifdef Status
#undef Status  // GLX/X11 macro must not rewrite SettingsStore::Status().
#endif
extern s52plib *ps52plib;

#include <wx/cmdline.h>
#include <wx/fileconf.h>
#include <wx/log.h>
#include <wx/menu.h>
#include <wx/msgdlg.h>
#include <wx/stdpaths.h>
#include <wx/filefn.h>
#include <wx/filename.h>
#include <wx/utils.h>
#include "diagnostics/TestEarlyStartupTrace.h"
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
std::unique_ptr<integration::OnlineWeather> online_weather;
std::shared_ptr<diagnostics::Commissioning> commissioning;
std::unique_ptr<NavigationBridge> navigation;
std::unique_ptr<integration::MarineBridge> marine;
std::unique_ptr<integration::SettingsStore> settings;
bool fresh_setup_profile = false;
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
  wxLogMessage("SKAGER human mode request: %s", safe ? "safe"
                                                 : mode == InterfaceMode::XNav
                                                     ? "xnav"
                                                     : "legacy");
  if (mode == InterfaceMode::XNav && recovery && recovery->RequiresSafe() &&
      !recovery->Retry()) {
    wxMessageBox("Cannot reset the startup recovery record. Inspect the "
                 "profile storage and diagnostics before retrying SKAGER.",
                 "SKAGER recovery", wxOK | wxICON_ERROR, host);
    return;
  }
  if (pilots) pilots->live.Enable(false, vessel::Clock::now());
  restart = mode;
  restart_safe=safe;
  // OpenCPN may refuse close while initialising, compressing or updating charts.
  // Only PrepareClose commits the request and releases the shell.
  // A canvas popup still unwinds and unbinds handlers after its menu callback.
  // Closing there would delete the canvas while that stack is still active.
  host->CallAfter([] {
    if (!host) { restart.reset(); return; }
    wxLogMessage("SKAGER mode request: asking OpenCPN to close");
    host->Close();
    if (host) {
      wxLogMessage(
          "SKAGER mode request: OpenCPN kept the current window open");
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
  parser.AddSwitch("", "xnav", "SKAGER interface");
  parser.AddSwitch("", "legacy", "Original OpenCPN interface");
  parser.AddSwitch("", "safe-mode", "Legacy recovery; SKAGER modules disabled");
#if XNAV_ENABLE_TEST_FIXTURES
  parser.AddSwitch("", "xnav-demo", "Explicit simulated SKAGER telemetry; no device commands");
#endif
#ifdef OPENNAV_ROUTE_TESTS
  parser.AddSwitch("", "xnav-route-fixture", "TEST BUILD ONLY: isolated route contract scenario");
  parser.AddSwitch("", "xnav-object-fixture", "TEST BUILD ONLY: isolated navigation object scenario");
#endif
}

bool ParseCommandLine(wxCmdLineParser& parser) {
  integration::CaptureUpdateStartupReceipt();
  if (integration::ParseInstallerSelfTest(parser)) return true;
  flags = {parser.Found("xnav"), parser.Found("legacy"),
           parser.Found("safe-mode") || parser.Found("safe_mode")};
#if XNAV_ENABLE_TEST_FIXTURES
  demo = parser.Found("xnav-demo");
#endif
  try { (void)integration::ResolveStartup(flags); }
  catch (const std::exception& error) {
    XNAV_EARLY_STARTUP_TRACE(ModeConflict, false);
    std::cerr << error.what() << '\n'; return false;
  }
  if (parser.Found("remote") && (flags.xnav || flags.legacy || flags.safe || demo)) {
    XNAV_EARLY_STARTUP_TRACE(RemoteConflict, false);
    std::cerr << "SKAGER startup options cannot be combined with --remote\n";
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
            "Cannot use the portable SKAGER profile as its working directory");
      if (parser.Found("remote"))
        throw std::runtime_error("portable SKAGER does not send remote "
                                 "commands to another OpenCPN instance");
    } else if(!configdir.empty()) diagnostic_directory=configdir.ToStdString(wxConvUTF8);
  } catch(const std::exception& e) {
    XNAV_EARLY_STARTUP_TRACE(PreviewPathFailure, false);
    std::cerr<<e.what()<<'\n';return false;
  }
  if (!configdir.empty()) {
    profile_arguments.push_back("--configdir");
    profile_arguments.push_back(configdir.ToStdString(wxConvUTF8));
  }
#ifdef OPENNAV_ROUTE_TESTS
  if(parser.Found("xnav-object-fixture")){
    if(!flags.xnav||flags.safe||flags.legacy||demo||configdir.empty()||parser.Found("xnav-route-fixture")||!wxFileExists(configdir+"/OPENNAV_OBJECT_FIXTURE")){
      XNAV_EARLY_STARTUP_TRACE(FixturePolicyFailure, false);
      std::cerr<<"Object fixture requires explicit SKAGER and a marked disposable profile\n";return false;
    }
    object_test_profile=configdir.ToStdString(wxConvUTF8);
  }
  if (parser.Found("xnav-route-fixture")) {
    if (!flags.xnav || flags.safe || flags.legacy || demo || configdir.empty() ||
        !wxFileExists(configdir + "/OPENNAV_ROUTE_FIXTURE")) {
      XNAV_EARLY_STARTUP_TRACE(FixturePolicyFailure, false);
      std::cerr << "Route fixture requires explicit SKAGER and a marked disposable profile\n";
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
    XNAV_EARLY_STARTUP_TRACE(FixturePolicyFailure, false);
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
    wxLogWarning("SKAGER automatic Safe Mode: %s",
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
      wxLogMessage("SKAGER installed resource defaults: original supported OpenCPN; configured selections preserved");
    }
  }
}

void SelectMode(wxFileConfig& config, bool upstream_safe) {
  // Snapshot before BeginXNav creates a startup journal. Earlier generations
  // always left that journal, including profiles with no vessel configuration.
  fresh_setup_profile = !preview_paths && !config.HasGroup("/OpenNav") &&
      !wxFileExists(wxFileName(g_BasePlatform->GetPrivateDataDir(),
                              "opennav-startup.state").GetFullPath());
  if (preview_paths) {
    const auto basemap = integration::PreviewBasemapDefault(
        preview_paths->root, gWorldShapefileLocation.ToStdString(wxConvUTF8));
    if (basemap) {
      gWorldShapefileLocation = wxString::FromUTF8(platform::PathUtf8(*basemap));
      wxLogMessage("SKAGER portable basemap: using bundled coastline at %s",
                   gWorldShapefileLocation);
    }
  }
  wxString value;
  std::optional<InterfaceMode> persisted;
  if (config.Read("/OpenNav/InterfaceMode", &value)) {
    persisted = integration::ParseInterfaceMode(value.ToStdString());
    if (!persisted) {
      wxLogWarning("Invalid SKAGER InterfaceMode; using Legacy recovery");
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
      wxLogWarning("SKAGER diagnostic directory unavailable: %s", folder);
  }
  integration::ConfigureChartPresentation(config, IsXNav());
  executable = wxStandardPaths::Get().GetExecutablePath().ToStdString(wxConvUTF8);
  wxLogMessage("SKAGER startup: %s", selected == StartupMode::Safe ? "safe" : IsXNav() ? "xnav" : "legacy");
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
    frame.SetTitle(selected == StartupMode::Safe ? application::brand::SafeModeTitle : application::brand::LegacyTitle);
    return;
  }
  frame.SetTitle(application::brand::WindowTitle);
  settings = std::make_unique<integration::SettingsStore>(config, fresh_setup_profile);
  marine = std::make_unique<integration::MarineBridge>();
  auto configure_sources = [] {
    if (pilots) pilots->live.Enable(false, vessel::Clock::now());
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
        if (pilots) pilots->live.Enable(false, vessel::Clock::now());
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
    value.radius_nm = online_ais->RadiusNm();
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
  actions.online_ais.set_radius_nm = [](int radius) {
    return online_ais ? online_ais->SetRadiusNm(radius)
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
          box.GetMinLat(), box.GetMaxLat(), box.GetMinLon(), box.GetMaxLon(),
          ais::AreaCenter{vp.clat, vp.clon});
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
  // SCRUM-324..331: optional GRIBstream forecast, XNav only, default OFF.
  // Construction never fetches; the worker owns all network and token reads.
  online_weather = std::make_unique<integration::OnlineWeather>(config,
      ais::CreateWeatherCredentials(),
      std::make_unique<weather::ForecastService>(ais::CreateWeatherCredentials(),
                                                 weather::CreateIxForecastTransport()));
  integration::SetWeatherSource([] {
    return online_weather ? online_weather->Read(std::chrono::system_clock::now())
                          : weather::ForecastSnapshot{};
  });
  actions.weather.read = [](vessel::Time) {
    // Freshness is decided from wall-clock fetch time, not the UI clock.
    return online_weather ? online_weather->Read(std::chrono::system_clock::now())
                          : weather::ForecastSnapshot{};
  };
  actions.weather.enable = [](bool enabled) {
    return online_weather ? online_weather->Enable(enabled)
        : application::CommandResult{false, "Weather unavailable during shutdown"};
  };
  actions.weather.store_token = [](const ais::Secret &token) {
    return online_weather ? online_weather->StoreToken(token)
        : application::CommandResult{false, "Credential storage unavailable during shutdown"};
  };
  actions.weather.remove_token = [] {
    return online_weather ? online_weather->RemoveToken()
        : application::CommandResult{false, "Credential storage unavailable during shutdown"};
  };
  actions.weather.token_present = [] { return online_weather && online_weather->TokenPresent(); };
  actions.weather.test_connection = [] {
    return online_weather ? online_weather->TestConnection()
        : application::CommandResult{false, "Weather unavailable during shutdown"};
  };
  actions.weather.request = [](const weather::ForecastQuery &query) {
    if (online_weather) online_weather->Request(query);
  };
  actions.weather.last_test = [] {
    return online_weather ? online_weather->LastTest() : weather::ConnectionTest{};
  };
  actions.weather.test_pending = [] { return online_weather && online_weather->TestPending(); };
  static std::vector<weather::Coordinate> weather_focus_route;
  static bool weather_query_dirty = false;
  actions.weather.focus_route = [](std::vector<weather::Coordinate> points) {
    if (points.size() > 1000) points.resize(1000);  // SampleRoute bounds the query.
    if (points.size() != weather_focus_route.size() ||
        !std::equal(points.begin(), points.end(), weather_focus_route.begin(),
                    [](const auto &a, const auto &b) {
                      return a.latitude_deg == b.latitude_deg && a.longitude_deg == b.longitude_deg;
                    })) {
      weather_focus_route = std::move(points);
      weather_query_dirty = true;  // Rebuild on the next tick, not in a minute.
    }
  };
  actions.weather_tick = [&frame, last = std::chrono::steady_clock::time_point{}](bool live_allowed) mutable {
    if (!online_weather || !online_weather->Enabled() || restart) return;
    const auto steady = std::chrono::steady_clock::now();
    if (!weather_query_dirty && last != std::chrono::steady_clock::time_point{} &&
        steady - last < std::chrono::seconds(60))
      return;
    last = steady;
    weather_query_dirty = false;
    weather::QueryInputs in;
    in.now = std::chrono::system_clock::now();
    if (live_allowed && (!commissioning || !commissioning->Replaying())) {
      // Fresh real fix only; never a simulated, replayed or stale position.
      const auto now = vessel::Clock::now();
      const auto lat = vessel::Assess(selected_navigation.navigation.latitude_deg, now);
      const auto lon = vessel::Assess(selected_navigation.navigation.longitude_deg, now);
      const auto fresh = [](vessel::Quality q) {
        return q == vessel::Quality::Live || q == vessel::Quality::Aging;
      };
      if (fresh(lat.quality) && fresh(lon.quality) && lat.value && lon.value)
        in.vessel = weather::Coordinate{*lat.value, *lon.value};
    }
    try {
      const auto route = integration::CopyActiveRoute(g_pRouteMan);
      if (route.active)
        for (const auto &point : route.points)
          in.route.push_back({point.latitude_deg, point.longitude_deg});
    } catch (const std::exception &) {
      in.route.clear();
    }
    // The active route has priority; otherwise the route the user is viewing.
    if (in.route.empty()) in.route = weather_focus_route;
    if (auto *canvas = frame.GetPrimaryCanvas()) {
      auto &vp = canvas->GetVP();
      const auto &box = vp.GetBBox();
      if (vp.IsValid() && box.GetValid())
        in.chart = weather::ChartBox{box.GetMinLat(), box.GetMaxLat(), box.GetMinLon(),
                                     box.GetMaxLon()};
    }
    online_weather->Request(weather::BuildForecastQuery(in));
  };
  actions.settings = [] { return settings->Read(); };
  actions.display = [] { return settings->Display(); };
  actions.save_display = [](const application::DisplayPreferences &value) {
    if (commissioning && commissioning->Replaying())
      return application::CommandResult{false,"Stop REPLAY before changing live settings"};
    return settings->SaveDisplay(value);
  };
  actions.vessel_name = [] { return settings->VesselName(); };
  integration::SetChartVesselNameProvider(
      [] { return settings ? settings->VesselName() : std::string(); });
  actions.chart_safety_depth_m = [] {
    return S52_getMarinerParam(S52_MAR_SAFETY_CONTOUR);
  };
  actions.save_vessel = [&frame](const application::Settings &next,
                                 const std::string &name,double safety_m) {
    if (commissioning && commissioning->Replaying())
      return application::CommandResult{false,"Stop REPLAY before changing live settings"};
    auto result = settings->SaveVessel(next,name,safety_m);
    if (!result.ok) return result;
    if (std::isfinite(safety_m)) {
      // The stock Options editor couples sounding depth and colour contour.
      S52_setMarinerParam(S52_MAR_SAFETY_DEPTH,safety_m);
      S52_setMarinerParam(S52_MAR_SAFETY_CONTOUR,safety_m);
      if (ps52plib) {
        ps52plib->UpdateMarinerParams();
        ps52plib->GenerateStateHash();
      }
      if (auto *canvas=frame.GetPrimaryCanvas()) canvas->ZoomCanvasSimple(1.0001);
      frame.InvalidateAllGL();
      frame.RefreshAllCanvas(false);
    }
    return result;
  };
  actions.backup_settings = [] {
    return application::SettingsBackup{settings->Read(),settings->Display(),
        settings->VesselName(),S52_getMarinerParam(S52_MAR_SAFETY_CONTOUR)};
  };
  actions.restore_settings = [&frame,configure_sources](const application::SettingsBackup &backup) {
    if (restart || (commissioning && commissioning->Replaying()))
      return application::CommandResult{false,"Stop REPLAY or finish restart before restoring settings"};
    const auto result=settings->RestoreBackup(backup);
    if (!result.ok) return result;
    if (pilots) {
      pilots->live.Enable(false,vessel::Clock::now());
      pilots->hardware.Configure(settings->Read().pilot);
    }
    configure_sources();
    if (std::isfinite(backup.chart_safety_depth_m)) {
      S52_setMarinerParam(S52_MAR_SAFETY_DEPTH,backup.chart_safety_depth_m);
      S52_setMarinerParam(S52_MAR_SAFETY_CONTOUR,backup.chart_safety_depth_m);
      if (ps52plib) { ps52plib->UpdateMarinerParams(); ps52plib->GenerateStateHash(); }
      if (auto *canvas=frame.GetPrimaryCanvas()) canvas->ZoomCanvasSimple(1.0001);
      frame.InvalidateAllGL(); frame.RefreshAllCanvas(false);
    }
    return result;
  };
  actions.setup_state = [] { return settings->SetupState(); };
  actions.request_setup = [] {
    if (restart || (commissioning && commissioning->Replaying()))
      return application::CommandResult{false,"Stop REPLAY or finish restart before opening setup"};
    return settings->RequestBoatSetup();
  };
  actions.save_setup = [&frame](const application::BoatSetupDraft& draft) {
    if (restart || (commissioning && commissioning->Replaying()))
      return application::CommandResult{false,"Stop REPLAY or finish restart before saving setup"};
    const auto result = settings->SaveBoatSetup(draft);
    if (result.ok && std::isfinite(draft.safety_depth_m)) {
      S52_setMarinerParam(S52_MAR_SAFETY_DEPTH,draft.safety_depth_m);
      S52_setMarinerParam(S52_MAR_SAFETY_CONTOUR,draft.safety_depth_m);
      if(ps52plib) { ps52plib->UpdateMarinerParams(); ps52plib->GenerateStateHash(); }
      if(auto* canvas=frame.GetPrimaryCanvas()) canvas->ZoomCanvasSimple(1.0001);
      frame.InvalidateAllGL(); frame.RefreshAllCanvas(false);
    }
    return result;
  };
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
         old_pilot.address != s.pilot.address ||
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
  actions.navigation=integration::MakeNavigationActions(frame,[]{return selected_navigation.navigation;},[](vessel::Time now){
    auto current=integration::ObserveAnchor(selected_navigation.navigation,now);
    application::RetainAnchorHistory(current,anchor_state);
    anchor_state=std::move(current);
    return anchor_state;
  });
  actions.navigation =
      application::GuardNavigationChanges(std::move(actions.navigation), [] {
        return !commissioning || !commissioning->Replaying();
      });
  actions.route_creating=[&frame]{return frame.GetPrimaryCanvas()->m_routeState>0;};
  pilots = std::make_unique<PilotServices>([] {
    return (integration::PilotLoopbackTestsEnabled() || integration::PilotManualSerialEnabled()) && pilots && !pilots->was_demo && !restart &&
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
  actions.pilot_detected = []() -> std::optional<adapters::St4000Binding> {
    if (!pilots) return std::nullopt;
    return pilots->hardware.DetectedBinding();
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
    view.output_unavailable = !simulated && !integration::PilotLoopbackTestsEnabled() && !integration::PilotManualSerialEnabled();
    view.adapter_status = simulated ? "DEMO / simulated feedback" : pilots->hardware.Description();
    if (!simulated && !view.output_unavailable)
      view.control_blocker = pilots->hardware.ControlBlocker();
    if (before != view.command.state)
      wxLogMessage("SKAGER manual pilot %llu: %s / %s",
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
    wxLogMessage("SKAGER manual autopilot [%s] request %llu: %s / %s",
                 simulated ? "DEMO" : "ST4000 live adapter",
                 static_cast<unsigned long long>(c.request.id),
                 wxString::FromUTF8(adapters::CommandStateName(c.state)),
                 wxString::FromUTF8(c.detail));
  };
  actions.pilot_enable = [](bool simulated, bool enabled) {
    if (enabled && !simulated && !pilots->hardware.Capabilities().manual_control)
      return;
    if (enabled && commissioning && !commissioning->AllowsHardwareControl())
      return;
    pilots->Select(simulated).Enable(enabled, vessel::Clock::now());
    wxLogMessage("SKAGER manual autopilot %s: %s",
                 simulated ? "DEMO" : "ST4000 live adapter",
                 enabled ? "enable requested" : "disabled");
  };
  actions.zoom_in = [&frame] { frame.GetPrimaryCanvas()->ZoomCanvas(2.0, false); };
  actions.zoom_out = [&frame] { frame.GetPrimaryCanvas()->ZoomCanvas(0.5, false); };
  actions.follow = [&frame] { frame.TogglebFollow(frame.GetPrimaryCanvas()); };
  actions.following = [&frame] {
    auto *canvas = frame.GetPrimaryCanvas();
    return canvas && canvas->GetbFollow();
  };
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
    runtime["chart_presentation"]["core"]["available"] = ps52plib != nullptr;
    if (ps52plib) {
      runtime["chart_presentation"]["core"]["saved_point_style"] =
          static_cast<int>(ps52plib->m_nSymbolStyle);
      runtime["chart_presentation"]["core"]["effective_point_style"] =
          static_cast<int>(ps52plib->GetEffectiveSymbolStyle());
    }
    const auto private_style = integration::ReadOChartsPointStyle();
    runtime["chart_presentation"]["private_ocharts"]["available"] = private_style.available;
    if (private_style.available)
      runtime["chart_presentation"]["private_ocharts"]["effective_point_style"] =
          static_cast<int>(private_style.effective_point_style);
    runtime["test_fixtures"] = integration::TestFixturesEnabled();
    runtime["build_purpose"] = wxString::FromUTF8(integration::BuildPurpose().data());
    if (online_ais) {
      const auto copy = online_ais->Read(now);
      auto &health = runtime["online_ais"];
      health["enabled"] = online_ais->Enabled();
      health["credential_present"] = online_ais->CredentialPresent();
      health["connection_state"] = static_cast<int>(copy.health.connection);
      health["subscription_confirmed"] = copy.health.subscription_confirmed;
      health["subscription_pending"] = copy.health.subscription_pending;
      health["subscription_awaiting_confirmation"] = copy.health.subscription_awaiting_confirmation;
      health["radius_nm"] = online_ais->RadiusNm();
      health["cached_positions"] = static_cast<int>(copy.cached_position_count);
      health["in_radius_positions"] = static_cast<int>(copy.targets.targets.size());
      health["received_messages"] = wxString::Format("%llu", static_cast<unsigned long long>(copy.health.received_messages));
      health["ignored_messages"] = wxString::Format("%llu", static_cast<unsigned long long>(copy.health.ignored_messages));
      health["targets"] = static_cast<int>(copy.targets.targets.size());
      health["accepted"] = wxString::Format("%llu", static_cast<unsigned long long>(copy.health.accepted));
      health["rejected"] = wxString::Format("%llu", static_cast<unsigned long long>(copy.health.rejected));
      health["reconnects"] = wxString::Format("%llu", static_cast<unsigned long long>(copy.health.reconnects));
    }
    if (online_weather) {
      // Credential-free: presence flag and provider status text only.
      const auto forecast = online_weather->Read(std::chrono::system_clock::now());
      auto &health = runtime["weather"];
      health["enabled"] = online_weather->Enabled();
      health["token_present"] = online_weather->TokenPresent();
      health["model"] = wxString::FromUTF8(forecast.model);
      health["state"] = static_cast<int>(forecast.state);
      health["status"] = wxString::FromUTF8(forecast.status);
      health["winds"] = static_cast<int>(forecast.winds.size());
      if (forecast.fetched_at)
        health["fetched_utc"] = wxString::FromUTF8(weather::gribstream::FormatUtc(*forecast.fetched_at));
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
      const auto footer=shell->FooterRegion();
      runtime["display"]["footer_region"]["x"]=footer.x;
      runtime["display"]["footer_region"]["y"]=footer.y;
      runtime["display"]["footer_region"]["width"]=footer.width;
      runtime["display"]["footer_region"]["height"]=footer.height;
      runtime["display"]["footer_middle_visible"]=shell->FooterMiddleVisible();
      const auto horizon=shell->HorizonRegions();
      const auto region=[](const wxRect &r){wxJSONValue v;v["x"]=r.x;v["y"]=r.y;v["width"]=r.width;v["height"]=r.height;return v;};
      runtime["display"]["horizon_region"]=region(horizon.horizon);
      runtime["display"]["horizon_heading"]=region(horizon.heading);
      runtime["display"]["horizon_full_passage"]=region(horizon.full_passage);
      runtime["display"]["horizon_events"]=wxJSONValue(wxJSONTYPE_ARRAY);
      for(const auto &event:horizon.events)runtime["display"]["horizon_events"].Append(region(event));
      const auto &footer_view=shell->NavigationFooter();
      runtime["navigation_footer"]["navigation_state"]=wxString::FromUTF8(footer_view.navigation_state);
      runtime["navigation_footer"]["position"]=wxString::FromUTF8(footer_view.position);
      runtime["navigation_footer"]["cog"]=wxString::FromUTF8(footer_view.cog);
      runtime["navigation_footer"]["xte"]=wxString::FromUTF8(footer_view.xte);
      runtime["navigation_footer"]["health_summary"]=wxString::FromUTF8(footer_view.health_summary);
      runtime["navigation_footer"]["historical"]=footer_view.historical;
      const auto signal_name=[](application::SignalState state) {
        switch(state) {
#define FOOTER_STATE(x) case application::SignalState::x: return #x
          FOOTER_STATE(Current); FOOTER_STATE(Aging); FOOTER_STATE(Stale);
          FOOTER_STATE(Estimated); FOOTER_STATE(Uncertain); FOOTER_STATE(Invalid); FOOTER_STATE(Unavailable);
#undef FOOTER_STATE
        }
        return "Invalid";
      };
      runtime["navigation_footer"]["position_state"]=wxString::FromUTF8(signal_name(footer_view.position_state));
      runtime["navigation_footer"]["cog_state"]=wxString::FromUTF8(signal_name(footer_view.cog_state));
      runtime["navigation_footer"]["health_state"]=wxString::FromUTF8(signal_name(footer_view.health_state));
      runtime["navigation_footer"]["health_source"]=wxString::FromUTF8(footer_view.health_source);
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
      pilot["serial_session_enabled"] = !state.simulated && pilots->hardware.SessionEnabled();
      pilot["configured_permission"] = settings->Read().pilot.permit_control;
      pilot["output_unavailable"] = !state.simulated && !integration::PilotLoopbackTestsEnabled() && !integration::PilotManualSerialEnabled();
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
      if (!state.simulated) {
        const auto discovery = pilots->hardware.DiscoveryDiagnostics(now);
        auto &observed = pilot["discovery"];
        observed["verified_identities"] = static_cast<int>(discovery.verified_identities);
        observed["fresh_mode_sources_without_identity"] =
            static_cast<int>(discovery.fresh_mode_sources_without_identity);
        observed["stale_mode_sources"] = static_cast<int>(discovery.stale_mode_sources);
        observed["identity_conflicts"] = static_cast<int>(discovery.identity_conflicts);
        observed["traffic_limit_exceeded"] = discovery.traffic_limit_exceeded;
        observed["conflict_source"] = wxString::FromUTF8(discovery.conflict_source);
        const auto traffic = pilots->hardware.TrafficDiagnostics();
        auto &receive = pilot["receive_diagnostics"];
        receive["scope"] = "Passive received envelopes; not pilot identity or command confirmation";
        const auto count = [](std::uint64_t value) {
          return wxString::Format("%llu", static_cast<unsigned long long>(value));
        };
        receive["accepted_count"] = count(traffic.accepted_count);
        receive["sources"] = wxJSONValue(wxJSONTYPE_ARRAY);
        for (const auto &source : traffic.sources) {
          wxJSONValue entry;
          entry["interface"] = wxString::FromUTF8(source.interface_id);
          entry["pgn"] = static_cast<int>(source.pgn);
          entry["address"] = static_cast<int>(source.source);
          entry["count"] = count(source.accepted_count);
          entry["age_ms"] = count(static_cast<std::uint64_t>(
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  now - source.last_observed).count()));
          receive["sources"].Append(entry);
        }
        auto &rejected = receive["rejected"];
        rejected["unsupported_pgn"] = count(traffic.rejected.unsupported_pgn);
        rejected["invalid_interface"] = count(traffic.rejected.invalid_interface);
        rejected["invalid_type"] = count(traffic.rejected.invalid_type);
        rejected["invalid_length"] = count(traffic.rejected.invalid_length);
        rejected["pgn_mismatch"] = count(traffic.rejected.pgn_mismatch);
        rejected["invalid_source"] = count(traffic.rejected.invalid_source);
        rejected["invalid_time"] = count(traffic.rejected.invalid_time);
        rejected["future"] = count(traffic.rejected.future);
        rejected["stale"] = count(traffic.rejected.stale);
        rejected["out_of_order"] = count(traffic.rejected.out_of_order);
        rejected["capacity"] = count(traffic.rejected.capacity);
      }
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
      "SKAGER session " + std::to_string(vessel::Clock::now().time_since_epoch().count()));
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
  if (shell && host && !demo && !preview_paths) {
    host->CallAfter([] { if (shell && host && !restart) shell->ShowBoatSetup(); });
  }
  if (!recovery_safe || recovery_notice_scheduled || !host) return;
  recovery_notice_scheduled = true;
  host->CallAfter([] {
    if (!host || restart) return;
    wxLogMessage("SKAGER recovery notice after deferred startup");
    wxMessageBox(
        "SKAGER did not complete startup reliably. OpenCPN is running in "
        "Safe Mode with SKAGER modules, plugins and OpenGL disabled. "
        "Navigation data has not been reset. Inspect the OpenCPN log; use "
        "Switch to SKAGER only when ready to retry.",
        "SKAGER startup recovery", wxOK | wxICON_INFORMATION, host);
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

bool HasXNavTransientSurface() {
  return shell && IsXNav() && shell->HasTransientSurface();
}

void AfterFrameRecapture() {
#ifdef __WXGTK__
  if (shell && IsXNav()) shell->RestackChartControls();
#endif
}

bool LoadPersistentPerspective(wxAuiManager &manager, const wxString &perspective) {
  if (shell && IsXNav() && shell->OwnsManager(manager)) {
    OpenNavDashboardLayoutScope dashboard_layout;
    return shell->LoadPersistentPerspective(perspective);
  }
  return manager.LoadPerspective(perspective, false);
}

void AfterAnchorWatch(){
  if (recovery && IsXNav()) {
    const auto now = vessel::Clock::now();
    integration::ObserveUpdateStartupHealth(g_bDeferredInitDone && shell && host, now);
    recovery->ObserveHealthy(g_bDeferredInitDone, now);
  }
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
bool ShowChartInformation(const wxString &html, double latitude, double longitude) {
  if (!IsXNav() || !shell || !host || restart) return false;
  if (std::isfinite(longitude)) longitude = std::remainder(longitude, 360.0);
  auto info = application::ParseChartInfo(html.ToStdString(wxConvUTF8), latitude, longitude);
  const wxWeakRef<ui::Shell> target(shell.get());
  const wxWeakRef<wxWindow> owner(host);
  host->CallAfter([info = std::move(info), target, owner]() mutable {
    if (!owner || !host || owner.get() != host || !target || shell.get() != target.get() ||
        !IsXNav() || restart) return;
    shell->ShowChartInformation(std::move(info));
  });
  return true;
}
bool ShowRouteContext(const std::string &id, bool hover) {
  if (!IsXNav() || !shell || !host || restart || id.empty()) return false;
  const wxWeakRef<ui::Shell> target(shell.get());
  const wxWeakRef<wxWindow> owner(host);
  host->CallAfter([id, hover, target, owner] {
    if (!owner || !host || owner.get() != host || !target || shell.get() != target.get() ||
        !IsXNav() || restart) return;
    shell->ShowRouteContext(id, hover);
  });
  return true;
}
bool IsAisSelected(int mmsi) { return IsXNav() && shell && mmsi > 0 && shell->SelectedAis() == mmsi; }
void DrawOnlineAis(ocpnDC &dc, ViewPort &vp, ChartCanvas *canvas) {
  if (IsXNav() && shell && canvas && !restart) {
    // Forecast wind sits under AIS targets (SCRUM-328); XNav presentation only.
    integration::DrawWeatherOverlay(dc, vp, *canvas);
    online_chart.Draw(dc, vp, *canvas);
  }
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
  menu.Append(id, IsXNav() ? "Open Legacy OpenCPN" : application::brand::SwitchToModern);
  menu.Bind(wxEVT_MENU, [](wxCommandEvent&) {
    RequestMode(IsXNav() ? InterfaceMode::Legacy : InterfaceMode::XNav);
  }, id);
}

bool PrepareClose(wxFileConfig& config) {
  if (pilots) pilots->live.Enable(false, vessel::Clock::now());
  wxLogMessage("SKAGER close preparation: preserving shared configuration");
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
                   "SKAGER restart", wxOK | wxICON_ERROR, host);
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
  integration::SetWeatherSource({});
  online_weather.reset(); // Cancels any request and joins the worker.
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
    wxLogError("SKAGER restart could not launch. Reopen OpenCPN to use the saved interface mode.");
  }
}

}  // namespace opennav

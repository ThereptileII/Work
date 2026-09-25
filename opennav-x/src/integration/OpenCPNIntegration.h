#pragma once
#include "vessel/RouteProgress.h"

class wxCmdLineParser;
class wxFileConfig;
class wxMenu;
class wxAuiManager;
class MyFrame;

namespace opennav {
namespace integration { struct ObservedRoutePass; }
using RouteObservation = std::shared_ptr<const integration::ObservedRoutePass>;
RouteObservation BeforeRouteProgress();
void AfterRouteProgress(const RouteObservation& before);
void AfterAnchorWatch();
bool ShowAisCard(int mmsi);
bool ShowNavigationObjectCard(const std::string& id,bool route);
// Application-thread acquisition; returned immutable values may be retained.
vessel::RouteProgress CurrentRouteProgress();
void AddCommandLine(wxCmdLineParser& parser);
bool ParseCommandLine(wxCmdLineParser& parser);
bool SafeRequested();
// Called after the upstream single-instance check, before its recovery dialog
// and before plugins/OpenGL initialization.
bool CheckStartupRecovery();
bool IsPortablePreview();
bool IsXNav();
void SelectMode(wxFileConfig& config, bool upstream_safe);
// After locale initialization, immediately before upstream resource defaults.
void InitializeResourceDefaults(wxFileConfig& config);
void Attach(MyFrame& frame, wxAuiManager& manager, wxFileConfig& config);
// Called at the end of normal deferred startup, after canvas/focus work.
void AfterDeferredInitialization();
void AppendModeMenu(wxMenu& menu);
bool PrepareClose(wxFileConfig& config);
void CompleteRestart();
bool HideLegacyToolbar(const void* toolbar);
}  // namespace opennav

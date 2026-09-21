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
// Application-thread acquisition; returned immutable values may be retained.
vessel::RouteProgress CurrentRouteProgress();
void AddCommandLine(wxCmdLineParser& parser);
bool ParseCommandLine(wxCmdLineParser& parser);
bool SafeRequested();
bool IsPortablePreview();
bool IsXNav();
void SelectMode(wxFileConfig& config, bool upstream_safe);
void Attach(MyFrame& frame, wxAuiManager& manager, wxFileConfig& config);
void AppendModeMenu(wxMenu& menu);
bool PrepareClose(wxFileConfig& config);
void CompleteRestart();
bool HideLegacyToolbar(const void* toolbar);
}  // namespace opennav

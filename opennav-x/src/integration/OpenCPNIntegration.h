#pragma once
#include "vessel/RouteProgress.h"

class wxCmdLineParser;
class wxFileConfig;
class wxMenu;
class wxAuiManager;
class MyFrame;
class wxWindow;
class wxString;
class ocpnDC;
class ViewPort;
class ChartCanvas;

namespace opennav {
namespace integration { struct ObservedRoutePass; }
using RouteObservation = std::shared_ptr<const integration::ObservedRoutePass>;
RouteObservation BeforeRouteProgress();
void AfterRouteProgress(const RouteObservation& before);
void AfterAnchorWatch();
bool ShowAisCard(int mmsi);
bool IsAisSelected(int mmsi);
void DrawOnlineAis(ocpnDC &dc, ViewPort &vp, ChartCanvas *canvas);
bool ShowOnlineAisAt(ChartCanvas &canvas, int x, int y);
bool ShowNavigationObjectCard(const std::string& id,bool route);
bool ShowChartContext(double latitude, double longitude);
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
// Reconcile XNav visibility after upstream Options rebuilds canvas panes.
void AfterSettingsReconfigured();
// Only actual XNav-owned temporary panes are excluded from saved workspaces.
bool IsTransientXNavPane(const wxWindow *window);
// Preserve an open XNav sheet/card during upstream's delayed resize raise.
bool HasXNavTransientSurface();
bool LoadPersistentPerspective(wxAuiManager &manager, const wxString &perspective);
void AppendModeMenu(wxMenu& menu);
bool PrepareClose(wxFileConfig& config);
void CompleteRestart();
bool HideLegacyToolbar(const void* toolbar);
}  // namespace opennav

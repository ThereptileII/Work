#pragma once

class wxCmdLineParser;
class wxFileConfig;
class wxMenu;
class wxAuiManager;
class MyFrame;

namespace opennav {
void AddCommandLine(wxCmdLineParser& parser);
bool ParseCommandLine(wxCmdLineParser& parser);
bool SafeRequested();
bool IsXNav();
void SelectMode(wxFileConfig& config, bool upstream_safe);
void Attach(MyFrame& frame, wxAuiManager& manager, wxFileConfig& config);
void AppendModeMenu(wxMenu& menu);
bool PrepareClose(wxFileConfig& config);
void CompleteRestart();
bool HideLegacyToolbar(const void* toolbar);
}  // namespace opennav

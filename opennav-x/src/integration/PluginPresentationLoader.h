#pragma once
#include <functional>
#include <memory>
class wxString;
class wxDynamicLibrary;
namespace opennav::integration {
class OriginalPluginTlsGuard;
// The plugin container owns inspection custody until checked module unload.
// A load helper's stack lifetime cannot cover forwarded factory resolution or
// a later declared delay-load from plugin code.
using PluginModuleInspectionCustody = std::unique_ptr<OriginalPluginTlsGuard>;
using PluginCompatibilityCheck = std::function<bool(const wxString &)>;
using PluginPresentationLoad = bool (*)(wxDynamicLibrary &, const wxString &,
                                      const PluginCompatibilityCheck &);
// Register on the application thread before normal plugin loading. Model-only
// tools/tests retain a null hook and therefore the exact upstream load path.
void RegisterPluginPresentationLoader(PluginPresentationLoad loader);
bool TryPluginPresentationLoader(wxDynamicLibrary &library,
    const wxString &original, const PluginCompatibilityCheck &compatible);
}

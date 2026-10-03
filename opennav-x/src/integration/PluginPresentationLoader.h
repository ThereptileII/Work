#pragma once
#include <functional>
class wxString;
class wxDynamicLibrary;
namespace opennav::integration {
using PluginCompatibilityCheck = std::function<bool(const wxString &)>;
using PluginPresentationLoad = bool (*)(wxDynamicLibrary &, const wxString &,
                                      const PluginCompatibilityCheck &);
// Register on the application thread before normal plugin loading. Model-only
// tools/tests retain a null hook and therefore the exact upstream load path.
void RegisterPluginPresentationLoader(PluginPresentationLoad loader);
bool TryPluginPresentationLoader(wxDynamicLibrary &library,
    const wxString &original, const PluginCompatibilityCheck &compatible);
}

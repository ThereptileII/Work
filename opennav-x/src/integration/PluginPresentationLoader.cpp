#include "integration/PluginPresentationLoader.h"
namespace opennav::integration {
namespace { PluginPresentationLoad load = nullptr; }
void RegisterPluginPresentationLoader(PluginPresentationLoad loader) { load = loader; }
bool TryPluginPresentationLoader(wxDynamicLibrary &library,
    const wxString &original, const PluginCompatibilityCheck &compatible) {
  return load && load(library, original, compatible);
}
}

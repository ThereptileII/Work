#pragma once
#include "integration/PluginPresentationLoader.h"
#include <wx/dynlib.h>
#ifdef _WIN32
#include <wx/msw/wrapwin.h>
#endif

namespace opennav::integration {
enum class PluginModuleLoadOutcome { Loaded, LoadFailed, UnloadFailed };

inline bool UnloadPluginModuleChecked(wxDynamicLibrary &library) {
  if (!library.IsLoaded()) return true;
#ifdef _WIN32
  // wx Unload returns void and clears its handle even when FreeLibrary fails.
  // Retain ownership on failure so a refused adapter cannot be overwritten.
  const auto handle = library.Detach();
  if (!FreeLibrary(handle)) {
    library.Attach(handle);
    return false;
  }
#else
  // Private presentation modules are Windows-only; retain upstream elsewhere.
  library.Unload();
#endif
  return true;
}

// Keep the original discovery/profile/helper identity. A refused private
// adapter may fall back only after it has completely left the destination.
// LoadFailed deliberately returns to OpenCPN's ordinary error/blacklist path.
inline PluginModuleLoadOutcome LoadPluginWithPresentationFallback(
    wxDynamicLibrary &library, const wxString &original,
    const PluginCompatibilityCheck &compatible) {
  if (!UnloadPluginModuleChecked(library))
    return PluginModuleLoadOutcome::UnloadFailed;
  if (TryPluginPresentationLoader(library, original, compatible))
    return library.IsLoaded() ? PluginModuleLoadOutcome::Loaded
                              : PluginModuleLoadOutcome::LoadFailed;
  if (library.IsLoaded()) return PluginModuleLoadOutcome::UnloadFailed;
  return library.Load(original) ? PluginModuleLoadOutcome::Loaded
                                : PluginModuleLoadOutcome::LoadFailed;
}
}

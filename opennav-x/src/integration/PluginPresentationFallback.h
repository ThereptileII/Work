#pragma once
#include "integration/PluginPresentationLoader.h"
#include <wx/dynlib.h>
#ifdef _WIN32
#include <wx/msw/wrapwin.h>
#include <wx/log.h>
#include "integration/OriginalPluginTlsGuard.h"
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
#ifdef _WIN32
  // This central original-load branch is shared by discovery, reload and a
  // clean adapter refusal. External/configured plugin roots need no duplicate
  // installer path resolver. The private adapter has its own exact-byte gate.
  OriginalPluginTlsGuard tls_guard(original);
  if (!tls_guard.Refusal().empty()) {
    wxLogWarning("SKAGER: refusing plugin %s: %s", original,
                 wxString::FromUTF8(tls_guard.Refusal().c_str()));
    return PluginModuleLoadOutcome::LoadFailed;
  }
#endif
  return library.Load(original) ? PluginModuleLoadOutcome::Loaded
                                : PluginModuleLoadOutcome::LoadFailed;
}
}

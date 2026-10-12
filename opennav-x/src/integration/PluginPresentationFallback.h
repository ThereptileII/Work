#pragma once
#include "integration/PluginPresentationLoader.h"
#include <wx/dynlib.h>
#ifdef _WIN32
#include <wx/msw/wrapwin.h>
#include <wx/log.h>
#include "integration/OriginalPluginTlsGuard.h"
#endif

namespace opennav::integration {
#ifndef _WIN32
// Model-only/non-Windows builds keep upstream loading. This complete empty type
// allows the container's out-of-line destructor to own the same nullable slot.
class OriginalPluginTlsGuard {};
#endif
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

inline bool UnloadPluginModuleChecked(
    wxDynamicLibrary &library, PluginModuleInspectionCustody &custody) {
  if (!UnloadPluginModuleChecked(library)) return false;
  // File protection remains during FreeLibrary, including detach callbacks.
  custody.reset();
  return true;
}

// Keep the original discovery/profile/helper identity. A refused private
// adapter may fall back only after it has completely left the destination.
// LoadFailed deliberately returns to OpenCPN's ordinary error/blacklist path.
inline PluginModuleLoadOutcome LoadPluginWithPresentationFallback(
    wxDynamicLibrary &library, const wxString &original,
    const PluginCompatibilityCheck &compatible,
    PluginModuleInspectionCustody &custody) {
  if (!UnloadPluginModuleChecked(library, custody))
    return PluginModuleLoadOutcome::UnloadFailed;
  if (TryPluginPresentationLoader(library, original, compatible))
    return library.IsLoaded() ? PluginModuleLoadOutcome::Loaded
                              : PluginModuleLoadOutcome::LoadFailed;
  if (library.IsLoaded()) return PluginModuleLoadOutcome::UnloadFailed;
#ifdef _WIN32
  // This central original-load branch is shared by discovery, reload and a
  // clean adapter refusal. External/configured plugin roots need no duplicate
  // installer path resolver. The private adapter has its own exact-byte gate.
  auto tls_guard = std::make_unique<OriginalPluginTlsGuard>(original);
  if (!tls_guard->Refusal().empty()) {
    wxLogWarning("SKAGER: refusing plugin %s: %s", original,
                 wxString::FromUTF8(tls_guard->Refusal().c_str()));
    return PluginModuleLoadOutcome::LoadFailed;
  }
#endif
  if (!library.Load(original)) return PluginModuleLoadOutcome::LoadFailed;
#ifdef _WIN32
  custody = std::move(tls_guard);
#endif
  return PluginModuleLoadOutcome::Loaded;
}
}

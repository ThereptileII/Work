#pragma once
#include "integration/PluginPresentationLoader.h"
#include "plugin-adapters/ChartPresentationBindingV1.h"
#include <wx/dynlib.h>
#include <wx/string.h>
#include <cstdint>
#include <string>
namespace opennav::integration {
inline constexpr char OriginalOChartsSha256[] =
    "99edcfd4419d606ef5c3fd554759e853cfda1fe4f38c05e26266f335a5a5b875";
// Constructed from actual startup policy and compiled package identity. No
// runtime user configuration supplies a trusted executable digest.
struct OChartsModuleRequest {
  wxString original, adapter, resources;
  bool package_available = false, main_thread = false, safe_mode = true;
  std::string adapter_sha256;
  std::uint64_t adapter_bytes = 0;
};
struct OChartsModuleResult {
  bool loaded = false, applicable = false;
  std::string reason;
};
// Actual Win32 locks/hash/load/bind; no injectable file or module operations.
// Refusal never executes original code. Caller must check residual module state
// before attempting original fallback if an adapter failed to unload.
OChartsModuleResult LoadOChartsModule(wxDynamicLibrary &, const OChartsModuleRequest &,
                                     const PluginCompatibilityCheck &);
bool ValidOChartsStatus(const SkagerChartPresentationStatusV1 &);
}

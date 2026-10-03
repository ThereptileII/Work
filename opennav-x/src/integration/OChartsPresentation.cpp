#include "integration/OChartsPresentation.h"
#include "integration/OChartsModuleLoader.h"
#include "integration/ChartPresentation.h"
#include "SkagerOChartsPackage.h"
#include "model/plugin_loader.h"
#include "model/safe_mode.h"
#include <wx/filename.h>
#include <wx/log.h>
#include <wx/stdpaths.h>
#include <wx/thread.h>
namespace opennav::integration {
namespace {
std::string last_refusal;
wxString loaded_original;
void *loaded_module = nullptr;
}
bool LoadQualifiedOChartsPresentation(wxDynamicLibrary &library,
    const wxString &original, const PluginCompatibilityCheck &compatible) {
#ifdef _WIN32
  OChartsModuleRequest request;
  request.main_thread = wxIsMainThread();
  request.safe_mode = safe_mode::get_mode();
  if (!request.main_thread || request.safe_mode ||
      wxFileName(original).GetFullName().CmpNoCase("o-charts_pi.dll") != 0)
    return false;
  request.original = original;
  request.resources = VerifiedPluginChartPresentationDirectory();
  request.package_available = skager_ocharts::available;
  request.adapter_sha256 = skager_ocharts::sha256;
  request.adapter_bytes = skager_ocharts::bytes;
  request.adapter = wxFileName(
      wxFileName(wxStandardPaths::Get().GetExecutablePath()).GetPath(),
      "skager-ocharts-adapter.dll").GetFullPath();
  const auto result = LoadOChartsModule(library, request, compatible);
  if (!result.applicable) return false;
  if (!result.loaded) {
    last_refusal = result.reason;
    loaded_original.clear(); loaded_module = nullptr;
    wxLogWarning("SKAGER o-charts presentation unavailable: %s",
                 wxString::FromUTF8(result.reason));
    return false;
  }
  loaded_original = original;
  loaded_module = library.GetLibHandle();
  last_refusal.clear();
  wxLogMessage("SKAGER o-charts presentation: original SHA256 %s; loaded %s SHA256 %s; awaiting renderer initialization",
               wxString::FromUTF8(OriginalOChartsSha256), request.adapter,
               wxString::FromUTF8(skager_ocharts::sha256));
  return true;
#else
  (void)library; (void)original; (void)compatible;
  return false;
#endif
}

std::string OChartsPresentationStatus() {
  if (!wxIsMainThread()) return "o-charts presentation: unavailable off application thread";
  if (loaded_module) {
    const auto *plugins = PluginLoader::GetInstance()->GetPlugInArray();
    for (unsigned i = 0; i < plugins->GetCount(); ++i) {
      auto *plugin = plugins->Item(i);
      if (plugin->m_plugin_file != loaded_original || !plugin->m_library.IsLoaded() ||
          plugin->m_library.GetLibHandle() != loaded_module) continue;
      auto query = reinterpret_cast<SkagerGetChartPresentationStatusV1>(
          plugin->m_library.GetSymbol(SKAGER_CHART_STATUS_EXPORT));
      SkagerChartPresentationStatusV1 state{};
      state.structBytes = sizeof(state);
      state.version = SKAGER_CHART_BINDING_VERSION;
      if (!query || query(&state) != 1 || !ValidOChartsStatus(state))
        return "o-charts presentation: invalid adapter status";
      switch (state.state) {
        case SKAGER_CHART_SELECTED: return "o-charts: verified SKAGER presentation";
        case SKAGER_CHART_BOUND_PENDING_INITIALIZATION: return "o-charts: awaiting chart renderer";
        case SKAGER_CHART_STANDARD_FALLBACK: return "o-charts: Standard fallback (private renderer)";
        default: return "o-charts: presentation unavailable";
      }
    }
    return "o-charts: private adapter not loaded";
  }
  return last_refusal.empty() ? "o-charts: no SKAGER adapter loaded"
                              : "o-charts: SKAGER presentation unavailable / " + last_refusal;
}
}

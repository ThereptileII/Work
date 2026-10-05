#include "plugin-adapters/ocharts/ChartPresentationAdapter.h"
#include "plugin-adapters/ocharts/BindingState.h"
#include "plugin-adapters/ocharts/PointStyleObservation.h"
#include "integration/ChartNameTypography.h"
#include "integration/ChartTextFace.h"
#include "integration/ChartLightLabel.h"
#include "integration/ChartSoundingFont.h"
#include "plugin-adapters/ocharts/ResourceVerification.h"
#include "s52plib.h"
#include "ocpn_plugin.h"
#include <wx/filename.h>
#include <wx/fontenum.h>
#include <wx/log.h>
#include <wx/thread.h>
#include <cstdint>
#include <memory>

extern s52plib* ps52plib;

namespace skager::ocharts {
namespace {
BindingState binding;
PointStyleObservation point_style;
using namespace opennav::integration;
wxFont *GeographicNameFont(const char *feature, const char *instruction, bool tx,
                           double *tracking, unsigned char *opacity, bool *light) {
  *light = false;
  if (IsGeneratedLightDescription(feature, instruction, tx)) {
    // Pinned FontMgr factory face/size are cached from wxNORMAL_FONT.
    static const wxFont system = *wxNORMAL_FONT;
    // Match the private renderer's original API-17 template request. The host
    // legacy API returns its first matching preference; do not create a new
    // differently-sized preference merely to ask whether it is factory-like.
#ifdef __WXOSX__
    const int default_size = system.GetPointSize() + 1;
#else
    const int default_size = system.GetPointSize() + 2;
#endif
    const wxFont* configured = GetOCPNScaledFont_PlugIn(_("ChartTexts"), default_size);
    *light = configured && FactoryLightTextFont(*configured, system,
                                                GetFontColour_PlugIn(_("ChartTexts")));
  }
  const auto role = GeographicChartName(feature, instruction, tx);
  if (role == ChartNameRole::Unchanged && !*light) return nullptr;
  *tracking = *light ? .12 : role == ChartNameRole::Land ? 1.0 : 5.0;
  *opacity = *light || role == ChartNameRole::Land ? 255 : 92; // round(.36 * 255)
  // The selected chart style owns geographic-name typography. Never rewrite
  // FontMgr's persisted ChartTexts preference; Standard retains it verbatim.
  static const wxString face = [] {
    for (const auto *candidate : {"Segoe UI Variable Display", "Segoe UI", "Arial"})
      if (wxFontEnumerator::IsValidFacename(candidate)) return wxString(candidate);
    return wxString("Arial");
  }();
  static const wxString light_face = wxFontEnumerator::IsValidFacename("Segoe UI")
      ? wxString("Segoe UI") : wxString("Arial");
  // Light descriptions: 8 CSS px = 6 points; geographic 12/16 px = 9/12 points.
  // The upstream renderer retains its
  // DIP/content scale and the user's chart-text scale; do not scale twice.
  return FindOrCreateFont_PlugIn(
      *light ? 6 : role == ChartNameRole::Land ? 9 : 12, wxFONTFAMILY_SWISS,
      *light || role == ChartNameRole::Land ? wxFONTSTYLE_NORMAL : wxFONTSTYLE_ITALIC,
      wxFONTWEIGHT_NORMAL, false, *light || role == ChartNameRole::Land ? light_face : face);
}


} // namespace
void SetChartPresentationActive(bool active) {
  point_style.SetActive(active && wxIsMainThread());
}
s52plib* CreateChartPresentation(const wxString& stockDirectory) {
  const auto path=binding.BeginInitialization();
  if (path[0] && wxIsMainThread()) {
    const wxString directory=wxString::FromUTF8(path.data());
    if (VerifyCompiledResources(directory)) {
      auto library=std::make_unique<s52plib>(
          wxFileName(directory,"S52RAZDS.RLE").GetFullPath(),false,false,true);
      if (library->m_bOK) {
        if (VerifyCompiledResources(directory)) {
          library->EnablePresentationSimplifiedSymbols();
          library->EnablePresentationLightSymbols();
          library->SetTextFontResolver(GeographicNameFont);
          library->SetPresentationTextFace(PrototypeChartTextFace());
          library->SetSoundingFontResolver(ChartSoundingFont);
          binding.Complete(true,SKAGER_CHART_REASON_NONE);
          wxLogMessage("SKAGER o-charts: verified private presentation selected");
          return library.release();
        }
        binding.Complete(false,SKAGER_CHART_REASON_RESOURCE_VERIFICATION);
      } else binding.Complete(false,SKAGER_CHART_REASON_RENDERER_INITIALIZATION);
    } else binding.Complete(false,SKAGER_CHART_REASON_RESOURCE_VERIFICATION);
    wxLogWarning("SKAGER o-charts: private presentation refused; stock fallback");
  } else if(path[0]) binding.Complete(false,SKAGER_CHART_REASON_RENDERER_INITIALIZATION);
  // Unbound is the original stock constructor behavior, including upstream CWD
  // policy. Bound failure refuses CWD so a failed owned style stays atomic.
  return new s52plib(stockDirectory,false,path[0]==0,false);
}
} // namespace skager::ocharts
#ifdef _WIN32
#define SKAGER_ADAPTER_EXPORT extern "C" __declspec(dllexport)
#else
#define SKAGER_ADAPTER_EXPORT extern "C" __attribute__((visibility("default")))
#endif
SKAGER_ADAPTER_EXPORT int32_t SKAGER_CHART_CALL skager_bind_chart_presentation_v1(
    const SkagerChartBindingV1* value) {
  try {return skager::ocharts::binding.Bind(value) ? 1 : 0;} catch (...) {return 0;}
}
SKAGER_ADAPTER_EXPORT int32_t SKAGER_CHART_CALL skager_chart_presentation_status_v1(
    SkagerChartPresentationStatusV1* status) {
  try {return skager::ocharts::binding.ReadStatus(status) ? 1 : 0;} catch (...) {return 0;}
}

SKAGER_ADAPTER_EXPORT int32_t SKAGER_CHART_CALL skager_chart_point_style_v1(
    SkagerChartPointStyleV1* observation) {
  try {
    // ps52plib is created/deleted by Init on this same thread. DeInit and the
    // plugin destructor revoke observation before their existing work; the only
    // failed-Init delete immediately nulls ps52plib. Never dereference off-thread.
    if (!wxIsMainThread()) return 0;
    SkagerChartPresentationStatusV1 status{};
    status.structBytes = sizeof(status);
    status.version = SKAGER_CHART_BINDING_VERSION;
    return skager::ocharts::binding.ReadStatus(&status) &&
        skager::ocharts::point_style.Read(observation, true,
            status.state == SKAGER_CHART_SELECTED, ps52plib) ? 1 : 0;
  } catch (...) { return 0; }
}

#include "integration/RuntimeDiagnostics.h"
#include "chartdb.h"
#include "chcanv.h"
#include "model/plugin_loader.h"
#include "viewport.h"
#include <wx/filename.h>
#include <wx/thread.h>
extern ChartDB *ChartData;
extern bool g_bopengl;
namespace opennav::integration {
wxJSONValue ReadRuntimeDiagnostics(ChartCanvas &canvas) {
  wxASSERT(wxIsMainThread());
  wxJSONValue r;
  const auto &vp = canvas.GetVP();
  r["chart"]["latitude"] = vp.clat;
  r["chart"]["longitude"] = vp.clon;
  r["chart"]["scale_ppm"] = vp.view_scale_ppm;
  r["chart"]["follow"] = canvas.GetbFollow();
  r["chart"]["quilt"] = canvas.GetQuiltMode();
  r["chart"]["opengl_enabled"] = g_bopengl;
  r["chart"]["canvas_pixels"]["width"] = canvas.GetClientSize().x;
  r["chart"]["canvas_pixels"]["height"] = canvas.GetClientSize().y;
  if (ChartData && ChartData->IsValid() && !ChartData->IsBusy()) {
    r["chart"]["database_entries"] = ChartData->GetChartTableEntries();
    r["chart"]["quilt_reference"] = canvas.GetQuiltReferenceChartIndex();
    r["chart"]["quilt_members"] = wxJSONValue(wxJSONTYPE_ARRAY);
    for (int i : canvas.GetQuiltIndexArray()) {
      if (i < 0 || i >= ChartData->GetChartTableEntries())
        continue;
      const auto &entry = ChartData->GetChartTableEntry(i);
      wxJSONValue c;
      c["index"] = i;
      c["file"] = wxFileName(entry.GetFullSystemPath()).GetFullName();
      c["type"] = entry.GetChartType();
      c["native_scale"] = entry.GetScale();
      r["chart"]["quilt_members"].Append(c);
    }
  }
  // The loader was initialized by OpenCPN before XNav attachment. Merely copy
  // its records: do not call plugin methods, refresh metadata or load a DLL.
  const auto *plugins = PluginLoader::GetInstance()->GetPlugInArray();
  r["plugins"] = wxJSONValue(wxJSONTYPE_ARRAY);
  for (const auto *p : *plugins) {
    if (!p)
      continue;
    wxJSONValue item;
    item["name"] = p->m_common_name;
    item["file"] = p->m_plugin_filename;
    item["enabled"] = p->m_enabled;
    item["initialized"] = p->m_init_state;
    item["api"] = p->m_api_version;
    item["version"] =
        wxString::Format("%d.%d", p->m_version_major, p->m_version_minor);
    r["plugins"].Append(item);
  }
  return r;
}
} // namespace opennav::integration

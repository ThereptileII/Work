#pragma once
#include <wx/string.h>
class wxJSONValue;
namespace opennav::integration {
// Explicit early maintenance check, before profile/plugin/transport startup.
// Only the compiled private module may execute. No factory, Init or fallback.
bool CheckChartModule(const wxString& original, const wxString& install,
                      wxJSONValue& report);
}

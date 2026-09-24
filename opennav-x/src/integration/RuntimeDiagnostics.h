#pragma once
#include <wx/jsonval.h>
class ChartCanvas;
namespace opennav::integration {
// GUI-thread copy. JSON owns all strings and values; no chart/plugin pointers
// escape.
wxJSONValue ReadRuntimeDiagnostics(ChartCanvas &canvas);
} // namespace opennav::integration

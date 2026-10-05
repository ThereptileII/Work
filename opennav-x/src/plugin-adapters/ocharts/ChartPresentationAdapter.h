#pragma once
#include <wx/string.h>
class s52plib;
namespace skager::ocharts {
// Called only at the plugin's existing private-library construction boundary.
// Init/DeInit/destruction gate observations without changing renderer ownership.
void SetChartPresentationActive(bool active);
s52plib* CreateChartPresentation(const wxString& stockDirectory);
}

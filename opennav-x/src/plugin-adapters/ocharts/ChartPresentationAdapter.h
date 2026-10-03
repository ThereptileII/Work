#pragma once
#include <wx/string.h>
class s52plib;
namespace skager::ocharts {
// Called only at the plugin's existing private-library construction boundary.
s52plib* CreateChartPresentation(const wxString& stockDirectory);
}

#pragma once
#include "application/NavigationObjects.h"
#include <wx/colour.h>
#include <wx/hashmap.h>
#include <wx/dynarray.h>
#include <wx/string.h>
#include "color_types.h"
class s52plib;
class wxFileConfig;
namespace opennav::integration {
// Application-thread only. The library is selected once before charts retain
// lookup pointers; changing a preference requires the ordinary controlled
// restart.
void ConfigureChartPresentation(wxFileConfig &config, bool xnav);
s52plib *CreateChartPresentation(const wxString &stock_path, bool force_legacy);
bool ChartBackground(ColorScheme scheme, wxColour &land, wxColour &water);
bool XNavChartRequested();
std::string ChartPresentationStatus();
application::CommandResult SetXNavChartRequested(bool enabled);
} // namespace opennav::integration

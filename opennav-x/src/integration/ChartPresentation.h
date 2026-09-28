#pragma once
#include "application/NavigationObjects.h"
#include <wx/colour.h>
#include <wx/hashmap.h>
#include <wx/dynarray.h>
#include <wx/string.h>
#include "color_types.h"
class s52plib;
class wxFileConfig;
class ChartCanvas;
class ocpnDC;
class wxRect;
namespace opennav::integration {
// Application-thread only. The library is selected once before charts retain
// lookup pointers; changing a preference requires the ordinary controlled
// restart.
void ConfigureChartPresentation(wxFileConfig &config, bool xnav);
s52plib *CreateChartPresentation(const wxString &stock_path, bool force_legacy);
bool ChartBackground(ColorScheme scheme, wxColour &land, wxColour &water);
// Returns true only after drawing the upstream-resolved chart depth unit.
// False preserves the stock emboss path, including Standard/Legacy/Safe.
bool DrawChartDepthUnit(ocpnDC &dc, ChartCanvas &canvas);
bool ChartScaleGeometry(ChartCanvas &canvas, int &x, int &y, int &reference_width);
bool DrawChartScale(ocpnDC &dc, ChartCanvas &canvas, const wxString &label,
                    int x, int y, int length, wxRect &bounds);
bool XNavChartRequested();
std::string ChartPresentationStatus();
application::CommandResult SetXNavChartRequested(bool enabled);
} // namespace opennav::integration

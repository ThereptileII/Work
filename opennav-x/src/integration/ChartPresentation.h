#pragma once
#include <string>
#include <functional>
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
class Route;
namespace opennav::integration {
// Application-thread only. The library is selected once before charts retain
// lookup pointers; changing a preference requires the ordinary controlled
// restart.
void ConfigureChartPresentation(wxFileConfig &config, bool xnav);
// For opt-in verified plugin adapters only. No shared-data API redirection.
wxString VerifiedPluginChartPresentationDirectory();
s52plib *CreateChartPresentation(const wxString &stock_path, bool force_legacy);
bool ChartBackground(ColorScheme scheme, wxColour &land, wxColour &water);
// Existing chart-selector vector keys only. Chart families, availability,
// eclipsing and interaction remain owned by OpenCPN's Piano.
bool ChartVectorSelectorInk(ColorScheme scheme, wxColour &selected,
                            wxColour &unselected);
// Paint-time only; caller retains upstream active/selected route semantics.
// Does not modify pens in RouteManager, route properties or navigation state.
bool ChartActiveRouteInk(ChartCanvas &canvas, wxColour &ink);
bool ChartRouteInk(ChartCanvas &canvas, Route &route, wxColour &ink);
// Untouched default route presentation; explicit custom/emergency states fall
// through to upstream. No stored route/global preference is changed.
bool DefaultChartRouteStyle(Route &route);
bool DrawChartRouteSegment(ocpnDC &dc, ChartCanvas &canvas, double ax, double ay,
                            double bx, double by, bool join_start, bool join_end);
// True when verified XNav presentation paints the predictor lines. OpenCPN
// pen preferences continue to apply in Standard/Legacy/Safe.
bool UseChartCogPredictorStyle(int width, int style, const wxString &color,
                               int density_width);
bool DrawChartCogPredictor(ocpnDC &dc, ChartCanvas &canvas,
                           double ax, double ay, double bx, double by);
// XNav heading line from the ship to a caller-projected end point. The
// caller derives its length from the same speed/time horizon as COG.
bool DrawChartHeadingPredictor(ocpnDC &dc, ChartCanvas &canvas,
                               double ax, double ay, double bx, double by);
// Default fixed or dimension-scaled bitmap ownship artwork. Explicit custom
// user icons retain upstream rendering. Projection, heading/COG choice and
// vessel dimensions remain upstream; invalid direction gets no oriented glyph.
// Angle is the existing clockwise screen angle; scale/beam preserve upstream size.
bool DrawChartOwnship(ocpnDC &dc, ChartCanvas &canvas, double x, double y,
                      double angle, double scale, double stretch_x = 1,
                      bool direction_available = true);
// Returns true only after drawing the upstream-resolved chart depth unit.
// False preserves the stock emboss path, including Standard/Legacy/Safe.
bool DrawChartDepthUnit(ocpnDC &dc, ChartCanvas &canvas);
// Paint only after the original overzoom indicator returns its current map.
// False preserves that same stock map; this never decides warning visibility.
bool DrawChartOverzoomWarning(ocpnDC &dc, ChartCanvas &canvas, int x, int y);
bool ChartScaleGeometry(ChartCanvas &canvas, int &x, int &y, int &reference_width);
bool DrawChartScale(ocpnDC &dc, ChartCanvas &canvas, const wxString &label,
                    int x, int y, int length, wxRect &bounds);
bool XNavChartRequested();
bool XNavChartPresentationActive();
// SCRUM-364: SKAGER mode draws no OpenCPN chart-stack (piano) bar. Render
// gate only; g_bShowChartBar and the saved setting stay as the user set them,
// so Legacy mode keeps its bar.
bool XNavChartBarHidden();
// SCRUM-364: the vessel name for the own-ship chip ("● Reptil · 6.3 kn").
void SetChartVesselNameProvider(std::function<std::string()> provider);
std::string ChartPresentationStatus();
application::CommandResult SetXNavChartRequested(bool enabled);
} // namespace opennav::integration

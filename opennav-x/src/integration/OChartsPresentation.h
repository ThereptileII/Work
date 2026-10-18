#pragma once
#include "integration/OChartsPointStyle.h"
#include <functional>
#include <string>
#include <wx/dynlib.h>
#include <wx/string.h>

namespace opennav::integration {
// Called only at the normal OpenCPN load boundary. False leaves the caller to
// load its original module; original discovery/config/helper paths never move.
bool LoadQualifiedOChartsPresentation(
    wxDynamicLibrary &library, const wxString &original,
    const std::function<bool(const wxString &)> &compatible);
// Main-thread copied diagnostics. No plugin/chart object or function pointer is
// retained after unloading. A query never initializes a renderer.
std::string OChartsPresentationStatus();
OChartsPointStyle ReadOChartsPointStyle();
// Re-resolves the private export only while the exact verified module remains
// loaded. A disabled adapter stages copied scale data until its next Init;
// no callback or renderer pointer is retained.
bool ConfigureOChartsDisplay(bool very_light, int percent);
}

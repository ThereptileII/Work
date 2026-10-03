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
}

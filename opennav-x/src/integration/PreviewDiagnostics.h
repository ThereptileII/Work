#pragma once
#include "application/Settings.h"
#include "smartnav/Energy.h"
#include "vessel/VesselState.h"
#include <string>
#include <vector>
#include <wx/jsonval.h>
namespace opennav::integration {
std::vector<std::string> PreviewBuildInfo(int dpi, const std::string &profile);
// state and energy are one evaluation frame; energy.calculated_at is its clock.
// Runtime observations and source candidates are collected independently.
void WritePreviewDiagnostics(const std::string &path,
                             const vessel::VesselState &state,
                             const std::vector<std::string> &info,
                             const smartnav::EnergyPrediction &energy,
                             const application::Settings &settings,
                             const std::vector<vessel::SourceHealth> &sources,
                             const std::string &ui_page,
                             const wxJSONValue &runtime);
} // namespace opennav::integration

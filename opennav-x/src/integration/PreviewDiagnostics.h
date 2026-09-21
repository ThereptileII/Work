#pragma once
#include "vessel/VesselState.h"
#include <string>
#include <vector>
namespace opennav::integration {
std::vector<std::string> PreviewBuildInfo(int dpi, const std::string &profile);
void WritePreviewDiagnostics(const std::string &path,
                             const vessel::VesselState &state,
                             const std::vector<std::string> &info);
} // namespace opennav::integration

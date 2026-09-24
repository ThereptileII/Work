#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace opennav::integration {
// Called only for a validated portable preview, after OpenCPN loads its config
// and before it creates the chart canvas. Explicit custom locations stay owned
// by OpenCPN. The returned value names existing bundled data, never invented data.
std::optional<std::filesystem::path> PreviewBasemapDefault(
    const std::filesystem::path& package_root, const std::string& configured);
}

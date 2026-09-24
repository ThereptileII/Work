#include "integration/PreviewResources.h"

#include <algorithm>

namespace opennav::integration {
namespace {
bool HasBasemap(const std::filesystem::path& directory) {
  // ShapeBaseChartSet::LoadBasemaps in pinned OpenCPN 5.12.4.
  for (const auto* quality : {"crude_10x10", "low", "medium", "high", "full"}) {
    std::error_code error;
    if (std::filesystem::is_regular_file(
            directory / (std::string("basemap_") + quality + ".shp"), error))
      return true;
  }
  return false;
}
}

std::optional<std::filesystem::path> PreviewBasemapDefault(
    const std::filesystem::path& root, const std::string& configured) {
  if (!configured.empty()) {
    auto spelling = configured;
    std::replace(spelling.begin(), spelling.end(), '\\', '/');
    // Older previews persisted NormalizePath("") as ./ (Windows: .\).
    // Do not replace a real basemap deliberately placed in that directory.
    if ((spelling != "." && spelling != "./") || HasBasemap(root / "profile"))
      return std::nullopt;
  }
  const auto bundled = root / "app/basemap_shp";
  if (!HasBasemap(bundled)) return std::nullopt;
  // A nonempty path round-trips through OpenCPN's existing portable path writer.
  return bundled;
}
}

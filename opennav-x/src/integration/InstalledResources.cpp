#include "integration/InstalledResources.h"
#include "platform/PortableProfile.h"

#include <algorithm>
#include <fstream>
#include <string>

namespace opennav::integration {
namespace fs = std::filesystem;
namespace {
bool NonemptyFile(const fs::path& path) {
  std::error_code ec;
  if (!fs::is_regular_file(path, ec) || ec) return false;
  const auto size = fs::file_size(path, ec);
  return !ec && size > 0;
}
}
std::optional<ResourceSelection> InstalledResourceDefaults(
    const fs::path& application) {
  try {
    const auto marker = application / "OPENNAV_INSTALLED_STOCK";
    std::error_code ec;
    const auto size = fs::file_size(marker, ec);
    if (ec || size == 0 || size > 4096) return std::nullopt;
    std::ifstream input(marker, std::ios::binary);
    std::string value(static_cast<std::size_t>(size), '\0');
    if (!input.read(value.data(), value.size()) ||
        value.find_first_of("\r\n") != std::string::npos ||
        value.find('\0') != std::string::npos) return std::nullopt;
    const auto executable = platform::PathFromUtf8(value);
    auto filename = platform::PathUtf8(executable.filename());
    std::transform(filename.begin(), filename.end(), filename.begin(),
                   [](unsigned char c) { return c >= 'A' && c <= 'Z' ? c + ('a'-'A') : c; });
    if (!executable.is_absolute() || filename != "opencpn.exe" ||
        !NonemptyFile(executable)) return std::nullopt;
    const auto stock = executable.parent_path();
    // Names are the pinned upstream defaults in MyApp::OnInit. Do not infer
    // harmonic formats or substitute arbitrary nearby/custom files.
    for (const auto* relative : {"tcdata/harmonics-dwf-20210110-free.tcd",
         "tcdata/HARMONICS_NO_US.IDX", "tcdata/HARMONICS_NO_US",
         "gshhs/poly-c-1.dat", "basemap_shp/basemap_low.shp", "sounds/2bells.wav"})
      if (!NonemptyFile(stock / relative)) return std::nullopt;
    return ResourceSelection{{platform::PathUtf8(stock / "tcdata/harmonics-dwf-20210110-free.tcd"),
                              platform::PathUtf8(stock / "tcdata/HARMONICS_NO_US.IDX")},
                             platform::PathUtf8(stock / "gshhs"), platform::PathUtf8(stock / "basemap_shp"),
                             platform::PathUtf8(stock / "sounds/2bells.wav")};
  } catch (const fs::filesystem_error&) {
    return std::nullopt;
  }
}
void ApplyResourceDefaults(ResourceSelection& selected,
                           const ResourceSelection& defaults) {
  if (selected.tides.empty()) selected.tides = defaults.tides;
  if (selected.coastline.empty()) selected.coastline = defaults.coastline;
  if (selected.basemap.empty()) selected.basemap = defaults.basemap;
  if (selected.ais_alarm.empty()) selected.ais_alarm = defaults.ais_alarm;
}
}  // namespace opennav::integration

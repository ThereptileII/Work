#include "integration/PreviewResources.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace fs = std::filesystem;
using opennav::integration::PreviewBasemapDefault;
void Check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  const auto root = fs::temp_directory_path() /
      ("OpenNav basemap " + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    fs::create_directories(root / "app/basemap_shp");
    fs::create_directories(root / "profile");
    Check(!PreviewBasemapDefault(root, ""), "Absent bundled data stays absent");
    // File presence only: these policy tests never claim to parse chart data.
    std::ofstream(root / "app/basemap_shp/basemap_low.shp") << "test marker";
    const auto expected = root / "app/basemap_shp";
    for (const auto* saved : {"", ".", "./", ".\\"})
      Check(PreviewBasemapDefault(root, saved) == expected,
            "Fresh default or old empty-path artifact resolves to existing bundle");
    for (const auto* saved : {"../app/basemap_shp", "../custom charts", "C:\\Charts",
                              "/custom/chart data", "missing custom charts"})
      Check(!PreviewBasemapDefault(root, saved), "Custom selection stays unchanged");
    for (const auto* quality : {"crude_10x10", "low", "medium", "high", "full"}) {
      const auto path = root / "profile" / (std::string("basemap_") + quality + ".shp");
      std::ofstream(path) << "test marker";
      Check(!PreviewBasemapDefault(root, "./"), "Real profile-relative choice preserved");
      fs::remove(path);
    }
    fs::remove(root / "app/basemap_shp/basemap_low.shp");
    Check(!PreviewBasemapDefault(root, "./"), "No silent fallback to nonexistent data");
    fs::remove_all(root);
    std::cout << "Preview basemap default, migration, missing data and custom paths passed\n";
  } catch (const std::exception& error) {
    fs::remove_all(root);
    std::cerr << error.what() << '\n';
    return 1;
  }
}

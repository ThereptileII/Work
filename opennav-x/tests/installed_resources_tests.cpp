#include "integration/InstalledResources.h"
#include "platform/PortableProfile.h"
using opennav::platform::PathFromUtf8;
using opennav::platform::PathUtf8;

#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace fs = std::filesystem;
using namespace opennav::integration;
void Check(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
void Write(const fs::path& path, const std::string& text) {
  fs::create_directories(path.parent_path());
  std::ofstream(path, std::ios::binary) << text;
}
int main() {
  const auto root = fs::temp_directory_path() / ("OpenNav resources " +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    const auto stock = root / PathFromUtf8("Stock Åland & sea");
    const auto app = root / "generation one/app";
    const auto marker = app / "OPENNAV_INSTALLED_STOCK";
    Check(!InstalledResourceDefaults(app), "Uninstalled/portable executable has no locator");
    Write(marker, PathUtf8(stock / "opencpn.exe"));
    Check(!InstalledResourceDefaults(app), "Missing stock stays unavailable");
    // Presence-only fixtures; actual harmonic decoding remains an integrated gate.
    for (const auto* relative : {"opencpn.exe", "tcdata/harmonics-dwf-20210110-free.tcd",
         "tcdata/HARMONICS_NO_US.IDX", "tcdata/HARMONICS_NO_US", "gshhs/poly-c-1.dat",
         "basemap_shp/basemap_low.shp", "sounds/2bells.wav"})
      Write(stock / relative, "fixture");
    const auto defaults = InstalledResourceDefaults(app);
    Check(defaults.has_value(), "Bounded UTF-8 stock locator with complete resources");
    ResourceSelection fresh;
    ApplyResourceDefaults(fresh, *defaults);
    Check(fresh.tides.size() == 2 && fresh.tides.front() ==
          PathUtf8(stock / "tcdata/harmonics-dwf-20210110-free.tcd"), "Pinned harmonic defaults use stable stock");
    Check(fresh.coastline == PathUtf8(stock / "gshhs") &&
          fresh.basemap == PathUtf8(stock / "basemap_shp") &&
          fresh.ais_alarm == PathUtf8(stock / "sounds/2bells.wav"), "Other persisted resource defaults use stable stock");
    ResourceSelection custom{{"missing custom tide", "regional tide"}, "custom coast", "custom basemap", "custom alarm"};
    ApplyResourceDefaults(custom, *defaults);
    Check(custom.tides == std::vector<std::string>({"missing custom tide", "regional tide"}) &&
          custom.coastline == "custom coast" && custom.basemap == "custom basemap" &&
          custom.ais_alarm == "custom alarm", "All existing selections, even missing ones, remain unchanged");
    ResourceSelection partial{{"only user tide"}, "", "explicit base", ""};
    ApplyResourceDefaults(partial, *defaults);
    Check(partial.tides.size() == 1 && partial.tides[0] == "only user tide" &&
          partial.basemap == "explicit base" && partial.coastline == fresh.coastline &&
          partial.ais_alarm == fresh.ais_alarm, "Fill only unset quantities without supplementing a user tide list");
    const auto replacement = root / "generation two/app";
    Write(replacement / marker.filename(), PathUtf8(stock / "opencpn.exe"));
    fs::remove_all(root / "generation one");
    Check(InstalledResourceDefaults(replacement)->tides == fresh.tides &&
          fs::exists(PathFromUtf8(fresh.tides[0])), "Generation removal does not invalidate saved defaults");
    for (const auto& invalid : {std::string(), std::string(4097, 'x'), std::string("relative/opencpn.exe"),
         PathUtf8(stock / "opencpn.exe") + "\n", std::string("bad\0marker", 10)}) {
      Write(replacement / marker.filename(), invalid);
      Check(!InstalledResourceDefaults(replacement), "Malformed/relative/oversized locator refused");
    }
    Write(replacement / marker.filename(), PathUtf8(stock / "opencpn.exe"));
    Write(stock / "tcdata/HARMONICS_NO_US", "");
    Check(!InstalledResourceDefaults(replacement), "Incomplete or empty stock resources stay unavailable");
    fs::remove_all(root);
    std::cout << "Stable installed resource defaults, custom preservation and bounded locator passed\n";
  } catch (const std::exception& e) {
    fs::remove_all(root);
    std::cerr << e.what() << '\n';
    return 1;
  }
}

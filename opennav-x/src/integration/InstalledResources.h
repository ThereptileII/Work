#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace opennav::integration {
// Only resource defaults, never chart catalogs or navigation/user data.
struct ResourceSelection {
  std::vector<std::string> tides;
  std::string coastline, basemap, ais_alarm;
};

// The installer writes this bounded marker after exact stock verification.
// It is a resource locator, not an installation authorization or executable.
std::optional<ResourceSelection> InstalledResourceDefaults(
    const std::filesystem::path& application);
void ApplyResourceDefaults(ResourceSelection& selected,
                           const ResourceSelection& defaults);
}  // namespace opennav::integration

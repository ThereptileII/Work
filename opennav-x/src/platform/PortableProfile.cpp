#include "platform/PortableProfile.h"
#include <stdexcept>
namespace opennav::platform {
std::optional<PreviewPaths> PreviewProfile(const std::filesystem::path &exe,
                                           const std::string &requested) {
  namespace fs = std::filesystem;
  if (!fs::exists(exe.parent_path() / "OPENNAV_PORTABLE_PREVIEW"))
    return std::nullopt;
  const auto app = fs::canonical(exe.parent_path());
  if (app.filename() != fs::path("app"))
    throw std::runtime_error(
        "Keep the portable OpenNav app folder inside the extracted package");
  const auto root = app.parent_path();
  const auto profile = root / "profile", logs = root / "logs";
  // Reject symlinks/junctions escaping the extracted package as well as
  // explicit external --configdir arguments. Never silently fall back to the
  // normal profile.
  if (fs::weakly_canonical(profile) != profile ||
      fs::weakly_canonical(logs) != logs)
    throw std::runtime_error("portable OpenNav profile/logs must stay inside "
                             "the extracted package");
  if (!requested.empty() &&
      fs::weakly_canonical(fs::u8path(requested)) != profile)
    throw std::runtime_error(
        "portable OpenNav refuses a profile outside its own profile folder");
  fs::create_directories(profile);
  fs::create_directories(logs);
  return PreviewPaths{root, profile, logs};
}
} // namespace opennav::platform

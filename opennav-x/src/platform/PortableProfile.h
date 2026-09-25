#pragma once
#include <filesystem>
#include <optional>
#include <string>
namespace opennav::platform {
struct PreviewPaths {
  std::filesystem::path root, profile, logs;
};
inline std::string PathUtf8(const std::filesystem::path &path) {
  const auto s = path.u8string();
  return std::string(s.begin(), s.end());
}
inline std::filesystem::path PathFromUtf8(const std::string& value) {
#ifdef __cpp_char8_t
  return std::filesystem::path(std::u8string(value.begin(), value.end()));
#else
  return std::filesystem::u8path(value);
#endif
}
// Marker shipped beside the executable activates fail-closed package isolation,
// including direct double-click of app/opencpn.exe. No installed paths are
// read.
std::optional<PreviewPaths>
PreviewProfile(const std::filesystem::path &executable,
               const std::string &requested_profile);
} // namespace opennav::platform

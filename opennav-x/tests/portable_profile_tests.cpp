#include "platform/PortableProfile.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
namespace fs = std::filesystem;
void Check(bool b, const char *why) {
  if (!b)
    throw std::runtime_error(why);
}
int main() {
  const auto root =
      fs::temp_directory_path() /
      ("OpenNav preview path " +
       std::to_string(
           std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    fs::create_directories(root / "app");
    const auto exe = root / "app/opencpn.exe";
    Check(!opennav::platform::PreviewProfile(exe, ""),
          "Unpackaged build unchanged");
    std::ofstream(root / "app/OPENNAV_PORTABLE_PREVIEW") << "0.1";
    auto paths = opennav::platform::PreviewProfile(exe, "");
    Check(paths && fs::equivalent(paths->profile, root / "profile"),
          "Direct launch isolated");
    Check(opennav::platform::PreviewProfile(
              exe, opennav::platform::PathUtf8(root / "profile"))
              .has_value(),
          "Explicit matching profile accepted");
    bool rejected = false;
    try {
      opennav::platform::PreviewProfile(
          exe, opennav::platform::PathUtf8(root / "normal profile"));
    } catch (...) {
      rejected = true;
    }
    Check(rejected && !fs::exists(root / "normal profile"),
          "External profile refused before mutation");
    fs::create_directories(root / "wrong");
    std::ofstream(root / "wrong/OPENNAV_PORTABLE_PREVIEW") << "0.1";
    rejected = false;
    try {
      opennav::platform::PreviewProfile(root / "wrong/opencpn.exe", "");
    } catch (...) {
      rejected = true;
    }
    Check(rejected, "Broken extraction fails closed");
    fs::remove_all(root);
    std::cout << "Portable profile isolation and paths with spaces passed\n";
  } catch (const std::exception &e) {
    fs::remove_all(root);
    std::cerr << e.what() << '\n';
    return 1;
  }
}

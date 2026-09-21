#include "platform/PlatformIntegration.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

int main(int argc, char** argv) {
  if (argc < 4) return 2;
  const std::filesystem::path marker(argv[2]);
  if (std::string(argv[1]) == "parent") {
    std::vector<std::string> args{"child", argv[2], argv[3], "with spaces", "quote\"slash\\", ""};
    if (!opennav::platform::RestartAfterExit(argv[0], args)) return 3;
    std::ofstream(marker.string() + ".armed") << "armed";
    std::this_thread::sleep_for(std::chrono::milliseconds(1800));
    return 0;
  }
  std::ofstream output(marker, std::ios::binary);
  for (int i = 3; i < argc; ++i) output << std::string(argv[i]).size() << ':' << argv[i] << '\n';
  return output ? 0 : 4;
}

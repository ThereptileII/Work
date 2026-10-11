// Reads synthetic PE data only; never loads or executes a DLL.
#include "integration/Win32PeImports.h"
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  using namespace opennav::integration;
  if (argc != 4) return 2;
  const auto read = [](const char* name) {
    std::ifstream file(name, std::ios::binary);
    return std::vector<unsigned char>{std::istreambuf_iterator<char>(file), {}};
  };
  const auto before = ReadWin32PeImports(read(argv[1]));
  const auto after = ReadWin32PeImports(read(argv[2]));
  if (!before.parsed || !before.forwarders.empty() || !before.modules.empty() ||
      !after.parsed || after.forwarders != std::vector<std::string>{argv[3]} ||
      !after.modules.empty()) {
    std::cerr << "FAIL " << before.error << " / " << after.error << '\n';
    return 1;
  }
  std::cout << "PASS exact production-reader declaration: " << argv[3] << '\n';
}

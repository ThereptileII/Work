// Byte transport only: never opens or executes a supplied PE path.
#include "integration/Win32PeImports.h"
#include <iostream>
#include <string>
#include <vector>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif
namespace {
void String(const std::string& value) {
  std::cout << '"';
  for (const unsigned char c : value) {
    if (c == '"' || c == '\\') std::cout << '\\';
    std::cout << c;
  }
  std::cout << '"';
}
}
int main() {
#ifdef _WIN32
  if (_setmode(_fileno(stdin), _O_BINARY) == -1) return 2;
#endif
  std::vector<unsigned char> bytes;
  char block[16384];
  while (std::cin.read(block, sizeof block) || std::cin.gcount()) {
    if (bytes.size() + static_cast<std::size_t>(std::cin.gcount()) > 128u * 1024u * 1024u)
      return 2;
    bytes.insert(bytes.end(), block, block + std::cin.gcount());
  }
  if (!std::cin.eof()) return 2;
  const auto result = opennav::integration::ReadWin32PeImports(bytes);
  std::cout << "{\"parsed\":" << (result.parsed ? "true" : "false") << ",\"error\":";
  String(result.error);
  std::cout << ",\"modules\":[";
  bool first = true;
  for (const auto& module : result.modules) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << "{\"name\":"; String(module.name);
    std::cout << ",\"delayed\":" << (module.delayed ? "true" : "false") << '}';
  }
  std::cout << "]}\n";
}

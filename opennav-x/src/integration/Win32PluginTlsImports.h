#pragma once
#include "integration/Win32PeImports.h"

namespace opennav::integration {
// Narrow policy over the entry DLL's declared imports, NOT a dependency-closure
// approval. Keep the general PE reader usable for evidence without trust policy.
inline std::string RefuseLegacyPluginTlsImports(
    const std::vector<unsigned char>& bytes) {
  const auto imports = ReadWin32PeImports(bytes);
  if (!imports.parsed) return "Cannot inspect plugin imports: " + imports.error;
  for (const auto& item : imports.modules) {
    auto name = item.name;
    for (auto& c : name)
      if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    // Include qualified paths, extension-less LoadLibrary names and Windows
    // trailing-dot/space aliases; never follow an import as a filesystem path.
    const auto slash = name.find_last_of("/\\");
    if (slash != std::string::npos) name.erase(0, slash + 1);
    if (name.size() > 1 && name[1] == ':') name.erase(0, 2);
    const auto colon = name.find(':');
    if (colon != std::string::npos) name.erase(colon);
    while (!name.empty() && (name.back() == '.' || name.back() == ' '))
      name.pop_back();
    if (name == "libeay32.dll" || name == "ssleay32.dll" ||
        name == "libeay32" || name == "ssleay32")
      return std::string("Unsupported legacy TLS ") +
             (item.delayed ? "delay import: " : "import: ") + item.name;
  }
  return {};
}
}

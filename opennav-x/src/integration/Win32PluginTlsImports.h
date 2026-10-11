#pragma once
#include "integration/Win32PeImports.h"
#include <deque>
#include <set>

namespace opennav::integration {
// Narrow policy over the entry DLL's declared imports, NOT a dependency-closure
// approval. Keep the general PE reader usable for evidence without trust policy.
inline std::string RefuseLegacyPluginTlsImports(const Win32PeImports& imports) {
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

inline std::string RefuseLegacyPluginTlsImports(
    const std::vector<unsigned char>& bytes) {
  return RefuseLegacyPluginTlsImports(ReadWin32PeImports(bytes));
}

struct PluginTlsLocalModule {
  bool found = false;
  std::vector<unsigned char> bytes;
  std::string error;
};

// Inspect the bounded, declarative import graph whose DLL basenames resolve
// beside the original plugin. `read_local` must read and hold each found file
// across the eventual LoadLibrary call. Missing sibling modules may resolve in
// the application/system/PATH loader search; this function makes no claim about
// those paths, dynamic loads or child processes.
template <typename ReadLocal>
std::string RefuseLegacyPluginLocalTlsClosure(
    const std::vector<unsigned char>& root, ReadLocal read_local) {
  constexpr std::size_t max_nodes = 256;
  constexpr std::size_t max_bytes = 256u * 1024u * 1024u;
  std::deque<std::vector<unsigned char>> pending;
  pending.push_back(root);
  std::set<std::string> seen;
  std::size_t total = root.size();
  while (!pending.empty()) {
    auto bytes = std::move(pending.front());
    pending.pop_front();
    const auto imports = ReadWin32PeImports(bytes);
    const auto denied = RefuseLegacyPluginTlsImports(imports);
    if (!denied.empty()) return denied;
    for (const auto& item : imports.modules) {
      // PE imports conventionally name a module basename. A qualified or
      // redirected name has no safe sibling-resolution interpretation here.
      if (item.name.empty() || item.name == "." || item.name == ".." ||
          item.name.find_first_of("/\\:*?\"<>|") != std::string::npos ||
          item.name.back() == ' ' || item.name.back() == '.')
        return "Unsupported plugin import module name";
      // LoadLibrary resolves a basename without an extension as <name>.dll.
      // Inspect that sibling too, and deduplicate it with an explicit .dll
      // import; otherwise an indirect legacy TLS import can evade this walk.
      auto key = item.name;
      if (key.find('.') == std::string::npos) key += ".dll";
      for (auto& c : key)
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
      if (!seen.insert(key).second) continue;
      if (seen.size() > max_nodes) return "Plugin import closure node budget exceeded";
      auto local = read_local(key);
      if (!local.error.empty()) return local.error;
      if (!local.found) continue;
      if (local.bytes.empty() || local.bytes.size() > max_bytes - total)
        return "Plugin import closure byte budget exceeded";
      total += local.bytes.size();
      pending.push_back(std::move(local.bytes));
    }
  }
  return {};
}
}

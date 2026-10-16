#pragma once
#include "integration/Win32PeImports.h"
#include <deque>
#include <set>
#include <utility>

namespace opennav::integration {
// Supported forwarder grammar is deliberately narrower than Windows' loader:
// one separator, no explicit .dll or dotted symbol/module. Ambiguous strings
// are unsupported here, not declared invalid Windows images. Native harmless
// probes must establish such semantics before this policy is broadened.
inline std::string PluginTlsForwarderModule(const std::string& target,
                                           std::string& module) {
  const auto dot = target.find('.');
  if (dot == std::string::npos || !dot || dot + 1 == target.size() ||
      target.find('.', dot + 1) != std::string::npos)
    return "Unsupported plugin export forwarder grammar: " + target;
  const auto symbol = target.substr(dot + 1);
  if (symbol.front() == '#') {
    std::uint32_t ordinal = 0;
    if (symbol.size() == 1) return "Unsupported plugin export forwarder ordinal";
    for (std::size_t i = 1; i < symbol.size(); ++i) {
      if (symbol[i] < '0' || symbol[i] > '9' || ordinal > 6553)
        return "Unsupported plugin export forwarder ordinal";
      ordinal = ordinal * 10 + static_cast<unsigned>(symbol[i] - '0');
      if (ordinal > 65535) return "Unsupported plugin export forwarder ordinal";
    }
    if (!ordinal) return "Unsupported plugin export forwarder ordinal";
  }
  module = target.substr(0, dot);
  return {};
}

// Narrow policy over declared imports/forwarders, NOT a dependency-closure
// approval. Keep the general PE reader usable for evidence without trust policy.
inline std::string RefuseLegacyPluginTlsImports(const Win32PeImports& imports) {
  if (!imports.parsed) return "Cannot inspect plugin imports: " + imports.error;
  const auto legacy = [](std::string name) {
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
    return name == "libeay32.dll" || name == "ssleay32.dll" ||
           name == "libeay32" || name == "ssleay32";
  };
  for (const auto& item : imports.modules) {
    if (legacy(item.name))
      return std::string("Unsupported legacy TLS ") +
             (item.delayed ? "delay import: " : "import: ") + item.name;
  }
  for (const auto& target : imports.forwarders) {
    std::string module;
    const auto unsupported = PluginTlsForwarderModule(target, module);
    if (!unsupported.empty()) return unsupported;
    if (legacy(module)) return "Unsupported legacy TLS export forwarder: " + target;
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

// Same exact stock OpenCPN 5.12.4 OpenSSL 3.0.5 identities as Lifecycle.ps1.
// This denies known bytes, including renamed copies; it does not qualify other
// OpenSSL builds or the complete Windows dependency search.
inline std::string RefuseKnownObsoletePluginTlsSha256(const std::string& digest) {
  if (digest == "2bede6f2840cfe0a4e88d5c3fa648c170509c9ceed438c7e8440340541ca0446")
    return "Known obsolete TLS runtime: OpenSSL 3.0.5 libssl-3.dll";
  if (digest == "05752a0550349337af8b7e25362d527f6a30261dd3d1e4db61bf707c9fac589d")
    return "Known obsolete TLS runtime: OpenSSL 3.0.5 libcrypto-3.dll";
  return {};
}

struct PluginTlsClosureLimits {
  // Found dependency PEs and bytes bound parsing/retention. Candidate probes
  // also include missing basenames in every root, so they need a separate cap.
  std::size_t max_nodes = 256;
  std::size_t max_bytes = 256u * 1024u * 1024u;
  std::size_t max_probes = 1024;
};

// Inspect one bounded declarative import/forwarder graph over all supplied
// directories. For EVERY declared basename, inspect each root candidate even
// when another root contains that name. Imports from any found file reenter
// this same queue, so chains crossing plugin, host and upstream roots are checked.
// Root identities must identify the actual directories, not their aliases.
// `read_candidate(index, basename)` must read and hold each found file across
// the module lifetime. No Windows search-order selection or complete loader
// closure is claimed: other PATH directories, preloaded modules, SxS, dynamic loads
// and child processes remain outside this declared-candidate graph.
template <typename ReadCandidate, typename InspectBytes>
std::string RefuseLegacyPluginTlsClosure(
    const std::vector<unsigned char>& root,
    const std::vector<std::string>& root_identities, ReadCandidate read_candidate,
    InspectBytes inspect_bytes, PluginTlsClosureLimits limits = {}) {
  if (root_identities.empty()) return "Cannot identify plugin import roots";
  if (root_identities.size() > limits.max_nodes)
    return "Plugin import closure node budget exceeded";
  for (const auto& identity : root_identities)
    if (identity.empty()) return "Cannot identify plugin import root";
  if (root.size() > limits.max_bytes)
    return "Plugin import closure byte budget exceeded";
  std::deque<std::vector<unsigned char>> pending;
  pending.push_back(root);
  std::set<std::pair<std::string, std::string>> seen;
  std::size_t found_nodes = 0;
  std::size_t total = root.size();
  while (!pending.empty()) {
    auto bytes = std::move(pending.front());
    pending.pop_front();
    const auto byte_refusal = inspect_bytes(bytes);
    if (!byte_refusal.empty()) return byte_refusal;
    const auto imports = ReadWin32PeImports(bytes);
    const auto denied = RefuseLegacyPluginTlsImports(imports);
    if (!denied.empty()) return denied;
    auto declarations = imports.modules;
    for (const auto& target : imports.forwarders) {
      std::string module;
      const auto unsupported = PluginTlsForwarderModule(target, module);
      if (!unsupported.empty()) return unsupported;
      declarations.push_back({std::move(module), false});
    }
    for (const auto& item : declarations) {
      // PE imports conventionally name a module basename. A qualified or
      // redirected name has no safe root-candidate interpretation here.
      if (item.name.empty() || item.name == "." || item.name == ".." ||
          item.name.find_first_of("/\\:*?\"<>|") != std::string::npos ||
          item.name.back() == ' ' || item.name.back() == '.')
        return "Unsupported plugin import module name";
      // Win32 reserved devices remain reserved with extensions. Refuse their
      // stems, including spaces before the first dot, before any candidate open.
      // The PE reader accepts ASCII only, excluding superscript-digit aliases.
      auto stem = item.name.substr(0, item.name.find('.'));
      while (!stem.empty() && stem.back() == ' ') stem.pop_back();
      for (auto& c : stem)
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
      if (stem == "con" || stem == "prn" || stem == "aux" || stem == "nul" ||
          stem == "conin$" || stem == "conout$" ||
          (stem.size() == 4 && (stem.compare(0, 3, "com") == 0 ||
                               stem.compare(0, 3, "lpt") == 0) &&
           stem[3] >= '1' && stem[3] <= '9'))
        return "Unsupported plugin reserved module name: " + item.name;
      // LoadLibrary resolves a basename without an extension as <name>.dll.
      // Keep original spelling for lookup; lowercase only the per-directory
      // deduplication key shared by extensionless and .dll imports.
      auto sibling = item.name;
      if (sibling.find('.') == std::string::npos) sibling += ".dll";
      auto key = sibling;
      for (auto& c : key)
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
      for (std::size_t index = 0; index < root_identities.size(); ++index) {
        const auto candidate = std::make_pair(root_identities[index], key);
        if (seen.find(candidate) != seen.end()) continue;
        // Missing candidates still consume bounded filesystem work, but do not
        // consume a found-PE slot merely because another import root exists.
        if (seen.size() >= limits.max_probes)
          return "Plugin import closure probe budget exceeded";
        seen.insert(candidate);
        auto local = read_candidate(index, sibling);
        if (!local.error.empty()) return local.error;
        if (!local.found) continue;
        if (found_nodes >= limits.max_nodes)
          return "Plugin import closure node budget exceeded";
        ++found_nodes;
        if (local.bytes.empty() || local.bytes.size() > limits.max_bytes - total)
          return "Plugin import closure byte budget exceeded";
        total += local.bytes.size();
        pending.push_back(std::move(local.bytes));
      }
    }
  }
  return {};
}

// Explicit single-directory adapter retained for existing reader/policy tests.
// Production uses the multi-root walk above with retained-byte hash inspection.
template <typename ReadLocal, typename InspectBytes>
std::string RefuseLegacyPluginLocalTlsClosure(
    const std::vector<unsigned char>& root, ReadLocal read_local,
    InspectBytes inspect_bytes) {
  return RefuseLegacyPluginTlsClosure(root, {"local"},
      [&](std::size_t, const std::string& module) { return read_local(module); },
      inspect_bytes);
}

// Reader-only compatibility seam; production always supplies the byte policy.
template <typename ReadLocal>
std::string RefuseLegacyPluginLocalTlsClosure(
    const std::vector<unsigned char>& root, ReadLocal read_local) {
  return RefuseLegacyPluginLocalTlsClosure(root, read_local,
      [](const std::vector<unsigned char>&) { return std::string{}; });
}
}

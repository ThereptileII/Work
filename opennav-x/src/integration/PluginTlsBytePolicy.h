#pragma once
#include "integration/Win32PluginTlsImports.h"
#include "picosha2.h"

namespace opennav::integration {
// Hash the complete bytes read from the still-held root/sibling handle. No
// filename, reopened path or caller-provided digest supplies this decision.
inline std::string PluginTlsBytesSha256(
    const std::vector<unsigned char>& bytes) {
  picosha2::hash256_one_by_one hash;
  hash.process(bytes.begin(), bytes.end());
  hash.finish();
  return picosha2::get_hash_hex_string(hash);
}
inline std::string RefuseKnownObsoletePluginTlsBytes(
    const std::vector<unsigned char>& bytes) {
  return RefuseKnownObsoletePluginTlsSha256(PluginTlsBytesSha256(bytes));
}
}

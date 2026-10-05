#pragma once
#include <cstring>

namespace opennav::integration {
// Only verified presentation instances may select owned light artwork. S-57
// attribute names are fixed six-byte fields. Any ORIENT presence keeps the
// original vector, including malformed/non-finite values handled upstream.
inline const char* PresentationLightAlias(bool enabled, const char* feature,
                                         const char* symbol, const char* attributes,
                                         int count) {
  if (!enabled || !feature || !symbol || std::memcmp(feature, "LIGHTS", 6) ||
      count < 0 || count > 4096 || (count && !attributes)) return nullptr;
  for (int i = 0; i < count; ++i)
    if (!std::memcmp(attributes + i * 6, "ORIENT", 6)) return nullptr;
  if (!std::memcmp(symbol, "LIGHTS11", 8)) return "XNLIT011";
  if (!std::memcmp(symbol, "LIGHTS12", 8)) return "XNLIT012";
  if (!std::memcmp(symbol, "LIGHTS13", 8)) return "XNLIT013";
  return nullptr;
}
} // namespace opennav::integration

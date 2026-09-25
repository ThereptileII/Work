#pragma once
#include <array>
#include <cstdint>
#include <string>

namespace opennav::integration {
// Resource/encoding preflight only. Syntax and values still use the pinned
// RapidJSON parser already used by OpenCPN. Bound recursion before parsing.
inline bool BoundedJsonText(const std::string &text) {
  if (text.empty() || text.size() > 262144) return false;
  std::array<char, 16> stack{};
  std::size_t depth = 0;
  bool quoted = false, escaped = false;
  for (std::size_t i = 0; i < text.size();) {
    const auto c = static_cast<unsigned char>(text[i++]);
    if (c >= 0x80) {
      unsigned tail = c >= 0xc2 && c <= 0xdf ? 1
                      : c >= 0xe0 && c <= 0xef ? 2
                      : c >= 0xf0 && c <= 0xf4 ? 3 : 0;
      if (!tail || i + tail > text.size() || !quoted) return false;
      std::uint32_t value = c & ((1u << (6 - tail)) - 1);
      for (unsigned n = 0; n < tail; ++n) {
        const auto b = static_cast<unsigned char>(text[i++]);
        if ((b & 0xc0) != 0x80) return false;
        value = (value << 6) | (b & 0x3f);
      }
      if ((tail == 1 && value < 0x80) || (tail == 2 && value < 0x800) ||
          (tail == 3 && value < 0x10000) || value > 0x10ffff ||
          (value >= 0xd800 && value <= 0xdfff) || escaped) return false;
      continue;
    }
    if (quoted) {
      if (c < 0x20) return false;
      if (escaped) { escaped = false; continue; }
      if (c == '\\') escaped = true;
      else if (c == '"') quoted = false;
    } else if (c == '"') quoted = true;
    else if (c == '{' || c == '[') {
      if (depth == stack.size()) return false;
      stack[depth++] = static_cast<char>(c);
    } else if (c == '}' || c == ']') {
      if (!depth || stack[--depth] != (c == '}' ? '{' : '[')) return false;
    } else if (c < 0x20 && c != '\n' && c != '\r' && c != '\t') return false;
  }
  return !quoted && !escaped && !depth;
}
} // namespace opennav::integration

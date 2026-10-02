#pragma once
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace opennav::integration {

// Bounded Latin labels can retain native kerning advances while applying the
// prototype tracking. Other scripts keep native whole-string shaping verbatim.
// Combining sequences retain whole-string shaping too: partial-extent indexing
// varies between native backends. Never split UTF-16 surrogate pairs.
inline std::vector<std::size_t> ChartNameClusterStarts(const std::u32string& text) {
  std::vector<std::size_t> starts;
  if (text.empty() || text.size() > 512) return starts;
  for (std::size_t i = 0; i < text.size(); ++i) {
    const auto c = text[i];
    if (!((c >= 0x20 && c <= 0x7e) || (c >= 0x00a0 && c <= 0x024f)))
      return {};
    starts.push_back(i);
  }
  return starts;
}

inline double ChartNameTrackingWidth(std::size_t clusters, double spacing) {
  // SVG letter-spacing includes the trailing advance. Reject corrupt scaling
  // rather than producing unbounded coordinates or allocating huge textures.
  return clusters <= 512 && std::isfinite(spacing) && spacing > 0 && spacing <= 64 &&
      clusters * spacing <= 4096
      ? clusters * spacing : 0;
}

} // namespace opennav::integration

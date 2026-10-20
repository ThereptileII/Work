#pragma once

#include <cstring>
#include <initializer_list>

namespace opennav::integration {
// AC paint only, after the native depth-area conditional has selected a role.
// Never replace DEPIT (drying), patterns, soundings, safety contour DEPSC, or
// DEPVS used by rocks/wrecks/obstructions. No chart attributes or depth/safety
// thresholds change. The selected Day/Dusk/Night DEPDW remains authoritative.
inline const char* VeryLightAreaColor(bool enabled, const char* feature,
                                      const char* color) {
  if (!enabled || !feature || !color ||
      (std::strncmp(feature, "DEPARE", 6) && std::strncmp(feature, "DRGARE", 6)))
    return color;
  for (const char* depth : {"DEPDW", "DEPMD", "DEPMS", "DEPVS"})
    if (!std::strncmp(color, depth, 5)) return "DEPDW";
  return color;
}

inline bool VeryLightOrdinaryContour(bool enabled, const char* feature,
                                     const char* color,
                                     bool linePrimitive = false) {
  // Native DEPCNT02 emits ordinary DEPCN for both DEPCNT and line DEPARE.
  // Area boundaries and safety DEPSC must retain their selected native role.
  return enabled && feature && color &&
         (!std::strncmp(feature, "DEPCNT", 6) ||
          (linePrimitive && !std::strncmp(feature, "DEPARE", 6))) &&
         !std::strncmp(color, "DEPCN", 5);
}

template<class Color>
inline Color FaintDepthContour(Color contour, const Color& water) {
  // Opaque preblend works identically in native DC, VBO and shader paths.
  // 50% ordinary-contour ink over water; safety DEPSC never enters this function.
  contour.R = (contour.R + water.R + 1) / 2;
  contour.G = (contour.G + water.G + 1) / 2;
  contour.B = (contour.B + water.B + 1) / 2;
  return contour;
}
}  // namespace opennav::integration

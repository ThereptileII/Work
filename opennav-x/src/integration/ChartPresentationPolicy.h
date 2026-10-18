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
                                     const char* color) {
  return enabled && feature && color && !std::strncmp(feature, "DEPCNT", 6) &&
         !std::strncmp(color, "DEPCN", 5);
}

template<class Color>
inline Color FaintDepthContour(Color contour, const Color& water) {
  // Opaque preblend works identically in native DC, VBO and shader paths.
  // Keep 35% ordinary-contour ink; safety DEPSC never enters this function.
  contour.R = (35 * contour.R + 65 * water.R + 50) / 100;
  contour.G = (35 * contour.G + 65 * water.G + 50) / 100;
  contour.B = (35 * contour.B + 65 * water.B + 50) / 100;
  return contour;
}
}  // namespace opennav::integration

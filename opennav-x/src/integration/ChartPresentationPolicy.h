#pragma once

#include <array>
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

// Fixed native symbol identities, not a feature-wide filter: several of these
// objects can instead select the magenta isolated-danger symbol or SOUND digits.
// Those independently selected marks must never inherit this neutral paint.
inline bool VeryLightObjectSymbol(bool enabled, const char* scheme,
                                  const char* symbol) {
  if (!enabled || !scheme || !symbol ||
      (std::strcmp(scheme, "DAY") && std::strcmp(scheme, "DAY_BRIGHT")))
    return false;
  for (const char* name : {
           "UWTROC03", "UWTROC04", "WRECKS01", "WRECKS04", "WRECKS05",
           "WRECKS07", "OBSTRN01", "OBSTRN02", "OBSTRN03", "OBSTRN11",
           "OBSTRN18", "DANGER01", "DANGER02", "DANGER03", "DANGER51",
           "DANGER52", "PILPNT02", "MORFAC03", "MORFAC04"})
    if (!std::strncmp(symbol, name, 8)) return true;
  return false;
}

using SymbolInkRgb = std::array<unsigned char, 3>;

// The owner's 65%-black Day ink: 35% white = #595959, keeping alpha coverage
// intact. Dusk/Night are intentionally excluded above to retain visible ink.
inline constexpr SymbolInkRgb kVeryLightObjectInk{89, 89, 89};

inline bool RecolorVeryLightObjectPixel(unsigned char* rgb,
                                       const SymbolInkRgb& black,
                                       const SymbolInkRgb& gray) {
  if (!rgb) return false;
  const SymbolInkRgb pixel{rgb[0], rgb[1], rgb[2]};
  // Exact palette matches only. The supplied rock/wreck artwork uses chart-text
  // ink; older native tiles retain exact stock black/gray. Never infer a role
  // from an RGB tolerance: depth/intertidal fills and chromatic ink stay intact.
  if (pixel != black && pixel != gray &&
      pixel != SymbolInkRgb{104, 123, 122} &&
      pixel != SymbolInkRgb{7, 7, 7} && pixel != SymbolInkRgb{125, 137, 140})
    return false;
  rgb[0] = kVeryLightObjectInk[0];
  rgb[1] = kVeryLightObjectInk[1];
  rgb[2] = kVeryLightObjectInk[2];
  return true;
}
}  // namespace opennav::integration

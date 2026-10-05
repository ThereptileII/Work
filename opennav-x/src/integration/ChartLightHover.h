#pragma once
#include "integration/ChartCanvasInk.h"
#include <wx/colour.h>
#include <wx/string.h>

namespace opennav::integration {
// Paint-only mapping of the pinned extended-sector renderer's three existing
// colour classes. Its yellow/default class is the upstream white-sector class;
// this helper does not reinterpret COLOUR, choose objects or change geometry.
inline wxColour ExtendedLightSectorInk(bool enabled, const wxColour &stock,
                                       const wxString &scheme) {
  const bool day = scheme == "DAY" || scheme == "DAY_BRIGHT";
  const bool dusk = scheme == "DUSK", night = scheme == "NIGHT";
  if (!enabled || !stock.IsOk() || (!day && !dusk && !night)) return stock;
  const bool red = stock.Red() == 255 && stock.Green() == 0 && stock.Blue() == 0;
  const bool green = stock.Red() == 0 && stock.Green() == 255 && stock.Blue() == 0;
  const bool white = stock.Red() == 255 && stock.Green() == 255 && stock.Blue() == 0;
  if (!red && !green && !white) return stock;
  // Same verified sector stroke tokens as CaFanColors (chart-symbols.css).
  std::uint32_t ink = red ? (day ? 0xb66e6c : dusk ? 0xd3948c : 0xae7870)
      : green ? (day ? 0x508d78 : dusk ? 0x8fbaa2 : 0x789d84)
              : (day ? 0xa1976a : dusk ? 0xc5bc92 : 0x9d987a);
  const auto mode = night ? ui::LightMode::Night : dusk ? ui::LightMode::Dusk : ui::LightMode::Day;
  ink = ChartCanvasInk(mode, ink);
  // Retain the pinned renderer's per-theme opacity levels, but resolve them at
  // paint time so a retained hover selection cannot keep the previous theme.
  const unsigned opacity = day ? 100 : dusk ? 50 : 20;
  return wxColour((ink >> 16) & 255, (ink >> 8) & 255, ink & 255,
                   white ? opacity * 13 / 10 : opacity);
}
inline wxColour ExtendedLightBoundaryInk(bool enabled, const wxString &scheme,
                                         unsigned opacity) {
  const bool day = scheme == "DAY" || scheme == "DAY_BRIGHT";
  const bool dusk = scheme == "DUSK", night = scheme == "NIGHT";
  if (!enabled || (!day && !dusk && !night)) return wxColour(0, 0, 0, opacity);
  const auto mode = night ? ui::LightMode::Night : dusk ? ui::LightMode::Dusk : ui::LightMode::Day;
  const auto ink = ui::FloatingTheme(mode).primary;
  return wxColour((ink >> 16) & 255, (ink >> 8) & 255, ink & 255, opacity);
}
} // namespace opennav::integration

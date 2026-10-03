#pragma once
#include "ui/Theme.h"

namespace opennav::integration {
// Only raw, owned chart-canvas paint inputs belong here. The prototype dims
// this ancestor in Night; floating controls and already-effective ink do not.
// Apply before alpha composition, preserving coverage, geometry and semantics.
constexpr std::uint32_t ChartCanvasInk(ui::LightMode mode, std::uint32_t raw) {
  if (mode != ui::LightMode::Night) return raw;
  const auto channel=[](unsigned value) { return (value*78+50)/100; };
  return (channel((raw>>16)&255)<<16) | (channel((raw>>8)&255)<<8) |
         channel(raw&255);
}
} // namespace opennav::integration

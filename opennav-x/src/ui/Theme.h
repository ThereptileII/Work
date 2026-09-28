#pragma once

#include <cstdint>

namespace opennav::ui {

enum class LightMode { Day, Dusk, Night };
struct Palette {
  std::uint32_t background, surface, selected, elevated, border;
  std::uint32_t primary, secondary, muted, accent, healthy, attention, alarm, ais;
};

constexpr Palette Theme(LightMode mode) {
  // Supplied v8 HTML: :root and #app[data-theme], including inheritance.
  // The active button uses --mint. --cyan remains a separate context accent.
  // See docs/design/prototype-tokens.json; never take tokens from old Beta UI.
  switch (mode) {
    case LightMode::Day:
      return {0x152326, 0x1D2D31, 0x26393D, 0x26393D, 0x35464A,
              0xF3F5EE, 0xAABDBD, 0x7E9699, 0xB6EFCE, 0xB6EFCE,
              0xECC48C, 0xEC8F87, 0xCD8DAC};
    case LightMode::Dusk:
      return {0x1D282E, 0x25343B, 0x30444B, 0x30444B, 0x405059,
              0xE2E5DB, 0xACB9B7, 0x819394, 0x9BC5B1, 0x9BC5B1,
              0xCFAC84, 0xEC8F87, 0xCD8DAC};
    case LightMode::Night:
      return {0x0C1115, 0x141C21, 0x1D282F, 0x1D282F, 0x29353B,
              0xB8B5A7, 0x91988E, 0x747D77, 0x85A995, 0x85A995,
              0xAA9170, 0xB77569, 0xA77F8A};
  }
  return Theme(LightMode::Night);
}

namespace spacing {
constexpr int base = 8;
constexpr int compact = 4;
constexpr int touch = 48;
constexpr int action_height = 56;
constexpr int panel_radius = 8;
constexpr int control_radius = 8;
constexpr int left_rail = 56;
constexpr int right_rail = 136;
}  // namespace spacing

}  // namespace opennav::ui

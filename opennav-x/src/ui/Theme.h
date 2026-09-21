#pragma once

#include <cstdint>

namespace opennav::ui {

enum class LightMode { Day, Dusk, Night };
struct Palette {
  std::uint32_t background, surface, selected, elevated, border;
  std::uint32_t primary, secondary, muted, accent, healthy, attention, alarm, ais;
};

constexpr Palette Theme(LightMode mode) {
  switch (mode) {
    case LightMode::Day:
      return {0x07141C, 0x0B1922, 0x10232E, 0x132B37, 0x284653,
              0xF2F6F8, 0xA9BAC3, 0x708791, 0x00B8E6, 0x00E08A,
              0xF5B942, 0xFF4D5A, 0xF04F9B};
    case LightMode::Dusk:
      return {0x080F14, 0x0D171D, 0x142129, 0x182730, 0x31434A,
              0xC9C4BA, 0xADA49A, 0x7B7974, 0x5192A0, 0x619A79,
              0xB99254, 0xCA6265, 0xA66B8A};
    case LightMode::Night:
      return {0x050606, 0x090A0A, 0x141010, 0x1A1111, 0x382425,
              0xB58C87, 0x91706B, 0x705654, 0xA24743, 0x57725A,
              0x9D7143, 0xBB4949, 0x875166};
  }
  return Theme(LightMode::Night);
}

namespace spacing {
constexpr int base = 8;
constexpr int compact = 4;
constexpr int touch = 48;
constexpr int action_height = 56;
constexpr int panel_radius = 8;
constexpr int control_radius = 6;
constexpr int left_rail = 56;
constexpr int right_rail = 136;
}  // namespace spacing

}  // namespace opennav::ui

#pragma once

#include <cstdint>

namespace opennav::ui {

enum class LightMode { Day, Dusk, Night };
struct Palette {
  std::uint32_t background, surface, selected, elevated, border;
  std::uint32_t primary, secondary, muted, accent, healthy, attention, alarm, ais;
};
struct FloatingPalette { std::uint32_t surface, primary, secondary, compass_light; };
struct OnlineChartPalette { std::uint32_t stroke, fill, selected, stale; };
constexpr std::uint32_t ActiveRouteInk(LightMode mode) {
  // Final prototype --route. Route selection and explicit stored properties
  // remain OpenCPN concerns, outside this presentation-only palette.
  return mode == LightMode::Day ? 0x267C76
       : mode == LightMode::Dusk ? 0xB0DFC8 : 0x91BCA2;
}
constexpr std::uint32_t NavigationContextInk(LightMode mode) {
  // --cyan, distinct from --mint (selection/confirmation).
  return mode == LightMode::Day ? 0x7BC8D7
       : mode == LightMode::Dusk ? 0x82B0C1 : 0x78989C;
}
constexpr OnlineChartPalette OnlineChartTheme(LightMode mode) {
  // .ais-ship stroke/selection are inherited in all three supplied HTML themes.
  // Aging marks are a documented data-validity extension, using theme muted ink.
  switch(mode) {
  case LightMode::Day: return {0x916477,0xF7F8F0,0xCB9CB1,0x7E9699};
  case LightMode::Dusk: return {0x916477,0x243A40,0xCB9CB1,0x819394};
  case LightMode::Night: return {0x916477,0x152129,0xCB9CB1,0x747D77};
  }
  return OnlineChartTheme(LightMode::Night);
}
constexpr FloatingPalette FloatingTheme(LightMode mode) {
  switch(mode) {
    case LightMode::Day: return {0xF7F8F0,0x233E3E,0x6B8380,0xB0C3BC};
    case LightMode::Dusk: return {0x243A40,0xE1E5D8,0xA6BCB7,0xB0C3BC};
    case LightMode::Night: return {0x152129,0xB6C3AF,0x869D91,0xB0C3BC};
  }
  return FloatingTheme(LightMode::Night);
}

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
constexpr int panel_radius = 14;
constexpr int control_radius = 9;
}  // namespace spacing

namespace prototype_ink {
// Literal alpha colors in the supplied CSS, not theme-semantic replacements.
constexpr std::uint32_t warning = 0xECC48C;
constexpr int warning_tag_alpha = 10, warning_callout_alpha = 9;
constexpr int warning_border_alpha = 48;
constexpr std::uint32_t active = 0xB6EFCE;
constexpr int active_tag_alpha = 10, active_border_alpha = 35;
} // namespace prototype_ink

}  // namespace opennav::ui

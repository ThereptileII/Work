#pragma once

#include <cstdint>

namespace opennav::ui {

enum class LightMode { Day, Dusk, Night };
struct Palette {
  std::uint32_t background, surface, selected, elevated, border;
  std::uint32_t primary, secondary, muted, accent, healthy, attention, alarm, ais;
};
struct FloatingPalette { std::uint32_t surface, primary, secondary, compass_light; };
struct OnlineChartPalette { std::uint32_t stroke, fill, selected, stale, label; };
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
  // Label fill is #835d70; Night's ancestor brightness(.78) yields #664957.
  // Apply that effect only to the new label, never to the real chart surface.
  case LightMode::Day: return {0x916477,0xF7F8F0,0xCB9CB1,0x7E9699,0x835D70};
  case LightMode::Dusk: return {0x916477,0x243A40,0xCB9CB1,0x819394,0x835D70};
  case LightMode::Night: return {0x916477,0x152129,0xCB9CB1,0x747D77,0x664957};
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

constexpr std::uint32_t MixInk(std::uint32_t a, std::uint32_t b, double t) {
  const auto c = [&](int shift) {
    const double x = ((a >> shift) & 0xFF) * (1 - t) + ((b >> shift) & 0xFF) * t;
    return static_cast<std::uint32_t>(x + .5) << shift;
  };
  return c(16) | c(8) | c(0);
}
// SCRUM-363: an inactive route keeps the prototype's route family (teal),
// muted toward the floating secondary ink, instead of a neutral grey.
constexpr std::uint32_t InactiveRouteInk(LightMode mode) {
  return MixInk(ActiveRouteInk(mode), FloatingTheme(mode).secondary, .55);
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

struct RadarPalette {
  std::uint32_t surface, center, middle, border, rings, crosshair, caption, legend;
  unsigned char border_alpha = 48, rings_alpha = 43, crosshair_alpha = 32;
};
constexpr RadarPalette RadarTheme() {
  // Supplied radar display has fixed green/black ink in all three themes.
  // CSS/Canvas alpha is preserved over the gradient; no synthetic echo palette.
  return {0x0D1D20,0x142E29,0x10241F,0x6E9A7E,0x90D2B5,0x83B49B,
          0xB5D8BF,0x9EBCAF};
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

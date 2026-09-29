#pragma once

namespace opennav::ui::prototype {
// CSS pixels at DPR1; FromDIP conversion belongs only at the native boundary.
// Final v8 cascade, docs/design/prototype/reference/*/capture.json.
constexpr int top = 68;
constexpr int navigation = 80;
constexpr int rail = 186;
constexpr int footer = 34;
constexpr int horizon = 132;
constexpr int drawer = 398;
constexpr int configuration_drawer = 432;
constexpr int compact_configuration_drawer = 410; // final max-width:1100px
constexpr int drawer_gap = 14;
constexpr int drawer_top = 12;
constexpr int drawer_radius = 18;
constexpr int icon = 22;
constexpr int icon_target = 44;
constexpr int button_height = 48;
constexpr int button_radius = 9;
constexpr int rail_value = 48;
constexpr int rail_label = 11;
constexpr int page_title = 30;
constexpr int context_title = 26;
// Passage Windows reference: content origin (705,190), first point y403.
constexpr int passage_points_y = 213;
constexpr int passage_point_row = 77;
constexpr int page_inset = 32;
constexpr int dashboard_y = 146;
constexpr int dashboard_gap = 18;
constexpr int instrument_tile_height = 126;
constexpr int instrument_tile_gap = 12;
constexpr int wind_card_height = 540;

struct DesktopLayout {
  int top, navigation, rail, horizon, nav_height, nav_gap, nav_inset;
  int divider_before, divider_after, rail_header, pilot_height, pilot_gap;
};
constexpr DesktopLayout Desktop(int width, int height) {
  // Final supplied desktop media cascade; CSS pixels, not physical pixels.
  // Reference measurements: prototype-responsive-in-progress.md.
  const bool narrow = width <= 1100;
  const bool compact = width > 760 && height <= 740;
  const bool short_helm = width > 760 && height <= 600;
  return {short_helm ? 56 : compact ? 60 : 68,
          narrow ? 70 : 80, narrow ? 156 : 186,
          short_helm ? 98 : compact ? 112 : 132,
          short_helm ? 43 : compact ? 51 : 61,
          short_helm ? 0 : compact ? 1 : 5, compact ? 8 : 14,
          short_helm ? 3 : compact ? 5 : 7,
          short_helm ? 3 : compact ? 6 : 12,
          short_helm ? 30 : compact ? 33 : 42,
          short_helm ? 66 : compact ? 74 : 87,
          short_helm ? 7 : compact ? 9 : 13};
}
}  // namespace opennav::ui::prototype

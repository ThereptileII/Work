#pragma once
#include "application/DisplayPreferences.h"

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
constexpr double health_summary = 69.1875;
constexpr double health_detail_row = 48.59375;
constexpr int health_gap = 8;

struct DesktopLayout {
  int top, navigation, rail, horizon, nav_height, nav_gap, nav_inset;
  int divider_before, divider_after, rail_header, pilot_height, pilot_gap;
  // Additional computed CSS geometry used by the 1500px+ desktop shell.
  int data_rail_padding_x, metric_value_font_size;
  int timeline_padding_top, timeline_padding_x, timeline_events_margin_top;
  int timeline_event_title_size, timeline_event_small_size;
  int chart_location_left, chart_location_top, next_turn_left, next_turn_top;
  int drawer_width, wide_drawer_width;
};
constexpr int MetricLabelSize(int width,int height) {
  return width<=760 || (width>760 && height<=740) ? 9 : width<=1100 ? 10 : 11;
}
constexpr DesktopLayout Desktop(int width, int height) {
  // Final supplied desktop media cascade; CSS pixels, not physical pixels.
  // Reference measurements: prototype-responsive-in-progress.md.
  const bool narrow = width <= 1100;
  const bool mobile = width <= 760;
  const bool compact = width > 760 && height <= 740;
  const bool short_helm = width > 760 && height <= 600;
  const bool large_desktop = width >= 1500;
  const int metric_value_size = mobile ? 32 : short_helm ? 33 : compact ? 40 :
      large_desktop ? height >= 900 ? 62 : 48 : narrow ? 47 : 48;
  return {large_desktop ? compact ? 60 : 76 :
              short_helm ? 56 : compact ? 60 : 68,
          large_desktop ? 88 : narrow ? 70 : 80,
          large_desktop ? 220 : narrow ? 156 : 186,
          large_desktop ? compact ? 112 : 150 :
              short_helm ? 98 : compact ? 112 : 132,
          short_helm ? 43 : compact ? 51 : large_desktop ? 69 : 61,
          short_helm ? 0 : compact ? 1 : 5, compact ? 8 : 14,
          short_helm ? 3 : compact ? 5 : 7,
          short_helm ? 3 : compact ? 6 : 12,
          short_helm ? 30 : compact ? 33 : 42,
          short_helm ? 66 : compact ? 74 : 87,
          short_helm ? 7 : compact ? 9 : 13,
          mobile ? 15 : narrow ? 13 : large_desktop ? 25 : 18,
          metric_value_size,
          mobile ? 12 : compact ? 9 : large_desktop ? 18 : 13,
          mobile ? 17 : narrow ? 20 : large_desktop ? 32 : 25,
          mobile ? 24 : short_helm ? 17 : compact ? 19 : large_desktop ? 29 : 23,
          mobile ? 11 : short_helm ? 11 : narrow ? 12 : 13,
          mobile ? 8 : short_helm ? 8 : narrow ? 9 : 10,
          mobile ? 17 : large_desktop ? 34 : 28,
          mobile ? 18 : short_helm ? 14 : compact ? 18 : large_desktop ? 32 : 25,
          mobile ? 17 : large_desktop ? 34 : 28,
          mobile ? 79 : short_helm ? 77 : compact ? 88 : large_desktop ? 116 : 102,
          mobile ? 0 : 398,
          mobile ? 0 : large_desktop ? 460 : narrow ? 410 : 432};
}
constexpr DesktopLayout DisplayDesktop(int width, int height,
                                      application::ChartLayout choice) {
  auto layout = Desktop(width, height);
  if (width <= 760) return layout; // Existing native compact-pane behavior remains.
  if (choice == application::ChartLayout::ChartFocus) {
    layout.rail = 155;
    layout.horizon = 110;
  } else if (choice == application::ChartLayout::InstrumentFocus) {
    layout.rail = 230;
  }
  return layout;
}
}  // namespace opennav::ui::prototype

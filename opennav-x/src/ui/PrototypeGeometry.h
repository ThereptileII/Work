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
}  // namespace opennav::ui::prototype

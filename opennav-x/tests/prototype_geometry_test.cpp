#include "ui/PrototypeGeometry.h"

#include <array>
#include <cstdlib>
#include <iostream>

namespace {
using opennav::ui::prototype::Desktop;
using opennav::ui::prototype::DesktopLayout;
using opennav::ui::prototype::DisplayDesktop;
using opennav::ui::prototype::MetricLabelSize;

std::array<int, 25> Values(const DesktopLayout &g) {
  return {g.top, g.navigation, g.rail, g.horizon, g.nav_height, g.nav_gap,
          g.nav_inset, g.divider_before, g.divider_after, g.rail_header,
          g.pilot_height, g.pilot_gap, g.data_rail_padding_x,
          g.metric_value_font_size, g.timeline_padding_top,
          g.timeline_padding_x, g.timeline_events_margin_top,
          g.timeline_event_title_size, g.timeline_event_small_size,
          g.chart_location_left, g.chart_location_top, g.next_turn_left,
          g.next_turn_top, g.drawer_width, g.wide_drawer_width};
}

void Check(const char *name, int width, int height,
           const std::array<int, 25> &expected) {
  const auto actual = Values(Desktop(width, height));
  if (actual == expected) return;
  std::cerr << "Prototype geometry mismatch at " << name << "\nactual: ";
  for (const int value : actual) std::cerr << value << ' ';
  std::cerr << "\nexpected: ";
  for (const int value : expected) std::cerr << value << ' ';
  std::cerr << '\n';
  std::exit(1);
}
}  // namespace

int main() {
  // Expected CSS values were measured from the immutable prototype in
  // Chromium at DPR1, including the active media cascade at each viewport.
  Check("1280x800", 1280, 800,
        {68, 80, 186, 132, 61, 5, 14, 7, 12, 42, 87, 13,
         18, 48, 13, 25, 23, 13, 10, 28, 25, 28, 102, 398, 432});
  Check("1280x600 compact", 1280, 600,
        {56, 80, 186, 98, 43, 0, 8, 3, 3, 30, 66, 7,
         18, 33, 9, 25, 17, 11, 8, 28, 14, 28, 77, 398, 432});
  Check("1500x800 large desktop below metric breakpoint", 1500, 800,
        {76, 88, 220, 150, 69, 5, 14, 7, 12, 42, 87, 13,
         25, 48, 18, 32, 29, 13, 10, 34, 32, 34, 116, 398, 460});
  Check("1500x900 metric breakpoint", 1500, 900,
        {76, 88, 220, 150, 69, 5, 14, 7, 12, 42, 87, 13,
         25, 62, 18, 32, 29, 13, 10, 34, 32, 34, 116, 398, 460});
  Check("1920x1080 large desktop", 1920, 1080,
        {76, 88, 220, 150, 69, 5, 14, 7, 12, 42, 87, 13,
         25, 62, 18, 32, 29, 13, 10, 34, 32, 34, 116, 398, 460});
  const auto below=DisplayDesktop(1500,800,opennav::application::ChartLayout::Balanced);
  const auto above=DisplayDesktop(1500,900,opennav::application::ChartLayout::Balanced);
  if(below.metric_value_font_size!=48 || above.metric_value_font_size!=62)
    std::exit(1);
  const auto chart=DisplayDesktop(1500,900,opennav::application::ChartLayout::ChartFocus);
  const auto instruments=DisplayDesktop(1500,900,opennav::application::ChartLayout::InstrumentFocus);
  if(chart.rail!=155 || chart.horizon!=110 || chart.timeline_padding_x!=32 ||
     instruments.rail!=230 || instruments.horizon!=150)
    std::exit(1);
  if(MetricLabelSize(1280,800)!=11 || MetricLabelSize(900,800)!=10 ||
     MetricLabelSize(900,600)!=9 || MetricLabelSize(700,800)!=9)
    std::exit(1);
}

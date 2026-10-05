#include "application/AnchorView.h"
#include <gtest/gtest.h>
#include <limits>
using namespace opennav;
using namespace std::chrono_literals;
namespace {
const vessel::Time stamp{100s};
struct Fixture {
  application::AnchorState watch;
  vessel::VesselState state;
  Fixture() {
    state.navigation.latitude_deg = {58., "selected GPS", stamp,
                                     vessel::Validity::Measured};
    state.navigation.longitude_deg = {16., "selected GPS", stamp,
                                      vessel::Validity::Measured};
    watch.waypoint_id = "owned-watch";
    watch.source = "test integration";
    watch.anchor = application::Coordinate{57.99, 16.};
    watch.radius_m = 50;
    watch.observed_at = stamp;
    watch.distance_m = {18., watch.source, stamp, vessel::Validity::Measured};
    watch.vessel_position =
        application::AnchorFix{{58., 16.}, stamp, 0., 18., "selected GPS"};
  }
  application::AnchorView View(vessel::Time now = stamp) {
    return application::PresentAnchor(watch, state, now);
  }
};
} // namespace
TEST(OpenNavAnchorView, CurrentPositionAndUnavailableStates) {
  Fixture f;
  EXPECT_EQ(f.View().distance_m, 18.);
  EXPECT_EQ(f.View().radius_m, 50.);
  EXPECT_FALSE(f.View().can_start);
  f.watch.alarm = true;
  EXPECT_FALSE(f.View(stamp + 6s).distance_m);
  EXPECT_TRUE(f.View(stamp + 6s).alarm);
  EXPECT_FALSE(f.View(stamp - 1s).distance_m);
  f.watch = {};
  EXPECT_FALSE(f.View().active);
  EXPECT_FALSE(f.View().distance_m);
  EXPECT_TRUE(f.View().can_start);
  f.state.simulated = true;
  EXPECT_FALSE(f.View().can_start);
  f.state.simulated = false;
  f.state.replayed = true;
  EXPECT_FALSE(f.View().can_start);
  f.state.replayed = false;
  f.state.navigation.latitude_deg = {};
  EXPECT_FALSE(f.View().can_start);
}
TEST(OpenNavAnchorView, ChangedPositionAndInvalidObservationsSuppressed) {
  for (int change = 0; change < 13; ++change) {
    Fixture f;
    switch (change) {
    case 0:
      f.state.navigation.latitude_deg.value.reset();
      break;
    case 1:
      f.state.navigation.latitude_deg.source = "other";
      break;
    case 2:
      f.state.navigation.longitude_deg.observed_at -= 1s;
      break;
    case 3:
      f.state.navigation.latitude_deg.value = 58.01;
      break;
    case 4:
      f.watch.distance_m.observed_at -= 1s;
      break;
    case 5:
      f.watch.distance_m.value = std::numeric_limits<double>::infinity();
      break;
    case 6:
      f.watch.distance_m.value = -1;
      break;
    case 7:
      f.watch.vessel_position.reset();
      break;
    case 8:
      f.watch.vessel_position->observed_at -= 1s;
      break;
    case 9:
      f.watch.vessel_position->east_m =
          std::numeric_limits<double>::quiet_NaN();
      break;
    case 10:
      f.watch.anchor->latitude_deg = 91;
      break;
    case 11:
      f.watch.observed_at += 1s;
      break;
    case 12:
      f.state.navigation.latitude_deg.source =
          f.state.navigation.longitude_deg.source = "new selected GPS";
      break;
    }
    EXPECT_FALSE(f.View().distance_m) << change;
    EXPECT_FALSE(f.View().vessel_position) << change;
  }
  Fixture f;
  f.watch.radius_m = -25;
  EXPECT_TRUE(f.View().inner_alarm);
  EXPECT_EQ(f.View().radius_m, 25.);
  f.watch.radius_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(f.View().radius_m);
  f.watch.distance_m.value = 0.; // A real observed zero is valid.
  EXPECT_EQ(f.View().distance_m, 0.);
}
TEST(OpenNavAnchorView, HistoryOwnedBoundedAndNeverRefreshedByReading) {
  Fixture f;
  application::RetainAnchorHistory(f.watch, {});
  ASSERT_EQ(f.watch.recent_positions.size(), 1u);
  auto retained = f.watch;
  application::RetainAnchorHistory(f.watch, retained);
  EXPECT_EQ(f.watch.recent_positions.size(), 1u);
  f.watch.vessel_position->observed_at -= 1s;
  application::RetainAnchorHistory(f.watch, retained);
  EXPECT_EQ(f.watch.recent_positions.size(), 1u);
  for (int i = 1; i <= 310; ++i) {
    f.watch.vessel_position->observed_at = stamp + i * 1s;
    f.watch.observed_at = stamp + i * 1s;
    application::RetainAnchorHistory(f.watch, retained);
    retained = f.watch;
  }
  EXPECT_EQ(f.watch.recent_positions.size(), 300u);
  const auto before = f.watch.recent_positions.back().observed_at;
  EXPECT_EQ(f.View(stamp + 320s).history_minutes, 299. / 60.);
  EXPECT_EQ(f.watch.recent_positions.back().observed_at, before);
  f.watch.anchor->longitude_deg += 1.;
  application::RetainAnchorHistory(f.watch, retained);
  EXPECT_EQ(f.watch.recent_positions.size(), 1u);
  f.watch.waypoint_id = "different";
  application::RetainAnchorHistory(f.watch, retained);
  EXPECT_EQ(f.watch.recent_positions.size(), 1u);
  f.watch = {};
  application::RetainAnchorHistory(f.watch, retained);
  EXPECT_TRUE(f.watch.recent_positions.empty());
  EXPECT_EQ(retained.recent_positions.size(), 300u);
}
TEST(OpenNavAnchorView, DistanceUnitsPreserveSmallMovementAndUnavailable) {
  Fixture f;
  EXPECT_EQ(f.View().display_distance, 18.);
  EXPECT_EQ(f.View().distance_unit, "m");
  EXPECT_EQ(f.View().distance_decimals, 0);
  for (const auto &unit : {std::pair<double, const char *>{1. / 1852., "NMi"},
                           {1.15078 / 1852., "mi"}, {.001, "km"},
                           {6076.12 / 1852., "ft"}}) {
    f.watch.distance_units_per_m = unit.first;
    f.watch.distance_unit = unit.second;
    EXPECT_EQ(f.View().display_distance, 18. * unit.first);
    EXPECT_EQ(f.View().distance_unit, unit.second);
    EXPECT_EQ(f.View().distance_decimals, unit.first < .01 ? 3 : 0);
    EXPECT_FALSE(f.View(stamp + 6s).display_distance);
    EXPECT_EQ(f.View().distance_m, 18.); // Canonical geometry is unchanged.
  }
  for (double factor : {0., -1., std::numeric_limits<double>::infinity()}) {
    f.watch.distance_units_per_m = factor;
    EXPECT_FALSE(f.View().display_distance);
  }
  f.watch = {};
  EXPECT_FALSE(f.View().display_distance);
}

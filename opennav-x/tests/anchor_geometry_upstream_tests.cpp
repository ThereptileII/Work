#include "integration/AnchorGeometry.h"
#include "application/AnchorView.h"
#include "model/georef.h"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
using namespace opennav;
using namespace std::chrono_literals;
TEST(OpenNavAnchorGeometry, PinnedMercatorProjectionIncludingAntimeridian) {
  const application::Coordinate anchor{58., 179.999};
  for (const auto position : {application::Coordinate{58.0001, 179.999},
                              {57.9999, 179.999},
                              {58., 179.9999},
                              {58., 179.9989},
                              {58., -179.999},
                              {58., 179.999}}) {
    double bearing = 0, distance = 0;
    DistanceBearingMercator(position.latitude_deg, position.longitude_deg,
                            anchor.latitude_deg, anchor.longitude_deg, &bearing,
                            &distance);
    const auto projection = integration::ProjectAnchorPosition(
        anchor, position, {}, "selected GPS");
    ASSERT_TRUE(projection);
    const auto radians = bearing * std::acos(-1.) / 180.;
    EXPECT_NEAR(*projection->east_m, 1852. * distance * std::sin(radians),
                1e-9);
    EXPECT_NEAR(*projection->north_m, 1852. * distance * std::cos(radians),
                1e-9);
    EXPECT_EQ(projection->position.longitude_deg, position.longitude_deg);
    EXPECT_EQ(projection->position_source, "selected GPS");
  }
}
TEST(OpenNavAnchorGeometry, InvalidCoordinatesUnavailable) {
  for (const auto bad : {application::Coordinate{91, 16},
                         {90, 16},
                         {58, 181},
                         {std::numeric_limits<double>::quiet_NaN(), 16},
                         {58, std::numeric_limits<double>::infinity()}}) {
    EXPECT_FALSE(
        integration::ProjectAnchorPosition(bad, {58, 16}, {}, "selected GPS"));
    EXPECT_FALSE(
        integration::ProjectAnchorPosition({58, 16}, bad, {}, "selected GPS"));
  }
  EXPECT_FALSE(integration::ProjectAnchorPosition({58, 16}, {58, 16}, {}, ""));
}
TEST(OpenNavAnchorGeometry, SelectedFixReplayBetweenNormalWatchTicks) {
  const vessel::Time start{100s};
  application::AnchorState watch;
  watch.waypoint_id = "accepted-watch";
  watch.source = "OpenCPN normal anchor watch";
  watch.anchor = application::Coordinate{58., 16.};
  watch.radius_m = 50;
  vessel::VesselState state;
  const auto feed = [&](int step) {
    const auto at = start + step * 100ms;
    state.navigation.latitude_deg =
        {58. + step * .0001, "selected GPS", at, vessel::Validity::Measured};
    state.navigation.longitude_deg =
        {16., "selected GPS", at, vessel::Validity::Measured};
    return at;
  };
  auto now = feed(0);
  watch.observed_at = now;
  integration::ObserveAnchorPosition(watch, state.navigation, now);
  EXPECT_EQ(application::PresentAnchor(watch, state, now).distance_m, 0.);
  double previous_distance = 0;
  // Several fixes arrive between one-second normal anchor-watch hooks. A
  // cached hook snapshot no longer matches, but a read of the accepted watch
  // and current fix must publish each movement without executing the alarm.
  for (int step = 1; step <= 8; ++step) {
    const auto cached = watch;
    now = feed(step);
    EXPECT_FALSE(application::PresentAnchor(cached, state, now).distance_m);
    watch.observed_at = now;
    integration::ObserveAnchorPosition(watch, state.navigation, now);
    application::RetainAnchorHistory(watch, cached);
    const auto view = application::PresentAnchor(watch, state, now);
    ASSERT_TRUE(view.distance_m);
    double bearing = 0, distance = 0;
    DistanceBearingMercator(58., 16., *state.navigation.latitude_deg.value,
                            16., &bearing, &distance);
    EXPECT_NEAR(*view.distance_m, distance * 1852., 1e-9);
    EXPECT_GT(*view.distance_m, previous_distance);
    previous_distance = *view.distance_m;
    EXPECT_EQ(watch.distance_m.observed_at, now);
    EXPECT_EQ(view.vessel_position->observed_at, now);
    EXPECT_FALSE(view.alarm); // Distance never recalculates upstream alarm.
  }
  const auto observed = watch.distance_m.observed_at;
  const auto previous = watch;
  watch.observed_at = now + 1s;
  integration::ObserveAnchorPosition(watch, state.navigation, now + 1s);
  application::RetainAnchorHistory(watch, previous);
  EXPECT_EQ(watch.distance_m.observed_at, observed);
  EXPECT_EQ(watch.recent_positions.size(), previous.recent_positions.size());
  watch.alarm = true;
  integration::ObserveAnchorPosition(watch, state.navigation, now + 6s);
  const auto stale = application::PresentAnchor(watch, state, now + 6s);
  EXPECT_FALSE(stale.distance_m);
  EXPECT_FALSE(stale.vessel_position);
  EXPECT_TRUE(stale.alarm);
}
TEST(OpenNavAnchorGeometry, InvalidFixResetAndMovedWatchDoNotReuseDistance) {
  const vessel::Time now{100s};
  application::AnchorState watch;
  watch.waypoint_id = "accepted-watch";
  watch.source = "OpenCPN normal anchor watch";
  watch.anchor = application::Coordinate{58., 16.};
  watch.observed_at = now;
  vessel::Navigation position;
  position.latitude_deg = {58.001, "selected GPS", now, vessel::Validity::Measured};
  position.longitude_deg = {16., "selected GPS", now, vessel::Validity::Measured};
  integration::ObserveAnchorPosition(watch, position, now);
  application::RetainAnchorHistory(watch, {});
  const auto accepted = watch;
  for (int change = 0; change < 7; ++change) {
    auto invalid = position;
    switch (change) {
    case 0: invalid.latitude_deg = {}; break;
    case 1: invalid.longitude_deg.source = "other"; break;
    case 2: invalid.longitude_deg.observed_at -= 1s; break;
    case 3: invalid.latitude_deg.validity = vessel::Validity::Estimated; break;
    case 4: invalid.latitude_deg.value = 91.; break;
    case 5: invalid.latitude_deg.observed_at = invalid.longitude_deg.observed_at = now + 1s; break;
    case 6: invalid.latitude_deg.observed_at = invalid.longitude_deg.observed_at = now - 6s; break;
    }
    watch = accepted;
    integration::ObserveAnchorPosition(watch, invalid, now);
    EXPECT_FALSE(watch.distance_m.value) << change;
    EXPECT_FALSE(watch.vessel_position) << change;
  }
  watch = accepted;
  watch.anchor = application::Coordinate{58.001, 16.};
  integration::ObserveAnchorPosition(watch, position, now);
  application::RetainAnchorHistory(watch, accepted);
  EXPECT_EQ(watch.distance_m.value, 0.);
  ASSERT_EQ(watch.recent_positions.size(), 1u);
  EXPECT_NEAR(*watch.recent_positions.front().north_m, 0., 1e-9);
  watch = {};
  integration::ObserveAnchorPosition(watch, position, now);
  application::RetainAnchorHistory(watch, accepted);
  EXPECT_FALSE(watch.distance_m.value);
  EXPECT_FALSE(watch.vessel_position);
  EXPECT_TRUE(watch.recent_positions.empty());
}

#include "integration/AnchorGeometry.h"
#include "model/georef.h"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
using namespace opennav;
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

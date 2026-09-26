#include "integration/AisObservationTime.h"
#include <gtest/gtest.h>
#include <limits>

namespace {
using namespace opennav;
using namespace std::chrono_literals;
const vessel::Time epoch{std::chrono::hours(48)};
TEST(OpenNavAisClock, UsesPinnedWxUtcTickConvention) {
  const wxDateTime local(26, wxDateTime::Sep, 2026, 12, 30, 45, 250);
  const auto native_report = local.ToUTC().GetTicks();
  const auto observed = integration::AisObservationAt(native_report, local, epoch);
  ASSERT_TRUE(observed);
  EXPECT_EQ(epoch - 250ms, *observed);
}
TEST(OpenNavAisClock, SameElapsedTimeInUtcAndEastAndWestTimezones) {
  // The decoder and native expiry timer shift BOTH operands. Pair elapsed
  // upstream time with the monotonic clock; never mix these ticks with Unix.
  constexpr std::time_t unix_now = 1790412345;
  for (const auto shift : {0, -2 * 3600, 5 * 3600}) {
    const auto native_now = unix_now + shift;
    const auto observed = integration::AisObservationFromClock(
        native_now - 4, native_now, 250, epoch);
    ASSERT_TRUE(observed);
    EXPECT_EQ(epoch - 4250ms, *observed);
  }
}
TEST(OpenNavAisClock, RetainsStaleObservationAge) {
  const auto at = integration::AisObservationFromClock(100000, 100070, 0, epoch);
  ASSERT_TRUE(at);
  vessel::Sample sample{4.2, "OpenCPN AIS model", *at, vessel::Validity::Measured};
  sample.freshness = {15s, 60s};
  EXPECT_EQ(vessel::Quality::Stale, vessel::Assess(sample, epoch).quality);
}
TEST(OpenNavAisClock, RejectsFutureMissingImplausibleAndOverflowReports) {
  EXPECT_FALSE(integration::AisObservationFromClock(100001, 100000, 0, epoch));
  EXPECT_FALSE(integration::AisObservationFromClock(0, 100000, 0, epoch));
  EXPECT_FALSE(integration::AisObservationFromClock(1, 100000, 0, epoch));
  EXPECT_FALSE(integration::AisObservationFromClock(
      std::numeric_limits<std::time_t>::max(), 100000, 0, epoch));
  EXPECT_FALSE(integration::AisObservationFromClock(
      std::numeric_limits<std::time_t>::min(), 100000, 0, epoch));
  EXPECT_FALSE(integration::AisObservationFromClock(100000, 100000, 1000, epoch));
  EXPECT_FALSE(integration::AisObservationAt(100000, wxDateTime{}, epoch));
}
} // namespace

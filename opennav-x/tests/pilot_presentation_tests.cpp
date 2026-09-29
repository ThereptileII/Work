#include "application/PilotPresentation.h"
#include <gtest/gtest.h>
#include <limits>
using namespace opennav;
using namespace std::chrono_literals;
namespace {
const vessel::Time stamp{100s};
adapters::PilotView Live() {
  adapters::PilotView p;
  p.capabilities = {false, true, true, false, false, true, true};
  p.enabled = p.fresh = true;
  p.feedback.mode = adapters::PilotMode::Auto;
  p.feedback.sequence = 12;
  p.feedback.source = "verified pilot";
  p.feedback.observed_at = stamp;
  p.feedback.heading_magnetic_deg = {143., "verified pilot", stamp,
                                     vessel::Validity::Measured};
  p.feedback.locked_heading_magnetic_deg = {145., "verified pilot", stamp,
                                            vessel::Validity::Measured};
  return p;
}
auto View(const adapters::PilotView &p, vessel::Time now = stamp,
          bool permission = true, bool replay = false) {
  return application::PresentPilot(p, now, permission, replay);
}
} // namespace
TEST(OpenNavPilotPresentation, MeasuredMagneticHeadingAndCommands) {
  auto p = Live();
  auto v = View(p);
  EXPECT_EQ(v.heading_magnetic_deg, 145.);
  EXPECT_EQ(v.actual_heading_magnetic_deg, 143.);
  EXPECT_TRUE(v.commanded);
  EXPECT_TRUE(v.standby);
  EXPECT_TRUE(v.auto_mode);
  EXPECT_TRUE(v.alter_course);
  EXPECT_FALSE(v.track);
  EXPECT_FALSE(v.wind);
  p.feedback.mode = adapters::PilotMode::Standby;
  v = View(p);
  EXPECT_EQ(v.heading_magnetic_deg, 143.);
  EXPECT_FALSE(v.commanded);
  EXPECT_FALSE(v.alter_course);
  p.feedback.heading_magnetic_deg.value = 0.;
  EXPECT_EQ(View(p).heading_magnetic_deg, 0.);
  p.feedback.heading_magnetic_deg = {};
  EXPECT_FALSE(View(p).heading_magnetic_deg);
  EXPECT_FALSE(View(p).auto_mode);
}
TEST(OpenNavPilotPresentation, PendingAndTimeoutNeverInventConfirmation) {
  auto p = Live();
  p.command.state = adapters::CommandState::Pending;
  p.command.request.action = adapters::PilotAction::AlterCourse;
  p.command.request.delta_deg = 10.;
  auto v = View(p);
  EXPECT_EQ(v.heading_magnetic_deg, 145.);
  EXPECT_TRUE(v.pending);
  EXPECT_FALSE(v.alter_course);
  EXPECT_FALSE(v.auto_mode);
  EXPECT_TRUE(v.standby);
  p.command.state = adapters::CommandState::TimedOut;
  v = View(p);
  EXPECT_EQ(v.mode, adapters::PilotMode::Auto);
  EXPECT_EQ(v.heading_magnetic_deg, 145.);
  EXPECT_NE(v.note.find("No confirmation"), std::string::npos);
  EXPECT_FALSE(View(p, stamp + 3s).heading_magnetic_deg);
  EXPECT_TRUE(View(p, stamp + 3s).standby);
}
TEST(OpenNavPilotPresentation, InvalidFreshnessAndHeading) {
  for (int change = 0; change < 7; ++change) {
    auto p = Live();
    switch (change) {
    case 0:
      p.fresh = false;
      break;
    case 1:
      p.feedback.sequence = 0;
      break;
    case 2:
      p.feedback.source.clear();
      break;
    case 3:
      p.feedback.observed_at += 1s;
      break;
    case 4:
      p.feedback.observed_at -= 3s;
      break;
    case 5:
      p.feedback.mode = adapters::PilotMode::Unavailable;
      break;
    case 6:
      p.feedback = {};
      break;
    }
    auto v = View(p);
    EXPECT_FALSE(v.heading_magnetic_deg) << change;
    EXPECT_FALSE(v.auto_mode) << change;
    EXPECT_FALSE(v.alter_course) << change;
    EXPECT_EQ(v.mode, adapters::PilotMode::Unavailable) << change;
  }
  for (int change = 0; change < 8; ++change) {
    auto p = Live();
    auto &h = p.feedback.locked_heading_magnetic_deg;
    switch (change) {
    case 0:
      h.value = std::numeric_limits<double>::quiet_NaN();
      break;
    case 1:
      h.value = std::numeric_limits<double>::infinity();
      break;
    case 2:
      h.value = -1.;
      break;
    case 3:
      h.value = 360.;
      break;
    case 4:
      h.observed_at -= 3s;
      break;
    case 5:
      h.observed_at += 1s;
      break;
    case 6:
      h.validity = vessel::Validity::Estimated;
      break;
    case 7:
      h.value.reset();
      break;
    }
    EXPECT_FALSE(View(p).heading_magnetic_deg) << change;
    EXPECT_FALSE(View(p).alter_course) << change;
  }
}
TEST(OpenNavPilotPresentation, PermissionAndReplay) {
  auto p = Live();
  auto v = View(p, stamp, false);
  EXPECT_TRUE(v.can_toggle); // Always allow disabling an enabled session.
  EXPECT_FALSE(v.standby);
  EXPECT_FALSE(v.auto_mode);
  EXPECT_FALSE(v.alter_course);
  p.enabled = false;
  EXPECT_FALSE(View(p, stamp, false).can_toggle);
  p.capabilities.manual_control = false;
  EXPECT_FALSE(View(p).can_toggle);
  p.capabilities.simulated = true;
  EXPECT_TRUE(View(p, stamp, false).can_toggle);
  p.enabled = true;
  v = View(p, stamp, true, true);
  EXPECT_FALSE(v.enabled);
  EXPECT_FALSE(v.can_toggle);
  EXPECT_FALSE(v.standby);
  EXPECT_FALSE(v.auto_mode);
  EXPECT_FALSE(v.heading_magnetic_deg);
  EXPECT_EQ(v.mode, adapters::PilotMode::Unavailable);
}
TEST(OpenNavPilotPresentation, ProductStatusOnlyOverridesOldPermission) {
  auto p = Live();
  p.output_unavailable = true;
  const auto v = View(p, stamp, true);
  EXPECT_TRUE(v.output_unavailable);
  EXPECT_FALSE(v.enabled);
  EXPECT_FALSE(v.can_toggle);
  EXPECT_FALSE(v.standby);
  EXPECT_FALSE(v.auto_mode);
  EXPECT_FALSE(v.track);
  EXPECT_FALSE(v.wind);
  EXPECT_FALSE(v.alter_course);
  EXPECT_TRUE(v.heading_magnetic_deg);
  EXPECT_EQ(v.mode, adapters::PilotMode::Auto);
  EXPECT_NE(v.note.find("Status only"), std::string::npos);
}

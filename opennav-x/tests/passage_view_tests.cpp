#include "application/PassageView.h"
#include "integration/RouteProgressInput.h"
#include "smartnav/VesselEnergy.h"
#include <iostream>
#include <limits>
#include <stdexcept>
#ifdef OPENNAV_PASSAGE_GTEST
#include <gtest/gtest.h>
#endif
using namespace opennav;
using namespace std::chrono_literals;
namespace {
int checks = 0;
void Check(bool ok, const char *message) {
  ++checks;
  if (!ok)
    throw std::runtime_error(message);
}
const vessel::Time stamp{100s};
vessel::Sample Sample(double n) {
  return {n, "test input", stamp, vessel::Validity::Measured};
}
vessel::VesselState Fixture() {
  integration::RouteRead r;
  r.route.active = true;
  r.route.id = "route";
  r.route.name = "Passage";
  r.route.active_index = 0;
  r.route.active_point_id = "a";
  r.route.points = {{"a", 10, 179.9, 0, "First", {}},
                    {"b", 10, -179.9, 2, "Middle", 32},
                    {"c", 11, -179.9, 3, "Destination", 90}};
  r.position.latitude_deg = Sample(10);
  r.position.longitude_deg = Sample(179.8);
  r.upstream_position_valid = true;
  r.upstream_latitude_deg = 10;
  r.upstream_longitude_deg = 179.8;
  r.range_to_active_nm = 1;
  r.bearing_to_active_true_deg = 0;
  integration::RouteProgressInput input("test");
  input.Complete(r, r, stamp);
  vessel::VesselState s;
  s.navigation = r.position;
  s.navigation.route = input.Current();
  s.navigation.sog_kn = Sample(6);
  s.navigation.cog_deg = Sample(0);
  s.battery.soc_percent = Sample(80);
  s.battery.net_discharge_kw = Sample(2);
  return s;
}
} // namespace
int RunPassageChecks() {
  try {
    auto state = Fixture();
    auto energy = smartnav::PredictVesselEnergy({20, 20, .5, "test capacity"},
                                                state, stamp);
    auto advice = smartnav::Advise(state, energy, {}, stamp);
    auto view = application::PresentPassage(state, advice, energy, stamp);
    Check(view.current && view.active && view.distance_nm == 6,
          "accepted remaining distance copied");
    Check(view.points.size() == 3 && view.points[2].distance_nm == 6,
          "SmartNav cumulative route distance copied");
    Check(view.seconds == 3600 && view.points[0].seconds == 600,
          "existing advisory timing copied");
    Check(view.arrival_soc == 70, "exact energy publication accepted");
    Check(view.points[0].turn_deg == 32 && view.points[0].course_true_deg == 32,
          "upstream stored course retained");
    Check(view.destination == "Destination" && view.points[0].ordinal == 1,
          "owned destination/ordinal");
    for (int kind = 0; kind < 4; ++kind) {
      auto other = advice;
      if (kind == 0)
        other.route_id = "other";
      if (kind == 1)
        ++other.route_revision;
      if (kind == 2)
        other.revision_scope = "another process";
      if (kind == 3)
        other.calculated_at -= 1s;
      const auto rejected =
          application::PresentPassage(state, other, energy, stamp);
      Check(!rejected.distance_nm && !rejected.seconds &&
                !rejected.arrival_soc && rejected.points.empty(),
            "unrelated or older advice withheld");
    }
    for (int kind = 0; kind < 3; ++kind) {
      auto other = energy;
      if (kind == 0)
        other.input_route = std::make_shared<vessel::RouteProgressSnapshot>(
            *state.navigation.route);
      if (kind == 1)
        other.calculated_at -= 1s;
      if (kind == 2)
        other.arrival.estimate->soc_percent =
            std::numeric_limits<double>::infinity();
      Check(
          !application::PresentPassage(state, advice, other, stamp).arrival_soc,
          "old or invalid energy suppressed");
    }
    for (auto when : {stamp + 20s, stamp - 1s}) {
      auto stale = application::PresentPassage(state, advice, energy, when);
      Check(!stale.distance_nm && !stale.seconds && !stale.arrival_soc,
            "stale/future observation cannot appear current");
    }
    auto old = state.navigation.route;
    for (auto status :
         {vessel::RouteState::NoActiveRoute, vessel::RouteState::RouteChanged,
          vessel::RouteState::ActivePointChanged,
          vessel::RouteState::InvalidLeg, vessel::RouteState::AmbiguousPoint,
          vessel::RouteState::MissingPosition}) {
      auto changed = std::make_shared<vessel::RouteProgressSnapshot>(*old);
      changed->state = status;
      state.navigation.route = changed;
      auto bad = application::PresentPassage(state, advice, energy, stamp);
      Check(!bad.distance_nm && !bad.arrival_soc && bad.points.empty(),
            "invalid route never becomes zero arrival");
    }
    state.navigation.route.reset();
    auto missing = application::PresentPassage(state, advice, energy, stamp);
    Check(!missing.active && !missing.distance_nm, "deleted route cleared");
    Check(view.points[1].name == "Middle" && view.distance_nm == 6,
          "owned view survives deletion");
    state = Fixture();
    state.navigation.latitude_deg = {};
    advice = smartnav::Advise(state, energy, {}, stamp);
    Check(
        !application::PresentPassage(state, advice, energy, stamp).distance_nm,
        "lost current GPS suppresses retained unexpired route distance");
    state = Fixture();
    state.navigation.sog_kn = Sample(.1);
    advice = smartnav::Advise(state, energy, {}, stamp);
    auto stopped = application::PresentPassage(state, advice, energy, stamp);
    Check(stopped.current && !stopped.seconds && !stopped.points[0].turn_deg,
          "slow vessel retains distance not timing");
    std::cout << "PASS " << checks << " passage provenance checks\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
#ifdef OPENNAV_PASSAGE_GTEST
TEST(OpenNavPassage, CurrentObservationProvenance) {
  EXPECT_EQ(RunPassageChecks(), 0);
}
#else
int main() { return RunPassageChecks(); }
#endif

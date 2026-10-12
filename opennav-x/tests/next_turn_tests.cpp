#include "application/PassageView.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace opennav;
using namespace opennav::application;
using namespace std::chrono_literals;
namespace {
int checks = 0;
void Check(bool ok, const char *why) { ++checks; if (!ok) throw std::runtime_error(why); }
const vessel::Time stamp{100s};
vessel::Sample Sample(double value) {
  return {value, "test input", stamp, vessel::Validity::Measured};
}
vessel::VesselState Fixture(std::optional<double> next_course = 75) {
  auto route = std::make_shared<vessel::RouteProgressSnapshot>();
  route->route_id = "route";
  route->revision_scope = "test";
  route->route_revision = 1;
  route->active_waypoint_id = "a";
  route->active_waypoint_index = 0;
  route->waypoint_count = 2;
  route->remaining_distance_nm = 2.8;
  route->observed_at = stamp;
  route->position_observed_at = stamp;
  route->state = vessel::RouteState::Valid;
  route->source = "test route progress";
  route->position_source = "test input";
  route->remaining_steps = {{"a", "L\xC3\xA5ngholmen", 10, 179.9, .7, 43},
                             {"b", "Ark\xC3\xB6sund", 10, -179.9, 2.1, next_course}};
  vessel::VesselState state;
  state.navigation.latitude_deg = Sample(10);
  state.navigation.longitude_deg = Sample(179.8);
  state.navigation.route = route;
  state.navigation.sog_kn = Sample(6);
  state.navigation.cog_deg = Sample(43);
  return state;
}
std::size_t Turns(const smartnav::NavigationAdvice &advice) {
  std::size_t count = 0;
  for (const auto &event : advice.events)
    if (event.kind == smartnav::EventKind::Turn) ++count;
  return count;
}
void Pipeline() {
  auto state = Fixture();
  auto advice = smartnav::Advise(state, {}, {}, stamp);
  auto passage = PresentPassage(state, advice, {}, stamp);
  auto view = PresentNextTurn(passage);
  Check(passage.current && view.visible && view.headline == "Starboard" &&
            view.turn_deg == 32 && Turns(advice) == 1,
        "computed turn reaches the card through advice and passage");
  Check(view.detail == "0.7 nm \xC2\xB7 in 7 min \xC2\xB7 new course 075\xC2\xB0",
        "pipeline retains course and timing");
  for (const auto change : {0.0, .5, -.5}) {
    state = Fixture(43 + change);
    advice = smartnav::Advise(state, {}, {}, stamp);
    passage = PresentPassage(state, advice, {}, stamp);
    view = PresentNextTurn(passage);
    Check(advice.route_valid && Turns(advice) == 0 &&
              advice.events.size() == 2 &&
              advice.events[0].kind == smartnav::EventKind::Waypoint &&
              advice.events[0].course_change_deg &&
              std::abs(*advice.events[0].course_change_deg - change) < 1e-9 &&
              advice.events[0].course_true_deg == 43 + change,
          "measured straight/small change stays on waypoint without Turn event");
    Check(passage.points[0].turn_deg == advice.events[0].course_change_deg &&
              passage.points[0].course_true_deg == 43 + change,
          "passage retains available zero/subdegree maneuver fields");
    Check(view.visible && view.headline == "Straight on" && !view.turn_deg &&
              view.detail.find("new course") != std::string::npos,
          "available zero/subdegree maneuver displays straight with course");
  }
  for (int missing = 0; missing < 4; ++missing) {
    state = Fixture(missing == 2 ? std::nullopt : std::optional<double>{75});
    if (missing == 0) state.navigation.sog_kn = Sample(0);
    if (missing == 1) state.navigation.cog_deg = {};
    if (missing == 3) state.navigation.cog_deg.validity = vessel::Validity::Uncertain;
    advice = smartnav::Advise(state, {}, {}, stamp);
    passage = PresentPassage(state, advice, {}, stamp);
    Check(advice.route_valid && Turns(advice) == 0 &&
              !advice.events[0].course_change_deg && !advice.events[0].course_true_deg,
          "stopped or unavailable course never emits maneuver or Turn event");
    Check(passage.current && !passage.points[0].turn_deg &&
              !passage.points[0].course_true_deg && !PresentNextTurn(passage).visible,
          "unavailable maneuver hides card instead of fabricating straight");
    Check(passage.points[0].seconds.has_value() == (missing != 0),
          "stopped timing withheld while moving route timing remains available");
  }
  for (const auto invalid : {std::numeric_limits<double>::quiet_NaN(), -1.0, 361.0}) {
    state = Fixture(invalid);
    advice = smartnav::Advise(state, {}, {}, stamp);
    passage = PresentPassage(state, advice, {}, stamp);
    Check(advice.route_valid && Turns(advice) == 0 &&
              !passage.points[0].turn_deg && !PresentNextTurn(passage).visible,
          "invalid stored course cannot fabricate straight guidance");
  }
}
PassageView Current() {
  PassageView p;
  p.active = p.current = true;
  PassagePointView a; a.name = "L\xC3\xA5ngholmen"; a.distance_nm = .7; a.seconds = 420;
  a.turn_deg = 32; a.course_true_deg = 75;
  PassagePointView b; b.name = "Ark\xC3\xB6sund"; b.distance_nm = 2.1;
  p.points = {a, b};
  return p;
}
}  // namespace
int main() {
  try {
    auto v = PresentNextTurn(Current());
    Check(v.visible && v.eyebrow == "NEXT \xC2\xB7 L\xC3\x85NGHOLMEN", "eyebrow names the waypoint");
    Check(v.headline == "Starboard" && v.turn_deg == 32 && v.direction == 1, "starboard 32");
    Check(v.detail == "0.7 nm \xC2\xB7 in 7 min \xC2\xB7 new course 075\xC2\xB0", "detail line");
    auto p = Current(); p.points[0].turn_deg = -48;
    Check(PresentNextTurn(p).headline == "Port" && PresentNextTurn(p).turn_deg == 48, "port");
    p.points[0].turn_deg = 2;
    Check(PresentNextTurn(p).headline == "Straight on" && !PresentNextTurn(p).turn_deg, "small turn");
    p.points.resize(1);
    v = PresentNextTurn(p);
    Check(v.headline == "Arrival" && v.detail.find("new course") == std::string::npos, "last point");
    p = Current(); p.current = false;
    Check(!PresentNextTurn(p).visible, "stale progress hides the card");
    p = Current(); p.points[0].distance_nm.reset();
    Check(!PresentNextTurn(p).visible, "no distance, no card");
    p = Current(); p.points[0].seconds.reset();
    Check(PresentNextTurn(p).detail == "0.7 nm \xC2\xB7 new course 075\xC2\xB0", "no ETA without speed");
    p = Current(); p.points[0].turn_deg.reset();
    Check(!PresentNextTurn(p).visible, "missing maneuver is not straight");
    p = Current(); p.points[0].course_true_deg.reset();
    Check(!PresentNextTurn(p).visible, "missing outgoing course hides maneuver");
    p.points.resize(1);
    Check(PresentNextTurn(p).visible && PresentNextTurn(p).headline == "Arrival",
          "arrival does not require an outgoing course");
    for (const auto invalid : {std::numeric_limits<double>::quiet_NaN(), -181.0, 181.0}) {
      p = Current(); p.points[0].turn_deg = invalid;
      Check(!PresentNextTurn(p).visible, "invalid maneuver angle hides card");
    }
    for (const auto invalid : {std::numeric_limits<double>::quiet_NaN(), -1.0, 360.0}) {
      p = Current(); p.points[0].course_true_deg = invalid;
      Check(!PresentNextTurn(p).visible, "invalid outgoing course hides card");
    }
    Pipeline();
    std::cout << checks << " next turn checks passed\n";
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}

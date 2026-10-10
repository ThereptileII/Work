#include "application/PassageView.h"
#include <iostream>
#include <stdexcept>
using namespace opennav::application;
namespace {
int checks = 0;
void Check(bool ok, const char *why) { ++checks; if (!ok) throw std::runtime_error(why); }
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
    std::cout << checks << " next turn checks passed\n";
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}

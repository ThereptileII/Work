#include "application/RailVisuals.h"
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace std::chrono_literals;
namespace {
int checks = 0;
void Check(bool ok, const char *why) { ++checks; if (!ok) throw std::runtime_error(why); }
vessel::Sample Live(double v, vessel::Time t) {
  return {v, "test", t, vessel::Validity::Measured};
}
}  // namespace
int main() {
  try {
    const vessel::Time t{1000s};
    vessel::VesselState s;
    application::RailHistory sog;
    // Sparkline needs three spaced live points; a gap clears the history.
    for (int i = 0; i < 3; ++i) {
      s.navigation.sog_kn = Live(5 + i, t + i * 10s);
      sog.Observe(s.navigation.sog_kn, t + i * 10s);
    }
    sog.Observe(Live(9, t + 25s), t + 25s);  // Too soon: not a new point.
    Check(sog.Values() == std::vector<double>({5, 6, 7}), "one point per interval");
    Check(application::RailVisualFor("sog", s, t + 20s, sog, {}, nullptr).kind ==
              application::RailVisual::Kind::Sparkline, "SOG sparkline");
    sog.Observe(vessel::Sample{}, t + 40s);
    Check(sog.Values().empty(), "lost SOG breaks the line");
    Check(application::RailVisualFor("sog", s, t, sog, {}, nullptr).kind ==
              application::RailVisual::Kind::None, "no sparkline without history");
    // Depth meter: safety mark and warning only with a configured safety depth.
    s.environment.depth_below_transducer_m = Live(8.4, t);
    auto d = application::RailVisualFor("depth", s, t, sog, 3.5, nullptr);
    Check(d.kind == application::RailVisual::Kind::Meter && !d.warning &&
              d.marker > .25 && d.marker < .35 && d.fraction > d.marker,
          "depth meter with safety mark near 30 %");
    Check(d.caption == "3.5 m safety", "safety caption");
    s.environment.depth_below_transducer_m = Live(1.2, t);
    Check(application::RailVisualFor("depth", s, t, sog, 3.5, nullptr).warning,
          "shallower than safety warns");
    auto plain = application::RailVisualFor("depth", s, t, sog, std::nullopt, nullptr);
    Check(plain.marker < 0 && plain.caption.empty(), "no invented safety mark");
    // Wind: angle and side from the measured angle only.
    s.wind.apparent_angle_deg = Live(-72, t);
    auto w = application::RailVisualFor("aws", s, t, sog, {}, nullptr);
    Check(w.kind == application::RailVisual::Kind::Direction && w.caption == "PORT" &&
              w.value_text == "72\xC2\xB0", "apparent wind on port");
    s.wind.true_angle_deg = Live(40, t);
    Check(application::RailVisualFor("tws", s, t, sog, {}, nullptr).caption == "STBD",
          "true wind on starboard");
    s.wind.apparent_angle_deg = Live(-72, t - 60s);
    Check(application::RailVisualFor("aws", s, t, sog, {}, nullptr).kind ==
              application::RailVisual::Kind::None, "stale angle draws nothing");
    // Battery: bar from SOC; "At destination" only with a prediction.
    s.battery.soc_percent = Live(68, t);
    auto b = application::RailVisualFor("soc", s, t, sog, {}, nullptr);
    Check(b.kind == application::RailVisual::Kind::Bar && b.fraction == .68 && b.caption.empty(),
          "battery bar without prediction");
    smartnav::EnergyPrediction p;
    p.arrival.estimate = smartnav::ArrivalEstimate{2.9, 6, 43., 0, false};
    b = application::RailVisualFor("soc", s, t, sog, {}, &p);
    Check(b.caption == "At destination" && b.value_text == "43%", "arrival SOC");
    Check(application::RailVisualFor("heading", s, t, sog, {}, nullptr).kind ==
              application::RailVisual::Kind::None, "other keys draw nothing");
    std::cout << checks << " rail visual checks passed\n";
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}

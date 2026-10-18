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
    sog.Observe(s.navigation.sog_kn, t + 21s);
    Check(sog.Values().size() == 3, "reading again does not create an observation");
    auto other_source = Live(8, t + 30s);
    other_source.source = "other receiver";
    sog.Observe(other_source, t + 30s);
    Check(sog.Values() == std::vector<double>{8}, "source switch starts a new history");
    sog.Observe(vessel::Sample{}, t + 40s);
    Check(sog.Values().empty(), "lost SOG breaks the line");
    Check(application::RailVisualFor("sog", s, t, sog, {}, nullptr).kind ==
              application::RailVisual::Kind::None, "no sparkline without history");
    // Transducer depth has no configured surface offset in this snapshot.
    // A chart safety depth is not a comparable measurement datum.
    s.environment.depth_below_transducer_m = Live(8.4, t);
    auto d = application::RailVisualFor("depth", s, t, sog, 3.5, nullptr);
    Check(d.kind == application::RailVisual::Kind::Meter && !d.warning &&
              d.marker < 0 && d.fraction > 0 && d.caption.empty(),
          "transducer measurement has no invented chart safety comparison");
    s.environment.depth_below_transducer_m = Live(1.2, t);
    Check(!application::RailVisualFor("depth", s, t, sog, 3.5, nullptr).warning,
          "different depth datums do not create a warning");
    auto plain = application::RailVisualFor("depth", s, t, sog, std::nullopt, nullptr);
    Check(plain.marker < 0 && plain.caption.empty(), "no invented safety mark");
    // Wind: angle and side from the measured angle only.
    s.wind.apparent_angle_deg = Live(-72, t);
    s.wind.apparent_speed_kn = Live(17.1, t);
    auto w = application::RailVisualFor("aws", s, t, sog, {}, nullptr);
    Check(w.kind == application::RailVisual::Kind::Direction && w.caption == "PORT" &&
              w.value_text == "72\xC2\xB0", "apparent wind on port");
    s.wind.true_angle_deg = Live(40, t);
    s.wind.true_speed_kn = Live(14.3, t);
    Check(application::RailVisualFor("tws", s, t, sog, {}, nullptr).caption == "STBD",
          "true wind on starboard");
    s.wind.true_speed_kn.source = "another wind source";
    Check(application::RailVisualFor("tws", s, t, sog, {}, nullptr).kind ==
              application::RailVisual::Kind::None, "different wind sources do not become one direction");
    s.wind.true_speed_kn = {};
    Check(application::RailVisualFor("tws", s, t, sog, {}, nullptr).kind ==
              application::RailVisual::Kind::None, "missing wind speed suppresses a paired visual");
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
    s.battery.soc_percent.validity = vessel::Validity::Estimated;
    Check(application::RailVisualFor("soc", s, t, sog, {}, nullptr).fraction == .68,
          "a labelled current SOC estimate retains its quantitative bar");
    s.battery.soc_percent.observed_at = t - 60s;
    Check(application::RailVisualFor("soc", s, t, sog, {}, &p).kind ==
              application::RailVisual::Kind::None, "stale SOC cannot draw a current bar");
    Check(application::RailVisualFor("heading", s, t, sog, {}, nullptr).kind ==
              application::RailVisual::Kind::None, "other keys draw nothing");
    std::cout << checks << " rail visual checks passed\n";
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}

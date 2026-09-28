#include "application/InstrumentView.h"
#include "application/Settings.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#ifdef OPENNAV_INSTRUMENT_GTEST
#include <gtest/gtest.h>
#endif
using namespace opennav;
using namespace std::chrono_literals;
namespace {
const vessel::Time stamp{100s};
int checks = 0;
void Check(bool ok, const char *message) {
  ++checks;
  if (!ok)
    throw std::runtime_error(message);
}
vessel::Sample Sample(double value) {
  return {value, "NMEA2000/test/source-0", stamp, vessel::Validity::Measured};
}
vessel::VesselState Fixture() {
  vessel::VesselState s;
  s.navigation.heading_true_deg = Sample(41);
  s.navigation.cog_deg = Sample(43);
  s.navigation.sog_kn = Sample(6.3);
  s.wind.true_angle_deg = Sample(-72);
  s.wind.true_angle_deg.validity = vessel::Validity::Estimated;
  s.wind.true_speed_kn = Sample(14.3);
  s.environment.depth_below_transducer_m = Sample(8.4);
  return s;
}
const application::InstrumentReading &Tile(const application::InstrumentView &v,
                                           const char *key) {
  for (const auto &t : v.tiles)
    if (t.key == key)
      return t;
  throw std::runtime_error("missing instrument");
}
} // namespace
int RunInstrumentChecks() {
  try {
    const auto keys = application::Settings{}.instruments;
    auto s = Fixture();
    auto view = application::PresentInstruments(s, keys, stamp);
    Check(view.wind_bearing_true_deg == 329,
          "north-up wind uses heading and signed relative angle");
    Check(view.heading.value == 41 && Tile(view, "cog").value == 43,
          "heading never substituted by COG");
    Check(view.true_angle.quality == vessel::Quality::Estimated,
          "true wind retains estimated provenance");
    Check(view.tiles.size() == 10 && view.tiles[0].key == "sog" &&
              view.tiles[7].key == "water_temp",
          "prototype order and additional configured fields preserved");
    Check(Tile(view, "depth").title == "DEPTH / TRANSDUCER",
          "transducer depth never called surface or keel depth");
    Check(view.heading.age == 0ms &&
              view.heading.source == s.navigation.heading_true_deg.source,
          "copied provenance");
    auto later = application::PresentInstruments(s, keys, stamp + 3s);
    Check(later.heading.age == 3s &&
              later.heading.quality == vessel::Quality::Aging &&
              later.heading.value == 41,
          "read ages existing sample");
    later = application::PresentInstruments(s, keys, stamp + 5s);
    Check(!later.heading.value && !later.wind_bearing_true_deg &&
              later.heading.quality == vessel::Quality::Stale,
          "stale suppresses directional indication");
    Check(!Tile(later, "depth").value &&
              Tile(later, "depth").quality == vessel::Quality::Stale,
          "no apparently live frozen sensor value");
    later = application::PresentInstruments(s, keys, stamp - 1ms);
    Check(!later.heading.value && !later.wind_bearing_true_deg &&
              later.heading.quality == vessel::Quality::Uncertain,
          "future observation withheld");
    s.navigation.heading_true_deg = {};
    view = application::PresentInstruments(s, keys, stamp);
    Check(!view.heading.value && !view.wind_bearing_true_deg &&
              view.true_angle.value == -72,
          "relative wind alone cannot become north-up bearing");
    s = Fixture();
    s.wind.true_angle_deg = {};
    view = application::PresentInstruments(s, keys, stamp);
    Check(view.heading.value == 41 && !view.wind_bearing_true_deg,
          "heading alone does not imply wind direction");
    s = Fixture();
    s.wind.true_angle_deg.observed_at -= 2001ms;
    view = application::PresentInstruments(s, keys, stamp);
    Check(view.heading.value && view.true_angle.value &&
              !view.wind_bearing_true_deg,
          "incoherent times suppress graphic without erasing valid individual "
          "observations");
    for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(), -1., 361.}) {
      s = Fixture();
      s.navigation.heading_true_deg.value = bad;
      view = application::PresentInstruments(s, keys, stamp);
      Check(!view.heading.value && !view.wind_bearing_true_deg,
            "nonfinite or invalid heading withheld");
    }
    for (double bad : {-181., 181., std::numeric_limits<double>::infinity()}) {
      s = Fixture();
      s.wind.true_angle_deg.value = bad;
      view = application::PresentInstruments(s, keys, stamp);
      Check(!view.true_angle.value && !view.wind_bearing_true_deg,
            "invalid relative angle withheld");
    }
    s = Fixture();
    s.navigation.heading_true_deg.validity = vessel::Validity::Uncertain;
    view = application::PresentInstruments(s, keys, stamp);
    Check(!view.heading.value &&
              view.heading.quality == vessel::Quality::Uncertain,
          "uncertainty cannot imply directional certainty");
    s = Fixture();
    s.navigation.heading_true_deg.freshness = {5s, 2s};
    Check(!application::PresentInstruments(s, keys, stamp).heading.value,
          "malformed freshness cannot throw through UI");
    s = Fixture();
    s.environment.depth_below_transducer_m.value = 0;
    Check(
        Tile(application::PresentInstruments(s, keys, stamp), "depth").value ==
            0,
        "measured zero retained");
    s.environment.depth_below_transducer_m.source.clear();
    Check(!Tile(application::PresentInstruments(s, keys, stamp), "depth").value,
          "missing provenance is unavailable");
    view = application::PresentInstruments(Fixture(), {"sog", "unknown", "sog"},
                                           stamp);
    Check(view.tiles.size() == 1 && !view.heading.selected &&
              !view.heading.value && !view.wind_bearing_true_deg,
          "selection deduplicates and suppresses hidden fields");
    s = Fixture();
    view = application::PresentInstruments(s, keys, stamp);
    s = {};
    Check(view.heading.value == 41 && view.true_speed.value == 14.3,
          "view owns copied data after input destruction");
    std::cout << "PASS " << checks << " instrument provenance checks\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
#ifdef OPENNAV_INSTRUMENT_GTEST
TEST(OpenNavInstruments, CurrentDirectionalProvenance) {
  EXPECT_EQ(RunInstrumentChecks(), 0);
}
#else
int main() { return RunInstrumentChecks(); }
#endif

#include "vessel/SensorRegistry.h"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace opennav::vessel;
using namespace std::chrono_literals;
void Check(bool v, const char *s) {
  if (!v)
    throw std::runtime_error(s);
}
const Time t{100s};
SensorObservation Depth(const std::string &source, double value, Time at,
                        unsigned priority = 10) {
  return {Quantity::Depth, source,
          Sample{value,
                 source + " / depth below transducer",
                 at,
                 Validity::Measured,
                 {},
                 "transducer-1"},
          priority};
}
void TestSelection() {
  SensorRegistry r;
  Check(!r.Select(Quantity::Depth, t).sample.value,
        "Missing source is not zero");
  r.Observe(Depth("NMEA0183/DPT", 8, t, 30), t);
  r.Observe(Depth("NMEA2000/128267", 9, t, 10), t);
  Check(r.Select(Quantity::Depth, t).sample.value == 9, "Protocol precedence");
  r.Observe(Depth("NMEA0183/DPT", 7, t + 4s, 30), t + 4s);
  auto fallback = r.Select(Quantity::Depth, t + 5s);
  Check(
      fallback.sample.value == 7 && fallback.sample.observed_at == t + 4s,
      "Aging selected source falls back to fresh alternate with its real age");
  r.Configure(Quantity::Depth, {"NMEA2000/128267", {}});
  auto pinned = r.Select(Quantity::Depth, t + 5s);
  Check(pinned.sample.value == 9 &&
            Assess(pinned.sample, t + 5s).quality == Quality::Stale,
        "Pinned source must not silently fall back");
  r.Configure(Quantity::Depth, {"not connected", {}});
  Check(!r.Select(Quantity::Depth, t + 5s).sample.value,
        "Absent pinned source");
  r.Configure(Quantity::Depth, {"", {3s, 8s}});
  Check(Assess(r.Select(Quantity::Depth, t + 7s).sample, t + 7s).quality ==
            Quality::Aging,
        "Quantity freshness is retained by the selected sample");
  const auto stamp = r.Select(Quantity::Depth, t + 7s).sample.observed_at;
  r.Health(t + 20s);
  r.Select(Quantity::Depth, t + 20s);
  Check(r.Select(Quantity::Depth, t + 20s).sample.observed_at == stamp,
        "Diagnostics reads do not renew age");
  VesselState nav;
  nav.navigation.sog_kn = {6, "OpenCPN selected", t, Validity::Measured};
  nav.navigation.latitude_deg = {50, "OpenCPN selected", t, Validity::Measured};
  auto merged = r.Merge(nav, t);
  Check(merged.navigation.sog_kn.value == 6 &&
            merged.navigation.latitude_deg.value == 50,
        "Instrument precedence cannot override selected navigation");
  auto retained = r.Select(Quantity::Depth, t);
  r.Clear();
  Check(retained.sample.value == 9 &&
            !r.Select(Quantity::Depth, t).sample.value,
        "Owned sample survives source destruction");
}
void TestValidity() {
  SensorRegistry r;
  Check(r.Observe(Depth("a", 0, t), t) == Admission::Accepted,
        "Measured zero depth valid");
  Check(r.Select(Quantity::Depth, t).sample.value == 0,
        "Zero stays distinct from missing");
  Check(r.Observe(Depth("a", 99, t - 1s), t) == Admission::Old,
        "Old observation rejected");
  Check(r.Observe(Depth("a", 99, t + 1s), t) == Admission::Future,
        "Future observation rejected");
  Check(r.Select(Quantity::Depth, t).sample.value == 0,
        "Rejected packet cannot replace good observation");
  Check(r.Observe(Depth("a", -1, t + 1s), t + 1s) == Admission::InvalidValue,
        "Invalid depth invalidates source");
  Check(!r.Select(Quantity::Depth, t + 1s).sample.value, "Invalid is not zero");
  for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(), 12001.0}) {
    Check(r.Observe(Depth("a", invalid, t + 2s), t + 2s) ==
              Admission::InvalidValue,
          "Nonfinite/domain validation");
    Check(!r.Select(Quantity::Depth, t + 2s).sample.value,
          "No nonfinite reading");
  }
  Check(r.Observe(Depth("", 8, t), t) == Admission::MissingSource,
        "Source identity mandatory");
  r.Clear();
  for (int n = 0; n < 32; ++n)
    r.Observe(Depth(std::to_string(n), n, t), t);
  Check(r.Observe(Depth("overflow", 4, t), t) == Admission::Full,
        "Candidate memory bounded");
  Check(r.Health(t).size() == 32, "No hidden unbounded candidates");
  bool invalid = false;
  try {
    r.Configure(Quantity::Depth, {"", {5s, 5s}});
  } catch (const std::invalid_argument &) {
    invalid = true;
  }
  Check(invalid, "Invalid freshness policy rejected");
}
void TestBattery() {
  VesselState s;
  s.battery.voltage_v = {400, "N2K voltage", t, Validity::Measured,
                         {},  "battery-1"};
  s.battery.current_native_a = {20, "N2K current", t, Validity::Measured,
                                {}, "battery-1"};
  NormalizeBatteryPower(s, "battery-1", CurrentConvention::Unconfigured, t);
  Check(!s.battery.net_discharge_kw.value && !s.battery.current_a.value,
        "No guessed current sign");
  NormalizeBatteryPower(s, "battery-1", CurrentConvention::PositiveDischarge,
                        t);
  Check(s.battery.net_discharge_kw.value == 8 &&
            s.battery.current_a.value == 20,
        "Coherent whole-pack conversion");
  Check(s.battery.net_discharge_kw.validity == Validity::Estimated,
        "Derived power labels its provenance");
  NormalizeBatteryPower(s, "battery-1", CurrentConvention::PositiveCharge, t);
  Check(s.battery.net_discharge_kw.value == -8,
        "Charging convention preserved");
  s.battery.voltage_v.device_id = "battery-2";
  NormalizeBatteryPower(s, "battery-1", CurrentConvention::PositiveDischarge,
                        t);
  Check(!s.battery.net_discharge_kw.value,
        "Never multiply voltage from another battery");
  s.battery.voltage_v.device_id = "battery-1";
  s.battery.voltage_v.observed_at = t - 1s;
  NormalizeBatteryPower(s, "battery-1", CurrentConvention::PositiveDischarge,
                        t);
  Check(!s.battery.net_discharge_kw.value,
        "Different measurement epochs are not coherent");
  s.battery.voltage_v.observed_at = t;
  NormalizeBatteryPower(s, "battery-1", CurrentConvention::PositiveDischarge,
                        t + 5s);
  Check(!s.battery.net_discharge_kw.value, "Stale power is withheld");
  s.battery.current_native_a.value = 0;
  NormalizeBatteryPower(s, "battery-1", CurrentConvention::PositiveDischarge,
                        t);
  Check(s.battery.net_discharge_kw.value == 0,
        "Measured zero current is real zero power");
  s.battery.voltage_v.value = std::numeric_limits<double>::max();
  s.battery.current_native_a.value = std::numeric_limits<double>::max();
  NormalizeBatteryPower(s, "battery-1", CurrentConvention::PositiveDischarge,
                        t);
  Check(!s.battery.net_discharge_kw.value,
        "Overflow withheld even without decoder domain guard");
}
int main(int argc, char **argv) {
  try {
    Check(argc == 2, "Choose test group");
    std::string group = argv[1];
    if (group == "selection")
      TestSelection();
    else if (group == "validity")
      TestValidity();
    else if (group == "battery")
      TestBattery();
    else
      throw std::runtime_error("Unknown group");
    std::cout << group << " passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

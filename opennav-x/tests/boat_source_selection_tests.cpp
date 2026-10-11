#include "adapters/BoatN2k.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace std::chrono_literals;
using vessel::Quantity;
namespace {
int checks = 0;
void Check(bool value, const char *why) {
  ++checks;
  if (!value) throw std::runtime_error(why);
}
const vessel::Time epoch{100s};
const adapters::BoatN2kBinding binding{"test-n2k", "40328200ffd23456"};
const std::string identity = binding.interface_id + "/NAME-" + binding.name;
const std::string device = "NMEA2000/" + identity + "/source-35/instance-0";
const std::string preferred = device + "/PGN-127506";
const std::string alternate = "SignalK/independent-pack/SOC";
vessel::SensorObservation Observation(bool boat, vessel::Time at) {
  const auto &source = boat ? preferred : alternate;
  return {Quantity::BatterySoc, source,
          {boat ? 68.0 : 67.0, source, at, vessel::Validity::Measured, {},
           boat ? device : "independent-pack"}, boat ? 10u : 30u};
}
void Test() {
  adapters::BoatN2k adapter;
  adapter.Configure(binding);
  vessel::SensorRegistry sources;
  const auto heartbeat = [&](vessel::Time at, unsigned char flags) {
    adapter.Observe(identity, 35, 61184,
                    {2, 2, 2, flags, 255, 255, 255, 255}, at, at);
  };
  heartbeat(epoch, 0xf1);
  sources.Observe(Observation(true, epoch + 1ms), epoch + 1ms);
  sources.Observe(Observation(false, epoch + 2ms), epoch + 2ms);
  vessel::VesselState navigation;
  navigation.navigation.latitude_deg = {58, "OpenCPN selected GPS", epoch,
                                        vessel::Validity::Measured};
  navigation.navigation.longitude_deg = {18, "OpenCPN selected GPS", epoch,
                                         vessel::Validity::Measured};
  navigation.navigation.sog_kn = {3, "OpenCPN selected SOG", epoch,
                                  vessel::Validity::Measured};
  navigation.navigation.cog_deg = {90, "OpenCPN selected COG", epoch,
                                   vessel::Validity::Measured};
  const auto check = [&](vessel::Time now, bool boat, bool available,
                         vessel::Validity validity, vessel::Time observation) {
    const auto state = adapter.Merge(sources, navigation, now);
    const auto &value = state.battery.soc_percent;
    Check(value.device_id == (boat ? device : "independent-pack"),
          "Select producer-valid candidate before choosing winner");
    Check(value.value.has_value() == available && value.validity == validity,
          "Selected value has current producer validity");
    Check(value.observed_at == observation, "Read preserves original sample time");
    Check(state.navigation.latitude_deg.value == 58 &&
          state.navigation.longitude_deg.value == 18 &&
          state.navigation.sog_kn.value == 3 && state.navigation.cog_deg.value == 90 &&
          state.navigation.latitude_deg.observed_at == epoch &&
          state.navigation.sog_kn.source == "OpenCPN selected SOG",
          "Producer selection never replaces OpenCPN navigation");
    const auto health = adapter.Health(sources, now);
    Check(health.size() == 2, "Both candidate diagnostics retained");
    const auto selected = std::find_if(health.begin(), health.end(),
                                     [](const auto &h) { return h.selected; });
    Check(selected != health.end() && selected->source_id == (boat ? preferred : alternate),
          "Health selection agrees with merged output");
    Check(selected->sample.value == value.value &&
          selected->sample.validity == validity &&
          selected->sample.observed_at == observation,
          "Health selected state preserves current validity and original age");
    Check(std::count_if(health.begin(), health.end(),
                        [](const auto &h) { return h.selected; }) == 1,
          "Exactly one candidate selected");
  };
  check(epoch + 3ms, true, true, vessel::Validity::Measured, epoch + 1ms);
  check(epoch + 499ms, true, true, vessel::Validity::Measured, epoch + 1ms);
  check(epoch + 500ms, false, true, vessel::Validity::Measured, epoch + 2ms);
  sources.Configure(Quantity::BatterySoc, {preferred, {}});
  check(epoch + 500ms, true, true, vessel::Validity::Uncertain, epoch + 1ms);
  sources.Configure(Quantity::BatterySoc, {});
  // A fresh heartbeat explicitly expires the power/SOC group while other groups live.
  heartbeat(epoch + 501ms, 0xe1);
  check(epoch + 502ms, false, true, vessel::Validity::Measured, epoch + 2ms);
  sources.Configure(Quantity::BatterySoc, {preferred, {}});
  check(epoch + 502ms, true, false, vessel::Validity::Invalid, epoch + 1ms);
  sources.Configure(Quantity::BatterySoc, {});
  heartbeat(epoch + 503ms, 0xf1);
  check(epoch + 504ms, false, true, vessel::Validity::Measured, epoch + 2ms);
  sources.Configure(Quantity::BatterySoc, {preferred, {}});
  check(epoch + 504ms, true, false, vessel::Validity::Invalid, epoch + 1ms);
  sources.Configure(Quantity::BatterySoc, {});
  // No expiry/health read can alter the retained measured observation.
  const auto retained = sources.Select(Quantity::BatterySoc, epoch + 504ms).sample;
  Check(retained.value == 68 && retained.validity == vessel::Validity::Measured &&
        retained.observed_at == epoch + 1ms && retained.source == preferred,
        "Producer assessment never mutates retained observations or provenance");
  sources.Observe(Observation(true, epoch + 505ms), epoch + 505ms);
  check(epoch + 506ms, true, true, vessel::Validity::Measured, epoch + 505ms);
  check(epoch + 507ms, true, true, vessel::Validity::Measured, epoch + 505ms);
  sources.Configure(Quantity::BatterySoc, {"missing source", {}});
  Check(!adapter.Merge(sources, navigation, epoch + 508ms).battery.soc_percent.value,
        "Absent pinned source cannot fall back");
  const auto missing = adapter.Health(sources, epoch + 508ms);
  Check(std::none_of(missing.begin(), missing.end(), [](const auto &h) { return h.selected; }),
        "Missing pinned source has no selected diagnostic candidate");
}
} // namespace
int main() {
  try {
    Test();
    std::cout << checks << " producer selection checks passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Check " << checks << ": " << e.what() << '\n';
    return 1;
  }
}

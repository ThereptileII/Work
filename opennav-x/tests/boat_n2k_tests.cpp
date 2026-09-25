#include "adapters/BoatN2k.h"
#include "application/Settings.h"
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace std::chrono_literals;
using vessel::Quantity;
const vessel::Time at{100s};
const adapters::BoatN2kBinding binding{"test-n2k", "40328200ffd23456"};
const std::string identity = binding.interface_id + "/NAME-" + binding.name;
const std::string device = "NMEA2000/" + identity + "/source-35/instance-0";
void Check(bool v, const char *why) {
  if (!v)
    throw std::runtime_error(why);
}
vessel::SensorObservation Sample(Quantity q, double v = 42,
                                 vessel::Time time = at) {
  vessel::Sample s{v, device + "/PGN-test", time, vessel::Validity::Measured};
  s.device_id = device;
  return {q, s.source, s, 10};
}
void Binding() {
  adapters::BoatN2k adapter;
  Check(!adapter.Matches(identity), "No implicit binding");
  adapter.Configure(binding);
  Check(adapter.Matches(identity), "Exact observed NAME");
  Check(!adapter.Matches(identity + "extra"), "No prefix identity admission");
  application::Settings s;
  s.boat_bridge = binding;
  auto d = application::DecodeSettings(application::EncodeSettings(s));
  Check(d.boat_bridge.name == binding.name &&
            d.boat_bridge.interface_id == binding.interface_id,
        "Binding persists");
  for (auto bad : {adapters::BoatN2kBinding{"test-n2k", "c0508700e76004d2"},
                   adapters::BoatN2kBinding{"", "40328200ffd23456"}}) {
    bool rejected = false;
    try {
      adapter.Configure(bad);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    Check(rejected, "Invalid identity rejected");
  }
  Check(adapter
            .Observe("other", 35, 61184, {1, 2, 1, 0xf1, 255, 255, 255, 255},
                     at, at)
            .empty(),
        "Other device custom PGN ignored");
}
void Mapping() {
  adapters::BoatN2k adapter;
  adapter.Configure(binding);
  std::vector<vessel::SensorObservation> samples{
      Sample(Quantity::CoolantTemperature, 62), Sample(Quantity::Fuel, 68),
      Sample(Quantity::BatterySoc, 68)};
  auto other = Sample(Quantity::Fuel, 20);
  other.sample.device_id = "unrelated pack";
  samples.push_back(other);
  adapter.Map(samples, at);
  Check(samples.size() == 3,
        "Virtual fuel suppressed, real other source retained");
  Check(samples[0].quantity == Quantity::MotorTemperature &&
            samples[0].sample.value == 62,
        "Explicit temperature meaning only");
  Check(samples[0].sample.observed_at == at &&
            samples[0].sample.device_id == device,
        "Copied provenance retained");
  adapter.Assess(samples[2].sample, samples[2].quantity, at);
  Check(samples[2].sample.validity == vessel::Validity::Measured,
        "Other source unchanged");
}
void Expiry() {
  adapters::BoatN2k adapter;
  adapter.Configure(binding);
  auto o = Sample(Quantity::BatterySoc, 68);
  adapter.Assess(o.sample, o.quantity, at);
  Check(o.sample.validity == vessel::Validity::Uncertain,
        "No producer freshness cannot be live");
  auto regen = adapter.Observe(identity, 35, 61184,
                               {1, 1, 2, 1, 255, 255, 255, 255}, at, at);
  Check(regen.size() == 1 &&
            regen[0].sample.validity == vessel::Validity::Uncertain,
        "v1 source readable but uncertain");
  regen =
      adapter.Observe(identity, 35, 61184, {2, 2, 2, 0xf1, 255, 255, 255, 255},
                      at + 1ms, at + 1ms);
  Check(regen[0].sample.validity == vessel::Validity::Measured &&
            regen[0].sample.value == 2,
        "v2 regen real setting");
  auto fresh = Sample(Quantity::BatterySoc, 68, at + 2ms);
  adapter.Assess(fresh.sample, fresh.quantity, at + 2ms);
  Check(fresh.sample.validity == vessel::Validity::Measured,
        "Verified producer mask permits measured value");
  auto lost = fresh;
  adapter.Assess(lost.sample, lost.quantity, at + 501ms);
  Check(lost.sample.validity == vessel::Validity::Uncertain &&
            lost.sample.observed_at == fresh.sample.observed_at,
        "Heartbeat loss cannot renew input");
  adapter.Observe(identity, 35, 61184, {3, 2, 2, 0xe1, 255, 255, 255, 255},
                  at + 502ms, at + 502ms);
  auto expired = Sample(Quantity::BatterySoc, 68, at + 503ms);
  adapter.Assess(expired.sample, expired.quantity, at + 503ms);
  Check(!expired.sample.value,
        "Expired producer group suppresses retained SOC");
  adapter.Observe(identity, 35, 61184, {4, 2, 2, 0xf1, 255, 255, 255, 255},
                  at + 504ms, at + 504ms);
  auto before_recovery = Sample(Quantity::BatterySoc, 68, at + 503ms);
  adapter.Assess(before_recovery.sample, before_recovery.quantity, at + 504ms);
  Check(!before_recovery.sample.value,
        "Heartbeat alone cannot resurrect a pre-recovery sample");
  Check(adapter
            .Observe(identity, 35, 61184, {2, 2, 2, 0xf1, 255, 255, 255, 255},
                     at, at + 503ms)
            .empty(),
        "Old heartbeat rejected");
  adapter.Reset();
  auto reconnect = Sample(Quantity::MotorRpm, 800);
  adapter.Assess(reconnect.sample, reconnect.quantity, at + 504ms);
  Check(reconnect.sample.validity == vessel::Validity::Uncertain,
        "Reconnect requires new producer contract");
  for (unsigned n = 0; n < 8; ++n) {
    auto bad =
        adapter.Observe(identity, 35, 61184, std::vector<std::uint8_t>(n),
                        at + 1s + std::chrono::milliseconds(n),
                        at + 1s + std::chrono::milliseconds(n));
    Check(bad.size() == 1 && !bad[0].sample.value,
          "Malformed custom frame missing, never zero");
  }
}
int main(int argc, char **argv) {
  try {
    const std::string group = argc > 1 ? argv[1] : "";
    if (group == "binding")
      Binding();
    else if (group == "mapping")
      Mapping();
    else if (group == "expiry")
      Expiry();
    else
      throw std::runtime_error("Unknown group");
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

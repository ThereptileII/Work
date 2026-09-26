#include "application/Settings.h"
#include "vessel/DisplayItems.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace std::chrono_literals;
void Check(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
void Reject(std::function<void()> f) {
  bool caught = false;
  try {
    f();
  } catch (const std::invalid_argument &) {
    caught = true;
  }
  Check(caught, "Invalid configuration admitted");
}
application::Settings Config() {
  application::Settings s;
  s.energy.battery = {24, 20, .5, "Explicit test pack / not a boat default"};
  s.energy.battery_device_id = "SignalK / pack-1";
  s.current = vessel::CurrentConvention::PositiveCharge;
  s.sources[vessel::Quantity::BatterySoc] = {"source-one", {1s, 3s}};
  s.hazard = {1.4, .6, 15};
  return s;
}
void RoundTrip() {
  auto blank = application::DecodeSettings(application::EncodeSettings({}));
  Check(std::isnan(blank.energy.battery.capacity_kwh) &&
            std::isnan(blank.energy.battery.reserve_soc_percent) &&
            blank.energy.battery_device_id.empty(),
        "Unconfigured remains missing");
  auto s = Config();
  s.energy.curve =
      smartnav::ImportPowerCurve("OpenNavXPowerCurve,1\nreference,STW\nbasis,"
                                 "whole-pack\nspeed_kn,power_kw\n2,1\n4,3\n",
                                 "Measured \"trip\" åäö / sample\\file.csv");
  s.energy.consumption = smartnav::ConsumptionModel::CalibratedCurve;
  const auto encoded = application::EncodeSettings(s);
  const auto decoded = application::DecodeSettings(encoded);
  Check(encoded == application::EncodeSettings(decoded),
        "Escaped multiline curve and Unicode lossless");
  Check(
      decoded.sources.at(vessel::Quantity::BatterySoc).freshness.stale_after ==
          3s,
      "Freshness persisted");
  Check(decoded.current == vessel::CurrentConvention::PositiveCharge &&
            decoded.energy.curve.points.size() == 2,
        "Sign and model persisted");
  vessel::SensorRegistry registry;
  auto sample =
      vessel::Sample{68, "SOC", vessel::Time{100s}, vessel::Validity::Measured};
  registry.Observe({vessel::Quantity::BatterySoc, "source-two", sample, 1},
                   vessel::Time{100s});
  registry.Configure(vessel::Quantity::BatterySoc,
                     decoded.sources.at(vessel::Quantity::BatterySoc));
  Check(!registry.Select(vessel::Quantity::BatterySoc, vessel::Time{100s})
             .sample.value,
        "Persisted pin never silently falls back");
}
void PilotConfiguration() {
  auto s = Config();
  const auto old = application::EncodeSettings(s);
  Check(!application::DecodeSettings(old).pilot.permit_control,
        "Alpha settings migrate with pilot control OFF");
  s.pilot = {"TCP:127.0.0.1:7777", "c0508700e76004d2", false};
  auto decoded = application::DecodeSettings(application::EncodeSettings(s));
  Check(decoded.pilot.interface_id == s.pilot.interface_id && decoded.pilot.name == s.pilot.name &&
        !decoded.pilot.permit_control, "Configured binding remains display only");
  s.pilot.permit_control = true;
  const auto text = application::EncodeSettings(s);
  decoded = application::DecodeSettings(text);
  Check(decoded.pilot.permit_control && text.find("enabled") == std::string::npos,
        "Saved permission is distinct from session enablement");
  s.pilot.name = "0000000000000000";
  Reject([&] { application::EncodeSettings(s); });
  auto bad = text;
  auto at = bad.find("\"manual\"");
  Check(at != std::string::npos,"Pilot permission fixture");
  bad.replace(at,8,"\"enabled\"");
  Reject([&] { application::DecodeSettings(bad); });
  bad = old + "\"pilot.permission\" \"manual\"\n";
  Reject([&] { application::DecodeSettings(bad); });
}
void Invalid() {
  auto s = Config();
  const auto record = application::EncodeSettings(s);
  for (const auto &bad : {std::string{}, record + "\"capacity\" \"42\"\n",
                          record + "\"unknown\" \"1\"\n", record + "broken",
                          std::string(65537, 'x')})
    Reject([&] { application::DecodeSettings(bad); });
  auto version = record;
  version.replace(0, 18, "OpenNavXSettings 2");
  Reject([&] { application::DecodeSettings(version); });
  for (const auto &bad : {"nan", "inf", "-inf", "1,2", "12 kWh", "1e999", " "})
    Reject([&] { application::ParseSettingNumber(bad); });
  for (double bad :
       {-1., 0., 100001., std::numeric_limits<double>::infinity()}) {
    s = Config();
    s.energy.battery.capacity_kwh = bad;
    Reject([&] { application::EncodeSettings(s); });
  }
  s = Config();
  s.energy.consumption = smartnav::ConsumptionModel::CalibratedCurve;
  Reject([&] { application::EncodeSettings(s); });
  s = Config();
  s.sources[vessel::Quantity::Depth].freshness = {3s, 1s};
  Reject([&] { application::EncodeSettings(s); });
  s = Config();
  s.sources[vessel::Quantity::Depth].freshness = {1s, 301s};
  Reject([&] { application::EncodeSettings(s); });
  s = Config();
  s.sources[vessel::Quantity::Depth].pinned_source = std::string(513, 'a');
  Reject([&] { application::EncodeSettings(s); });
  s = Config();
  s.energy.battery_device_id = std::string("bad\0device", 10);
  Reject([&] { application::EncodeSettings(s); });
  s = Config();
  s.hazard.corridor_half_width_m = 0;
  Reject([&] { application::EncodeSettings(s); });
}
void Mappings() {
  auto s = Config();
  s.signal_k_mappings = application::ImportSignalKMappings(
      "OpenNavXSignalK,1\r\npath,quantity,scale,offset\r\n"
      "propulsion.main.motorTemperature,motor_temperature,1,-273.15\r\n"
      "propulsion.main.electricalPower,motor_power,0.001,0\r\n");
  auto decoded = application::DecodeSettings(application::EncodeSettings(s));
  Check(decoded.signal_k_mappings.size() == 2 &&
            decoded.signal_k_mappings[1].scale == .001,
        "Persisted explicit mapping conversions");
  Check(application::ExportSignalKMappings(decoded.signal_k_mappings) ==
            application::ExportSignalKMappings(s.signal_k_mappings),
        "Mapping roundtrip");
  const std::string prefix = "OpenNavXSignalK,1\npath,quantity,scale,offset\n";
  for (const auto &row : {"propulsion.main.voltage,battery_voltage,1,0\n",
                          "navigation.foo,motor_power,1,0\n",
                          "propulsion.main.foo,motor_power,0,0\n",
                          "propulsion.main.foo,motor_power,nan,0\n",
                          "propulsion.main.foo,motor_power,1,inf\n",
                          "propulsion.main.foo,motor_power,1,\n",
                          "propulsion..foo,motor_power,1,0\n",
                          "propulsion.main.coolantTemperature,motor_temperature,1,0\n",
                          "propulsion.main.foo,motor_power,1,0,extra\n",
                          "propulsion.main.foo,motor_power,1,0\npropulsion."
                          "main.foo,shaft_power,1,0\n"})
    Reject([&] { application::ImportSignalKMappings(prefix + row); });
  s.signal_k_mappings.resize(17, s.signal_k_mappings[0]);
  Reject([&] { application::EncodeSettings(s); });
  Reject([&] { application::ImportSignalKMappings(std::string(16385, 'x')); });
  Check(application::DecodeSettings(application::EncodeSettings(Config()))
            .signal_k_mappings.empty(),
        "Existing records acquire no implicit mappings");
}
void Live() {
  auto c = application::DecodeSettings(application::EncodeSettings(Config()));
  const vessel::Time t{100s};
  vessel::VesselState s;
  auto observed = [&](double n) {
    return vessel::Sample{n,  "Recorded marine test observation",
                          t,  vessel::Validity::Measured,
                          {}, c.energy.battery_device_id};
  };
  s.battery.soc_percent = observed(80);
  s.battery.voltage_v = observed(48);
  s.battery.current_native_a = observed(-50);
  s.navigation.sog_kn = observed(5);
  vessel::NormalizeBatteryPower(s, c.energy.battery_device_id, c.current, t);
  auto prediction = smartnav::PredictConfiguredEnergy(c.energy, s, t);
  Check(prediction.range.estimate &&
            std::abs(prediction.range.estimate->range_nm - 30) < 1e-9,
        "Configured whole-pack range");
  Check(!prediction.arrival.estimate, "No route cannot become arrival");
  s.battery.soc_percent.freshness = {100ms, 200ms};
  Check(
      !smartnav::PredictConfiguredEnergy(c.energy, s, t + 201ms).range.estimate,
      "Short source stale threshold suppresses energy");
  s.battery.soc_percent.freshness = {10s, 20s};
  Check(!smartnav::PredictConfiguredEnergy(c.energy, s, t + 5s).range.estimate,
        "Long configured threshold cannot relax established energy ceiling");
  s.battery.soc_percent.freshness = {};
  s.battery.voltage_v.observed_at = t - 1ms;
  vessel::NormalizeBatteryPower(s, c.energy.battery_device_id, c.current, t);
  Check(!smartnav::PredictConfiguredEnergy(c.energy, s, t).range.estimate,
        "Epoch mismatch cannot become consumption");
  Check(!s.battery.usable_capacity_kwh.value,
        "Configuration did not fabricate sensor capacity");
}
void Display() {
  application::Settings legacy;
  legacy.data_rail={"aws","depth","sog","cog","heading"};
  const auto migrated=application::DecodeSettings(application::EncodeSettings(legacy));
  Check(migrated.data_rail==std::vector<std::string>{"sog","depth","aws","heading"},
        "Untouched old five-value rail migrates to visible four-value helm layout");
  Check(migrated.instruments==legacy.instruments,
        "Rail migration does not remove instruments or sensor data");
  legacy.data_rail={"soc","pack_power","rpm","depth","sog"};
  Check(application::DecodeSettings(application::EncodeSettings(legacy)).data_rail==legacy.data_rail,
        "Custom older rail settings remain stored for deliberate user selection");
  application::Settings s;
  s.data_rail = {"soc", "pack_power", "rpm"};
  s.instruments = {"sog", "stw", "depth", "water_temp", "rudder"};
  const auto decoded =
      application::DecodeSettings(application::EncodeSettings(s));
  Check(decoded.data_rail == s.data_rail &&
            decoded.instruments == s.instruments,
        "Display selections round trip");
  auto bad = s;
  bad.data_rail = {"depth", "depth"};
  Reject([&] { application::ValidateSettings(bad); });
  bad = s;
  bad.instruments = {};
  Reject([&] { application::ValidateSettings(bad); });
  bad = s;
  bad.data_rail = {"sog", "cog", "heading", "stw", "depth", "aws", "soc"};
  Reject([&] { application::ValidateSettings(bad); });
  bad = s;
  bad.instruments = {"invented_sensor"};
  Reject([&] { application::ValidateSettings(bad); });
  vessel::VesselState state;
  const auto observed = vessel::Time{} + 10s;
  state.battery.soc_percent = {23, "test battery", observed,
                               vessel::Validity::Measured};
  for (const auto &item : vessel::DisplayItems(state)) {
    if (std::string(item.key) == "soc") {
      Check(item.sample->observed_at == observed,
            "Display selection preserves observation time");
      Check(vessel::Assess(*item.sample, observed + 10s).quality ==
                vessel::Quality::Stale,
            "Display does not freshen stale SOC");
    }
    if (std::string(item.key) == "depth")
      Check(!item.sample->value, "Missing displayed depth remains unavailable");
  }
}
int main(int argc, char **argv) {
  try {
    Check(argc == 2, "Choose group");
    std::string arg = argv[1];
    if (arg == "roundtrip")
      RoundTrip();
    else if (arg == "invalid")
      Invalid();
    else if (arg == "pilot")
      PilotConfiguration();
    else if (arg == "mappings")
      Mappings();
    else if (arg == "display")
      Display();
    else if (arg == "live")
      Live();
    else
      throw std::runtime_error("Unknown group");
    std::cout << arg << " passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

#include "diagnostics/Calibration.h"
#include <algorithm>
#include <set>
#include <sstream>
#include <stdexcept>
namespace opennav::diagnostics {
namespace {
std::string Quote(const std::string &s) {
  // Text cells are explicitly labelled to avoid spreadsheet formula execution.
  std::string result = "\"source: ";
  for (char c : s) {
    if (c == '\"')
      result += '\"';
    result += c;
  }
  return result + '\"';
}
} // namespace
CalibrationExport ExportCalibration(const Recording &r,
                                    const CalibrationSelection &c) {
  if (c.device.empty() ||
      (c.speed != smartnav::SpeedReference::ThroughWater &&
       c.speed != smartnav::SpeedReference::OverGround) ||
      (c.power != smartnav::PowerBasis::WholePack &&
       c.power != smartnav::PowerBasis::MotorElectrical &&
       c.power != smartnav::PowerBasis::Shaft))
    throw std::invalid_argument(
        "Calibration needs an explicit reference, power basis and device");
  (void)EncodeRecording(r);
  CalibrationExport out;
  std::ostringstream csv;
  csv << "OpenNavXCalibration,1\nreference,"
      << (c.speed == smartnav::SpeedReference::ThroughWater ? "STW" : "SOG")
      << "\nbasis,"
      << (c.power == smartnav::PowerBasis::WholePack ? "whole-pack"
          : c.power == smartnav::PowerBasis::MotorElectrical
              ? "motor-electrical"
              : "shaft")
      << "\nelapsed_ms,speed_kn,power_kw,speed_age_ms,power_age_ms,speed_"
         "source,power_source,device,quality\n";
  std::set<std::string> seen;
  for (const auto &f : r.frames) {
    const auto &s = f.state;
    const auto &speed = c.speed == smartnav::SpeedReference::ThroughWater
                            ? s.navigation.stw_kn
                            : s.navigation.sog_kn;
    const auto &power = c.power == smartnav::PowerBasis::WholePack
                            ? s.battery.net_discharge_kw
                        : c.power == smartnav::PowerBasis::MotorElectrical
                            ? s.propulsion.electrical_power_kw
                            : s.propulsion.shaft_power_kw;
    const vessel::Time now{
        std::chrono::duration_cast<vessel::Clock::duration>(f.elapsed)};
    auto fresh = [&](const vessel::Sample &value) {
      return value.value &&
             (value.validity == vessel::Validity::Measured ||
              value.validity == vessel::Validity::Estimated) &&
             !value.source.empty() && value.observed_at <= now &&
             now - value.observed_at <=
                 std::min(value.freshness.stale_after, vessel::Duration(2000));
    };
    if (!fresh(speed) || !fresh(power) || power.device_id != c.device ||
        *speed.value < .1 || *power.value <= 0 ||
        std::max(speed.observed_at, power.observed_at) -
                std::min(speed.observed_at, power.observed_at) >
            vessel::Duration(1000)) {
      ++out.rejected;
      continue;
    }
    const auto key =
        speed.source + '\n' + power.source + '\n' +
        std::to_string(speed.observed_at.time_since_epoch().count()) + '\n' +
        std::to_string(power.observed_at.time_since_epoch().count());
    if (!seen.insert(key).second) {
      ++out.duplicate;
      continue;
    }
    csv << f.elapsed.count() << ',' << application::SettingNumber(*speed.value)
        << ',' << application::SettingNumber(*power.value) << ','
        << std::chrono::duration_cast<vessel::Duration>(now - speed.observed_at)
               .count()
        << ','
        << std::chrono::duration_cast<vessel::Duration>(now - power.observed_at)
               .count()
        << ',' << Quote(speed.source) << ',' << Quote(power.source) << ','
        << Quote(power.device_id) << ','
        << (s.simulated ? "DEMO"
            : speed.validity == vessel::Validity::Estimated ||
                    power.validity == vessel::Validity::Estimated
                ? "DERIVED_OR_ESTIMATED"
                : "MEASURED")
        << '\n';
    ++out.pairs;
  }
  out.csv = csv.str();
  return out;
}
} // namespace opennav::diagnostics

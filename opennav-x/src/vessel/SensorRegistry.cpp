#include "vessel/SensorRegistry.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace opennav::vessel {
// Units and admissible domains are physical contracts, not display clamps.
#define QUANTITIES(X)                                                          \
  X(Heading, navigation.heading_true_deg, "heading", "Heading", "deg true", 0, \
    360)                                                                       \
  X(WaterSpeed, navigation.stw_kn, "stw", "Speed through water", "kn", 0, 200) \
  X(ApparentWindSpeed, wind.apparent_speed_kn, "aws", "Apparent wind speed",   \
    "kn", 0, 250)                                                              \
  X(ApparentWindAngle, wind.apparent_angle_deg, "awa", "Apparent wind angle",  \
    "deg", -180, 180)                                                          \
  X(TrueWindSpeed, wind.true_speed_kn, "tws", "True wind speed", "kn", 0, 250) \
  X(TrueWindAngle, wind.true_angle_deg, "twa", "True wind angle", "deg", -180, \
    180)                                                                       \
  X(Depth, environment.depth_below_transducer_m, "depth",                      \
    "Depth below transducer", "m", 0, 12000)                                   \
  X(WaterTemperature, environment.water_temperature_c, "water_temperature",    \
    "Water temperature", "C", -10, 100)                                        \
  X(Pressure, environment.pressure_hpa, "pressure", "Atmospheric pressure",    \
    "hPa", 100, 1200)                                                          \
  X(Rudder, rudder.angle_deg, "rudder", "Rudder", "deg", -180, 180)            \
  X(Heel, rudder.heel_deg, "heel", "Heel", "deg", -180, 180)                   \
  X(MotorRpm, propulsion.motor_rpm, "rpm", "Motor speed", "RPM", 0, 100000)    \
  X(MotorTemperature, propulsion.motor_temperature_c, "motor_temperature",     \
    "Motor temperature", "C", -100, 500)                                       \
  X(CoolantTemperature, propulsion.coolant_temperature_c,                      \
    "coolant_temperature", "Engine coolant temperature", "C", -100, 500)       \
  X(MotorPower, propulsion.electrical_power_kw, "motor_power",                 \
    "Motor electrical power", "kW", -10000, 10000)                             \
  X(ShaftPower, propulsion.shaft_power_kw, "shaft_power", "Shaft power", "kW", \
    -10000, 10000)                                                             \
  X(BatteryVoltage, battery.voltage_v, "battery_voltage", "Battery voltage",   \
    "V", 0, 2000)                                                              \
  X(BatteryNativeCurrent, battery.current_native_a, "battery_native_current",  \
    "Battery current (source convention)", "A", -100000, 100000)               \
  X(BatteryCurrent, battery.current_a, "battery_current",                      \
    "Battery current (+ discharge)", "A", -100000, 100000)                     \
  X(BatterySoc, battery.soc_percent, "soc", "Battery SOC", "%", 0, 100)        \
  X(BatterySoh, battery.soh_percent, "soh", "Battery SOH", "%", 0, 100)        \
  X(BatteryPower, battery.net_discharge_kw, "battery_power",                   \
    "Whole-pack net discharge", "kW", -10000, 10000)                           \
  X(BatteryCapacity, battery.usable_capacity_kwh, "battery_capacity",          \
    "Usable pack capacity", "kWh", 0, 100000)                                  \
  X(FreshWater, tanks.fresh_water_percent, "fresh_water", "Fresh water tank",  \
    "%", 0, 100)                                                               \
  X(Fuel, tanks.fuel_percent, "fuel", "Fuel tank", "%", 0, 100)                \
  X(Waste, tanks.waste_percent, "waste", "Waste tank", "%", 0, 100)            \
  X(Gear, propulsion.gear_code, "gear", "Transmission gear",                   \
    "0 forward / 1 neutral / 2 reverse", 0, 2)                                 \
  X(OtherTank, tanks.other_percent, "other_tank", "Other fluid tank", "%", 0,  \
    100)                                                                      \
  X(Regeneration, propulsion.regeneration_code, "regeneration",              \
    "Regeneration setting", "0 off / 1 one bar / 2 two bars", 0, 2)

const std::vector<QuantityInfo> &Quantities() {
  static const std::vector<QuantityInfo> values = {
#define ENTRY(q, member, key, name, unit, lo, hi)                              \
  {Quantity::q, key, name, unit, lo, hi},
      QUANTITIES(ENTRY)
#undef ENTRY
  };
  return values;
}
const QuantityInfo &Describe(Quantity q) {
  const auto index = static_cast<std::size_t>(q);
  if (index >= Quantities().size())
    throw std::invalid_argument("Unknown quantity");
  return Quantities()[index];
}
Sample &Field(VesselState &s, Quantity q) {
  switch (q) {
#define FIELD(q, member, key, name, unit, lo, hi)                              \
  case Quantity::q:                                                            \
    return s.member;
    QUANTITIES(FIELD)
#undef FIELD
  default:
    throw std::invalid_argument("Unknown quantity");
  }
}
const Sample &Field(const VesselState &s, Quantity q) {
  switch (q) {
#define FIELD(q, member, key, name, unit, lo, hi)                              \
  case Quantity::q:                                                            \
    return s.member;
    QUANTITIES(FIELD)
#undef FIELD
  default:
    throw std::invalid_argument("Unknown quantity");
  }
}
#undef QUANTITIES

SourcePolicy SensorRegistry::Policy(Quantity q) const {
  Describe(q);
  const auto it = policies_.find(q);
  return it == policies_.end() ? SourcePolicy{} : it->second;
}
void SensorRegistry::Configure(Quantity q, SourcePolicy p) {
  Describe(q);
  if (p.pinned_source.size() > 512 ||
      p.freshness.aging_after < Duration::zero() ||
      p.freshness.stale_after <= p.freshness.aging_after ||
      p.freshness.stale_after > std::chrono::minutes(5))
    throw std::invalid_argument("Invalid source policy");
  policies_[q] = std::move(p);
}
Admission SensorRegistry::Observe(SensorObservation o, Time now) {
  const auto &info = Describe(o.quantity);
  const auto controls = [](const std::string &text) {
    return std::any_of(text.begin(), text.end(), [](unsigned char c) {
      return c < 0x20 || c == 0x7f;
    });
  };
  if (o.source_id.empty() || o.source_id.size() > 512 ||
      o.sample.source.empty() || o.sample.source.size() > 1024 ||
      o.sample.device_id.size() > 512 || controls(o.source_id) ||
      controls(o.sample.source) || controls(o.sample.device_id))
    return Admission::MissingSource;
  if (o.sample.observed_at > now)
    return Admission::Future;
  auto &candidates = sources_[o.quantity];
  const auto previous = candidates.find(o.source_id);
  if (previous != candidates.end() &&
      o.sample.observed_at < previous->second.sample.observed_at)
    return Admission::Old;
  if (previous == candidates.end() && candidates.size() >= 32)
    return Admission::Full;
  const bool valid = o.sample.value && std::isfinite(*o.sample.value) &&
                     *o.sample.value >= info.minimum &&
                     *o.sample.value <= info.maximum &&
                     (o.sample.validity == Validity::Measured ||
                      o.sample.validity == Validity::Estimated ||
                      o.sample.validity == Validity::Uncertain) &&
                     ((o.quantity != Quantity::Gear && o.quantity != Quantity::Regeneration) ||
                      std::floor(*o.sample.value) == *o.sample.value);
  if (!valid) {
    o.sample.value.reset();
    o.sample.validity = Validity::Invalid;
  }
  if (o.quantity == Quantity::Heading && o.sample.value == 360)
    o.sample.value = 0;
  auto &stats = statistics_[o.quantity][o.source_id];
  // Same-epoch duplicates cannot inflate rate. Invalid observations still
  // count as received input, but never count as a valid sensor value.
  if (!stats.observations || o.sample.observed_at > stats.last) {
    if (stats.observations) {
      const double dt =
          std::chrono::duration<double>(o.sample.observed_at - stats.last)
              .count();
      stats.interval_seconds = stats.interval_seconds
                                   ? .75 * *stats.interval_seconds + .25 * dt
                                   : dt;
    }
    stats.last = o.sample.observed_at;
    if (stats.observations != std::numeric_limits<std::uint64_t>::max())
      ++stats.observations;
    if (!valid && stats.invalid != std::numeric_limits<std::uint64_t>::max())
      ++stats.invalid;
  }
  candidates[o.source_id] = std::move(o);
  return valid ? Admission::Accepted : Admission::InvalidValue;
}
SourceSelection SensorRegistry::Select(Quantity q, Time now) const {
  const auto policy = Policy(q);
  SourceSelection result;
  result.sample.freshness = policy.freshness;
  result.reason = policy.pinned_source.empty()
                      ? "No observed source"
                      : "Configured source unavailable";
  const auto set = sources_.find(q);
  if (set == sources_.end())
    return result;
  const SensorObservation *selected = nullptr;
  int best = 4;
  for (const auto &entry : set->second) {
    const auto &o = entry.second;
    if (!policy.pinned_source.empty() && entry.first != policy.pinned_source)
      continue;
    const auto a = Assess(o.sample, now, policy.freshness);
    const int rank = a.quality == Quality::Live || a.quality == Quality::Aging
                         ? 0
                     : a.quality == Quality::Estimated ? 1
                     : a.quality == Quality::Uncertain ? 2
                                                       : 3;
    if (!selected || rank < best ||
        (rank == best && o.priority < selected->priority)) {
      selected = &o;
      best = rank;
    }
  }
  if (selected) {
    result.sample = selected->sample;
    result.sample.freshness = policy.freshness;
    result.selected_source = selected->source_id;
    result.reason = !policy.pinned_source.empty()
                        ? "Configured source (no fallback)"
                    : best < 2 ? "Automatic precedence among fresh sources"
                               : "Retained source; no usable fresh input";
  }
  return result;
}
VesselState SensorRegistry::Merge(VesselState s, Time now) const {
  for (const auto &q : Quantities())
    Field(s, q.quantity) = Select(q.quantity, now).sample;
  NormalizePropulsionStates(s);
  return s;
}
void NormalizePropulsionStates(VesselState &s) {
  const auto enum_value=[](const Sample &v) {
    return v.value && v.validity!=Validity::Invalid && std::isfinite(*v.value) &&
           *v.value>=0 && *v.value<=2 && std::floor(*v.value)==*v.value;
  };
  const auto &gear = s.propulsion.gear_code;
  s.propulsion.gear = {{},
                       gear.source,
                       gear.observed_at,
                       gear.validity,
                       gear.freshness,
                       gear.device_id};
  if (enum_value(gear)) {
    static const char *names[] = {"Forward", "Neutral", "Reverse"};
    s.propulsion.gear.value = names[static_cast<unsigned>(*gear.value)];
  }
  const auto &regen = s.propulsion.regeneration_code;
  s.propulsion.regeneration = {{}, regen.source, regen.observed_at,
                               regen.validity, regen.freshness, regen.device_id};
  if (enum_value(regen)) {
    static const char *names[] = {"Off", "One bar", "Two bars"};
    s.propulsion.regeneration.value = names[static_cast<unsigned>(*regen.value)];
  }
}
std::vector<SourceHealth> SensorRegistry::Health(Time now) const {
  std::vector<SourceHealth> result;
  for (const auto &group : sources_) {
    const auto selected = Select(group.first, now);
    for (const auto &entry : group.second) {
      auto sample = entry.second.sample;
      sample.freshness = Policy(group.first).freshness;
      result.push_back({group.first, entry.first, sample,
                        selected.selected_source == entry.first,
                        entry.second.priority});
      const auto &stats = statistics_.at(group.first).at(entry.first);
      auto &health = result.back();
      health.observations = stats.observations;
      health.invalid_observations = stats.invalid;
      const auto quality = Assess(sample, now).quality;
      if (stats.interval_seconds && *stats.interval_seconds > 0 &&
          now >= stats.last &&
          now - stats.last < sample.freshness.stale_after &&
          quality != Quality::Stale) {
        // Rate decays during a dropout; never retain a live-looking rate.
        const auto elapsed =
            std::chrono::duration<double>(now - stats.last).count();
        health.frequency_hz = 1.0 / std::max(*stats.interval_seconds, elapsed);
      }
    }
  }
  return result;
}
void SensorRegistry::Clear() {
  sources_.clear();
  statistics_.clear();
}

void NormalizeBatteryPower(VesselState &s, const std::string &device,
                           CurrentConvention convention, Time now) {
  s.battery.current_a = {};
  s.battery.net_discharge_kw = {};
  if (device.empty() || convention == CurrentConvention::Unconfigured)
    return;
  const auto &raw = s.battery.current_native_a;
  const auto &volts = s.battery.voltage_v;
  auto usable = [now, &device](const Sample &v) {
    const auto a = Assess(v, now);
    return v.device_id == device && v.validity == Validity::Measured &&
           (a.quality == Quality::Live || a.quality == Quality::Aging);
  };
  if (!usable(raw))
    return;
  auto current = raw;
  current.value =
      *raw.value * (convention == CurrentConvention::PositiveCharge ? -1 : 1);
  current.source += " / configured whole-pack current sign";
  s.battery.current_a = current;
  // Voltage and current must be a coherent measurement from one message/epoch.
  if (!usable(volts) || volts.observed_at != raw.observed_at ||
      *volts.value <= 0)
    return;
  const double power = *volts.value * *current.value / 1000;
  if (!std::isfinite(power) || std::abs(power) > 10000)
    return;
  auto net = current;
  net.value = power;
  net.validity = Validity::Estimated;
  net.source += " / V x I, includes all loads at configured pack shunt";
  s.battery.net_discharge_kw = std::move(net);
}
} // namespace opennav::vessel

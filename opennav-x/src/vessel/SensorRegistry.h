#pragma once
#include "vessel/VesselState.h"
#include <cstdint>
#include <map>
#include <vector>

namespace opennav::vessel {
// Position/SOG/COG are deliberately absent: OpenCPN's selected navigation bus
// continues to own those quantities and their source precedence.
enum class Quantity {
  Heading,
  WaterSpeed,
  ApparentWindSpeed,
  ApparentWindAngle,
  TrueWindSpeed,
  TrueWindAngle,
  Depth,
  WaterTemperature,
  Pressure,
  Rudder,
  Heel,
  MotorRpm,
  MotorTemperature,
  CoolantTemperature,
  MotorPower,
  ShaftPower,
  BatteryVoltage,
  BatteryNativeCurrent,
  BatteryCurrent,
  BatterySoc,
  BatterySoh,
  BatteryPower,
  BatteryCapacity,
  FreshWater,
  Fuel,
  Waste,
  Gear,
  OtherTank,
  Count
};
struct QuantityInfo {
  Quantity quantity;
  const char *key, *name, *unit;
  double minimum, maximum;
};
const std::vector<QuantityInfo> &Quantities();
const QuantityInfo &Describe(Quantity quantity);
Sample &Field(VesselState &state, Quantity quantity);
const Sample &Field(const VesselState &state, Quantity quantity);

struct SourcePolicy {
  // Empty selects the highest-priority usable source. Explicit pins fail closed
  // when missing/stale and never fall back silently to another device.
  std::string pinned_source;
  Freshness freshness{};
};
struct SensorObservation {
  Quantity quantity;
  std::string source_id; // transport + PGN/path + source + instance
  Sample sample;
  unsigned priority = 100; // lower wins; ties use stable source identity
};
enum class Admission {
  Accepted,
  InvalidValue,
  Old,
  Future,
  MissingSource,
  Full
};
struct SourceSelection {
  Sample sample;
  std::string selected_source;
  std::string reason;
};
struct SourceHealth {
  Quantity quantity;
  std::string source_id;
  Sample sample;
  bool selected = false;
  unsigned priority = 0;
  std::optional<double> frequency_hz;
  std::uint64_t observations = 0;
  std::uint64_t invalid_observations = 0;
};

// Application-thread reducer, with owned values only. Reads never mutate the
// receipt watermark or renew age. Each quantity has a bounded candidate set.
class SensorRegistry {
public:
  Admission Observe(SensorObservation observation, Time now);
  void Configure(Quantity quantity, SourcePolicy policy);
  SourcePolicy Policy(Quantity quantity) const;
  SourceSelection Select(Quantity quantity, Time now) const;
  VesselState Merge(VesselState selected_navigation, Time now) const;
  std::vector<SourceHealth> Health(Time now) const;
  void Clear();

private:
  struct Statistics {
    Time last{};
    std::optional<double> interval_seconds;
    std::uint64_t observations = 0, invalid = 0;
  };
  std::map<Quantity, std::map<std::string, SensorObservation>> sources_;
  std::map<Quantity, std::map<std::string, Statistics>> statistics_;
  std::map<Quantity, SourcePolicy> policies_;
};

// Whole-pack conversion is opt-in for a selected device/instance and a verified
// installation sign convention. It cannot combine values from unrelated packs.
enum class CurrentConvention {
  Unconfigured,
  PositiveDischarge,
  PositiveCharge
};
void NormalizeBatteryPower(VesselState &state, const std::string &device,
                           CurrentConvention convention, Time now);
} // namespace opennav::vessel

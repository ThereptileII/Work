#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace opennav::vessel {

using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
using Duration = std::chrono::milliseconds;
enum class Validity { Measured, Estimated, Uncertain, Invalid };
enum class Quality { Live, Aging, Stale, Unavailable, Estimated, Uncertain };

struct Freshness {
  Duration aging_after{2000};
  Duration stale_after{5000};
};

struct Sample {
  std::optional<double> value;
  std::string source;
  Time observed_at{};
  Validity validity = Validity::Invalid;
  Freshness freshness{};
  // Identity of the physical device/instance, independent of the message PGN.
  // Required when combining battery observations across different messages.
  std::string device_id{};
};

// Discrete telemetry has the same provenance/age requirements as numeric data.
struct TextSample {
  std::optional<std::string> value;
  std::string source;
  Time observed_at{};
  Validity validity = Validity::Invalid;
  Freshness freshness{};
  std::string device_id{};
};

struct Assessment {
  Quality quality = Quality::Unavailable;
  std::optional<double> value;
  std::optional<Duration> age;
};

Assessment Assess(const Sample &sample, Time now, Freshness freshness);
inline Assessment Assess(const Sample &sample, Time now) {
  return Assess(sample, now, sample.freshness);
}
struct TextAssessment {
  Quality quality = Quality::Unavailable;
  std::optional<std::string> value;
  std::optional<Duration> age;
};
TextAssessment AssessText(const TextSample &sample, Time now,
                          Freshness freshness);
inline TextAssessment AssessText(const TextSample &sample, Time now) {
  return AssessText(sample, now, sample.freshness);
}
const char *QualityName(Quality quality);
const char *ValidityName(Validity validity);

// Field names specify canonical units and physical meaning. Never reinterpret
// heading as COG, apparent as true wind, or depth as under-keel clearance.
struct RouteProgressSnapshot;
struct Navigation {
  Sample latitude_deg, longitude_deg, sog_kn, cog_deg, heading_true_deg, stw_kn;
  std::shared_ptr<const RouteProgressSnapshot> route;
};
struct Environment {
  Sample depth_below_transducer_m, water_temperature_c, pressure_hpa;
};
struct Wind {
  Sample apparent_speed_kn, apparent_angle_deg, true_speed_kn, true_angle_deg;
};
struct Propulsion {
  Sample electrical_power_kw, motor_rpm, motor_temperature_c, shaft_power_kw;
  Sample coolant_temperature_c;
  // Normalized transmission enum: 0 forward, 1 neutral, 2 reverse.
  // Unknown/NA is missing, never interpreted as forward.
  Sample gear_code;
  TextSample gear, regeneration;
};
struct Battery {
  Sample soc_percent, voltage_v, current_a, usable_capacity_kwh, soh_percent;
  // Positive whole-pack net discharge, including hotel loads. Not motor power.
  Sample net_discharge_kw;
  // Raw source sign is intentionally not relabelled '+ discharge'. An explicit
  // installation mapping is needed before it becomes current_a/net_discharge.
  Sample current_native_a;
};
struct Rudder {
  Sample angle_deg;
  Sample heel_deg;
};
struct Tanks {
  Sample fresh_water_percent, fuel_percent, waste_percent;
  Sample
      other_percent; // Instance and fluid type retained in source provenance.
};
struct Connectivity {
  Sample received_messages_per_second;
  TextSample status;
};

struct VesselState {
  Navigation navigation;
  Environment environment;
  Wind wind;
  Propulsion propulsion;
  Battery battery;
  Rudder rudder;
  Tanks tanks;
  Connectivity connectivity;
  bool simulated = false;
};

// Explicit, deterministic fixture. Never installed as a live-data fallback.
VesselState SimulatorFixture(Time observed_at);

} // namespace opennav::vessel

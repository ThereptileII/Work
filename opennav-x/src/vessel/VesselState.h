#pragma once

#include <chrono>
#include <optional>
#include <string>

namespace opennav::vessel {

using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
using Duration = std::chrono::milliseconds;
enum class Validity { Measured, Estimated, Uncertain, Invalid };
enum class Quality { Live, Aging, Stale, Unavailable, Estimated, Uncertain };

struct Sample {
  std::optional<double> value;
  std::string source;
  Time observed_at{};
  Validity validity = Validity::Invalid;
};

struct Freshness {
  Duration aging_after{2000};
  Duration stale_after{5000};
};

struct Assessment {
  Quality quality = Quality::Unavailable;
  std::optional<double> value;
  std::optional<Duration> age;
};

Assessment Assess(const Sample& sample, Time now, Freshness freshness = {});

// Field names specify canonical units and physical meaning. Never reinterpret
// heading as COG, apparent as true wind, or depth as under-keel clearance.
struct Navigation { Sample latitude_deg, longitude_deg, sog_kn, cog_deg, heading_true_deg; };
struct Environment { Sample depth_below_transducer_m, water_temperature_c; };
struct Wind { Sample apparent_speed_kn, apparent_angle_deg, true_speed_kn, true_angle_deg; };
struct Propulsion { Sample electrical_power_kw, motor_rpm, motor_temperature_c; };
struct Battery { Sample soc_percent, voltage_v, current_a, usable_capacity_kwh; };
struct Rudder { Sample angle_deg; };
struct Tanks { Sample fresh_water_percent, fuel_percent, waste_percent; };
struct Connectivity { Sample received_messages_per_second; };

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

}  // namespace opennav::vessel

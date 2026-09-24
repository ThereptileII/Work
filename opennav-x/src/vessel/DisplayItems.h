#pragma once
#include "vessel/VesselState.h"
#include <vector>
namespace opennav::vessel {
struct DisplayItem {
  const char *key;
  const char *title;
  const char *unit;
  const Sample *sample;
};
// Stable configuration keys; borrowed values are inspected synchronously from
// one OpenNav-owned snapshot. Selection never refreshes observations.
inline std::vector<DisplayItem> DisplayItems(const VesselState &s) {
  return {
      {"sog", "SOG", "kn", &s.navigation.sog_kn},
      {"cog", "COG", "deg true", &s.navigation.cog_deg},
      {"heading", "HEADING", "deg true", &s.navigation.heading_true_deg},
      {"stw", "STW", "kn", &s.navigation.stw_kn},
      {"aws", "APPARENT WIND", "kn", &s.wind.apparent_speed_kn},
      {"awa", "APPARENT ANGLE", "deg relative", &s.wind.apparent_angle_deg},
      {"tws", "TRUE WIND", "kn", &s.wind.true_speed_kn},
      {"twa", "TRUE ANGLE", "deg relative", &s.wind.true_angle_deg},
      {"depth", "DEPTH", "m / transducer",
       &s.environment.depth_below_transducer_m},
      {"water_temp", "WATER TEMP", "°C", &s.environment.water_temperature_c},
      {"pressure", "PRESSURE", "hPa", &s.environment.pressure_hpa},
      {"rudder", "RUDDER", "deg", &s.rudder.angle_deg},
      {"heel", "HEEL", "deg", &s.rudder.heel_deg},
      {"soc", "BATTERY SOC", "%", &s.battery.soc_percent},
      {"voltage", "BATTERY", "V", &s.battery.voltage_v},
      {"current", "PACK CURRENT", "A / positive discharge",
       &s.battery.current_a},
      {"pack_power", "PACK DISCHARGE", "kW", &s.battery.net_discharge_kw},
      {"motor_power", "MOTOR POWER", "kW", &s.propulsion.electrical_power_kw},
      {"rpm", "MOTOR SPEED", "RPM", &s.propulsion.motor_rpm},
      {"motor_temp", "MOTOR TEMP", "°C", &s.propulsion.motor_temperature_c},
      {"fresh_water", "FRESH WATER", "%", &s.tanks.fresh_water_percent},
      {"fuel", "FUEL", "%", &s.tanks.fuel_percent},
      {"waste", "WASTE", "%", &s.tanks.waste_percent}};
}
} // namespace opennav::vessel

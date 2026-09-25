#pragma once
#include "vessel/VesselState.h"
#include <vector>

namespace opennav::vessel {
struct DataItem {
  const char *name;
  const char *unit;
  const Sample *sample;
};
// Borrowed only for synchronous inspection of an owned VesselState.
inline std::vector<DataItem> DataItems(const VesselState &s) {
  return {
      {"Latitude", "deg", &s.navigation.latitude_deg},
      {"Longitude", "deg", &s.navigation.longitude_deg},
      {"Speed over ground", "kn", &s.navigation.sog_kn},
      {"Course over ground", "deg true", &s.navigation.cog_deg},
      {"Heading", "deg true", &s.navigation.heading_true_deg},
      {"Speed through water", "kn", &s.navigation.stw_kn},
      {"Apparent wind speed", "kn", &s.wind.apparent_speed_kn},
      {"Apparent wind angle", "deg", &s.wind.apparent_angle_deg},
      {"True wind speed", "kn", &s.wind.true_speed_kn},
      {"True wind angle", "deg", &s.wind.true_angle_deg},
      {"Depth below transducer", "m", &s.environment.depth_below_transducer_m},
      {"Water temperature", "C", &s.environment.water_temperature_c},
      {"Atmospheric pressure", "hPa", &s.environment.pressure_hpa},
      {"Rudder", "deg", &s.rudder.angle_deg},
      {"Heel", "deg", &s.rudder.heel_deg},
      {"Motor electrical power", "kW", &s.propulsion.electrical_power_kw},
      {"Shaft power", "kW", &s.propulsion.shaft_power_kw},
      {"Motor speed", "RPM", &s.propulsion.motor_rpm},
      {"Motor temperature", "C", &s.propulsion.motor_temperature_c},
      {"Engine coolant temperature", "C", &s.propulsion.coolant_temperature_c},
      {"Battery voltage", "V", &s.battery.voltage_v},
      {"Battery current (+ discharge)", "A", &s.battery.current_a},
      {"Battery current (source convention)", "A", &s.battery.current_native_a},
      {"Battery SOC", "%", &s.battery.soc_percent},
      {"Battery SOH", "%", &s.battery.soh_percent},
      {"Usable pack capacity", "kWh", &s.battery.usable_capacity_kwh},
      {"Whole-pack net discharge", "kW", &s.battery.net_discharge_kw},
      {"Fresh water tank", "%", &s.tanks.fresh_water_percent},
      {"Fuel tank", "%", &s.tanks.fuel_percent},
      {"Waste tank", "%", &s.tanks.waste_percent},
      {"Other fluid tank", "%", &s.tanks.other_percent},
      {"Input rate", "msg/s", &s.connectivity.received_messages_per_second}};
}
struct TextDataItem {
  const char *name;
  const TextSample *sample;
};
inline std::vector<TextDataItem> TextDataItems(const VesselState &s) {
  return {{"Gear", &s.propulsion.gear},
          {"Regeneration", &s.propulsion.regeneration},
          {"Connectivity", &s.connectivity.status}};
}
} // namespace opennav::vessel

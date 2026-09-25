#pragma once
#include "vessel/VesselState.h"
#include <type_traits>
#include <vector>

namespace opennav::vessel {
template <class T> struct BasicDataItem {
  const char *name;
  const char *unit;
  T *sample;
};
// Borrowed only for synchronous inspection of an owned VesselState.
using DataItem = BasicDataItem<const Sample>;
using MutableDataItem = BasicDataItem<Sample>;
template <class State> inline auto MakeDataItems(State &s) {
  using T = std::remove_reference_t<decltype((s.navigation.latitude_deg))>;
  return std::vector<BasicDataItem<T>>{
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
      {"Transmission gear", "0 forward / 1 neutral / 2 reverse",
       &s.propulsion.gear_code},
      {"Regeneration setting", "0 off / 1 one bar / 2 two bars",
       &s.propulsion.regeneration_code},
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
inline std::vector<DataItem> DataItems(const VesselState &s) {
  return MakeDataItems(s);
}
inline std::vector<MutableDataItem> MutableDataItems(VesselState &s) {
  return MakeDataItems(s);
}
template <class T> struct BasicTextDataItem {
  const char *name;
  T *sample;
};
using TextDataItem = BasicTextDataItem<const TextSample>;
using MutableTextDataItem = BasicTextDataItem<TextSample>;
template <class State> inline auto MakeTextDataItems(State &s) {
  using T = std::remove_reference_t<decltype((s.propulsion.gear))>;
  return std::vector<BasicTextDataItem<T>>{
      {"Gear", &s.propulsion.gear},
      {"Regeneration", &s.propulsion.regeneration},
      {"Connectivity", &s.connectivity.status}};
}
inline std::vector<TextDataItem> TextDataItems(const VesselState &s) {
  return MakeTextDataItems(s);
}
inline std::vector<MutableTextDataItem> MutableTextDataItems(VesselState &s) {
  return MakeTextDataItems(s);
}
} // namespace opennav::vessel

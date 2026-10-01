#pragma once
#include <stdexcept>
#include <string>

namespace opennav::application {
enum class ChartLayout { Balanced, ChartFocus, InstrumentFocus };
struct DisplayPreferences {
  int scale_percent = 100;
  ChartLayout layout = ChartLayout::Balanced;
};
inline bool ValidDisplayPreferences(const DisplayPreferences &value) {
  return (value.scale_percent == 100 || value.scale_percent == 125 ||
          value.scale_percent == 150) &&
         (value.layout == ChartLayout::Balanced ||
          value.layout == ChartLayout::ChartFocus ||
          value.layout == ChartLayout::InstrumentFocus);
}
inline std::string EncodeDisplayPreferences(const DisplayPreferences &value) {
  if (!ValidDisplayPreferences(value))
    throw std::invalid_argument("Invalid display preferences");
  const char *layout = value.layout == ChartLayout::Balanced ? "balanced" :
      value.layout == ChartLayout::ChartFocus ? "chart" : "instruments";
  return "v1|" + std::to_string(value.scale_percent) + "|" + layout;
}
inline DisplayPreferences DecodeDisplayPreferences(const std::string &record) {
  for (const int scale : {100, 125, 150})
    for (const auto layout : {ChartLayout::Balanced, ChartLayout::ChartFocus,
                              ChartLayout::InstrumentFocus}) {
      DisplayPreferences value{scale, layout};
      if (record == EncodeDisplayPreferences(value)) return value;
    }
  throw std::invalid_argument("Unsupported display preferences record");
}
} // namespace opennav::application

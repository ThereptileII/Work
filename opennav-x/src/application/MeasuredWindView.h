#pragma once

#include "vessel/VesselState.h"
#include <cmath>
#include <iomanip>
#include <locale>
#include <optional>
#include <sstream>
#include <string>

namespace opennav::application {

struct MeasuredWindReading {
  std::optional<double> value;
  vessel::Quality quality = vessel::Quality::Unavailable;
  std::string source;
  std::optional<vessel::Duration> age;
  bool invalid_value = false;
};

inline MeasuredWindReading WindReading(const vessel::Sample &sample, vessel::Time now,
                                      double minimum, double maximum) {
  const auto assessed = vessel::Assess(sample, now);
  const bool valid_range = !assessed.value ||
      (*assessed.value >= minimum && *assessed.value <= maximum);
  const bool current = assessed.quality == vessel::Quality::Live ||
                       assessed.quality == vessel::Quality::Aging;
  return {current && valid_range ? assessed.value : std::nullopt,
          assessed.quality, sample.source, assessed.age, !valid_range};
}

struct MeasuredWindView {
  MeasuredWindReading true_speed;
  MeasuredWindReading apparent_speed;
  MeasuredWindReading apparent_angle;
};

inline MeasuredWindView MeasureWind(const vessel::Wind &wind, vessel::Time now) {
  return {WindReading(wind.true_speed_kn, now, 0, 250),
          WindReading(wind.apparent_speed_kn, now, 0, 250),
          WindReading(wind.apparent_angle_deg, now, -180, 180)};
}

inline std::string WindStatusText(const MeasuredWindReading &reading) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << (reading.invalid_value ? "INVALID" : vessel::QualityName(reading.quality));
  if (reading.age)
    out << " \xC2\xB7 " << std::fixed << std::setprecision(1)
        << reading.age->count() / 1000.0 << " s";
  return out.str();
}

inline std::string WindSourceText(const MeasuredWindReading &reading) {
  return reading.source.empty() ? "No source" : "Source: " + reading.source;
}

inline std::string WindSpeedText(const MeasuredWindReading &reading) {
  if (!reading.value) return "\xE2\x80\x94";
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(1) << *reading.value << " kn";
  return out.str();
}

inline std::string WindAngleText(const MeasuredWindReading &reading) {
  if (!reading.value) return "\xE2\x80\x94";
  const double angle = *reading.value;
  const char *side = std::abs(angle) < 0.5 ? "ahead"
                     : std::abs(angle) >= 179.5 ? "astern"
                     : angle < 0 ? "port" : "starboard";
  return std::to_string(std::lround(std::abs(angle))) + "\xC2\xB0 " + side;
}

} // namespace opennav::application

#pragma once
#include "vessel/VesselState.h"
#include <vector>

namespace opennav::application {
// Owned presentation values only. Reading does not change an observation's age.
struct InstrumentReading {
  std::string key, title, unit, source;
  std::optional<double> value;
  std::optional<vessel::Duration> age;
  vessel::Quality quality = vessel::Quality::Unavailable;
  bool selected = true;
  int decimals = 1;
};
struct InstrumentView {
  InstrumentReading heading, true_speed, true_angle;
  // The compass card may explicitly show measured apparent wind when true
  // wind is unavailable. These owned readings keep that distinction visible.
  InstrumentReading wind_speed, wind_angle;
  bool wind_is_apparent = false;
  std::vector<InstrumentReading> tiles;
  // North-up visual direction, never a sensor publication or SmartNav input.
  // A relative angle needs coherent true heading before being drawn north-up.
  // "true" here describes the north reference, not the wind calculation.
  std::optional<double> wind_bearing_true_deg;
  std::string wind_direction_reason;
  bool replayed = false, simulated = false;
};
InstrumentView PresentInstruments(const vessel::VesselState &,
                                  const std::vector<std::string> &selection,
                                  vessel::Time now);
} // namespace opennav::application

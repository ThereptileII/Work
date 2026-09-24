#pragma once
#include "vessel/SensorRegistry.h"
namespace opennav::application {
// Configuration only. A wire path is interpreted by the integration decoder,
// never by an instrument widget or SmartNav consumer.
struct SignalKMapping {
  std::string path;
  vessel::Quantity quantity = vessel::Quantity::MotorPower;
  double scale = 1, offset = 0;
};
std::vector<SignalKMapping> ImportSignalKMappings(const std::string &csv);
std::string ExportSignalKMappings(const std::vector<SignalKMapping> &mappings);
void ValidateSignalKMappings(const std::vector<SignalKMapping> &mappings);
} // namespace opennav::application

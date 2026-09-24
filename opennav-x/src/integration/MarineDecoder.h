#pragma once
#include "vessel/SensorRegistry.h"
#include <cstdint>
#include <vector>

namespace opennav::integration {
const std::vector<std::uint64_t> &InstrumentPgns();
const std::vector<std::string> &InstrumentSentences();
std::vector<vessel::SensorObservation> DecodeN2kInstruments(
    std::uint64_t pgn, const std::vector<unsigned char> &payload,
    const std::string &interface_identity, vessel::Time observation);
std::vector<vessel::SensorObservation>
Decode0183Instruments(const std::string &sentence,
                      const std::string &interface_identity,
                      vessel::Time observation);
// Explicit extension mapping. Canonical-unit scale/offset must be configured;
// no proprietary propulsion/battery wire meanings are guessed.
struct SignalKBinding {
  std::string path;
  vessel::Quantity quantity = vessel::Quantity::MotorPower;
  double scale = 1, offset = 0;
};
std::vector<vessel::SensorObservation> DecodeSignalKInstruments(
    const std::string &json, const std::string &self_context,
    const std::string &interface_identity, vessel::Time received,
    std::chrono::system_clock::time_point wall_received,
    const std::vector<SignalKBinding> &bindings = {});
} // namespace opennav::integration

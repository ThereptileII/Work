#pragma once
#include "vessel/SensorRegistry.h"
#include <array>
#include <cstdint>

namespace opennav::adapters {
// Explicit mapping for the inspected boat-side marine producer. This adapter
// contains no EV CAN identifiers or decoder and never sends a message.
struct BoatN2kBinding {
  std::string interface_id, name;
};
void ValidateBoatN2kBinding(const BoatN2kBinding &binding);
class BoatN2k final {
public:
  void Configure(const BoatN2kBinding &binding);
  void Reset();
  bool Matches(const std::string &claimed_identity) const;
  std::vector<vessel::SensorObservation>
  Observe(const std::string &claimed_identity, unsigned address,
          std::uint32_t pgn, const std::vector<std::uint8_t> &data,
          vessel::Time at, vessel::Time now);
  void Map(std::vector<vessel::SensorObservation> &samples,
           vessel::Time now) const;
  void Assess(vessel::Sample &sample, vessel::Quantity quantity,
              vessel::Time now) const;
  std::string Status(vessel::Time now) const;

private:
  BoatN2kBinding binding_;
  std::optional<vessel::Time> observed_;
  unsigned version_ = 0, mask_ = 0;
  std::array<vessel::Time, 4> usable_since_{};
};
} // namespace opennav::adapters

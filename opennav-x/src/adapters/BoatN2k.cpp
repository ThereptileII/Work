#include "adapters/BoatN2k.h"
#include <algorithm>
#include <stdexcept>

namespace opennav::adapters {
using vessel::Quantity;
void ValidateBoatN2kBinding(const BoatN2kBinding &b) {
  if (b.interface_id.empty() && b.name.empty())
    return;
  if (b.interface_id.empty() || b.interface_id.size() > 140 ||
      b.name.size() != 16 ||
      !std::all_of(b.interface_id.begin(), b.interface_id.end(),
                   [](unsigned char c) { return c >= 32 && c != 127; }) ||
      !std::all_of(b.name.begin(), b.name.end(), [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
      }))
    throw std::invalid_argument("Boat bridge needs exact interface and 16 "
                                "lowercase hexadecimal NAME digits");
  const auto n = std::stoull(b.name, nullptr, 16);
  if (((n >> 21) & 2047) != 2046 || ((n >> 40) & 255) != 130 ||
      ((n >> 49) & 127) != 25 || ((n >> 60) & 7) != 4)
    throw std::invalid_argument(
        "NAME does not match the inspected boat propulsion bridge class");
}
void BoatN2k::Configure(const BoatN2kBinding &b) {
  ValidateBoatN2kBinding(b);
  if (b.interface_id != binding_.interface_id || b.name != binding_.name)
    Reset();
  binding_ = b;
}
void BoatN2k::Reset() {
  observed_.reset();
  version_ = mask_ = 0;
}
bool BoatN2k::Matches(const std::string &identity) const {
  return !binding_.interface_id.empty() &&
         identity == binding_.interface_id + "/NAME-" + binding_.name;
}
std::vector<vessel::SensorObservation>
BoatN2k::Observe(const std::string &identity, unsigned address,
                 std::uint32_t pgn, const std::vector<std::uint8_t> &data,
                 vessel::Time at, vessel::Time now) {
  if (!Matches(identity) || address >= 254 || pgn != 61184 || at > now ||
      now - at >= std::chrono::seconds(1) || (observed_ && at <= *observed_))
    return {};
  const auto previous_mask =
      version_ == 2 && observed_ &&
              at - *observed_ < std::chrono::milliseconds(500)
          ? mask_
          : 0u;
  observed_ = at;
  version_ = mask_ = 0;
  const std::string device = "NMEA2000/" + identity + "/source-" +
                             std::to_string(address) + "/instance-0";
  const auto source = device + "/PGN-61184/explicit-boat-regeneration";
  vessel::Sample sample{{}, source, at, vessel::Validity::Invalid};
  sample.device_id = device;
  // v1 was inspected but cannot attest underlying sensor freshness. v2 adds
  // independent producer expiry bits: power/SOC, RPM, temperature, gear/regen.
  if (data.size() == 8 && (data[1] == 1 || data[1] == 2) && !(data[3] & 8) &&
      (data[1] != 1 || !(data[3] & 0xf0))) {
    version_ = data[1];
    mask_ = data[3] >> 4;
    if (version_ == 2)
      for (unsigned i = 0; i < 4; ++i)
        if ((mask_ & (1u << i)) && !(previous_mask & (1u << i)))
          usable_since_[i] = at;
    if ((data[3] & 1) && data[2] <= 2) {
      sample.value = data[2];
      sample.validity = vessel::Validity::Measured;
    }
  }
  Assess(sample, Quantity::Regeneration, now);
  return {{Quantity::Regeneration, source, sample, 10}};
}
void BoatN2k::Assess(vessel::Sample &s, Quantity q, vessel::Time now) const {
  if (binding_.interface_id.empty())
    return;
  const auto prefix = "NMEA2000/" + binding_.interface_id + "/NAME-" +
                      binding_.name + "/source-";
  if (s.device_id.rfind(prefix, 0) != 0)
    return;
  const auto reason = [&s](const char *text) {
    if (s.source.find(text) == std::string::npos)
      s.source += std::string(" / ") + text;
  };
  unsigned group = 0;
  switch (q) {
  case Quantity::BatteryVoltage:
  case Quantity::BatteryNativeCurrent:
  case Quantity::BatterySoc:
    group = 1;
    break;
  case Quantity::MotorRpm:
    group = 2;
    break;
  case Quantity::MotorTemperature:
  case Quantity::CoolantTemperature:
    group = 4;
    break;
  case Quantity::Gear:
  case Quantity::Regeneration:
    group = 8;
    break;
  default:
    return;
  }
  const bool heartbeat = observed_ && now >= *observed_ &&
                         now - *observed_ < std::chrono::milliseconds(500);
  if (!heartbeat || version_ != 2) {
    if (s.value && s.validity != vessel::Validity::Invalid)
      s.validity = vessel::Validity::Uncertain;
    reason("producer expiry unverified");
  } else if (!(mask_ & group)) {
    s.value.reset();
    s.validity = vessel::Validity::Invalid;
    reason("producer sensor expired");
  } else {
    const unsigned index = group == 1 ? 0 : group == 2 ? 1 : group == 4 ? 2 : 3;
    if (s.observed_at < usable_since_[index]) {
      s.value.reset();
      s.validity = vessel::Validity::Invalid;
      reason("awaiting new sample after producer expiry");
    }
  }
}
void BoatN2k::Map(std::vector<vessel::SensorObservation> &samples,
                  vessel::Time now) const {
  if (binding_.interface_id.empty())
    return;
  const auto prefix = "NMEA2000/" + binding_.interface_id + "/NAME-" +
                      binding_.name + "/source-";
  samples.erase(
      std::remove_if(samples.begin(), samples.end(),
                     [&](const auto &o) {
                       return o.quantity == Quantity::Fuel &&
                              o.sample.device_id.rfind(prefix, 0) == 0 &&
                              o.sample.device_id.size() >= 11 &&
                              o.sample.device_id.compare(
                                  o.sample.device_id.size() - 11, 11,
                                  "/instance-0") == 0;
                     }),
      samples.end()); // Virtual SOC tank is never presented as physical fuel.
  for (auto &o : samples) {
    if (o.quantity == Quantity::CoolantTemperature &&
        o.sample.device_id.rfind(prefix, 0) == 0 &&
        o.sample.device_id.compare(o.sample.device_id.size() - 11, 11,
                                   "/instance-0") == 0) {
      o.quantity = Quantity::MotorTemperature;
      o.source_id += "/explicit-boat-motor-temperature";
      o.sample.source += " / configured boat motor-temperature field";
    }
    // Do not alter retained validity here: Assess is repeated on copied output
    // so losing/recovering heartbeat never refreshes a measurement timestamp.
    (void)now;
  }
}
std::string BoatN2k::Status(vessel::Time now) const {
  if (binding_.interface_id.empty())
    return "Boat propulsion mapping unconfigured";
  if (!observed_ || now < *observed_ ||
      now - *observed_ >= std::chrono::milliseconds(500))
    return "Boat bridge heartbeat unavailable/stale; producer expiry "
           "unverified";
  if (version_ != 2)
    return "Boat bridge requires reviewed v2 producer expiry firmware; values "
           "uncertain";
  return "Boat v2 expiry contract observed; sensor freshness mask " +
         std::to_string(mask_) + " / physical validation required";
}
} // namespace opennav::adapters

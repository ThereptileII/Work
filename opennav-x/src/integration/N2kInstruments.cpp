#include "N2kMessages.h"
#include "integration/MarineDecoder.h"
#include <algorithm>
#include <cmath>

namespace opennav::integration {
using vessel::Quantity;
const std::vector<std::uint64_t> &InstrumentPgns() {
  static const std::vector<std::uint64_t> pgns = {
      127245, 127250, 127257, 127488, 127505, 127506,
      127508, 128259, 128267, 130306, 130310, 130316};
  return pgns;
}
std::vector<vessel::SensorObservation>
DecodeN2kInstruments(std::uint64_t pgn, const std::vector<unsigned char> &bytes,
                     const std::string &iface, vessel::Time at) {
  std::vector<vessel::SensorObservation> out;
  if (iface.empty() || iface.size() > 200 || bytes.size() < 14 ||
      bytes[0] != 0x93 ||
      std::find(InstrumentPgns().begin(), InstrumentPgns().end(), pgn) ==
          InstrumentPgns().end())
    return out;
  const auto header = std::uint64_t(bytes[3]) | (std::uint64_t(bytes[4]) << 8) |
                      (std::uint64_t(bytes[5]) << 16);
  const std::size_t length = bytes[12], minimum = pgn == 127489   ? 26
                                                  : pgn == 127506 ? 11
                                                                  : 8;
  if (header != pgn || length < minimum || length > tN2kMsg::MaxDataLen ||
      bytes.size() != length + 14 || bytes[7] >= 254)
    return out;
  // Validate the normalized OpenCPN receive envelope before the pinned parser.
  // No parser wrapper may read a short header or uninitialized trailing bytes.
  tN2kMsg message;
  message.SetPGN(static_cast<unsigned long>(pgn));
  message.Source = bytes[7];
  message.Destination = bytes[6];
  message.Priority = bytes[2];
  message.DataLen = static_cast<int>(length);
  std::copy(bytes.begin() + 13, bytes.begin() + 13 + length, message.Data);
  const std::string base =
      "NMEA2000/" + iface + "/source-" + std::to_string(bytes[7]);
  unsigned char sid = 0, instance = 0;
  double a = N2kDoubleNA, b = N2kDoubleNA, c = N2kDoubleNA;
  auto put = [&](Quantity q, double value, unsigned inst = 0,
                 vessel::Validity validity = vessel::Validity::Measured,
                 const std::string &meaning = "") {
    const std::string device = base + "/instance-" + std::to_string(inst);
    const std::string source = device + "/PGN-" + std::to_string(pgn) + meaning;
    vessel::Sample sample{{}, source, at, vessel::Validity::Invalid};
    sample.device_id = device;
    if (std::isfinite(value) && !N2kIsNA(value)) {
      sample.value = value;
      sample.validity = validity;
    }
    const auto selection_id = q == Quantity::Heading ? device + "/PGN-" + std::to_string(pgn) : source;
    out.push_back({q, selection_id, std::move(sample), 10});
  };
  auto degrees = [](double x) {
    return N2kIsNA(x) ? x : static_cast<double>(RadToDeg(x));
  };
  auto signed_angle = [&](double x) {
    return N2kIsNA(x) ? x : std::remainder(degrees(x), 360.0);
  };
  switch (pgn) {
  case 127250: {
    tN2kHeadingReference reference;
    if (ParseN2kPGN127250(message, sid, a, b, c, reference)) {
      if (reference == N2khr_true)
        put(Quantity::Heading, degrees(a));
      else if (reference == N2khr_magnetic && !N2kIsNA(a) && !N2kIsNA(c))
        put(Quantity::Heading, std::fmod(degrees(a + c) + 720.0, 360.0), 0,
            vessel::Validity::Estimated, "/magnetic-plus-message-variation");
      else
        put(Quantity::Heading, N2kDoubleNA);
    }
    break;
  }
  case 127245: {
    tN2kRudderDirectionOrder order;
    if (ParseN2kPGN127245(message, a, instance, order, b))
      put(Quantity::Rudder, degrees(a), instance);
    break;
  }
  case 127257:
    if (ParseN2kPGN127257(message, sid, a, b, c))
      put(Quantity::Heel, degrees(c));
    break;
  case 128259: {
    tN2kSpeedWaterReferenceType reference;
    if (ParseN2kPGN128259(message, sid, a, b, reference))
      put(Quantity::WaterSpeed, msToKnots(a));
    break;
  }
  case 128267:
    if (ParseN2kPGN128267(message, sid, a, b, c))
      put(Quantity::Depth, a);
    break;
  case 130306: {
    tN2kWindReference reference;
    if (ParseN2kPGN130306(message, sid, a, b, reference)) {
      if (reference == N2kWind_Apparent) {
        put(Quantity::ApparentWindSpeed, msToKnots(a));
        put(Quantity::ApparentWindAngle, signed_angle(b));
      } else if (reference == N2kWind_True_boat ||
                 reference == N2kWind_True_water) {
        const auto provenance = reference == N2kWind_True_water
                                    ? "/calculated-water-reference"
                                    : "/calculated-ground-reference";
        put(Quantity::TrueWindSpeed, msToKnots(a), 0,
            vessel::Validity::Estimated, provenance);
        put(Quantity::TrueWindAngle, signed_angle(b), 0,
            vessel::Validity::Estimated, provenance);
      }
    }
    break;
  }
  case 130310:
    if (ParseN2kPGN130310(message, sid, a, b, c)) {
      put(Quantity::WaterTemperature, KelvinToC(a));
      put(Quantity::Pressure, PascalTohPA(c));
    }
    break;
  case 130316: {
    tN2kTempSource type;
    if (ParseN2kPGN130316(message, sid, instance, type, a, b) &&
        type == N2kts_SeaTemperature)
      put(Quantity::WaterTemperature, KelvinToC(a), instance);
    break;
  }
  case 127488: {
    int8_t trim;
    if (ParseN2kPGN127488(message, instance, a, b, trim))
      put(Quantity::MotorRpm, a, instance);
    break;
  }
  case 127505: {
    tN2kFluidType type;
    if (ParseN2kPGN127505(message, instance, type, a, b)) {
      if (type == N2kft_Water)
        put(Quantity::FreshWater, a, instance);
      else if (type == N2kft_Fuel || type == N2kft_FuelGasoline)
        put(Quantity::Fuel, a, instance);
      else if (type == N2kft_BlackWater || type == N2kft_GrayWater)
        put(Quantity::Waste, a, instance);
    }
    break;
  }
  case 127506: {
    tN2kDCType type;
    unsigned char soc, soh;
    if (ParseN2kPGN127506(message, sid, instance, type, soc, soh, a, b, c) &&
        type == N2kDCt_Battery) {
      put(Quantity::BatterySoc, N2kIsNA(soc) ? N2kDoubleNA : double(soc),
          instance);
      put(Quantity::BatterySoh, N2kIsNA(soh) ? N2kDoubleNA : double(soh),
          instance);
    }
    break;
  }
  case 127508:
    if (ParseN2kPGN127508(message, instance, a, b, c, sid)) {
      // The pinned PGN parser uses signed 0.01 V; its encoder saturates at
      // 327.66 V. Never present that clipped endpoint as a high-voltage pack.
      put(Quantity::BatteryVoltage, a >= 327.66 ? N2kDoubleNA : a, instance);
      put(Quantity::BatteryNativeCurrent, b, instance);
    }
    break;
  }
  return out;
}
} // namespace opennav::integration

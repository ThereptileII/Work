#include "N2kMessages.h"
#include "integration/MarineDecoder.h"
#include <cmath>
#include <gtest/gtest.h>
#include <iomanip>
#include <sstream>
using namespace opennav;
using namespace vessel;
using namespace integration;
using namespace std::chrono_literals;
namespace {
const Time epoch{100s};
std::vector<unsigned char> Envelope(const tN2kMsg &m) {
  std::vector<unsigned char> b = {0x93,
                                  0x13,
                                  m.Priority,
                                  static_cast<unsigned char>(m.PGN),
                                  static_cast<unsigned char>(m.PGN >> 8),
                                  static_cast<unsigned char>(m.PGN >> 16),
                                  255,
                                  45,
                                  255,
                                  255,
                                  255,
                                  255,
                                  static_cast<unsigned char>(m.DataLen)};
  b.insert(b.end(), m.Data, m.Data + m.DataLen);
  b.push_back(0x55);
  return b;
}
VesselState State(const std::vector<SensorObservation> &values) {
  SensorRegistry registry;
  for (auto v : values)
    registry.Observe(v, epoch);
  return registry.Merge({}, epoch);
}
std::string Sentence(const std::string &body) {
  unsigned check = 0;
  for (unsigned char c : body)
    check ^= c;
  std::ostringstream s;
  s << '$' << body << '*' << std::hex << std::uppercase << std::setw(2)
    << std::setfill('0') << check << "\r\n";
  return s.str();
}
std::vector<SensorObservation>
Sk(const std::string &values,
   const std::string &time = "2026-09-24T12:00:00.000Z",
   const std::string &context = "vessels.test") {
  return DecodeSignalKInstruments(
      "{\"context\":\"" + context + "\",\"updates\":[{\"timestamp\":\"" + time +
          "\",\"$source\":\"test\",\"values\":[" + values + "]}]}",
      "vessels.test", "loopback", epoch,
      std::chrono::system_clock::time_point{std::chrono::seconds(1790251200)});
}
} // namespace
TEST(OpenNavMarine, N2kBatteryPreservesInstanceAndSign) {
  tN2kMsg message;
  SetN2kPGN127508(message, 2, 48, -21, N2kDoubleNA, 7);
  const auto observations =
      DecodeN2kInstruments(message.PGN, Envelope(message), "test-bus", epoch);
  ASSERT_EQ(observations.size(), 2u);
  const auto s = State(observations);
  EXPECT_NEAR(*s.battery.voltage_v.value, 48, .01);
  EXPECT_NEAR(*s.battery.current_native_a.value, -21, .1);
  EXPECT_EQ(s.battery.voltage_v.observed_at,
            s.battery.current_native_a.observed_at);
  EXPECT_EQ(s.battery.voltage_v.device_id,
            s.battery.current_native_a.device_id);
  EXPECT_FALSE(s.battery.net_discharge_kw.value);
  EXPECT_FALSE(s.battery.current_a.value);
  EXPECT_NE(s.battery.voltage_v.device_id.find("instance-2"),
            std::string::npos);
}
TEST(OpenNavMarine, N2kHighVoltageSaturationIsUnavailable) {
  tN2kMsg message;
  SetN2kPGN127508(message, 2, 343, -21, N2kDoubleNA, 7);
  auto s = State(
      DecodeN2kInstruments(message.PGN, Envelope(message), "test", epoch));
  EXPECT_FALSE(s.battery.voltage_v.value);
  auto bytes = Envelope(message);
  bytes[14] = 0xfc;
  bytes[15] =
      0x85; // Unsigned 343 V cannot be reinterpreted as this signed contract.
  s = State(DecodeN2kInstruments(message.PGN, bytes, "test", epoch));
  EXPECT_FALSE(s.battery.voltage_v.value);
}
TEST(OpenNavMarine, N2kSocAndSohDoNotInventCapacity) {
  tN2kMsg m;
  SetN2kPGN127506(m, 1, 2, N2kDCt_Battery, 68, 94, 3600, .1, 72000);
  auto s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "test-bus", epoch));
  EXPECT_EQ(s.battery.soc_percent.value, 68);
  EXPECT_EQ(s.battery.soh_percent.value, 94);
  EXPECT_FALSE(s.battery.usable_capacity_kwh.value);
  SetN2kPGN127506(m, 1, 2, N2kDCt_Battery, 255, 255, N2kDoubleNA, N2kDoubleNA,
                  N2kDoubleNA);
  s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "test-bus", epoch));
  EXPECT_FALSE(s.battery.soc_percent.value);
  EXPECT_FALSE(s.battery.soh_percent.value);
}
TEST(OpenNavMarine, N2kRejectsEveryTruncatedEnvelope) {
  tN2kMsg m;
  SetN2kPGN127508(m, 0, 12, 2);
  auto bytes = Envelope(m);
  for (std::size_t length = 0; length < bytes.size(); ++length)
    EXPECT_TRUE(DecodeN2kInstruments(m.PGN,
                                     {bytes.begin(), bytes.begin() + length},
                                     "test", epoch)
                    .empty());
  bytes[12] = 255;
  EXPECT_TRUE(DecodeN2kInstruments(m.PGN, bytes, "test", epoch).empty());
  bytes = Envelope(m);
  bytes[0] = 0x94;
  EXPECT_TRUE(DecodeN2kInstruments(m.PGN, bytes, "test", epoch).empty());
  EXPECT_TRUE(DecodeN2kInstruments(127506, Envelope(m), "test", epoch).empty());
  EXPECT_TRUE(DecodeN2kInstruments(m.PGN, Envelope(m), "", epoch).empty());
}
TEST(OpenNavMarine, N2kHeadingRequiresReferenceProvenance) {
  tN2kMsg m;
  SetN2kPGN127250(m, 1, DegToRad(80), N2kDoubleNA, N2kDoubleNA, N2khr_true);
  auto s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch));
  ASSERT_TRUE(s.navigation.heading_true_deg.value);
  EXPECT_NEAR(*s.navigation.heading_true_deg.value, 80, .01);
  EXPECT_EQ(s.navigation.heading_true_deg.validity, Validity::Measured);
  SetN2kPGN127250(m, 1, DegToRad(80), N2kDoubleNA, DegToRad(4), N2khr_magnetic);
  s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch));
  EXPECT_NEAR(*s.navigation.heading_true_deg.value, 84, .01);
  EXPECT_EQ(s.navigation.heading_true_deg.validity, Validity::Estimated);
  SetN2kPGN127250(m, 1, DegToRad(80), N2kDoubleNA, N2kDoubleNA, N2khr_magnetic);
  EXPECT_FALSE(State(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch))
                   .navigation.heading_true_deg.value);
}
TEST(OpenNavMarine, N2kWindDoesNotConfuseNorthAndRelativeAngles) {
  tN2kMsg m;
  SetN2kPGN130306(m, 1, 5, DegToRad(270), N2kWind_Apparent);
  auto s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch));
  EXPECT_NEAR(*s.wind.apparent_angle_deg.value, -90, .01);
  EXPECT_NEAR(*s.wind.apparent_speed_kn.value, msToKnots(5), .01);
  EXPECT_FALSE(s.wind.true_angle_deg.value);
  SetN2kPGN130306(m, 1, 5, DegToRad(270), N2kWind_True_North);
  EXPECT_TRUE(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch).empty());
  SetN2kPGN130306(m, 1, 5, DegToRad(270), N2kWind_True_water);
  s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch));
  EXPECT_EQ(s.wind.true_angle_deg.validity, Validity::Estimated);
}
TEST(OpenNavMarine, N2kDepthRudderRpmAndTankUnits) {
  tN2kMsg m;
  SetN2kPGN128267(m, 1, 8.4, -1.2, 50);
  auto s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch));
  EXPECT_NEAR(*s.environment.depth_below_transducer_m.value, 8.4, .01);
  SetN2kPGN127245(m, DegToRad(-10));
  s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch));
  EXPECT_NEAR(*s.rudder.angle_deg.value, -10, .01);
  SetN2kPGN127488(m, 0, 820, N2kDoubleNA, N2kInt8NA);
  s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch));
  EXPECT_EQ(s.propulsion.motor_rpm.value, 820);
  SetN2kPGN127505(m, 0, N2kft_Water, 72, 200);
  s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch));
  EXPECT_NEAR(*s.tanks.fresh_water_percent.value, 72, .01);
}
TEST(OpenNavMarine, NmeaMissingValuesNeverBecomeZero) {
  auto s = State(Decode0183Instruments(Sentence("IIDPT,,0"), "test", epoch));
  EXPECT_FALSE(s.environment.depth_below_transducer_m.value);
  s = State(Decode0183Instruments(Sentence("IIDPT,0,0"), "test", epoch));
  EXPECT_EQ(s.environment.depth_below_transducer_m.value, 0);
  s = State(Decode0183Instruments(Sentence("IIDPT,nan,0"), "test", epoch));
  EXPECT_FALSE(s.environment.depth_below_transducer_m.value);
  s = State(Decode0183Instruments(Sentence("IIDPT,8.4,-2"), "test", epoch));
  EXPECT_EQ(s.environment.depth_below_transducer_m.value, 8.4);
  s = State(Decode0183Instruments(Sentence("IIVHW,80,T,75,M,6,N,11.1,K"),
                                  "test", epoch));
  EXPECT_EQ(s.navigation.heading_true_deg.value, 80);
  EXPECT_EQ(s.navigation.stw_kn.value, 6);
}
TEST(OpenNavMarine, NmeaChecksumsStatusAndWindUnits) {
  auto good = Sentence("IIMWV,270,R,5,M,A");
  auto s = State(Decode0183Instruments(good, "test", epoch));
  ASSERT_TRUE(s.wind.apparent_speed_kn.value);
  EXPECT_NEAR(*s.wind.apparent_speed_kn.value, 5 * 3600.0 / 1852, .001);
  EXPECT_EQ(s.wind.apparent_angle_deg.value, -90);
  good[8] = '1';
  EXPECT_TRUE(Decode0183Instruments(good, "test", epoch).empty());
  EXPECT_TRUE(
      Decode0183Instruments("$IIMWV,270,R,5,M,A", "test", epoch).empty());
  s = State(
      Decode0183Instruments(Sentence("IIMWV,270,R,5,M,V"), "test", epoch));
  EXPECT_FALSE(s.wind.apparent_speed_kn.value);
  s = State(Decode0183Instruments(Sentence("IIRSA,-4,A,,V"), "test", epoch));
  EXPECT_EQ(s.rudder.angle_deg.value, -4);
  s = State(Decode0183Instruments(Sentence("IIMTW,15.4,C"), "test", epoch));
  EXPECT_EQ(s.environment.water_temperature_c.value, 15.4);
}
TEST(OpenNavMarine, SignalKIntegerRatioAndPackIdentity) {
  const auto out = Sk(
      R"({"path":"electrical.batteries.pack.voltage","value":343},{"path":"electrical.batteries.pack.current","value":21},{"path":"electrical.batteries.pack.capacity.stateOfCharge","value":0.68})");
  auto s = State(out);
  EXPECT_EQ(s.battery.voltage_v.value, 343);
  EXPECT_EQ(s.battery.current_native_a.value, 21);
  EXPECT_NEAR(*s.battery.soc_percent.value, 68, .0001);
  EXPECT_EQ(s.battery.soc_percent.device_id, s.battery.voltage_v.device_id);
  EXPECT_FALSE(s.battery.net_discharge_kw.value);
  NormalizeBatteryPower(s, s.battery.soc_percent.device_id,
                        CurrentConvention::PositiveDischarge, epoch);
  EXPECT_NEAR(*s.battery.net_discharge_kw.value, 7.203, .0001);
}
TEST(OpenNavMarine, SignalKContextTimestampAndNulls) {
  const std::string value =
      R"({"path":"environment.depth.belowTransducer","value":8.4})";
  EXPECT_TRUE(Sk(value, "2026-09-24T12:00:00Z", "vessels.other").empty());
  EXPECT_TRUE(Sk(value, "2026-09-24T12:00:01Z").empty());
  EXPECT_TRUE(Sk(value, "2026-02-30T12:00:00Z").empty());
  EXPECT_TRUE(Sk(value, "2026-09-24T12:00:00+01:00").empty());
  EXPECT_TRUE(Sk(value, "").empty());
  auto s = State(Sk(value, "2026-09-24T11:59:55Z"));
  EXPECT_EQ(Assess(s.environment.depth_below_transducer_m, epoch).quality,
            Quality::Stale);
  for (const auto bad : {"null", "true", "\"8.4\""}) {
    s = State(
        Sk(std::string(
               "{\"path\":\"environment.depth.belowTransducer\",\"value\":") +
           bad + "}"));
    EXPECT_FALSE(s.environment.depth_below_transducer_m.value);
  }
}
TEST(OpenNavMarine, SignalKSourceMissingAndUnknownPaths) {
  auto v = DecodeSignalKInstruments(
      R"({"context":"vessels.test","updates":[{"timestamp":"2026-09-24T12:00:00Z","values":[{"path":"environment.depth.belowTransducer","value":8}]}]})",
      "vessels.test", "test", epoch,
      std::chrono::system_clock::time_point{std::chrono::seconds(1790251200)});
  EXPECT_TRUE(v.empty());
  EXPECT_TRUE(Sk(R"({"path":"unknown.leaf.rpm","value":800})").empty());
  EXPECT_TRUE(
      Sk(R"({"path":"navigation.position","value":{"latitude":10,"longitude":12}})")
          .empty());
}
TEST(OpenNavMarine, SignalKPropulsionAndExplicitExtensionMapping) {
  auto s = State(Sk(
      R"({"path":"propulsion.port.revolutions","value":15},{"path":"propulsion.port.coolantTemperature","value":320})"));
  EXPECT_EQ(s.propulsion.motor_rpm.value, 900);
  EXPECT_NEAR(*s.propulsion.coolant_temperature_c.value, 46.85, .001);
  EXPECT_FALSE(s.propulsion.motor_temperature_c.value);
  const auto json =
      R"({"context":"vessels.test","updates":[{"timestamp":"2026-09-24T12:00:00Z","$source":"test","values":[{"path":"propulsion.port.electricalPower","value":7200}]}]})";
  const auto wall =
      std::chrono::system_clock::time_point{std::chrono::seconds(1790251200)};
  EXPECT_TRUE(
      DecodeSignalKInstruments(json, "vessels.test", "test", epoch, wall)
          .empty());
  s = State(DecodeSignalKInstruments(
      json, "vessels.test", "test", epoch, wall,
      {{"propulsion.port.electricalPower", Quantity::MotorPower, .001, 0}}));
  EXPECT_NEAR(*s.propulsion.electrical_power_kw.value, 7.2, .001);
  EXPECT_FALSE(s.battery.net_discharge_kw.value);
}

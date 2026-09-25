#include "N2kMessages.h"
#include "application/Settings.h"
#include "integration/MarineDecoder.h"
#include "integration/ExternalJson.h"
#include "integration/N2kSourceIdentity.h"
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
TEST(OpenNavMarine, RealAddressClaimUnifiesPackAcrossDifferentPgns) {
  N2kSourceIdentity identities;
  const std::vector<unsigned char> name{0x45,0x23,0xc1,0xff,0,0x87,0x50,0xc0};
  ASSERT_EQ(identities.Observe("real-bus",45,name,epoch,epoch),ClaimResult::Changed);
  SensorRegistry registry;
  tN2kMsg dc,soc;
  SetN2kPGN127508(dc,0,48,-21,N2kDoubleNA,1);
  SetN2kPGN127506(soc,1,0,N2kDCt_Battery,68,255,N2kDoubleNA,N2kDoubleNA,N2kDoubleNA);
  for(const auto* m:{&dc,&soc})
    for(auto observation:DecodeN2kInstruments(m->PGN,Envelope(*m),identities.Label("real-bus",45),epoch+1ms))
      registry.Observe(std::move(observation),epoch+1ms);
  auto state=registry.Merge({},epoch+1ms);
  ASSERT_TRUE(state.battery.soc_percent.value);
  ASSERT_EQ(state.battery.soc_percent.device_id,state.battery.voltage_v.device_id);
  ASSERT_EQ(state.battery.soc_percent.device_id,state.battery.current_native_a.device_id);
  EXPECT_NE(state.battery.soc_percent.device_id.find("NAME-c0508700ffc12345"),std::string::npos);
  NormalizeBatteryPower(state,state.battery.soc_percent.device_id,CurrentConvention::PositiveCharge,epoch+1ms);
  ASSERT_TRUE(state.battery.net_discharge_kw.value);
  EXPECT_NEAR(*state.battery.net_discharge_kw.value,1.008,.001);
  auto changed=name;changed[0]++;
  ASSERT_EQ(identities.Observe("real-bus",45,changed,epoch+2ms,epoch+2ms),ClaimResult::Changed);
  registry.Clear();
  for(auto observation:DecodeN2kInstruments(dc.PGN,Envelope(dc),identities.Label("real-bus",45),epoch+3ms))
    registry.Observe(std::move(observation),epoch+3ms);
  state=registry.Merge({},epoch+3ms);
  EXPECT_FALSE(state.battery.soc_percent.value);
  EXPECT_FALSE(state.battery.net_discharge_kw.value);
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
TEST(OpenNavMarine, SignalKFractionalTimestampNativeClockResolution) {
  const auto observations =
      Sk(R"({"path":"environment.depth.belowTransducer","value":8.4})",
         "2026-09-24T11:59:59.123456789Z");
  ASSERT_EQ(observations.size(), 1u);
  const auto age = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       epoch - observations.front().sample.observed_at)
                       .count();
  // Windows system_clock is 100 ns; Linux is normally 1 ns. Truncation must
  // make this observation older, never move its timestamp into the future.
  EXPECT_GE(age, 876543211);
  EXPECT_LE(age, 876543311);
  EXPECT_TRUE(Sk(R"({"path":"environment.depth.belowTransducer","value":8.4})",
                 "2026-09-24T12:00:00.000000100Z")
                  .empty());
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

// This joins the pinned Signal K decoder, registry, persisted explicit settings
// and energy core. Route geometry itself remains covered by
// OpenNavRouteGeometry.
TEST(OpenNavMarine, ConfiguredLiveBatteryFeedsEnergyWithoutDemoDefaults) {
  auto state = State(Sk(
      R"({"path":"electrical.batteries.propulsion.voltage","value":48},{"path":"electrical.batteries.propulsion.current","value":-50},{"path":"electrical.batteries.propulsion.capacity.stateOfCharge","value":0.8})"));
  ASSERT_TRUE(state.battery.soc_percent.value);
  state.navigation.sog_kn = {5, "Test selected navigation", epoch,
                             Validity::Measured};
  auto route = std::make_shared<RouteProgressSnapshot>();
  route->state = RouteState::Valid;
  route->route_id = "contract-route";
  route->route_revision = 1;
  route->revision_scope = "test";
  route->active_waypoint_id = "destination";
  route->active_waypoint_index = 0;
  route->waypoint_count = 1;
  route->remaining_distance_nm = 10;
  route->observed_at = epoch;
  route->position_observed_at = epoch;
  route->source = "Test route publication / no geometry inference";
  route->position_source = "Test selected navigation";
  state.navigation.route = route;
  application::Settings setting;
  setting.energy.battery = {24, 20, .5, "Explicit test calibration"};
  setting.energy.battery_device_id = state.battery.soc_percent.device_id;
  setting.current = CurrentConvention::PositiveCharge;
  const auto loaded =
      application::DecodeSettings(application::EncodeSettings(setting));
  NormalizeBatteryPower(state, loaded.energy.battery_device_id, loaded.current,
                        epoch);
  ASSERT_EQ(state.battery.net_discharge_kw.value, 2.4);
  ASSERT_FALSE(state.simulated);
  auto prediction =
      smartnav::PredictConfiguredEnergy(loaded.energy, state, epoch);
  ASSERT_TRUE(prediction.range.estimate);
  EXPECT_NEAR(prediction.range.estimate->range_nm, 30, 1e-9);
  ASSERT_TRUE(prediction.arrival.estimate);
  EXPECT_NEAR(*prediction.arrival.estimate->soc_percent, 60, 1e-9);
  EXPECT_EQ(prediction.input_route, state.navigation.route);
  EXPECT_FALSE(state.battery.usable_capacity_kwh.value);
  EXPECT_FALSE(
      smartnav::PredictConfiguredEnergy(loaded.energy, state, epoch + 5s)
          .arrival.estimate);
  state.battery.soc_percent.device_id = "different-pack";
  EXPECT_FALSE(smartnav::PredictConfiguredEnergy(loaded.energy, state, epoch)
                   .arrival.estimate);
}

TEST(OpenNavMarine, PersistedPropulsionMappingsPreserveAgeAndDomain) {
  application::Settings configured;
  configured.signal_k_mappings = application::ImportSignalKMappings(
      "OpenNavXSignalK,1\npath,quantity,scale,offset\n"
      "propulsion.main.electricalPower,motor_power,0.001,0\n"
      "propulsion.main.motorTemperature,motor_temperature,1,-273.15\n");
  const auto settings =
      application::DecodeSettings(application::EncodeSettings(configured));
  const auto wall =
      std::chrono::system_clock::time_point{std::chrono::seconds(1790251200)};
  const std::string json =
      R"({"context":"vessels.test","updates":[{"timestamp":"2026-09-24T11:59:58Z","$source":"boat-bridge","values":[{"path":"propulsion.main.electricalPower","value":7200},{"path":"propulsion.main.motorTemperature","value":335.15}]}]})";
  auto samples = DecodeSignalKInstruments(json, "vessels.test", "boat", epoch,
                                          wall, settings.signal_k_mappings);
  ASSERT_EQ(samples.size(), 2u);
  auto state = State(samples);
  ASSERT_TRUE(state.propulsion.electrical_power_kw.value);
  EXPECT_NEAR(*state.propulsion.electrical_power_kw.value, 7.2, 1e-9);
  EXPECT_NEAR(*state.propulsion.motor_temperature_c.value, 62, 1e-9);
  EXPECT_EQ(state.propulsion.electrical_power_kw.observed_at, epoch - 2s);
  EXPECT_EQ(Assess(state.propulsion.electrical_power_kw, epoch + 3s).quality,
            Quality::Stale);
  EXPECT_NE(state.propulsion.electrical_power_kw.source.find("boat-bridge"),
            std::string::npos);
  EXPECT_FALSE(state.battery.net_discharge_kw.value);
  EXPECT_TRUE(
      DecodeSignalKInstruments(json, "vessels.test", "boat", epoch, wall, {})
          .empty());
  auto invalid = json;
  invalid.replace(invalid.find("7200"), 4, "null");
  state = State(DecodeSignalKInstruments(invalid, "vessels.test", "boat", epoch,
                                         wall, settings.signal_k_mappings));
  EXPECT_FALSE(state.propulsion.electrical_power_kw.value);
  invalid = json;
  invalid.replace(invalid.find("7200"), 4, "1e308");
  state = State(DecodeSignalKInstruments(invalid, "vessels.test", "boat", epoch,
                                         wall, settings.signal_k_mappings));
  EXPECT_FALSE(state.propulsion.electrical_power_kw.value);
}

namespace {
tN2kMsg Wire(unsigned pgn, std::initializer_list<unsigned char> bytes) {
  tN2kMsg m;
  m.SetPGN(pgn);
  m.DataLen = static_cast<int>(bytes.size());
  std::copy(bytes.begin(), bytes.end(), m.Data);
  return m;
}
} // namespace
TEST(OpenNavMarine, DcWideRangeVoltageCurrentFromBoatWire) {
  // Independent byte fixture from boat firmware's n2kSend127751: 343.0 V,
  // -21.00 A (positive charging convention), connection 0. No Leaf frames.
  auto m = Wire(127751, {7, 0, 0x66, 0x0d, 0xcc, 0xf7, 0xff, 0xff});
  auto s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "boat", epoch));
  ASSERT_EQ(s.battery.voltage_v.value, 343);
  ASSERT_EQ(s.battery.current_native_a.value, -21);
  EXPECT_EQ(s.battery.voltage_v.device_id,
            s.battery.current_native_a.device_id);
  NormalizeBatteryPower(s, s.battery.voltage_v.device_id,
                        CurrentConvention::PositiveCharge, epoch);
  ASSERT_TRUE(s.battery.net_discharge_kw.value);
  EXPECT_NEAR(*s.battery.net_discharge_kw.value, 7.203, .000001);
  EXPECT_EQ(s.battery.net_discharge_kw.validity, Validity::Estimated);
  EXPECT_EQ(Assess(s.battery.net_discharge_kw, epoch + 5s).quality,
            Quality::Stale);
}
TEST(OpenNavMarine, DcWideRangeSentinelsAndUnsignedVoltage) {
  for (unsigned raw : {0x7ffffdu, 0x7ffffeu, 0x7fffffu, 0x800000u}) {
    auto m = Wire(127751, {0, 0, 0xff, 0xff, static_cast<unsigned char>(raw),
                           static_cast<unsigned char>(raw >> 8),
                           static_cast<unsigned char>(raw >> 16), 0xff});
    const auto s =
        State(DecodeN2kInstruments(m.PGN, Envelope(m), "boat", epoch));
    EXPECT_FALSE(s.battery.voltage_v.value);
    EXPECT_FALSE(s.battery.current_native_a.value);
  }
  // Boat's older signed-NA 0x7fff exceeds the accepted battery domain and is
  // invalid, not a 3276.7 V valid measurement. Standard voltage is unsigned.
  auto m = Wire(127751, {0, 0, 0xff, 0x7f, 0, 0, 0, 0xff});
  auto s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "boat", epoch));
  EXPECT_FALSE(s.battery.voltage_v.value);
  EXPECT_EQ(s.battery.current_native_a.value, 0);
  m.Data[1] = 255;
  EXPECT_TRUE(DecodeN2kInstruments(m.PGN, Envelope(m), "boat", epoch).empty());
}
TEST(OpenNavMarine, DcWideRangePrecedenceKeepsEpochCoherent) {
  SensorRegistry r;
  auto wide = Wire(127751, {7, 0, 0x66, 0x0d, 0xcc, 0xf7, 0xff, 0xff});
  tN2kMsg narrow;
  SetN2kPGN127508(narrow, 0, N2kDoubleNA, -20);
  for (const auto &m : {wide, narrow})
    for (auto o : DecodeN2kInstruments(m.PGN, Envelope(m), "boat", epoch))
      r.Observe(o, epoch);
  auto s = r.Merge({}, epoch);
  EXPECT_EQ(s.battery.current_native_a.value, -21);
  NormalizeBatteryPower(s, s.battery.voltage_v.device_id,
                        CurrentConvention::PositiveCharge, epoch);
  EXPECT_TRUE(s.battery.net_discharge_kw.value);
  // New narrow current must not mask the coherent 751 pair while fresh.
  for (auto o :
       DecodeN2kInstruments(narrow.PGN, Envelope(narrow), "boat", epoch + 1s))
    r.Observe(o, epoch + 1s);
  s = r.Merge({}, epoch + 1s);
  EXPECT_EQ(s.battery.current_native_a.observed_at, epoch);
  s = r.Merge({}, epoch + 5s);
  NormalizeBatteryPower(s, s.battery.voltage_v.device_id,
                        CurrentConvention::PositiveCharge, epoch + 5s);
  EXPECT_FALSE(s.battery.net_discharge_kw.value);
}
TEST(OpenNavMarine, EngineTemperaturePreservesFieldMeaningAndLength) {
  tN2kMsg m;
  m.SetPGN(127489);
  m.DataLen = 26;
  std::fill(m.Data, m.Data + m.DataLen, 0xff);
  m.Data[0] = 2;
  // 62 C = 335.15 K = 33515 at the inspected pinned coolant offset.
  m.Data[5] = 0xeb;
  m.Data[6] = 0x82;
  auto s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "boat", epoch));
  ASSERT_TRUE(s.propulsion.coolant_temperature_c.value);
  EXPECT_NEAR(*s.propulsion.coolant_temperature_c.value, 62, .0001);
  EXPECT_FALSE(s.propulsion.motor_temperature_c.value);
  for (unsigned raw : {0xfffd, 0xfffe, 0xffff}) {
    m.Data[5] = static_cast<unsigned char>(raw);
    m.Data[6] = static_cast<unsigned char>(raw >> 8);
    s = State(DecodeN2kInstruments(m.PGN, Envelope(m), "boat", epoch));
    EXPECT_FALSE(s.propulsion.coolant_temperature_c.value);
  }
  m.DataLen = 25;
  EXPECT_TRUE(DecodeN2kInstruments(m.PGN, Envelope(m), "boat", epoch).empty());
}
TEST(OpenNavMarine, TransmissionGearUsesPinnedDecoder) {
  for (int code = 0; code != 4; ++code) {
    tN2kMsg m;
    SetN2kPGN127493(m, 2, static_cast<tN2kTransmissionGear>(code), N2kDoubleNA,
                    N2kDoubleNA, 255);
    const auto s =
        State(DecodeN2kInstruments(m.PGN, Envelope(m), "boat", epoch));
    if (code < 3) {
      EXPECT_EQ(s.propulsion.gear_code.value, code);
      EXPECT_TRUE(s.propulsion.gear.value);
      EXPECT_EQ(s.propulsion.gear.observed_at, epoch);
    } else
      EXPECT_FALSE(s.propulsion.gear.value);
  }
}
TEST(OpenNavMarine, BetaPgnTruncationAndReservedIdentity) {
  for (const auto pgn : {127489u, 127493u, 127751u}) {
    tN2kMsg m;
    m.SetPGN(pgn);
    m.DataLen = pgn == 127489 ? 26 : 8;
    std::fill(m.Data, m.Data + m.DataLen, 0);
    const auto wire = Envelope(m);
    for (std::size_t length = 0; length < wire.size(); ++length)
      EXPECT_TRUE(DecodeN2kInstruments(
                      pgn, {wire.begin(), wire.begin() + length}, "boat", epoch)
                      .empty());
    auto bad = wire;
    bad[7] = 254;
    EXPECT_TRUE(DecodeN2kInstruments(pgn, bad, "boat", epoch).empty());
  }
}
TEST(OpenNavMarine, ErrorCodesCannotBecomePlausibleMeasurements) {
  tN2kMsg m;
  SetN2kPGN127488(m, 0, 820, N2kDoubleNA, N2kInt8NA);
  m.Data[1] = 0xfe;
  m.Data[2] = 0xff;
  EXPECT_FALSE(State(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch))
                   .propulsion.motor_rpm.value);
  SetN2kPGN130306(m, 1, 5, DegToRad(70), N2kWind_Apparent);
  m.Data[3] = 0xfe;
  m.Data[4] = 0xff;
  EXPECT_FALSE(State(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch))
                   .wind.apparent_angle_deg.value);
  SetN2kPGN127508(m, 0, 12, 20);
  m.Data[3] = 0xfe;
  m.Data[4] = 0x7f;
  EXPECT_FALSE(State(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch))
                   .battery.current_native_a.value);
  SetN2kPGN127250(m, 1, DegToRad(70), N2kDoubleNA, DegToRad(4), N2khr_magnetic);
  m.Data[1] = 0xfe;
  m.Data[2] = 0xff;
  EXPECT_FALSE(State(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch))
                   .navigation.heading_true_deg.value);
  SetN2kPGN127505(m, 0, N2kft_Error, 70, 100);
  EXPECT_TRUE(DecodeN2kInstruments(m.PGN, Envelope(m), "test", epoch).empty());
}

TEST(OpenNavMarine, BoundBoatFieldsUseActualPinnedMarineDecoder) {
  adapters::BoatN2k bridge;
  bridge.Configure({"boat", "40328200ffd23456"});
  const std::string identity="boat/NAME-40328200ffd23456";
  bridge.Observe(identity,45,61184,{1,2,2,0xf1,255,255,255,255},epoch,epoch);
  tN2kMsg coolant,tank,soc;
  coolant.SetPGN(127489);coolant.DataLen=26;
  std::fill(coolant.Data,coolant.Data+26,255);coolant.Data[0]=0;
  coolant.Data[5]=0xeb;coolant.Data[6]=0x82;
  SetN2kPGN127505(tank,0,N2kft_Fuel,68,100);
  SetN2kPGN127506(soc,1,0,N2kDCt_Battery,68,255,N2kDoubleNA,N2kDoubleNA,N2kDoubleNA);
  SensorRegistry registry;
  for(const auto *m:{&coolant,&tank,&soc}) {
    auto observations=DecodeN2kInstruments(m->PGN,Envelope(*m),identity,epoch+1ms);
    bridge.Map(observations,epoch+1ms);
    for(auto o:observations)registry.Observe(std::move(o),epoch+1ms);
  }
  auto s=registry.Merge({},epoch+1ms);
  for(const auto&q:Quantities())bridge.Assess(Field(s,q.quantity),q.quantity,epoch+1ms);
  EXPECT_NEAR(*s.propulsion.motor_temperature_c.value,62,.001);
  EXPECT_FALSE(s.propulsion.coolant_temperature_c.value);
  EXPECT_FALSE(s.tanks.fuel_percent.value);
  EXPECT_EQ(s.battery.soc_percent.value,68);
  EXPECT_EQ(s.battery.soc_percent.validity,Validity::Measured);
  bridge.Assess(s.battery.soc_percent,Quantity::BatterySoc,epoch+501ms);
  EXPECT_EQ(s.battery.soc_percent.validity,Validity::Uncertain);
}

TEST(OpenNavMarine, NmeaRejectsUntrustedBytesBeforeWxParsing) {
  for (int c = 0; c < 256; ++c) {
    if (c >= 0x20 && c <= 0x7e) continue;
    const auto body = std::string("IIDPT,8") + static_cast<char>(c) + ".4,-2";
    EXPECT_TRUE(Decode0183Instruments(Sentence(body), "test", epoch).empty()) << c;
  }
  EXPECT_TRUE(Decode0183Instruments(Sentence("IIDPT," + std::string(300, '9') + ",0"), "test", epoch).empty());
  for (const auto *value : {"1e309", "-1e309", "nan", "inf", "-inf"})
    EXPECT_FALSE(State(Decode0183Instruments(Sentence(std::string("IIDPT,") + value + ",0"), "test", epoch)).environment.depth_below_transducer_m.value);
  EXPECT_EQ(State(Decode0183Instruments(Sentence("IIDPT,0,0"), "test", epoch)).environment.depth_below_transducer_m.value, 0);
}

TEST(OpenNavMarine, N2kRejectsInvalidPriorityAndBoundedRandomEnvelopes) {
  tN2kMsg m;
  SetN2kPGN127508(m, 0, 48, -2);
  for (unsigned p = 8; p < 256; ++p) {
    auto wire = Envelope(m); wire[2] = static_cast<unsigned char>(p);
    EXPECT_TRUE(DecodeN2kInstruments(m.PGN, wire, "test", epoch).empty());
  }
  std::uint32_t random = 0x584e4156;
  for (std::size_t length = 0; length < 300; ++length) {
    std::vector<unsigned char> wire(length);
    for (auto &b : wire) { random = random * 1664525u + 1013904223u; b = static_cast<unsigned char>(random >> 24); }
    for (auto pgn : InstrumentPgns()) {
      const auto observations = DecodeN2kInstruments(pgn, wire, "test", epoch);
      EXPECT_LE(observations.size(), 4u);
      for (const auto &o : observations) {
        if (o.sample.value) { EXPECT_TRUE(std::isfinite(*o.sample.value)); }
      }
    }
  }
}

TEST(OpenNavMarine, SignalKRejectsDeepOrMalformedExternalText) {
  const auto wall = std::chrono::system_clock::time_point{1790251200s};
  const auto decode = [&](const std::string &s) { return DecodeSignalKInstruments(s, "vessels.test", "test", epoch, wall); };
  for (int depth : {17, 256, 60000})
    EXPECT_TRUE(decode(std::string(depth, '[') + "0" + std::string(depth, ']')).empty());
  for (const auto &bad : {std::string("{\"bad\":\"\xff\"}"),
                          std::string("{\"bad\":\"\xc0\x80\"}"),
                          std::string("{\"bad\":\"\xed\xa0\x80\"}"),
                          std::string("{\"bad\":\"\xf4\x90\x80\x80\"}"),
                          std::string("{\"bad\":\"\0\"}", 11),
                          std::string("{[}]"), std::string(262145, ' ')})
    EXPECT_TRUE(decode(bad).empty());
  auto observations = Sk("{\"path\":\"environment.depth.belowTransducer\",\"value\":8.4}");
  ASSERT_EQ(observations.size(), 1u);
  EXPECT_EQ(observations.front().sample.value, 8.4);
  // Resource guards do not reject legitimate UTF-8 or braces inside strings.
  EXPECT_TRUE(BoundedJsonText("{\"label\":\"\xc3\x85land \xe6\xb5\xb7 \xf0\x9f\x9a\xa4\",\"escaped\":\"\\\"{[}]\\\"\"}"));
  EXPECT_TRUE(BoundedJsonText(std::string(16, '[') + "0" + std::string(16, ']')));
  EXPECT_FALSE(BoundedJsonText(R"({"unterminated":"value})"));
}

TEST(OpenNavMarine, SignalKUnicodeSourceAndDriverFraming) {
  const auto wall = std::chrono::system_clock::time_point{1790251200s};
  auto json = std::string(R"({"context":"vessels.test","updates":[{"timestamp":"2026-09-24T12:00:00.000Z","$source":"test.\u00c5land","values":[{"path":"environment.depth.belowTransducer","value":8.4}]}]})") + "\r\n";
  const auto escaped = DecodeSignalKInstruments(json, "vessels.test", "loopback", epoch, wall);
  ASSERT_EQ(escaped.size(), 1u);
  EXPECT_EQ(escaped.front().sample.value, 8.4);
  EXPECT_NE(escaped.front().source_id.find("test.\xc3\x85land"), std::string::npos);
  json.replace(json.find("\\u00c5"), 6, "\xc3\x85");
  const auto raw = DecodeSignalKInstruments(json, "vessels.test", "loopback", epoch, wall);
  ASSERT_EQ(raw.size(), 1u);
  EXPECT_EQ(raw.front().source_id, escaped.front().source_id);
}

#include "FreshTelemetry.h"
#include "adapters/BoatN2k.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
using namespace opennav;
using namespace std::chrono_literals;
struct Telemetry {
  float packV = 343, packA = 21, socPct = 68, motorTempC = 62;
  int rpm = 820;
  bool rpmValid = true;
  std::uint8_t gear = 1, regen = 2, expiryMask = 0;
};
struct Packet {
  unsigned pgn;
  std::vector<std::uint8_t> data;
};
std::vector<Packet> packets;
bool n2kTransmit(unsigned pgn, const uint8_t *data, uint8_t size,
                 uint8_t = 255) {
  packets.push_back({pgn, {data, data + size}});
  return true;
}
bool n2kFastPacketTransmit(unsigned pgn, const uint8_t *data, uint8_t size) {
  return n2kTransmit(pgn, data, size);
}
void setU16(uint8_t *p, uint16_t v) {
  p[0] = v & 255;
  p[1] = v >> 8;
}
void setS16(uint8_t *p, int16_t v) { setU16(p, uint16_t(v)); }
void setS24(uint8_t *p, int32_t v) {
  for (unsigned i = 0; i < 3; ++i)
    p[i] = (uint32_t(v) >> (8 * i)) & 255;
}
void setU32(uint8_t *p, uint32_t v) {
  for (unsigned i = 0; i < 4; ++i)
    p[i] = (v >> (8 * i)) & 255;
}
template <class T, class A, class B> T constrain(T x, A a, B b) {
  return x < a ? T(a) : x > b ? T(b) : x;
}
uint8_t nextSid() { return 7; }
unsigned n2kFuelTankTxFrames = 0;
constexpr unsigned PGN_ENG_RAPID = 127488, PGN_TRANS_DYN = 127493,
                   PGN_FLUID_LEVEL = 127505, PGN_BATT_STATUS = 127508,
                   PGN_DC_VI = 127751, PGN_DC_STATUS = 127506,
                   PGN_ENG_DYNAMIC = 127489, PGN_PROP_A = 61184,
                   N2K_BCAST = 255, SOC_FUEL_TANK_INSTANCE = 0;
constexpr float SOC_FUEL_TANK_CAPACITY_L = 100;
using std::isfinite;
#include "FirmwareCodec.inc"
void Check(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
void Send(const Telemetry &t) {
  packets.clear();
  n2kSend127488(t.rpm, t.rpmValid);
  n2kSend127493(t.gear);
  n2kSend127508(t.packV, t.packA);
  n2kSend127751(t.packV, t.packA);
  n2kSend127506(t.socPct);
  n2kSend127505_FuelSoc(t.socPct);
  n2kSend127489_EngineTemp(t.motorTempC);
  n2kSend61184(7, t.regen, t.packV, t.packA, t.expiryMask);
}
const std::vector<uint8_t> &Data(unsigned pgn) {
  for (const auto &p : packets)
    if (p.pgn == pgn)
      return p.data;
  throw std::runtime_error("Missing frame");
}
int main() {
  try {
    using Fresh = boat_bridge::FreshTelemetry;
    Fresh f;
    Telemetry raw;
    auto empty = f.Snapshot(raw, 0);
    Check(empty.expiryMask == 0 && !isfinite(empty.packV) && !empty.rpmValid,
          "Startup never manufactures valid zero");
    f.Observe(Fresh::PowerSoc, 0);
    f.Observe(Fresh::Rpm, 0);
    f.Observe(Fresh::Temperature, 0);
    f.Observe(Fresh::GearRegen, 0);
    auto live = f.Snapshot(raw, 2499);
    Check(live.expiryMask == 15 && live.rpmValid,
          "Zero-time observation supported");
    Send(live);
    Check(Data(127751)[2] == 0x66 && Data(127751)[3] == 0x0d,
          "Actual producer unsigned 343 V");
    Check(Data(127506)[3] == 68 && Data(61184)[1] == 2 &&
              (Data(61184)[3] >> 4) == 15,
          "Actual codec v2 and SOC");
    adapters::BoatN2k adapter;
    adapter.Configure({"test", "40328200ffd23456"});
    auto regen =
        adapter.Observe("test/NAME-40328200ffd23456", 35, 61184, Data(61184),
                        vessel::Time{100s}, vessel::Time{100s});
    Check(regen.size() == 1 && regen[0].sample.value == 2 &&
              regen[0].sample.validity == vessel::Validity::Measured,
          "Actual firmware wire consumed by adapter");
    f.Observe(Fresh::GearRegen, 2499);
    auto partial = f.Snapshot(raw, 2500);
    Check(partial.expiryMask == 8 && partial.gear == 1 &&
              !isfinite(partial.packV) && !partial.rpmValid,
          "Gear traffic cannot refresh battery or RPM");
    Send(partial);
    Check(Data(127751)[2] == 255 && Data(127751)[3] == 255,
          "Expired HV uses unsigned NA");
    Check(Data(127488)[1] == 255 && Data(127488)[2] == 255 &&
              Data(127506)[3] == 255,
          "Expired RPM and SOC use NA");
    Check(Data(127489)[5] == 255 && Data(127489)[6] == 255 &&
              Data(127505)[1] == 255,
          "Expired temperature and virtual tank clear receivers");
    Check(Data(61184)[3] == 0x81, "Only actual fresh gear/regen group remains");
    f.Snapshot(raw, 5000);
    auto wrapped = f.Snapshot(raw, 0);
    Check(wrapped.expiryMask == 0,
          "Expired latch cannot revive after counter wrap");
    f.Observe(Fresh::PowerSoc, UINT32_MAX - 1000);
    Check(f.Snapshot(raw, 999).expiryMask == 1,
          "Counter wrap short interval stays valid");
    Check(f.Snapshot(raw, 1500).expiryMask == 0, "Counter wrap expiration");
    f.Observe(Fresh::PowerSoc, 1501);
    Check(f.Snapshot(raw, 1501).packV == 343,
          "Recovery requires new observed source");
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

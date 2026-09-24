#pragma once
#include <stddef.h>
#include <stdint.h>
#include <math.h>

namespace bridge {
constexpr double pi = 3.14159265358979323846;
inline double wrap(double v) { v = fmod(v, 360.0); return v < 0 ? v + 360 : v; }
inline double difference(double to, double from) { return wrap(to - from + 180) - 180; }
inline bool fresh(uint32_t now, uint32_t at, uint32_t limit) { return uint32_t(now-at) < limit; }
inline uint16_t u16(const uint8_t *p) { return p[0] | uint16_t(p[1]) << 8; }
inline uint32_t u32(const uint8_t *p) { return p[0] | uint32_t(p[1])<<8 | uint32_t(p[2])<<16 | uint32_t(p[3])<<24; }
inline bool angleValid(uint16_t v) { return v <= 62832; }
inline double angle(uint16_t v) { return wrap(v * 0.0001 * 180 / pi); }
inline void put16(uint8_t *p, uint16_t v) { p[0]=v; p[1]=v>>8; }

enum class Mode : uint8_t { Unknown, Standby, Auto, Wind, Track };
const char *modeName(Mode mode);
struct Pilot {
  Mode mode = Mode::Unknown;
  double heading = NAN, target = NAN, rudder = NAN;
  bool offCourse = false, windShift = false;
  uint32_t at = 0, generation = 0;
  bool valid = false;
  uint8_t seaTalkStatus[9] = {}; // Last validated physical pilot status.
  bool alive(uint32_t now) const;
  bool receive(const uint8_t *p, size_t n, uint32_t now);
};
struct Sample {
  double value = NAN;
  uint32_t at = 0;
  bool valid = false;
  void set(double v, uint32_t now) { value=v; at=now; valid=isfinite(v); }
  void clear() { valid=false; value=NAN; }
  bool alive(uint32_t now, uint32_t timeout) const { return valid && fresh(now,at,timeout); }
};
struct Navigation {
  uint8_t source = 255;
  Sample xte, distance, bearing, variation;
  uint8_t reference = 3;
  uint32_t waypoint = 0xffffffff, epoch = 0;
  bool terminated = true, xteSeen = false;
  bool acceptSource(uint8_t sa);
  bool receive(uint32_t pgn, uint8_t sa, const uint8_t *p, size_t n, uint32_t now);
  double magneticBearing(uint32_t now) const;
  bool ready(uint32_t now) const;
};
struct Wind {
  uint8_t source = 255;
  Sample speed, angle;
  Sample localAngle; // Existing SeaTalk wind instrument: do not retransmit it.
  bool receive(uint8_t sa, const uint8_t *p, size_t n, uint32_t now);
  bool ready(uint32_t now) const;
  bool nmeaReady(uint32_t now) const;
};

struct Datagram { uint8_t data[18] = {}; uint8_t size = 0; };
Datagram key(uint8_t code);
Datagram evolutionStatus(const Pilot &pilot,uint32_t now);
bool publishLockedHeading(const Pilot &pilot,uint32_t now,bool autoTrackCompatibility);
Datagram navigation(const Navigation &nav, uint32_t now);
Datagram waypointToken(uint32_t epoch);
Datagram windAngle(double degrees);
Datagram windSpeed(double knots);
Datagram waterSpeed(double knots);

enum class CommandType : uint8_t { None, Mode, Heading, Step };
struct Command { CommandType type=CommandType::None; Mode mode=Mode::Unknown; double value=0; };
// Strict PGN 126208 command parser. Field 6 in 65360 is ONLY an angle.
bool parseCommand(const uint8_t *p, size_t n, Command &out);

// Validate frame boundaries and lifetime before the library reassembles control
// and navigation messages. Especially important for short DLC and late frames.
class FastPacketGuard {
 public:
  bool accept(uint32_t pgn,uint8_t source,uint8_t destination,const uint8_t *p,size_t n,uint32_t now);
  void clear();
 private:
  struct Slot { bool active=false; uint32_t pgn=0,at=0; uint8_t source=0,destination=0,next=0,remaining=0; };
  Slot slots[16];
};

enum class Phase : uint8_t { Idle, Queued, Transmitting, Confirming };
class Controller {
 public:
  Pilot pilot;
  Navigation nav;
  Wind wind;
  Phase phase = Phase::Idle;
  const char *result = "boot: waiting for pilot";
  uint32_t completed=0, failed=0, rejected=0;
  bool request(const Command &cmd, uint32_t now);
  bool next(uint32_t now, Datagram &out);
  void started(uint32_t now);
  void transmitted(bool echoOk, uint32_t now);
  void receivePilot(const uint8_t *p, size_t n, uint32_t now);
  void standbyKey();
  void tick(uint32_t now);
  void cancel(const char *why);
  bool busy() const { return phase != Phase::Idle; }
 private:
  Command command;
  uint32_t requestedAt=0, phaseAt=0, beforeGeneration=0, routeEpoch=0;
  double expectedHeading=NAN, previousHeading=NAN;
  Mode expectedMode=Mode::Unknown;
  bool reject(const char *why);
  void finish();
};

// Decode hardware-captured runs of identical bits into 9-bit SeaTalk characters.
// Call endCapture after each RMT idle boundary; partial datagrams are discarded.
class WireDecoder {
 public:
  using Callback = void (*)(void *, const uint8_t *, size_t);
  WireDecoder(Callback cb, void *ctx) : callback(cb), context(ctx) {}
  void run(bool high, uint32_t us);
  void endCapture(bool idleHigh);
  uint32_t errors=0;
 private:
  Callback callback;
  void *context;
  Datagram packet;
  uint16_t character=0;
  uint8_t bit=0, expected=0;
  void feed(bool high);
  void reset();
};
}

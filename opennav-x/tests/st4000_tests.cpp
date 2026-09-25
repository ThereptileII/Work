#include "adapters/St4000Pilot.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace opennav;
using namespace adapters;
using namespace std::chrono_literals;
const vessel::Time epoch{100s};
const std::uint64_t name =
    (std::uint64_t(0xc0) << 56) | (std::uint64_t(80) << 48) |
    (std::uint64_t(135) << 40) | (std::uint64_t(1851) << 21) | 1234;
void Check(bool v, const char *why) {
  if (!v)
    throw std::runtime_error(why);
}
template <class F> void Reject(F f) {
  try {
    f();
  } catch (const std::invalid_argument &) {
    return;
  }
  throw std::runtime_error("Expected invalid argument");
}
class Transport final : public IN2kPilotTransport {
public:
  PilotTransportStatus status{true, true, 1, "TEST transport"};
  bool accept = true;
  struct Sent {
    std::string iface;
    unsigned destination, pgn, priority;
    std::vector<std::uint8_t> data;
  };
  std::vector<Sent> sent;
  PilotTransportStatus Status(const std::string &iface) const override {
    return iface == "test-n2k" ? status : PilotTransportStatus{};
  }
  bool Send(const std::string &iface, std::uint8_t destination,
            std::uint32_t pgn, std::uint8_t priority,
            const std::vector<std::uint8_t> &data) override {
    sent.push_back({iface, destination, pgn, priority, data});
    return accept;
  }
};
PilotN2kFrame Frame(unsigned pgn, std::vector<std::uint8_t> data,
                    vessel::Time at, unsigned address = 204) {
  return {"test-n2k", pgn, static_cast<std::uint8_t>(address), data, at};
}
PilotN2kFrame Claim(vessel::Time at, unsigned address = 204,
                    std::uint64_t identity = name) {
  std::vector<std::uint8_t> data(8);
  for (unsigned i = 0; i < 8; ++i)
    data[i] = (identity >> (i * 8)) & 255;
  return Frame(60928, data, at, address);
}
PilotN2kFrame Mode(PilotMode mode, vessel::Time at) {
  const unsigned value = mode == PilotMode::Auto      ? 0x40
                         : mode == PilotMode::Track   ? 0x180
                         : mode == PilotMode::Wind    ? 0x100
                         : mode == PilotMode::Standby ? 0
                                                      : 0xffff;
  return Frame(65379,
               {0x3b, 0x9f, static_cast<std::uint8_t>(value & 255),
                static_cast<std::uint8_t>(value >> 8), 255, 255, 255, 255},
               at);
}
PilotN2kFrame Heading(double value, vessel::Time at, bool locked = false) {
  const auto angle = static_cast<unsigned>(
      std::lround(value / 180.0 * 3.14159265358979323846 / .0001));
  const auto lo = static_cast<std::uint8_t>(angle & 255),
             hi = static_cast<std::uint8_t>(angle >> 8);
  return locked ? Frame(65360, {0x3b, 0x9f, 1, 255, 255, lo, hi, 255}, at)
                : Frame(127250, {1, lo, hi, 255, 255, 255, 255, 0xfd}, at);
}
void Identity() {
  Check(ParsePilotName(FormatPilotName(name)) == name, "Exact 64-bit NAME");
  Reject([] { ParsePilotName("0000000000000000"); });
  Reject([] { ParsePilotName("c0508700e76004d"); });
  Reject([] { ParsePilotName("C0508700E76004D2"); });
  Reject([] { ValidateSt4000Binding({"test-n2k", "", true}); });
  Reject([] {
    ValidateSt4000Binding({"bad\niface", FormatPilotName(name), true});
  });
  Transport transport;
  St4000Pilot pilot(transport);
  pilot.Configure({"test-n2k", FormatPilotName(name), false});
  pilot.Observe(Mode(PilotMode::Auto, epoch), epoch);
  Check(pilot.GetState().mode == PilotMode::Unavailable,
        "No unverified source");
  auto wrong = Claim(epoch);
  wrong.interface = "another-interface";
  pilot.Observe(wrong, epoch);
  Check(!pilot.Address(), "Interface is part of identity");
  pilot.Observe(Claim(epoch, 204, name + 1), epoch);
  Check(!pilot.Address(), "Same vendor is not the configured device");
  pilot.Observe(Claim(epoch + 1ms), epoch + 1ms);
  pilot.Observe(Mode(PilotMode::Standby, epoch + 2ms), epoch + 2ms);
  Check(pilot.Address() == 204 && pilot.GetState().mode == PilotMode::Standby,
        "Verified source can provide read-only state");
  Check(!pilot.Capabilities().manual_control && transport.sent.empty(),
        "Configuration is display-only by default; observing never sends");
  pilot.Observe(Claim(epoch + 3ms, 205), epoch + 3ms);
  Check(!pilot.Address() && pilot.GetState().mode == PilotMode::Unavailable,
        "Ambiguous NAME/address invalidates retained state");
  pilot.Observe(Claim(epoch + 4ms), epoch + 4ms);
  Check(!pilot.Address(), "Conflict does not auto-heal into control");
  pilot.Configure({"test-n2k", FormatPilotName(name), true});
  pilot.Observe(Claim(epoch + 5ms), epoch + 5ms);
  Check(!pilot.Address(),
        "Permission changes cannot clear an identity conflict");
  pilot.Configure({});
  pilot.Configure({"test-n2k", FormatPilotName(name), true});
  pilot.Observe(Claim(epoch + 5ms), epoch + 5ms);
  Check(pilot.Capabilities().manual_control,
        "Deliberate verified configuration");
  pilot.Observe(Claim(epoch + 6ms, 204, name + 1), epoch + 6ms);
  Check(!pilot.Capabilities().manual_control,
        "Claimed address reused by another device");
}
void Feedback() {
  Transport t;
  St4000Pilot a(t);
  a.Configure({"test-n2k", FormatPilotName(name), true});
  a.Observe(Claim(epoch), epoch);
  a.Observe(Mode(PilotMode::Auto, epoch + 10ms), epoch + 10ms);
  a.Observe(Heading(330, epoch + 20ms), epoch + 20ms);
  a.Observe(Heading(331, epoch + 30ms, true), epoch + 30ms);
  Check(a.GetState().mode == PilotMode::Auto &&
            std::abs(*a.GetState().heading_magnetic_deg.value - 330) < .01 &&
            std::abs(*a.GetState().locked_heading_magnetic_deg.value - 331) <
                .01,
        "Actual and commanded magnetic heading are distinct physical "
        "observations");
  const auto copy = a.GetState();
  a.Poll(epoch + 2s);
  Check(a.GetState().observed_at == copy.observed_at,
        "Poll never renews feedback");
  auto bad = Heading(10, epoch + 40ms);
  bad.data[7] = 0xfc;
  a.Observe(bad, epoch + 40ms);
  Check(!a.GetState().heading_magnetic_deg.value,
        "True heading not relabelled magnetic");
  bad = Heading(10, epoch + 50ms, true);
  bad.data[5] = 255;
  bad.data[6] = 255;
  a.Observe(bad, epoch + 50ms);
  Check(!a.GetState().locked_heading_magnetic_deg.value,
        "NA clears locked heading");
  a.Observe(Heading(100, epoch + 45ms, true), epoch + 60ms);
  Check(!a.GetState().locked_heading_magnetic_deg.value,
        "Out-of-order cannot resurrect data");
  bad = Mode(PilotMode::Auto, epoch + 70ms);
  bad.data[0] = 0;
  a.Observe(bad, epoch + 70ms);
  Check(a.GetState().mode == PilotMode::Unavailable,
        "Wrong vendor header invalidates mode");
  a.Observe(Mode(PilotMode::Track, epoch + 80ms), epoch + 80ms);
  Check(a.GetState().mode == PilotMode::Track && !a.Capabilities().track &&
            !a.Capabilities().wind,
        "Observed modes do not grant unvalidated commands");
  bad = Mode(PilotMode::Auto, epoch + 90ms);
  bad.data.resize(2);
  a.Observe(bad, epoch + 90ms);
  Check(a.GetState().mode == PilotMode::Unavailable,
        "Malformed mode fails closed");
  a.Observe(Mode(PilotMode::Auto, epoch + 100ms), epoch + 4s);
  Check(a.GetState().mode == PilotMode::Unavailable,
        "Delayed queued data not live");
  a.Observe(Mode(PilotMode::Auto, epoch + 5s), epoch + 4s);
  Check(a.GetState().mode == PilotMode::Unavailable, "Future data rejected");
  Check(copy.mode == PilotMode::Auto && copy.locked_heading_magnetic_deg.value,
        "Retained copied state survives later mutations");
}
void Encoding() {
  for (const auto action :
       {PilotAction::Standby, PilotAction::Auto, PilotAction::AlterCourse}) {
    const std::vector<double> deltas = action == PilotAction::AlterCourse
                                           ? std::vector<double>{-10, -1, 1, 10}
                                           : std::vector<double>{0};
    for (const double delta : deltas) {
      const auto bytes = EncodeSt4000Command({1, action, delta, epoch});
      const unsigned key = action == PilotAction::Standby ? 0
                           : action == PilotAction::Auto  ? 0x40
                           : delta == -10                 ? 0x50
                           : delta == -1                  ? 0x7f
                           : delta == 1                   ? 0x51
                                                          : 0xd1;
      const std::vector<std::uint8_t> expected{1,
                                               0x63,
                                               0xff,
                                               0,
                                               0xff,
                                               3,
                                               1,
                                               0x3b,
                                               7,
                                               3,
                                               4,
                                               6,
                                               static_cast<std::uint8_t>(key)};
      Check(bytes == expected, "Pinned firmware structural command fixture");
    }
  }
  Check(EncodeSt4000Command({1, PilotAction::Track, 0, epoch}).empty() &&
            EncodeSt4000Command({1, PilotAction::Wind, 0, epoch}).empty(),
        "TRACK/WIND never transmitted by Beta live adapter");
  for (const auto delta :
       {0., 2., -360., std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()})
    Check(EncodeSt4000Command({1, PilotAction::AlterCourse, delta, epoch})
              .empty(),
          "Malformed/unsupported command refused");
}
void Commands() {
  Transport t;
  St4000Pilot a(t);
  ManualAutopilot p(a);
  a.Configure({"test-n2k", FormatPilotName(name), true});
  a.Observe(Claim(epoch), epoch);
  a.Observe(Mode(PilotMode::Standby, epoch + 10ms), epoch + 10ms);
  a.Observe(Heading(330, epoch + 10ms), epoch + 10ms);
  Check(p.Request(PilotAction::Auto, 0, epoch + 20ms).state ==
            CommandState::Disabled,
        "Stored permission never enables a session");
  p.Enable(true, epoch + 20ms);
  Check(p.Request(PilotAction::Auto, 0, epoch + 20ms).state ==
                CommandState::Pending &&
            t.sent.size() == 1 && t.sent[0].destination == 204 &&
            t.sent[0].pgn == 126208 && t.sent[0].priority == 3,
        "Only configured destination receives a manual request");
  Check(p.Request(PilotAction::Standby, 0, epoch + 21ms).state ==
                CommandState::Rejected &&
            t.sent.size() == 1 &&
            p.GetState(epoch + 21ms).command.state == CommandState::Pending,
        "Touch repeats cannot storm or erase pending command evidence");
  p.Tick(epoch + 1s);
  Check(p.GetState(epoch + 1s).command.state == CommandState::Pending,
        "Transport return/echo alone never confirms");
  Check(p.Request(PilotAction::Standby, 99, epoch + 1s).state ==
            CommandState::Rejected &&
            p.GetState(epoch + 1s).command.state == CommandState::Pending,
        "Malformed standby cannot erase pending command evidence");
  a.Observe(Mode(PilotMode::Auto, epoch + 1100ms), epoch + 1100ms);
  a.Observe(Heading(330, epoch + 1100ms, true), epoch + 1100ms);
  p.Tick(epoch + 1100ms);
  Check(p.GetState(epoch + 1100ms).command.state == CommandState::Confirmed,
        "Subsequent measured mode confirms");
  p.Request(PilotAction::AlterCourse, 1, epoch + 1200ms);
  a.Observe(Mode(PilotMode::Auto, epoch + 1300ms), epoch + 1300ms);
  p.Tick(epoch + 1300ms);
  Check(p.GetState(epoch + 1300ms).command.state == CommandState::Pending,
        "Mode refresh cannot renew a retained locked heading");
  a.Observe(Heading(331, epoch + 1400ms, true), epoch + 1400ms);
  p.Tick(epoch + 1400ms);
  Check(p.GetState(epoch + 1400ms).command.state == CommandState::Confirmed,
        "New correct physical target confirms manual step");
  p.Request(PilotAction::Standby, 0, epoch + 1600ms);
  a.Observe(Mode(PilotMode::Standby, epoch + 4600ms), epoch + 4600ms);
  p.Tick(epoch + 4600ms);
  Check(p.GetState(epoch + 4600ms).command.state == CommandState::TimedOut,
        "Late feedback cannot retroactively confirm");
  Check(t.sent.size() == 3, "No automatic retries");
  t.status.connected = false;
  p.Tick(epoch + 5s);
  Check(!p.GetState(epoch + 5s).enabled && !a.Address(),
        "Loss disables session and binding");
  t.status.connected = true;
  ++t.status.epoch;
  a.Observe(Claim(epoch + 6s), epoch + 6s);
  a.Observe(Mode(PilotMode::Standby, epoch + 6100ms), epoch + 6100ms);
  p.Tick(epoch + 6100ms);
  Check(!p.GetState(epoch + 6100ms).enabled,
        "Reconnect never re-enables control");
}
void Discovery() {
  Transport t;
  St4000Pilot a(t);
  Check(!a.RequestIdentity(epoch), "No discovery on unconfigured connections");
  a.Configure({"test-n2k", FormatPilotName(name), false});
  Check(a.RequestIdentity(epoch) && t.sent.size() == 1 &&
            t.sent[0].pgn == 59904 && t.sent[0].destination == 255 &&
            t.sent[0].data == std::vector<std::uint8_t>({0, 0xee, 0}),
        "Read-only ISO identity request");
  Check(!a.RequestIdentity(epoch + 1s), "Bound discovery cadence");
  t.status.writable = false;
  Check(!a.RequestIdentity(epoch + 6s),
        "Never write to an input-only connection");
  t.status.writable = true;
  a.Observe(Claim(epoch + 7s), epoch + 7s);
  Check(!a.Send({1, PilotAction::Standby, 0, epoch + 8s}) && t.sent.size() == 1,
        "Discovery permission cannot enable control");
}
int main(int argc, char **argv) {
  try {
    const std::string group = argc > 1 ? argv[1] : "";
    if (group == "identity")
      Identity();
    else if (group == "feedback")
      Feedback();
    else if (group == "encoding")
      Encoding();
    else if (group == "commands")
      Commands();
    else if (group == "discovery")
      Discovery();
    else
      throw std::runtime_error("Unknown test");
    std::cout << "PASS " << group << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

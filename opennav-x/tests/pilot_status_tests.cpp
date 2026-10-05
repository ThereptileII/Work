#include "integration/PilotStatusDiscovery.h"
#include "application/PilotPresentation.h"
#include <iostream>
#include <stdexcept>

using namespace opennav;
using namespace std::chrono_literals;
namespace {
const vessel::Time start{100s};
constexpr std::uint64_t identity =
    (std::uint64_t(0xc0) << 56) | (std::uint64_t(80) << 48) |
    (std::uint64_t(135) << 40) | (std::uint64_t(1851) << 21) | 1234;
void Check(bool value, const char *why) {
  if (!value) throw std::runtime_error(why);
}
class Transport final : public adapters::IN2kPilotTransport {
public:
  adapters::PilotTransportStatus connection{true, true, 1, "Fixture"};
  int sends = 0;
  adapters::PilotTransportStatus Status(const std::string &) const override {
    return connection;
  }
  bool Send(const std::string &, std::uint8_t, std::uint32_t, std::uint8_t,
            const std::vector<std::uint8_t> &) override { ++sends; return true; }
};
adapters::PilotN2kFrame Claim(vessel::Time at, unsigned address = 204,
                            std::uint64_t name = identity,
                            const std::string &iface = "existing-opencpn") {
  std::vector<std::uint8_t> data(8);
  for (unsigned i = 0; i < 8; ++i) data[i] = (name >> (8 * i)) & 255;
  return {iface, 60928, static_cast<std::uint8_t>(address), data, at};
}
adapters::PilotN2kFrame Mode(vessel::Time at, unsigned address = 204,
                           const std::string &iface = "existing-opencpn") {
  return {iface, 65379, static_cast<std::uint8_t>(address),
          {0x3b, 0x9f, 0x40, 0, 255, 255, 255, 255}, at};
}
void Discovery() {
  Transport transport;
  integration::PilotStatusDiscovery pilot(transport);
  pilot.Observe(Mode(start), start);
  Check(pilot.GetState(start).mode == adapters::PilotMode::Unavailable,
        "Mode traffic without observed identity cannot establish availability");
  pilot.Observe(Claim(start), start);
  Check(pilot.GetState(start).mode == adapters::PilotMode::Unavailable,
        "An identity claim is not physical pilot status");
  pilot.Observe(Mode(start + 1ms, 205), start + 1ms);
  pilot.Observe(Mode(start + 2ms, 204, "other-interface"), start + 2ms);
  Check(pilot.GetState(start + 2ms).mode == adapters::PilotMode::Unavailable,
        "An unrelated address or connection cannot supply pilot status");
  pilot.Observe(Mode(start + 3ms), start + 3ms);
  const auto feedback = pilot.GetState(start + 3ms);
  Check(feedback.mode == adapters::PilotMode::Auto && feedback.sequence,
        "Live accepted feedback is discovered without separate XNav setup");
  Check(feedback.source.find("NAME-" + adapters::FormatPilotName(identity)) !=
            std::string::npos, "Observed exact identity is retained");
  adapters::PilotView view;
  view.feedback = feedback;
  view.fresh = true;
  view.output_unavailable = true;
  view.enabled = true; // Old saved/session permissions must have no effect.
  view.capabilities = {false, true, true, true, true, true, true};
  auto shown = application::PresentPilot(view, start + 3ms, true, false);
  Check(shown.available && shown.mode == adapters::PilotMode::Auto,
        "Status-only policy must not hide observed availability");
  Check(!shown.enabled && !shown.can_toggle && !shown.standby &&
            !shown.auto_mode && !shown.track && !shown.wind && !shown.alter_course,
        "Detected equipment grants no control path");
  Check(transport.sends == 0, "Passive discovery must send no commands or requests");
}
void Rejection() {
  for (int invalid = 0; invalid < 5; ++invalid) {
    Transport transport;
    integration::PilotStatusDiscovery pilot(transport);
    auto claim = Claim(start);
    if (invalid == 0) claim.data.assign(8, 0);
    if (invalid == 1) claim.data.pop_back();
    if (invalid == 2) claim.source = 254;
    if (invalid == 3) claim.observed_at = start + 1s;
    if (invalid == 4) claim.observed_at = start - 3s;
    pilot.Observe(claim, start);
    pilot.Observe(Mode(start + 1ms), start + 1ms);
    Check(pilot.GetState(start + 1ms).mode == adapters::PilotMode::Unavailable,
          "Invalid, unknown, stale and future identities remain unavailable");
  }
  Transport transport;
  integration::PilotStatusDiscovery pilot(transport);
  pilot.Observe(Claim(start), start);
  auto bad = Mode(start + 1ms);
  bad.data[0] = 0;
  pilot.Observe(bad, start + 1ms);
  Check(pilot.GetState(start + 1ms).mode == adapters::PilotMode::Unavailable,
        "Wrong vendor feedback cannot establish availability");
  pilot.Observe(Mode(start + 2ms), start + 2ms);
  bad = Mode(start + 3ms);
  bad.data[2] = bad.data[3] = 255;
  pilot.Observe(bad, start + 3ms);
  Check(pilot.GetState(start + 3ms).mode == adapters::PilotMode::Unavailable,
        "Explicit unavailable feedback removes previously live mode");
  Check(transport.sends == 0, "Invalid input never triggers a bus request");
}
void LossAndRecovery() {
  Transport transport;
  integration::PilotStatusDiscovery pilot(transport);
  pilot.Observe(Claim(start), start);
  pilot.Observe(Mode(start + 1ms), start + 1ms);
  pilot.Poll(start + 3001ms);
  auto lost = pilot.GetState(start + 3001ms);
  Check(lost.mode == adapters::PilotMode::Unavailable && lost.sequence &&
            !lost.heading_magnetic_deg.value && !lost.locked_heading_magnetic_deg.value,
        "Expiry preserves loss evidence but never old mode or headings");
  adapters::PilotView view;
  view.feedback = lost;
  view.output_unavailable = true;
  auto shown = application::PresentPilot(view, start + 3001ms, false, false);
  Check(!shown.available && shown.degraded && shown.state == "FEEDBACK LOST",
        "Stale status is visibly degraded despite the status-only policy");
  pilot.Observe(Mode(start + 4s), start + 4s);
  Check(pilot.GetState(start + 4s).mode == adapters::PilotMode::Auto,
        "New valid physical feedback restores status");
  transport.connection.connected = false;
  pilot.Poll(start + 4001ms);
  Check(pilot.GetState(start + 4001ms).mode == adapters::PilotMode::Unavailable,
        "Connection loss immediately invalidates otherwise fresh mode");
  transport.connection.connected = true;
  ++transport.connection.epoch;
  pilot.Poll(start + 4002ms);
  pilot.Observe(Mode(start + 4003ms), start + 4003ms);
  Check(pilot.GetState(start + 4003ms).mode == adapters::PilotMode::Unavailable,
        "A new connection requires a newly observed identity");
  pilot.Observe(Claim(start + 4004ms), start + 4004ms);
  pilot.Observe(Mode(start + 4005ms), start + 4005ms);
  Check(pilot.GetState(start + 4005ms).mode == adapters::PilotMode::Auto,
        "Observed identity plus feedback recovers after connection loss");
  Check(transport.sends == 0, "Recovery is passive");
}
void Ambiguity() {
  Transport transport;
  integration::PilotStatusDiscovery pilot(transport);
  pilot.Observe(Claim(start), start);
  pilot.Observe(Mode(start + 1ms), start + 1ms);
  pilot.Observe(Claim(start + 2ms, 205, identity + 1), start + 2ms);
  pilot.Observe(Mode(start + 3ms, 205), start + 3ms);
  Check(pilot.GetState(start + 3ms).mode == adapters::PilotMode::Unavailable,
        "Two live pilots must not result in an arbitrary selected pilot");
  Check(pilot.Description(start + 3ms).find("ambiguous") != std::string::npos,
        "Ambiguity has an explicit diagnostic");
  integration::PilotStatusDiscovery conflict(transport);
  conflict.Observe(Claim(start), start);
  conflict.Observe(Mode(start + 1ms), start + 1ms);
  conflict.Observe(Claim(start + 2ms, 205), start + 2ms);
  conflict.Observe(Mode(start + 3ms), start + 3ms);
  Check(conflict.GetState(start + 3ms).mode == adapters::PilotMode::Unavailable,
        "Conflicting claims cannot retain available status");
  Check(transport.sends == 0, "Ambiguity must not trigger discovery output");
}
}
int main() {
  try {
    Discovery();
    Rejection();
    LossAndRecovery();
    Ambiguity();
    std::cout << "Passive pilot discovery and status tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

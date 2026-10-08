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
  // SCRUM-295: like AutoTrack, vendor-coded physical status is enough to see
  // the pilot when this PC joined after the bridge's one-time address claim.
  pilot.Observe(Mode(start), start);
  auto seen = pilot.GetState(start);
  Check(seen.mode == adapters::PilotMode::Auto && seen.sequence &&
            seen.source.find("status-address/source-204") != std::string::npos,
        "Vendor-coded mode traffic establishes status without an observed claim");
  auto detected = pilot.Detected(start);
  Check(detected && detected->interface_id == "existing-opencpn" &&
            detected->address == "204" && detected->name.empty() &&
            !detected->permit_control,
        "The detected pilot is offered as an address binding, never as permission");
  pilot.Observe(Claim(start + 1ms), start + 1ms);
  Check(pilot.GetState(start + 1ms).mode == adapters::PilotMode::Auto,
        "A claim without new status keeps the live address status");
  pilot.Observe(Mode(start + 2ms, 204, "other-interface"), start + 2ms);
  Check(pilot.GetState(start + 2ms).mode == adapters::PilotMode::Unavailable,
        "A second live pilot source is ambiguous, not an arbitrary choice");
  Check(!pilot.Detected(start + 2ms), "Ambiguity offers no binding");
  pilot.Poll(start + 3003ms);
  pilot.Observe(Mode(start + 3003ms), start + 3003ms);
  const auto feedback = pilot.GetState(start + 3003ms);
  Check(feedback.mode == adapters::PilotMode::Auto && feedback.sequence,
        "Live accepted feedback is discovered without separate XNav setup");
  Check(feedback.source.find("NAME-" + adapters::FormatPilotName(identity)) !=
            std::string::npos, "An observed exact identity takes precedence");
  detected = pilot.Detected(start + 3003ms);
  Check(detected && detected->name == adapters::FormatPilotName(identity) &&
            detected->address.empty(), "A NAME identity is offered when observed");
  adapters::PilotView view;
  view.feedback = feedback;
  view.fresh = true;
  view.output_unavailable = true;
  view.enabled = true; // Old saved/session permissions must have no effect.
  view.capabilities = {false, true, true, true, true, true, true};
  auto shown = application::PresentPilot(view, start + 3003ms, true, false);
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
    const auto state = pilot.GetState(start + 1ms);
    Check(state.source.find("NAME-") == std::string::npos &&
              pilot.GetDiagnostics(start + 1ms).verified_identities == 0,
          "Invalid, unknown, stale and future identities are never accepted");
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
  const auto reconnected = pilot.GetState(start + 4003ms);
  Check(reconnected.mode == adapters::PilotMode::Auto &&
            reconnected.source.find("status-address") != std::string::npos,
        "After reconnect, fresh vendor-coded status restores availability without waiting for a claim");
  pilot.Observe(Claim(start + 4004ms), start + 4004ms);
  pilot.Observe(Mode(start + 4005ms), start + 4005ms);
  Check(pilot.GetState(start + 4005ms).mode == adapters::PilotMode::Auto &&
            pilot.GetState(start + 4005ms).source.find("NAME-") != std::string::npos,
        "Observed identity plus feedback recovers its NAME after connection loss");
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
  const auto diagnostics = conflict.GetDiagnostics(start + 3ms);
  Check(diagnostics.identity_conflicts == 1 &&
            diagnostics.verified_identities == 0 &&
            diagnostics.conflict_source ==
                "existing-opencpn/NAME-" + adapters::FormatPilotName(identity),
        "Conflict diagnostics identify the actual interface and NAME");
  Check(conflict.Description(start + 3ms).find("address/NAME conflict") !=
            std::string::npos,
        "A known conflict must not be hidden behind generic feedback loss");
  ++transport.connection.epoch;
  conflict.Poll(start + 4ms);
  conflict.Observe(Claim(start + 5ms), start + 5ms);
  conflict.Observe(Mode(start + 6ms), start + 6ms);
  Check(conflict.GetDiagnostics(start + 6ms).identity_conflicts == 1 &&
            conflict.GetState(start + 6ms).mode == adapters::PilotMode::Unavailable,
        "Reconnect and repeated claims cannot silently reset a conflict");
  Check(transport.sends == 0, "Ambiguity must not trigger discovery output");
  integration::PilotStatusDiscovery unconfirmed_conflict(transport);
  unconfirmed_conflict.Observe(Claim(start), start);
  unconfirmed_conflict.Observe(Claim(start + 1ms, 205), start + 1ms);
  Check(unconfirmed_conflict.Description(start + 1ms).find("address/NAME conflict") !=
            std::string::npos &&
            !unconfirmed_conflict.GetState(start + 1ms).sequence,
        "Conflict before any physical feedback is explicit rather than generic waiting");
}
void AddressConflict() {
  Transport transport;
  integration::PilotStatusDiscovery pilot(transport);
  pilot.Observe(Mode(start), start);
  Check(pilot.GetState(start).mode == adapters::PilotMode::Auto, "Address status live");
  // Another class of device claims the pilot's address: no longer this pilot.
  pilot.Observe(Claim(start + 1ms, 204, 0x1234), start + 1ms);
  pilot.Observe(Mode(start + 2ms), start + 2ms);
  Check(pilot.GetState(start + 2ms).mode == adapters::PilotMode::Unavailable &&
            !pilot.Detected(start + 2ms),
        "A foreign claim on the pilot address revokes address status");
  Check(transport.sends == 0, "Address conflict handling is passive");
}
void TrafficDiagnostics() {
  Transport transport;
  integration::PilotStatusDiscovery pilot(transport);
  Check(pilot.GetDiagnostics(start).fresh_mode_sources_without_identity == 0 &&
            pilot.GetDiagnostics(start).verified_identities == 0,
        "No received traffic cannot imply either mode or identity");
  // Joining an already running bus may miss its earlier address claim.
  pilot.Observe(Mode(start), start);
  auto diagnostics = pilot.GetDiagnostics(start);
  Check(diagnostics.fresh_mode_sources_without_identity == 1 &&
            diagnostics.verified_identities == 0 &&
            pilot.Description(start).find("Live pilot feedback") != std::string::npos &&
            pilot.GetState(start).mode == adapters::PilotMode::Auto,
        "Mode-only startup shows status while diagnostics still report the missing NAME");
  pilot.Observe(Claim(start + 1ms, 204, identity, "other-interface"), start + 1ms);
  Check(pilot.GetDiagnostics(start + 1ms).fresh_mode_sources_without_identity == 1,
        "A claim on another interface cannot identify this mode source");
  Check(pilot.GetDiagnostics(start + 3s).fresh_mode_sources_without_identity == 0 &&
            pilot.GetDiagnostics(start + 3s).stale_mode_sources == 1,
        "Mode diagnostics expire at the same strict three-second boundary");
  ++transport.connection.epoch;
  pilot.Poll(start + 3001ms);
  Check(pilot.GetDiagnostics(start + 3001ms).stale_mode_sources == 0 &&
            pilot.GetDiagnostics(start + 3001ms).verified_identities == 0,
        "An old connection cannot contribute current identity or traffic diagnostics");
  pilot.Observe(Mode(start + 3002ms), start + 3002ms);
  pilot.Observe(Claim(start + 3003ms), start + 3003ms);
  Check(pilot.GetDiagnostics(start + 3003ms).verified_identities == 1 &&
            pilot.GetDiagnostics(start + 3003ms).fresh_mode_sources_without_identity == 0 &&
            pilot.GetState(start + 3003ms).source.find("status-address") != std::string::npos,
        "A claim does not retroactively attach earlier mode to the NAME");
  pilot.Observe(Mode(start + 3004ms), start + 3004ms);
  Check(pilot.GetState(start + 3004ms).mode == adapters::PilotMode::Auto &&
            pilot.GetState(start + 3004ms).source.find("NAME-") != std::string::npos,
        "Subsequent physical feedback attaches to the verified NAME");
  transport.connection.connected = false;
  pilot.Poll(start + 3005ms);
  Check(pilot.GetDiagnostics(start + 3005ms).verified_identities == 0 &&
            pilot.GetDiagnostics(start + 3005ms).fresh_mode_sources_without_identity == 0,
        "Disconnected transport cannot contribute fresh diagnostics");
  Check(transport.sends == 0, "Diagnostic discovery remains entirely passive");
}
void InvalidTrafficDiagnostics() {
  for (int invalid = 0; invalid < 8; ++invalid) {
    Transport transport;
    integration::PilotStatusDiscovery pilot(transport);
    auto frame = Mode(start);
    if (invalid == 0) frame.data.pop_back();
    if (invalid == 1) frame.data[0] = 0;
    if (invalid == 2) frame.data[2] = frame.data[3] = 255;
    if (invalid == 3) frame.source = 254;
    if (invalid == 4) frame.observed_at = start + 1ms;
    if (invalid == 5) frame.observed_at = start - 3s;
    if (invalid == 6) frame.interface_id = "bad\ninterface";
    if (invalid == 7) transport.connection.connected = false;
    pilot.Observe(frame, start);
    Check(pilot.GetDiagnostics(start).fresh_mode_sources_without_identity == 0 &&
              pilot.GetDiagnostics(start).stale_mode_sources == 0,
          "Malformed, future, stale and disconnected traffic cannot supply diagnostics");
    Check(transport.sends == 0, "Invalid diagnostics never cause output");
  }
  Transport transport;
  integration::PilotStatusDiscovery bounded(transport);
  for (unsigned source = 0; source < 33; ++source)
    bounded.Observe(Mode(start, source), start);
  const auto diagnostics = bounded.GetDiagnostics(start);
  Check(diagnostics.fresh_mode_sources_without_identity == 32 &&
            diagnostics.traffic_limit_exceeded &&
            bounded.GetState(start).mode == adapters::PilotMode::Unavailable &&
            !bounded.Detected(start),
        "Unidentified traffic storage is bounded and an overflow grants no status");
}
}
int main() {
  try {
    Discovery();
    Rejection();
    LossAndRecovery();
    Ambiguity();
    AddressConflict();
    TrafficDiagnostics();
    InvalidTrafficDiagnostics();
    std::cout << "Passive pilot discovery and status tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

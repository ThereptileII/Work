#include "integration/OpenCPNPilot.h"
#include "integration/PilotOutputPolicy.h"
#include "model/comm_drv_n2k_net.h"
#include "model/comm_drv_n2k_serial.h"
#include "model/comm_drv_registry.h"
#include "model/comm_drv_stats.h"
#include <limits>
#include <cmath>
#include <stdexcept>
#include <wx/thread.h>

namespace opennav::integration {
namespace {
void MainThread() {
  if (!wxIsMainThread())
    throw std::logic_error("Pilot bridge requires the application thread");
}
CommDriverN2K *Driver(const std::string &iface) {
  MainThread();
  for (const auto &d : CommDriverRegistry::GetInstance().GetDrivers())
    if (d->bus == NavAddr::Bus::N2000 && d->iface == iface)
      return dynamic_cast<CommDriverN2K *>(d.get());
  return nullptr;
}
PilotOutputEndpoint Endpoint(CommDriverN2K *driver) {
  PilotOutputEndpoint endpoint;
  if (auto *serial = dynamic_cast<CommDriverN2KSerial *>(driver)) {
    endpoint.serial = true;
    endpoint.bidirectional = serial->m_params.IOSelect == DS_TYPE_INPUT_OUTPUT;
    endpoint.enabled = serial->m_params.bEnabled;
    endpoint.connected = serial->GetSerialState().connected;
    endpoint.actisense = true; // This concrete driver parses binary complete-PGN Actisense.
    return endpoint;
  }
  auto *network = dynamic_cast<CommDriverN2KNet *>(driver);
  if (!network) return endpoint;
  const auto params = network->GetParams();
  auto *socket = network->GetSock();
  endpoint.tcp = params.NetProtocol == TCP;
  endpoint.bidirectional = params.IOSelect == DS_TYPE_INPUT_OUTPUT;
  endpoint.enabled = params.bEnabled;
  endpoint.connected = socket && socket->IsOk() && socket->IsConnected();
  endpoint.actisense = network->GetN2kFormat() == N2KFormat_Actisense_N2K_ASCII;
  endpoint.configured_address = params.NetworkAddress.ToStdString(wxConvUTF8);
  endpoint.configured_port = params.NetworkPort;
  wxIPV4address peer;
  if (endpoint.connected && socket->GetPeer(peer)) {
    endpoint.peer_address = peer.IPAddress().ToStdString(wxConvUTF8);
    endpoint.peer_port = peer.Service();
  }
  return endpoint;
}
} // namespace
OpenCPNPilot::OpenCPNPilot(std::function<bool()> allowed)
    : output_allowed_(std::move(allowed)) {
  MainThread();
  auto changes = std::make_unique<ObsListener>();
  changes->Init(CommDriverRegistry::GetInstance().evt_driverlist_change,
                [this](ObservedEvt &) {
                  if (registry_generation_ < UINT32_MAX)
                    ++registry_generation_;
                  Poll(vessel::Clock::now());
                });
  listeners_.push_back(std::move(changes));
  for (const auto pgn : {60928u, 65379u, 65360u, 127250u, 65359u, 126720u}) {
    auto listener = std::make_unique<ObsListener>();
    listener->Init(Nmea2000Msg(pgn), [this, pgn](ObservedEvt &event) {
      const auto m = UnpackEvtPointer<Nmea2000Msg>(event);
      if (!m || !m->source) return;
      const auto now = vessel::Clock::now();
      const auto age = std::chrono::system_clock::now() - m->created_at;
      auto at =
          now - std::chrono::duration_cast<vessel::Clock::duration>(age);
      if (auto *serial = dynamic_cast<CommDriverN2KSerial *>(Driver(m->source->iface))) {
        const auto received = std::dynamic_pointer_cast<const Nmea2000SerialMsg>(m);
        const auto state = serial->GetSerialState();
        if (!received || !state.connected || received->connection_generation != state.epoch ||
            received->received_at < state.changed_at || received->received_at > now ||
            now - received->received_at >= std::chrono::seconds(3)) return;
        at = received->received_at;
      }
      // Passive commissioning counters include AutoTrack's compatibility PGNs.
      // They do not create a pilot identity, mode, acknowledgement or permission.
      traffic_.Observe(m->source->iface, pgn, m->payload, at, now);
      if (pgn == 65359 || pgn == 126720) return;
      if (m->payload.size() != 22 || m->payload[0] != 0x93 ||
          m->payload[12] != 8 || m->payload[7] >= 254)
        return;
      const auto encoded_pgn = unsigned(m->payload[3]) |
                               (unsigned(m->payload[4]) << 8) |
                               (unsigned(m->payload[5]) << 16);
      if (encoded_pgn != pgn)
        return;
      if (age < std::chrono::system_clock::duration::zero() ||
          age >= std::chrono::seconds(3))
        return;
      if (const auto *network =
              dynamic_cast<CommDriverN2KNet *>(Driver(m->source->iface)))
        if (at < network->GetConnectionChangedAt())
          return;
      const adapters::PilotN2kFrame frame{m->source->iface, pgn, m->payload[7],
                      std::vector<std::uint8_t>(m->payload.begin() + 13,
                                                m->payload.begin() + 21),
                      at};
      status_.Observe(frame, now);
      // Always ingest real state, including unavailable mode/identity conflict.
      // GetState exposes write provenance separately for acknowledgement only.
      pilot_.Observe(frame, now);
      Poll(now); // Revoke queued output immediately on invalid mode or identity conflict.
    });
    listeners_.push_back(std::move(listener));
  }
}
OpenCPNPilot::~OpenCPNPilot() { SetControlEnabled(false); }
void OpenCPNPilot::Configure(const adapters::St4000Binding &binding) {
  MainThread();
  SetControlEnabled(false);
  pilot_.Configure(binding);
  binding_ = binding;
}
adapters::PilotFeedback OpenCPNPilot::GetState() const {
  MainThread();
  // A saved NAME or status-address binding uses the configured adapter;
  // otherwise status comes from passive discovery (display only).
  const bool bound = !binding_.name.empty() || !binding_.address.empty();
  auto feedback = bound ? pilot_.GetState()
                        : status_.GetState(vessel::Clock::now());
  if (awaiting_serial_write_) {
    auto *serial = dynamic_cast<CommDriverN2KSerial *>(Driver(binding_.interface_id));
    const auto state = serial ? serial->GetSerialState() : N2kSerialState::Snapshot{};
    feedback.command_confirmation_allowed = state.connected &&
        awaited_write_ticket_ != 0 && state.pilot_written_ticket == awaited_write_ticket_ &&
        Status(binding_.interface_id).epoch == session_epoch_;
    feedback.command_written_at = state.pilot_written_at;
  }
  return feedback;
}
std::string OpenCPNPilot::Description() const {
  MainThread();
  return !binding_.name.empty() || !binding_.address.empty()
             ? pilot_.Status()
             : status_.Description(vessel::Clock::now());
}
std::string OpenCPNPilot::ControlBlocker() const {
  MainThread();
  auto iface = binding_.interface_id;
  if (iface.empty())
    if (const auto detected = status_.Detected(vessel::Clock::now()))
      iface = detected->interface_id;
  auto *driver = iface.empty() ? nullptr : Driver(iface);
  if (!driver) return {};
  return PilotControlBlocker(Endpoint(driver), iface);
}
std::optional<adapters::St4000Binding> OpenCPNPilot::DetectedBinding() const {
  MainThread();
  return status_.Detected(vessel::Clock::now());
}
PilotStatusDiscovery::Diagnostics
OpenCPNPilot::DiscoveryDiagnostics(vessel::Time now) const {
  MainThread();
  return status_.GetDiagnostics(now);
}
PilotTrafficDiagnostics::Snapshot OpenCPNPilot::TrafficDiagnostics() const {
  MainThread();
  return traffic_.GetSnapshot();
}
void OpenCPNPilot::Poll(vessel::Time now) {
  MainThread();
  pilot_.Poll(now);
  status_.Poll(now);
  const auto feedback = pilot_.GetState();
  if (session_enabled_ && (!Status(binding_.interface_id).writable ||
      Status(binding_.interface_id).epoch != session_epoch_ ||
      !pilot_.Capabilities().manual_control || !feedback.sequence ||
      feedback.mode == adapters::PilotMode::Unavailable || feedback.observed_at > now ||
      now - feedback.observed_at >= std::chrono::seconds(3))) SetControlEnabled(false);
  if (navigation_token_ && (!NavigationSession() || !navigation_current_ || !navigation_current_()))
    InvalidateNavigationOutput("Route or measured position changed; waiting for normal route progress");
}
adapters::PilotTransportStatus
OpenCPNPilot::Status(const std::string &iface) const {
  MainThread();
  adapters::PilotTransportStatus status;
  status.epoch = registry_generation_ << 32;
  if (registry_generation_ == UINT32_MAX) {
    status.detail = "Connection identity exhausted; restart required";
    return status;
  }
  auto *driver = Driver(iface);
  if (!driver) {
    status.detail = "Configured OpenCPN N2K connection unavailable";
    return status;
  }
  if (auto *serial = dynamic_cast<CommDriverN2KSerial *>(driver)) {
    const auto state = serial->GetSerialState();
    if (state.epoch >= UINT32_MAX) { status.detail = "Serial identity exhausted; restart required"; return status; }
    status.epoch |= state.epoch;
    status.connected = state.connected && serial->m_params.bEnabled;
    status.writable = status.connected && output_allowed_ && output_allowed_() &&
                      PilotOutputPermitted(Endpoint(driver));
    status.detail = status.writable ? "Serial commissioning / physical feedback required / session starts OFF"
                                   : "Serial status only / bidirectional enabled connection required";
    return status;
  }
  auto *network = dynamic_cast<CommDriverN2KNet *>(driver);
  if (!network) {
    const auto *stats = dynamic_cast<DriverStatsProvider *>(driver);
    status.connected = stats && stats->GetDriverStats().available;
    status.detail =
        "Status only; this transport has no qualified SKAGER control path";
    return status;
  }
  const auto params = network->GetParams();
  if (network->GetConnectionGeneration() >= UINT32_MAX) {
    status.detail = "Network identity exhausted; restart required";
    return status;
  }
  status.epoch |= network->GetConnectionGeneration();
  const auto *socket = network->GetSock();
  status.connected = params.bEnabled && socket && socket->IsOk() &&
                     (params.NetProtocol != TCP || socket->IsConnected());
  status.writable = status.connected && params.NetProtocol == TCP &&
                    params.IOSelect == DS_TYPE_INPUT_OUTPUT &&
                    network->GetN2kFormat() == N2KFormat_Actisense_N2K_ASCII &&
                    output_allowed_ && output_allowed_() &&
                    PilotOutputPermitted(Endpoint(driver));
  status.detail =
      !status.connected ? "OpenCPN network disconnected"
      : !PilotLoopbackTestsEnabled() ? "Status only; SKAGER equipment output unavailable in this product"
      : params.NetProtocol != TCP ? "Status only; N2K UDP output is unsupported"
      : params.IOSelect != DS_TYPE_INPUT_OUTPUT
          ? "Status only; bidirectional OpenCPN connection required"
      : network->GetN2kFormat() != N2KFormat_Actisense_N2K_ASCII
          ? "Status only; qualified output requires Actisense complete-PGN "
            "ASCII"
      : !status.writable ? "Test output withheld: local peer or session isolation"
                         : "TEST LOOPBACK ONLY / OpenCPN TCP / Actisense N2K ASCII / feedback "
                           "confirmation required";
  return status;
}
adapters::PilotCapabilities OpenCPNPilot::Capabilities() const {
  MainThread();
  if (!PilotLoopbackTestsEnabled() && !PilotManualSerialEnabled()) return {};
  auto capabilities = pilot_.Capabilities();
  capabilities.track = capabilities.track && NavigationBlocker().empty();
  return capabilities;
}
bool OpenCPNPilot::Send(const adapters::PilotRequest &r) {
  MainThread();
  if (r.action == adapters::PilotAction::Track && !NavigationBlocker().empty()) {
    InvalidateNavigationOutput("TRACK needs fresh navigation output from the current active route");
    return false;
  }
  return session_enabled_ && output_allowed_ && output_allowed_() && pilot_.Send(r);
}
std::optional<NavigationOutputSession> OpenCPNPilot::NavigationSession() const {
  MainThread();
  const auto now = vessel::Clock::now();
  const auto transport = Status(binding_.interface_id);
  const auto feedback = pilot_.GetState();
  if (!session_enabled_ || !binding_.permit_control || !transport.writable ||
      transport.epoch != session_epoch_ || !control_generation_ ||
      !dynamic_cast<CommDriverN2KSerial *>(Driver(binding_.interface_id)) ||
      !pilot_.Capabilities().manual_control || !feedback.sequence ||
      (feedback.mode != adapters::PilotMode::Auto && feedback.mode != adapters::PilotMode::Track) ||
      feedback.observed_at > now || now - feedback.observed_at >= std::chrono::seconds(3))
    return std::nullopt;
  return NavigationOutputSession{binding_.interface_id, session_epoch_, control_generation_};
}
bool OpenCPNPilot::OwnsNavigationInterface(const std::string &iface) const {
  MainThread();
  // Reserve the explicitly selected pilot connection even while control is OFF:
  // ordinary route output must not bypass this session's guarded feed.
  return binding_.permit_control && !iface.empty() && iface == binding_.interface_id;
}
void OpenCPNPilot::SetNavigationValidator(std::function<bool()> current) {
  MainThread();
  navigation_current_ = std::move(current);
}
void OpenCPNPilot::InvalidateNavigationOutput(const std::string &reason) {
  MainThread();
  if (auto *serial = dynamic_cast<CommDriverN2KSerial *>(Driver(binding_.interface_id)))
    serial->ClearNavigationOutput();
  navigation_token_ = 0;
  navigation_source_at_ = {};
  navigation_blocker_ = reason;
}
bool OpenCPNPilot::SendNavigationBatch(const NavigationOutputSession &session,
    const std::vector<NavigationOutputFrame> &frames, std::uint16_t token,
    vessel::Time source_at) {
  MainThread();
  const auto current = NavigationSession();
  const auto now = vessel::Clock::now();
  if (!current || current->interface_id != session.interface_id ||
      current->epoch != session.epoch || current->generation != session.generation ||
      !navigation_current_ || !navigation_current_() || !token || token > 65532 ||
      source_at > now || now - source_at >= std::chrono::seconds(3) || frames.size() != 3)
    return false;
  auto *serial = dynamic_cast<CommDriverN2KSerial *>(Driver(session.interface_id));
  if (!serial) return false;
  const std::uint32_t order[] = {129285, 129284, 129283};
  std::vector<std::shared_ptr<const Nmea2000Msg>> messages;
  const auto destination = std::make_shared<const NavAddr2000>(session.interface_id, 255);
  for (std::size_t i = 0; i < frames.size(); ++i) {
    if (frames[i].pgn != order[i] || frames[i].data.empty() || frames[i].data.size() > 223)
      return false;
    messages.push_back(std::make_shared<const Nmea2000Msg>(order[i], frames[i].data, destination, 6));
  }
  // Normal management packets remain in the ordinary queue, which drains
  // before a route batch can receive its separate local-write receipt.
  for (auto pgn : order) serial->AddTxPGN(pgn);
  if (!serial->SendNavigationBatch(messages, session.epoch & UINT32_MAX, token, source_at))
    return false;
  navigation_token_ = token;
  navigation_source_at_ = source_at;
  navigation_blocker_ = "Waiting for the current route batch to be written locally";
  return true;
}
std::string OpenCPNPilot::NavigationBlocker() const {
  MainThread();
  if (!NavigationSession()) return "Enable control with fresh physical AUTO or TRACK feedback";
  if (!navigation_current_ || !navigation_current_())
    return "TRACK needs a fresh, stable active route and measured position";
  if (!navigation_token_) return navigation_blocker_;
  const auto now = vessel::Clock::now();
  auto *serial = dynamic_cast<CommDriverN2KSerial *>(Driver(binding_.interface_id));
  const auto state = serial ? serial->GetSerialState() : N2kSerialState::Snapshot{};
  if (!state.connected || state.epoch != (session_epoch_ & UINT32_MAX) ||
      state.navigation_written_token != navigation_token_ ||
      state.navigation_valid_until != navigation_source_at_ + std::chrono::seconds(3) ||
      state.navigation_written_at > now || now >= state.navigation_valid_until ||
      navigation_source_at_ > now || now - navigation_source_at_ >= std::chrono::seconds(3))
    return "Waiting for fresh route output on the selected pilot connection";
  // This proves a local write only. The bridge still qualifies true-bearing
  // variation and navigation; only subsequent physical mode can confirm TRACK.
  return {};
}
void OpenCPNPilot::SetControlEnabled(bool enabled) {
  MainThread();
  const auto now = vessel::Clock::now();
  const auto feedback = pilot_.GetState();
  const auto transport = Status(binding_.interface_id);
  enabled = enabled && pilot_.Capabilities().manual_control && feedback.sequence &&
            feedback.mode != adapters::PilotMode::Unavailable && feedback.observed_at <= now &&
            now - feedback.observed_at < std::chrono::seconds(3);
  if (enabled && !session_enabled_) {
    if (control_generation_ == (std::numeric_limits<std::uint64_t>::max)()) enabled = false;
    else ++control_generation_;
  }
  InvalidateNavigationOutput("Waiting for a coherent active route in this control session");
  if (auto *serial = dynamic_cast<CommDriverN2KSerial *>(Driver(binding_.interface_id)))
    serial->EnablePilotOutput(enabled);
  session_enabled_ = enabled;
  if (!enabled) awaiting_serial_write_ = false;
  session_epoch_ = enabled ? transport.epoch : 0;
}
bool OpenCPNPilot::RequestIdentity(vessel::Time now) {
  MainThread();
  if (binding_.interface_id.empty() || !output_allowed_ || !output_allowed_() ||
      (last_discovery_ && (now < *last_discovery_ || now - *last_discovery_ < std::chrono::seconds(5)))) return false;
  last_discovery_ = now;
  return Send(binding_.interface_id, 255, 59904, 6, {0, 0xee, 0});
}
bool OpenCPNPilot::Send(const std::string &iface, std::uint8_t destination,
                        std::uint32_t pgn, std::uint8_t priority,
                        const std::vector<std::uint8_t> &data) {
  MainThread();
  const auto status = Status(iface);
  if (!status.writable || iface != binding_.interface_id ||
      !((pgn == 126208 && binding_.permit_control && destination < 254 &&
         data.size() == 13) ||
        (pgn == 59904 && destination == 255 &&
         data == std::vector<std::uint8_t>{0, 0xee, 0})))
    return false;
  auto *driver = Driver(iface);
  if (!driver) return false;
  if (pgn == 126208) {
    const auto now = vessel::Clock::now();
    const auto feedback = pilot_.GetState();
    if (!session_enabled_ || session_epoch_ != status.epoch || !pilot_.Address() ||
        *pilot_.Address() != destination || !pilot_.Capabilities().manual_control ||
        !feedback.sequence || feedback.mode == adapters::PilotMode::Unavailable ||
        feedback.observed_at > now || now - feedback.observed_at >= std::chrono::seconds(3) || priority != 3)
      return false;
    bool exact = false;
    for (double delta : {-10., -1., 0., 1., 10.}) {
      adapters::PilotRequest candidate{1, delta == 0 ? adapters::PilotAction::Standby : adapters::PilotAction::AlterCourse, delta, now};
      exact = exact || data == adapters::EncodeSt4000Command(candidate);
    }
    exact = exact || data == adapters::EncodeSt4000Command({1, adapters::PilotAction::Auto, 0, now});
    exact = exact || data == adapters::EncodeSt4000Command({1, adapters::PilotAction::Track, 0, now});
    exact = exact || data == adapters::EncodeSt4000Command({1, adapters::PilotAction::Wind, 0, now});
    if (!exact) return false;
    const auto valid_heading = [now](const vessel::Sample& sample) {
      return sample.validity == vessel::Validity::Measured && sample.value &&
             std::isfinite(*sample.value) && *sample.value >= 0 && *sample.value < 360 &&
             sample.observed_at <= now && now - sample.observed_at < std::chrono::seconds(3);
    };
    const auto key = data.back();
    if (key == 0x40 && !valid_heading(feedback.heading_magnetic_deg)) return false;
    const bool alters_course = key == 0x51 || key == 0x7f || key == 0xd1 || key == 0x50;
    if (alters_course &&
        (feedback.mode != adapters::PilotMode::Auto || !valid_heading(feedback.locked_heading_magnetic_deg))) return false;
    // The bridge independently requires fresh navigation before native TRACK.
    if (key == 0x80 && feedback.mode != adapters::PilotMode::Auto &&
        feedback.mode != adapters::PilotMode::Track) return false;
    if (key == 0x80 && !NavigationBlocker().empty()) {
      InvalidateNavigationOutput("TRACK withheld because route output is no longer current");
      return false;
    }
    // WIND uses the common fresh physical feedback gate above; the bridge
    // decides apparent-wind readiness without a SKAGER heading substitute.
  } else if (priority != 6) return false;
  auto destination_address = std::make_shared<NavAddr2000>(iface, destination);
  auto message =
      std::make_shared<Nmea2000Msg>(pgn, data, destination_address, priority);
  // Driver serialization/transport remains OpenCPN-owned. Its true return does
  // NOT establish network delivery; only new physical status can confirm.
  return DispatchPilotOutput(Endpoint(driver), [&] {
    if (auto *serial = dynamic_cast<CommDriverN2KSerial *>(driver)) {
      uint64_t ticket = 0;
      if (!serial->SendPilotMessage(message, destination_address, status.epoch & UINT32_MAX, &ticket)) return false;
      if (pgn == 126208) {
        awaited_write_ticket_ = ticket;
        awaiting_serial_write_ = true;
      }
      return true;
    }
    return driver->SendMessage(message, destination_address);
  });
}
} // namespace opennav::integration

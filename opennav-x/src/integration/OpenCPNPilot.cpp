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
  auto feedback = !binding_.name.empty() ? pilot_.GetState()
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
  return !binding_.name.empty() ? pilot_.Status()
                                   : status_.Description(vessel::Clock::now());
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
  return pilot_.Capabilities();
}
bool OpenCPNPilot::Send(const adapters::PilotRequest &r) {
  MainThread();
  return session_enabled_ && output_allowed_ && output_allowed_() && pilot_.Send(r);
}
void OpenCPNPilot::SetControlEnabled(bool enabled) {
  MainThread();
  const auto now = vessel::Clock::now();
  const auto feedback = pilot_.GetState();
  const auto transport = Status(binding_.interface_id);
  enabled = enabled && pilot_.Capabilities().manual_control && feedback.sequence &&
            feedback.mode != adapters::PilotMode::Unavailable && feedback.observed_at <= now &&
            now - feedback.observed_at < std::chrono::seconds(3);
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
    if (!exact) return false;
    const auto valid_heading = [now](const vessel::Sample& sample) {
      return sample.validity == vessel::Validity::Measured && sample.value &&
             std::isfinite(*sample.value) && *sample.value >= 0 && *sample.value < 360 &&
             sample.observed_at <= now && now - sample.observed_at < std::chrono::seconds(3);
    };
    const auto key = data.back();
    if (key == 0x40 && !valid_heading(feedback.heading_magnetic_deg)) return false;
    if (key != 0 && key != 0x40 &&
        (feedback.mode != adapters::PilotMode::Auto || !valid_heading(feedback.locked_heading_magnetic_deg))) return false;
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

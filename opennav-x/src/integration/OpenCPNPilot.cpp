#include "integration/OpenCPNPilot.h"
#include "model/comm_drv_n2k_net.h"
#include "model/comm_drv_registry.h"
#include "model/comm_drv_stats.h"
#include <limits>
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
} // namespace
OpenCPNPilot::OpenCPNPilot(std::function<bool()> allowed)
    : output_allowed_(std::move(allowed)) {
  MainThread();
  auto changes = std::make_unique<ObsListener>();
  changes->Init(CommDriverRegistry::GetInstance().evt_driverlist_change,
                [this](ObservedEvt &) {
                  if (registry_generation_ < UINT32_MAX)
                    ++registry_generation_;
                  pilot_.Poll(vessel::Clock::now());
                });
  listeners_.push_back(std::move(changes));
  for (const auto pgn : {60928u, 65379u, 65360u, 127250u}) {
    auto listener = std::make_unique<ObsListener>();
    listener->Init(Nmea2000Msg(pgn), [this, pgn](ObservedEvt &event) {
      const auto m = UnpackEvtPointer<Nmea2000Msg>(event);
      if (!m || !m->source || m->source->iface != binding_.interface_id ||
          m->payload.size() != 22 || m->payload[0] != 0x93 ||
          m->payload[12] != 8 || m->payload[7] >= 254)
        return;
      const auto encoded_pgn = unsigned(m->payload[3]) |
                               (unsigned(m->payload[4]) << 8) |
                               (unsigned(m->payload[5]) << 16);
      if (encoded_pgn != pgn)
        return;
      const auto now = vessel::Clock::now();
      const auto age = std::chrono::system_clock::now() - m->created_at;
      if (age < std::chrono::system_clock::duration::zero() ||
          age >= std::chrono::seconds(3))
        return;
      const auto at =
          now - std::chrono::duration_cast<vessel::Clock::duration>(age);
      if (const auto *network =
              dynamic_cast<CommDriverN2KNet *>(Driver(binding_.interface_id)))
        if (at < network->GetConnectionChangedAt())
          return;
      pilot_.Observe({m->source->iface, pgn, m->payload[7],
                      std::vector<std::uint8_t>(m->payload.begin() + 13,
                                                m->payload.begin() + 21),
                      at},
                     now);
    });
    listeners_.push_back(std::move(listener));
  }
}
void OpenCPNPilot::Configure(const adapters::St4000Binding &binding) {
  MainThread();
  pilot_.Configure(binding);
  binding_ = binding;
}
adapters::PilotFeedback OpenCPNPilot::GetState() const {
  MainThread();
  return pilot_.GetState();
}
void OpenCPNPilot::Poll(vessel::Time now) {
  MainThread();
  pilot_.Poll(now);
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
  auto *network = dynamic_cast<CommDriverN2KNet *>(driver);
  if (!network) {
    const auto *stats = dynamic_cast<DriverStatsProvider *>(driver);
    status.connected = stats && stats->GetDriverStats().available;
    status.detail =
        "Status only; this transport has no qualified XNav control path";
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
                    output_allowed_ && output_allowed_();
  status.detail =
      !status.connected           ? "OpenCPN network disconnected"
      : params.NetProtocol != TCP ? "Status only; N2K UDP output is unsupported"
      : params.IOSelect != DS_TYPE_INPUT_OUTPUT
          ? "Status only; bidirectional OpenCPN connection required"
      : network->GetN2kFormat() != N2KFormat_Actisense_N2K_ASCII
          ? "Status only; qualified output requires Actisense complete-PGN "
            "ASCII"
      : !status.writable ? "Output isolated during DEMO/REPLAY/transition"
                         : "OpenCPN TCP / Actisense N2K ASCII / feedback "
                           "confirmation required";
  return status;
}
adapters::PilotCapabilities OpenCPNPilot::Capabilities() const {
  MainThread();
  return pilot_.Capabilities();
}
bool OpenCPNPilot::Send(const adapters::PilotRequest &r) {
  MainThread();
  return output_allowed_ && output_allowed_() && pilot_.Send(r);
}
bool OpenCPNPilot::RequestIdentity(vessel::Time now) {
  MainThread();
  return output_allowed_ && output_allowed_() && pilot_.RequestIdentity(now);
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
  if (!driver)
    return false;
  auto destination_address = std::make_shared<NavAddr2000>(iface, destination);
  auto message =
      std::make_shared<Nmea2000Msg>(pgn, data, destination_address, priority);
  // Driver serialization/transport remains OpenCPN-owned. Its true return does
  // NOT establish network delivery; only new physical status can confirm.
  return driver->SendMessage(message, destination_address);
}
} // namespace opennav::integration

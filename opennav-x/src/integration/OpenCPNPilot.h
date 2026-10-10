#pragma once
#include "adapters/St4000Pilot.h"
#include "integration/PilotStatusDiscovery.h"
#include "integration/PilotTrafficDiagnostics.h"
#include "observable.h"
#include <functional>
#include <memory>

namespace opennav::integration {
// Main-thread transport boundary. No driver pointer escapes this class.
class OpenCPNPilot final : public adapters::IAutopilot,
                           private adapters::IN2kPilotTransport {
public:
  explicit OpenCPNPilot(std::function<bool()> output_allowed);
  ~OpenCPNPilot() override;
  void Configure(const adapters::St4000Binding &binding);
  adapters::PilotCapabilities Capabilities() const override;
  adapters::PilotFeedback GetState() const override;
  void Poll(vessel::Time now) override;
  bool Send(const adapters::PilotRequest &request) override;
  bool RequestIdentity(vessel::Time now);
  void SetControlEnabled(bool enabled) override;
  bool ControlEnabled() const override { return session_enabled_; }
  bool SessionEnabled() const { return session_enabled_; }
  std::string Description() const;
  // The single live pilot seen on OpenCPN's receive connections, offered for
  // an explicit operator binding (never permission). Empty when ambiguous.
  std::optional<adapters::St4000Binding> DetectedBinding() const;
  // Plain-language reason the pilot's connection cannot carry commands (the
  // operator can fix it in OpenCPN), or empty. Unlike AutoTrack, SKAGER only
  // transmits where the user enabled output on the connection.
  std::string ControlBlocker() const;
  PilotStatusDiscovery::Diagnostics DiscoveryDiagnostics(vessel::Time now) const;
  PilotTrafficDiagnostics::Snapshot TrafficDiagnostics() const;

private:
  adapters::PilotTransportStatus
  Status(const std::string &interface_id) const override;
  bool Send(const std::string &interface_id, std::uint8_t destination,
            std::uint32_t pgn, std::uint8_t priority,
            const std::vector<std::uint8_t> &data) override;
  std::vector<std::unique_ptr<ObsListener>> listeners_;
  std::function<bool()> output_allowed_;
  std::uint64_t registry_generation_ = 1;
  adapters::St4000Binding binding_;
  bool session_enabled_ = false;
  std::uint64_t session_epoch_ = 0, awaited_write_ticket_ = 0;
  bool awaiting_serial_write_ = false;
  std::optional<vessel::Time> last_discovery_;
  adapters::St4000Pilot pilot_{*this};
  PilotStatusDiscovery status_{*this};
  PilotTrafficDiagnostics traffic_;
};
} // namespace opennav::integration

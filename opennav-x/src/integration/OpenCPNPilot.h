#pragma once
#include "adapters/St4000Pilot.h"
#include "observable.h"
#include <functional>
#include <memory>

namespace opennav::integration {
// Main-thread transport boundary. No driver pointer escapes this class.
class OpenCPNPilot final : public adapters::IAutopilot,
                           private adapters::IN2kPilotTransport {
public:
  explicit OpenCPNPilot(std::function<bool()> output_allowed);
  void Configure(const adapters::St4000Binding &binding);
  adapters::PilotCapabilities Capabilities() const override;
  adapters::PilotFeedback GetState() const override;
  void Poll(vessel::Time now) override;
  bool Send(const adapters::PilotRequest &request) override;
  bool RequestIdentity(vessel::Time now);
  std::string Description() const { return pilot_.Status(); }

private:
  adapters::PilotTransportStatus
  Status(const std::string &interface) const override;
  bool Send(const std::string &interface, std::uint8_t destination,
            std::uint32_t pgn, std::uint8_t priority,
            const std::vector<std::uint8_t> &data) override;
  std::vector<std::unique_ptr<ObsListener>> listeners_;
  std::function<bool()> output_allowed_;
  std::uint64_t registry_generation_ = 1;
  adapters::St4000Binding binding_;
  adapters::St4000Pilot pilot_{*this};
};
} // namespace opennav::integration

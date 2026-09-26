#pragma once
#include "adapters/Autopilot.h"

namespace opennav::adapters {
// Defined only by the explicit developer/test fixture library.
class SimulatedAutopilot final : public IAutopilot {
public:
  explicit SimulatedAutopilot(vessel::Time now);
  PilotCapabilities Capabilities() const override;
  PilotFeedback GetState() const override { return feedback_; }
  void Poll(vessel::Time now) override;
  bool Send(const PilotRequest &request) override;
  void SetFailure(bool reject, bool lose_feedback) {
    reject_ = reject;
    lose_feedback_ = lose_feedback;
  }

private:
  PilotFeedback feedback_;
  std::optional<PilotRequest> pending_;
  bool reject_ = false, lose_feedback_ = false;
};
} // namespace opennav::adapters

#include "adapters/SimulatedAutopilot.h"
#include <cmath>

// Linked only into dedicated tests or explicitly enabled developer builds.
namespace opennav::adapters {
namespace {
double Wrap(double d) { return std::fmod(d + 360.0, 360.0); }
PilotMode RequestedMode(PilotAction a) {
  switch (a) {
  case PilotAction::Standby:
    return PilotMode::Standby;
  case PilotAction::Auto:
  case PilotAction::AlterCourse:
    return PilotMode::Auto;
  case PilotAction::Track:
    return PilotMode::Track;
  case PilotAction::Wind:
    return PilotMode::Wind;
  }
  return PilotMode::Unavailable;
}
} // namespace
SimulatedAutopilot::SimulatedAutopilot(vessel::Time now) {
  feedback_.mode = PilotMode::Standby;
  feedback_.observed_at = now;
  feedback_.sequence = 1;
  feedback_.source = "DEMO simulated pilot feedback";
  feedback_.heading_magnetic_deg = {80, feedback_.source, now,
                                    vessel::Validity::Measured};
}
PilotCapabilities SimulatedAutopilot::Capabilities() const {
  return {true, true, true, true, true, true};
}
bool SimulatedAutopilot::Send(const PilotRequest &request) {
  if (reject_)
    return false;
  if (pending_ && request.action != PilotAction::Standby)
    return false;
  pending_ = request;
  return true; // Transport acceptance only.
}
void SimulatedAutopilot::Poll(vessel::Time now) {
  if (lose_feedback_ || now < feedback_.observed_at)
    return;
  const bool response =
      pending_ && now > pending_->issued_at &&
      now - pending_->issued_at >= std::chrono::milliseconds(500);
  if (!response && now - feedback_.observed_at < std::chrono::seconds(1))
    return;
  if (response) {
    feedback_.mode = RequestedMode(pending_->action);
    if (pending_->action == PilotAction::Auto)
      feedback_.locked_heading_magnetic_deg = feedback_.heading_magnetic_deg;
    if (pending_->action == PilotAction::AlterCourse &&
        feedback_.locked_heading_magnetic_deg.value)
      feedback_.locked_heading_magnetic_deg.value = Wrap(
          *feedback_.locked_heading_magnetic_deg.value + pending_->delta_deg);
    if (pending_->action == PilotAction::Standby)
      feedback_.locked_heading_magnetic_deg = {};
    pending_.reset();
  }
  feedback_.observed_at = now;
  ++feedback_.sequence;
  feedback_.heading_magnetic_deg.observed_at = now;
  if (feedback_.locked_heading_magnetic_deg.value)
    feedback_.locked_heading_magnetic_deg.observed_at = now;
}
} // namespace opennav::adapters

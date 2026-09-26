#include "adapters/Autopilot.h"
#include <cmath>
#include <limits>

namespace opennav::adapters {
namespace {
bool Fresh(const PilotFeedback &s, vessel::Time now) {
  return s.mode != PilotMode::Unavailable && s.sequence && !s.source.empty() &&
         s.observed_at <= now && now - s.observed_at < std::chrono::seconds(3);
}
bool Heading(const vessel::Sample &s, vessel::Time now) {
  const auto a = vessel::Assess(
      s, now, {std::chrono::seconds(1), std::chrono::seconds(3)});
  return s.validity == vessel::Validity::Measured && a.value && *a.value >= 0 &&
         *a.value < 360 &&
         (a.quality == vessel::Quality::Live ||
          a.quality == vessel::Quality::Aging);
}
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
const char *PilotModeName(PilotMode m) {
  switch (m) {
  case PilotMode::Standby:
    return "STANDBY";
  case PilotMode::Auto:
    return "AUTO";
  case PilotMode::Track:
    return "TRACK";
  case PilotMode::Wind:
    return "WIND";
  default:
    return "UNAVAILABLE";
  }
}
const char *CommandStateName(CommandState s) {
  switch (s) {
#define STATE(x)                                                               \
  case CommandState::x:                                                        \
    return #x
    STATE(None);
    STATE(Pending);
    STATE(Confirmed);
    STATE(Rejected);
    STATE(TimedOut);
    STATE(Disabled);
    STATE(StaleFeedback);
    STATE(Requested);
#undef STATE
  }
  return "Unknown";
}
void ManualAutopilot::Record(CommandState state, const std::string &detail,
                             vessel::Time now) {
  command_.state = state;
  command_.detail = detail;
  command_.updated_at = now;
  if (log_.size() >= 128)
    log_.erase(log_.begin());
  log_.push_back(command_);
}
void ManualAutopilot::Enable(bool value, vessel::Time now) {
  enabled_ = value;
  if (!value && command_.state == CommandState::Pending)
    Record(CommandState::Disabled,
           "Control disabled; an already transmitted command may still take "
           "effect. Use physical STANDBY.",
           now);
}
PilotView ManualAutopilot::GetState(vessel::Time now) const {
  const auto feedback = adapter_.GetState();
  return {feedback, adapter_.Capabilities(), command_, enabled_,
          Fresh(feedback, now)};
}
PilotCommand ManualAutopilot::Request(PilotAction action, double delta,
                                      vessel::Time now) {
  const auto capabilities = adapter_.Capabilities();
  if (last_sent_ && (now < *last_sent_ ||
                    now - *last_sent_ < std::chrono::milliseconds(250))) {
    auto refusal = command_;
    refusal.state = CommandState::Rejected;
    refusal.detail = "Touch repeat suppressed; no additional command sent";
    return refusal;
  }
  // A second non-standby request never replaces an unacknowledged command.
  if (command_.state == CommandState::Pending &&
      (action != PilotAction::Standby || delta != 0 || !std::isfinite(delta))) {
    auto refusal = command_;
    refusal.state = CommandState::Rejected;
    refusal.detail = "One command is pending; wait for feedback or use STANDBY";
    return refusal;
  }
  const auto feedback = adapter_.GetState();
  if (command_.state == CommandState::Pending)
    Record(CommandState::Rejected,
           "Superseded by manual STANDBY; earlier outcome is unknown", now);
  command_ = {{next_id_, action, delta, now}, CommandState::None, {}, now};
  expected_heading_.reset();
  if (next_id_ == std::numeric_limits<std::uint64_t>::max()) {
    Record(CommandState::Disabled,
           "Command identity exhausted; restart required", now);
    return command_;
  }
  ++next_id_;
  if (!enabled_ || (!capabilities.simulated && !capabilities.manual_control)) {
    Record(
        CommandState::Disabled,
        "Control OFF; explicit permission and a verified adapter are required",
        now);
    return command_;
  }
  const bool supported =
      action == PilotAction::Standby ? capabilities.standby
      : action == PilotAction::Auto  ? capabilities.auto_mode
      : action == PilotAction::Track ? capabilities.track
      : action == PilotAction::Wind
          ? capabilities.wind
          : action == PilotAction::AlterCourse && capabilities.alter_course;
  if (!supported || !std::isfinite(delta) ||
      (action == PilotAction::AlterCourse
           ? !(delta == 1 || delta == -1 || delta == 10 || delta == -10)
           : delta != 0)) {
    Record(CommandState::Rejected, "Unsupported command or course increment",
           now);
    return command_;
  }
  if (action != PilotAction::Standby && !Fresh(feedback, now)) {
    Record(CommandState::StaleFeedback,
           "Fresh measured pilot state is required", now);
    return command_;
  }
  if (action == PilotAction::Auto &&
      !Heading(feedback.heading_magnetic_deg, now)) {
    Record(CommandState::StaleFeedback,
           "Fresh measured magnetic heading required for AUTO", now);
    return command_;
  }
  if (action == PilotAction::AlterCourse) {
    if (feedback.mode != PilotMode::Auto ||
        !Heading(feedback.locked_heading_magnetic_deg, now)) {
      Record(
          CommandState::Rejected,
          "Course change requires confirmed AUTO and locked magnetic heading",
          now);
      return command_;
    }
    expected_heading_ =
        Wrap(*feedback.locked_heading_magnetic_deg.value + delta);
  }
  feedback_sequence_ = feedback.sequence;
  connection_epoch_ = feedback.connection_epoch;
  feedback_source_ = feedback.source;
  Record(CommandState::Requested, "Manual request validated", now);
  last_sent_ = now; // Also bound retries after a rejected transport attempt.
  if (!adapter_.Send(command_.request))
    Record(CommandState::Rejected, "Adapter rejected transmission", now);
  else
    Record(CommandState::Pending, "Sent; awaiting new matching pilot feedback",
           now);
  return command_;
}
void ManualAutopilot::Tick(vessel::Time now) {
  adapter_.Poll(now);
  const auto capabilities = adapter_.Capabilities();
  if (enabled_ && !capabilities.simulated && !capabilities.manual_control) {
    Enable(false, now); // Reconnection never silently re-enables live output.
    return;
  }
  if (command_.state != CommandState::Pending)
    return;
  // Late feedback cannot retroactively turn an expired command into success.
  if (now < command_.request.issued_at ||
      now - command_.request.issued_at >= std::chrono::seconds(3)) {
    Record(CommandState::TimedOut,
           "No timely acknowledgement; outcome unknown, no automatic retry",
           now);
    return;
  }
  const auto feedback = adapter_.GetState();
  if (feedback.connection_epoch != connection_epoch_ ||
      feedback.source != feedback_source_) {
    Record(CommandState::Rejected,
           "Pilot identity/connection changed; previous outcome unknown", now);
    return;
  }
  bool matches = Fresh(feedback, now) &&
                 feedback.sequence > feedback_sequence_ &&
                 feedback.observed_at > command_.request.issued_at &&
                 feedback.mode == RequestedMode(command_.request.action);
  if (expected_heading_)
    matches =
        matches && Heading(feedback.locked_heading_magnetic_deg, now) &&
        feedback.locked_heading_magnetic_deg.observed_at >
            command_.request.issued_at &&
        std::abs(std::remainder(*feedback.locked_heading_magnetic_deg.value -
                                    *expected_heading_,
                                360.0)) <= .5;
  if (matches)
    Record(CommandState::Confirmed,
           "Confirmed by subsequent matching pilot feedback", now);
}

} // namespace opennav::adapters

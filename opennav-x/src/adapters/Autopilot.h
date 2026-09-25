#pragma once
#include "vessel/VesselState.h"
#include <cstdint>
#include <vector>

namespace opennav::adapters {
enum class PilotMode { Unavailable, Standby, Auto, Track, Wind };
enum class PilotAction { Standby, Auto, Track, Wind, AlterCourse };
enum class CommandState {
  None,
  Pending,
  Confirmed,
  Rejected,
  TimedOut,
  Disabled,
  StaleFeedback,
  Requested
};
struct PilotCapabilities {
  bool simulated = false, standby = false, auto_mode = false, track = false,
       wind = false, alter_course = false;
  // True only for an explicitly configured, identity-verified live adapter.
  // This is permission/capability, never an assertion of hardware acceptance.
  bool manual_control = false;
};
struct PilotFeedback {
  PilotMode mode = PilotMode::Unavailable;
  vessel::Sample heading_magnetic_deg, locked_heading_magnetic_deg;
  vessel::Time observed_at{};
  std::uint64_t sequence = 0;
  std::string source;
  std::uint64_t connection_epoch = 0;
};
struct PilotRequest {
  std::uint64_t id = 0;
  PilotAction action = PilotAction::Standby;
  double delta_deg = 0;
  vessel::Time issued_at{};
};
// Read state, explicit polling, and command transport are separate operations.
// A transport acknowledgement is never a physical-mode acknowledgement.
class IAutopilot {
public:
  virtual ~IAutopilot() = default;
  virtual PilotCapabilities Capabilities() const = 0;
  virtual PilotFeedback GetState() const = 0;
  virtual void Poll(vessel::Time now) = 0;
  virtual bool Send(const PilotRequest &request) = 0;
};
struct PilotCommand {
  PilotRequest request;
  CommandState state = CommandState::None;
  std::string detail;
  vessel::Time updated_at{};
};
struct PilotView {
  PilotFeedback feedback;
  PilotCapabilities capabilities;
  PilotCommand command;
  bool enabled = false, fresh = false;
};
class ManualAutopilot {
public:
  explicit ManualAutopilot(IAutopilot &adapter) : adapter_(adapter) {}
  void Enable(bool enabled, vessel::Time now);
  PilotCommand Request(PilotAction action, double delta_deg, vessel::Time now);
  void Tick(vessel::Time now);
  PilotView GetState(vessel::Time now) const;
  const std::vector<PilotCommand> &Log() const { return log_; }

private:
  void Record(CommandState state, const std::string &detail, vessel::Time now);
  IAutopilot &adapter_;
  bool enabled_ = false;
  std::uint64_t next_id_ = 1, feedback_sequence_ = 0;
  std::uint64_t connection_epoch_ = 0;
  std::string feedback_source_;
  std::optional<vessel::Time> last_sent_;
  std::optional<double> expected_heading_;
  PilotCommand command_;
  std::vector<PilotCommand> log_;
};
class UnavailableAutopilot final : public IAutopilot {
public:
  PilotCapabilities Capabilities() const override { return {}; }
  PilotFeedback GetState() const override { return {}; }
  void Poll(vessel::Time) override {}
  bool Send(const PilotRequest &) override { return false; }
};
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
const char *PilotModeName(PilotMode mode);
const char *CommandStateName(CommandState state);
} // namespace opennav::adapters

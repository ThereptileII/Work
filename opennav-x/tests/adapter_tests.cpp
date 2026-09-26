#include "adapters/SimulatedAutopilot.h"
#include "adapters/RadarSimulator.h"
#include "adapters/Autopilot.h"
#include "adapters/Radar.h"
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace adapters;
using namespace std::chrono_literals;
const vessel::Time epoch{100s};
void Check(bool value, const char *why) {
  if (!value)
    throw std::runtime_error(why);
}
void Manual() {
  SimulatedAutopilot simulator(epoch);
  ManualAutopilot pilot(simulator);
  Check(pilot.Request(PilotAction::Auto, 0, epoch).state ==
            CommandState::Disabled,
        "Default global disable");
  pilot.Enable(true, epoch);
  auto command = pilot.Request(PilotAction::Auto, 0, epoch);
  Check(command.state == CommandState::Pending &&
            simulator.GetState().mode == PilotMode::Standby,
        "Transport acknowledgement is not state");
  Check(pilot.Request(PilotAction::Auto, 0, epoch + 100ms).state ==
            CommandState::Rejected,
        "One pending command");
  pilot.Tick(epoch + 499ms);
  Check(pilot.GetState(epoch + 499ms).command.state == CommandState::Pending,
        "No premature acknowledgement");
  pilot.Tick(epoch + 500ms);
  Check(pilot.GetState(epoch + 500ms).command.state == CommandState::Confirmed,
        "Subsequent measured feedback confirms");
  Check(pilot.Request(PilotAction::AlterCourse, 10, epoch + 600ms).state ==
            CommandState::Pending,
        "Manual ten degree request");
  Check(simulator.GetState().locked_heading_magnetic_deg.value == 80,
        "Requested heading not displayed as observed");
  pilot.Tick(epoch + 1100ms);
  Check(simulator.GetState().locked_heading_magnetic_deg.value == 90 &&
            pilot.GetState(epoch + 1100ms).command.state ==
                CommandState::Confirmed,
        "Heading confirmation");
  Check(pilot.Request(PilotAction::AlterCourse, 20, epoch + 1200ms).state ==
            CommandState::Rejected,
        "Invalid course delta");
  pilot.Request(PilotAction::Wind, 0, epoch + 1300ms);
  pilot.Tick(epoch + 1800ms);
  Check(pilot.GetState(epoch + 1800ms).feedback.mode == PilotMode::Wind,
        "Capability-supported simulated wind mode");
  Check(pilot.Request(PilotAction::AlterCourse, 1, epoch + 1900ms).state ==
            CommandState::Rejected,
        "Manual course requires AUTO");
  pilot.Request(PilotAction::Track, 0, epoch + 2s);
  pilot.Tick(epoch + 2500ms);
  Check(pilot.GetState(epoch + 2500ms).feedback.mode == PilotMode::Track,
        "Capability-supported simulated track mode");
  pilot.Request(PilotAction::Standby, 0, epoch + 2600ms);
  pilot.Tick(epoch + 3100ms);
  Check(pilot.GetState(epoch + 3100ms).feedback.mode == PilotMode::Standby,
        "Standby observed");
  Check(!pilot.Log().empty() &&
            pilot.Log().back().state == CommandState::Confirmed,
        "Command log");
}
void Failure() {
  SimulatedAutopilot simulator(epoch);
  ManualAutopilot pilot(simulator);
  pilot.Enable(true, epoch);
  simulator.SetFailure(true, false);
  Check(pilot.Request(PilotAction::Auto, 0, epoch).state ==
            CommandState::Rejected,
        "Adapter rejection explicit");
  simulator.SetFailure(false, true);
  Check(pilot.Request(PilotAction::Auto, 0, epoch).state ==
            CommandState::Rejected,
        "Immediate retry after transport rejection is bounded");
  Check(pilot.Request(PilotAction::Auto, 0, epoch + 250ms).state ==
            CommandState::Pending,
        "Dropped response pending");
  pilot.Tick(epoch + 3250ms);
  Check(pilot.GetState(epoch + 3250ms).command.state == CommandState::TimedOut,
        "Exact timeout boundary");
  Check(!pilot.GetState(epoch + 3250ms).fresh, "Read never refreshes feedback");
  Check(pilot.Request(PilotAction::Auto, 0, epoch + 3250ms).state ==
            CommandState::StaleFeedback,
        "Stale state suppresses commands");
  Check(pilot.Request(PilotAction::Standby, 0, epoch + 3250ms).state ==
            CommandState::Pending,
        "Standby can be attempted with stale state");
  simulator.SetFailure(false, false);
  pilot.Tick(epoch + 3750ms);
  Check(pilot.GetState(epoch + 3750ms).feedback.mode == PilotMode::Standby,
        "Standby supersedes queued auto");
  pilot.Request(PilotAction::Auto, 0, epoch + 4s);
  pilot.Enable(false, epoch + 4100ms);
  pilot.Tick(epoch + 4500ms);
  Check(pilot.GetState(epoch + 4500ms).command.state == CommandState::Disabled,
        "Disable does not pretend in-flight command was cancelled");
  Check(pilot.GetState(epoch + 4500ms).feedback.mode == PilotMode::Auto,
        "Observed physical outcome remains visible after disabling");
  UnavailableAutopilot absent;
  ManualAutopilot live(absent);
  live.Enable(true, epoch);
  Check(live.Request(PilotAction::Standby, 0, epoch).state ==
            CommandState::Disabled,
        "No live output path in Alpha");
}
class ControlledFeedback final : public IAutopilot {
public:
  PilotFeedback feedback;
  int sends = 0;
  PilotCapabilities Capabilities() const override {
    return {true, true, true, false, false, true};
  }
  PilotFeedback GetState() const override { return feedback; }
  void Poll(vessel::Time) override {}
  bool Send(const PilotRequest &) override {
    ++sends;
    return true;
  }
};
void Evidence() {
  ControlledFeedback source;
  source.feedback = SimulatedAutopilot(epoch).GetState();
  ManualAutopilot pilot(source);
  pilot.Enable(true, epoch);
  Check(pilot.Request(PilotAction::Track, 0, epoch).state ==
            CommandState::Rejected,
        "Unsupported capability refused");
  pilot.Request(PilotAction::Auto, 0, epoch);
  source.feedback.mode = PilotMode::Auto;
  pilot.Tick(epoch + 1s);
  Check(pilot.GetState(epoch + 1s).command.state == CommandState::Pending,
        "Cached state and sequence not acknowledgement");
  source.feedback.sequence++;
  source.feedback.observed_at = epoch + 1s;
  pilot.Tick(epoch + 1s);
  Check(pilot.GetState(epoch + 1s).command.state == CommandState::Confirmed,
        "New matching state evidence");
  source.feedback.locked_heading_magnetic_deg = {
      359, "test feedback", epoch + 1s, vessel::Validity::Measured};
  pilot.Request(PilotAction::AlterCourse, 1, epoch + 1s);
  source.feedback.sequence++;
  source.feedback.observed_at = epoch + 2s;
  source.feedback.locked_heading_magnetic_deg.observed_at = epoch + 2s;
  source.feedback.locked_heading_magnetic_deg.value = 90;
  pilot.Tick(epoch + 2s);
  Check(pilot.GetState(epoch + 2s).command.state == CommandState::Pending,
        "Wrong heading not acknowledged");
  source.feedback.locked_heading_magnetic_deg.value = 0;
  pilot.Tick(epoch + 2s);
  Check(pilot.GetState(epoch + 2s).command.state == CommandState::Confirmed,
        "Magnetic north wrap acknowledgement");
  pilot.Request(PilotAction::Standby, 0, epoch + 2s);
  source.feedback.mode = PilotMode::Standby;
  source.feedback.observed_at = epoch + 5s;
  source.feedback.sequence++;
  pilot.Tick(epoch + 5s);
  Check(pilot.GetState(epoch + 5s).command.state == CommandState::TimedOut,
        "Late state cannot retroactively confirm");
  Check(source.sends == 3, "No automatic retransmit");
}
void Radar() {
  UnavailableRadar absent;
  Check(!absent.GetState().available &&
            !absent.SetPresentation(RadarPresentation::Overlay),
        "No fabricated live radar");
  RadarStatusSimulator radar;
  radar.Observe(true, epoch);
  Check(radar.GetState().capabilities.simulated, "Demo capability provenance");
  Check(radar.SetPresentation(RadarPresentation::Overlay) &&
            radar.GetState().presentation == RadarPresentation::Overlay,
        "Overlay status");
  Check(radar.SetPresentation(RadarPresentation::Focus), "Focus status");
  Check(radar.GetState().observed_at == epoch,
        "Presentation changes never freshen sensor age");
  radar.Observe(false, epoch + 1s);
  Check(radar.GetState().presentation == RadarPresentation::Off &&
            !radar.GetState().available,
        "Disconnected state");
}
int main(int argc, char **argv) {
  try {
    std::string group = argc > 1 ? argv[1] : "";
    if (group == "manual")
      Manual();
    else if (group == "failure")
      Failure();
    else if (group == "evidence")
      Evidence();
    else if (group == "radar")
      Radar();
    else
      throw std::runtime_error("Unknown group");
    std::cout << "PASS " << group << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

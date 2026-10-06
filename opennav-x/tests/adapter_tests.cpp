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
class SessionFeedback final : public IAutopilot {
public:
  PilotFeedback feedback;
  bool session=false;
  bool revoke_on_stale=false;
  unsigned changes=0, sends=0;
  PilotCapabilities Capabilities() const override { return {false,true,true,false,false,true,true}; }
  PilotFeedback GetState() const override { return feedback; }
  void Poll(vessel::Time now) override {
    if(revoke_on_stale && now-feedback.observed_at>=3s) session=false;
  }
  bool Send(const PilotRequest&) override { if(!session)return false; ++sends;return true; }
  void SetControlEnabled(bool enabled) override { session=enabled; ++changes; }
  bool ControlEnabled() const override { return session; }
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
  source.feedback.command_confirmation_allowed=false;
  pilot.Tick(epoch+1s);
  Check(pilot.GetState(epoch+1s).command.state==CommandState::Pending,"fresh receive before queued serial write cannot acknowledge");
  source.feedback.command_confirmation_allowed=true;
  source.feedback.command_written_at=epoch+1500ms;
  pilot.Tick(epoch+1s);
  Check(pilot.GetState(epoch+1s).command.state==CommandState::Pending,"receive predating completed serial write cannot acknowledge");
  source.feedback.command_written_at=epoch+500ms;
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
  SessionFeedback live;
  live.feedback=SimulatedAutopilot(epoch).GetState();
  ManualAutopilot manual(live);
  Check(!manual.GetState(epoch).enabled && !live.session,"both control layers start OFF");
  manual.Enable(true,epoch);
  Check(live.session,"explicit enable reaches transport barrier");
  live.session=false; // Worker disconnected before the next UI tick.
  manual.Tick(epoch+1s);
  Check(!manual.GetState(epoch+1s).enabled && !live.session,"worker revocation disables controller, no silent session recovery");
  manual.Enable(true,epoch+1s);
  manual.Tick(epoch+3s);
  Check(!live.session && !manual.GetState(epoch+3s).enabled,"stale live feedback disables transport as well as buttons");
  Check(live.changes==4,"every live session change reaches transport cancellation hook");

  for(const bool adapter_revokes:{false,true}) {
    SessionFeedback source;
    source.feedback=SimulatedAutopilot(epoch).GetState();
    source.revoke_on_stale=adapter_revokes;
    ManualAutopilot tracked(source);
    tracked.Enable(true,epoch);
    const auto request=tracked.Request(PilotAction::Auto,0,epoch+2s);
    Check(request.state==CommandState::Pending && source.sends==1,"one request accepted before feedback loss");
    tracked.Tick(epoch+3s); // Feedback stale; request still has two seconds left.
    auto state=tracked.GetState(epoch+3s);
    Check(!state.enabled && !source.session && !state.fresh &&
          state.command.state==CommandState::Pending && state.command.request.id==request.request.id,
          "automatic stale revocation stops output without cancelling the accepted request deadline");
    source.feedback.mode=PilotMode::Auto;
    source.feedback.sequence++;
    source.feedback.observed_at=epoch+3500ms;
    // A cancelled queue may now report fresh telemetry with no old write proof.
    source.feedback.command_confirmation_allowed=true;
    tracked.Tick(epoch+3500ms);
    Check(tracked.GetState(epoch+3500ms).fresh && !tracked.GetState(epoch+3500ms).enabled &&
          tracked.GetState(epoch+3500ms).command.state==CommandState::Pending && source.sends==1,
          "fresh recovery neither re-enables nor confirms a revoked command");
    if(adapter_revokes) tracked.Enable(true,epoch+4s);
    tracked.Tick(epoch+4999ms);
    Check(tracked.GetState(epoch+4999ms).command.state==CommandState::Pending,
          "even explicit re-enable cannot recreate old write proof before the deadline");
    tracked.Tick(epoch+5s);
    Check(tracked.GetState(epoch+5s).command.state==CommandState::TimedOut &&
          tracked.GetState(epoch+5s).command.request.id==request.request.id && source.sends==1,
          "original accepted request times out exactly once without retransmit");
    const auto log_size=tracked.Log().size();
    tracked.Tick(epoch+6s);
    Check(tracked.GetState(epoch+6s).command.state==CommandState::TimedOut &&
          tracked.Log().size()==log_size && source.sends==1,"timeout remains terminal after fresh recovery");
    tracked.Enable(true,epoch+6s);
    Check(tracked.Request(PilotAction::Standby,0,epoch+6s).state==CommandState::Pending && source.sends==2,
          "new explicit session and command can still request standby");
    tracked.Enable(false,epoch+6100ms);
    tracked.Tick(epoch+10s);
    Check(tracked.GetState(epoch+10s).command.state==CommandState::Disabled && !source.session && source.sends==2,
          "explicit user disable keeps its unknown-outcome cancellation semantics");
  }
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

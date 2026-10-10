#include "application/PilotPresentation.h"
#include <chrono>

namespace opennav::application {
namespace {
std::optional<double> Heading(const vessel::Sample &sample, vessel::Time now) {
  const auto a = vessel::Assess(
      sample, now, {std::chrono::seconds(1), std::chrono::seconds(3)});
  if (sample.validity == vessel::Validity::Measured && a.value &&
      *a.value >= 0 && *a.value < 360 &&
      (a.quality == vessel::Quality::Live ||
       a.quality == vessel::Quality::Aging))
    return a.value;
  return {};
}
} // namespace
PilotPresentation PresentPilot(const adapters::PilotView &pilot,
                               vessel::Time now, bool permit_control,
                               bool replayed) {
  PilotPresentation p;
  p.output_unavailable = pilot.output_unavailable;
  const auto &f = pilot.feedback;
  const auto &caps = pilot.capabilities;
  const bool fresh = !replayed && pilot.fresh &&
                     f.mode != adapters::PilotMode::Unavailable && f.sequence &&
                     !f.source.empty() && f.observed_at <= now &&
                     now - f.observed_at < std::chrono::seconds(3);
  p.available = fresh;
  p.degraded = !replayed && !fresh && f.sequence;
  p.mode = fresh ? f.mode : adapters::PilotMode::Unavailable;
  p.commanded = p.mode == adapters::PilotMode::Auto;
  if (fresh) {
    p.actual_heading_magnetic_deg = Heading(f.heading_magnetic_deg, now);
    p.heading_magnetic_deg = p.commanded
                                 ? Heading(f.locked_heading_magnetic_deg, now)
                                 : p.actual_heading_magnetic_deg;
  }
  p.pending = pilot.command.state == adapters::CommandState::Pending ||
              pilot.command.state == adapters::CommandState::Requested;
  const bool authorized =
      !replayed && !pilot.output_unavailable && (caps.simulated || (permit_control && caps.manual_control));
  p.enabled = !replayed && !pilot.output_unavailable && pilot.enabled;
  p.can_toggle = !replayed && (p.enabled || authorized);
  p.standby = p.enabled && authorized && caps.standby;
  p.auto_mode = p.enabled && authorized && fresh && !p.pending &&
                caps.auto_mode && bool(p.actual_heading_magnetic_deg);
  p.track = p.enabled && authorized && fresh && !p.pending && caps.track;
  p.wind = p.enabled && authorized && fresh && !p.pending && caps.wind;
  p.alter_course = p.enabled && authorized && fresh && !p.pending &&
                   caps.alter_course && p.mode == adapters::PilotMode::Auto &&
                   bool(p.heading_magnetic_deg);
  p.state = p.pending ? "AWAITING ACKNOWLEDGEMENT"
            : fresh   ? adapters::PilotModeName(p.mode)
                      : "STATUS UNAVAILABLE";
  p.connection = fresh ? "Receiving feedback"
                       : p.degraded ? "Feedback lost" : "Waiting for feedback";
  p.note = p.enabled ? "Manual control enabled for this session. Mode and "
                       "heading require measured pilot feedback."
                     : "Control is off. Switch it on in the autopilot drawer; "
                       "keep the physical helm within reach.";
  if (!fresh)
    p.note = "Pilot feedback unavailable. Check the connection and use the "
             "physical helm.";
  if (p.pending)
    p.note = "Command sent. Waiting for new matching pilot feedback; the "
             "outcome is not yet known.";
  switch (pilot.command.state) {
  case adapters::CommandState::TimedOut:
    p.note = "No confirmation received. Check the physical helm. No command "
             "was retried.";
    break;
  case adapters::CommandState::Rejected:
    p.note =
        "Command rejected. Check pilot diagnostics before another request.";
    break;
  case adapters::CommandState::StaleFeedback:
    p.note = "Command not sent: current pilot feedback is required.";
    break;
  case adapters::CommandState::Disabled:
    if (!pilot.enabled)
      p.note = "Control is off. An earlier transmitted request may still take "
               "effect; use physical STANDBY.";
    break;
  default:
    break;
  }
  if (replayed) {
    p.pending = false;
    p.state = "CONTROL UNAVAILABLE";
    p.note = "Historical replay. Live equipment controls are disabled.";
  }
  if (pilot.output_unavailable && !replayed) {
    p.pending = false;
    p.state = fresh ? adapters::PilotModeName(p.mode)
                    : p.degraded ? "FEEDBACK LOST" : "STATUS UNAVAILABLE";
    p.note = fresh ? "Live pilot feedback. Status only; use the physical helm for control."
                  : p.degraded ? "Pilot feedback is stale or lost. Status only; check the OpenCPN connection and use the physical helm."
                               : "Waiting for accepted pilot feedback from OpenCPN. Status only; use the physical helm.";
  }
  return p;
}
} // namespace opennav::application

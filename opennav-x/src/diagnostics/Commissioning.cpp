#include "diagnostics/Commissioning.h"
namespace opennav::diagnostics {
application::CommandResult
Commissioning::StartRecording(const application::Settings &config,
                              bool navigation, vessel::Time now) {
  if (replay_)
    return {false, "Stop REPLAY before recording live or DEMO input"};
  if (recorder_ && recorder_->Status().active)
    return {false, "Recording already active"};
  try {
    recorder_ = std::make_unique<Recorder>(root_, config, navigation, now);
    recorded_simulation_.reset();
    return {
        true,
        navigation
            ? "Recording started, including vessel positions and route names"
            : "Recording started; vessel positions and route names excluded"};
  } catch (const std::exception &e) {
    return {false, e.what()};
  }
}
application::CommandResult Commissioning::StopRecording() {
  if (!recorder_)
    return {false, "No recording session"};
  recorder_->Stop();
  const auto status = recorder_->Status();
  return {status.error.empty(),
          status.error.empty() ? "Recording stopped; complete checkpoints saved"
                               : status.error};
}
void Commissioning::Capture(const vessel::VesselState &s, vessel::Time now) {
  if (!recorder_ || !recorder_->Status().active || replay_)
    return;
  // Mode changes must not blend live and synthetic input in one recording.
  if (recorded_simulation_ && *recorded_simulation_ != s.simulated) {
    recorder_->Stop();
    return;
  }
  recorded_simulation_ = s.simulated;
  recorder_->Capture(s, now);
}
RecorderStatus Commissioning::RecordingStatus() const {
  return recorder_ ? recorder_->Status() : RecorderStatus{};
}
application::CommandResult
Commissioning::OpenReplay(const std::filesystem::path &file, vessel::Time now) {
  if (recorder_ && recorder_->Status().active)
    return {false, "Stop recording before opening REPLAY"};
  try {
    if (replay_preflight_) {
      auto reason = replay_preflight_();
      if (!reason.empty())
        return {false, reason};
    }
    auto session = std::make_unique<ReplaySession>(LoadRecording(file), now);
    replay_ = std::move(session);
    return {true,
            "REPLAY loaded. Historical data only; hardware controls disabled. "
            "Chart and live OpenCPN navigation remain separate."};
  } catch (const std::exception &e) {
    return {false, e.what()};
  }
}
std::optional<ReplayView> Commissioning::ReadReplay(vessel::Time now) const {
  return replay_ ? std::optional<ReplayView>{replay_->Read(now)} : std::nullopt;
}
application::Settings Commissioning::ReplayAssumptions() const {
  return replay_ ? replay_->Assumptions() : application::Settings{};
}
} // namespace opennav::diagnostics

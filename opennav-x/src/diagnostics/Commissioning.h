#pragma once
#include "application/NavigationObjects.h"
#include "diagnostics/Calibration.h"
#include "diagnostics/Recorder.h"
#include "diagnostics/Replay.h"
namespace opennav::diagnostics {
// Application-thread service. Owns only copied data and private session files.
// Integration checks AllowsHardwareControl as well as explicit pilot
// permission.
class Commissioning {
public:
  explicit Commissioning(std::filesystem::path root,
                         std::function<std::string()> replay_preflight = {})
      : root_(std::move(root)), replay_preflight_(std::move(replay_preflight)) {
  }
  application::CommandResult StartRecording(const application::Settings &,
                                            bool navigation, vessel::Time now);
  application::CommandResult StopRecording();
  void Capture(const vessel::VesselState &, vessel::Time now);
  RecorderStatus RecordingStatus() const;
  application::CommandResult OpenReplay(const std::filesystem::path &,
                                        vessel::Time now);
  void StopReplay() { replay_.reset(); }
  void PauseReplay(bool pause, vessel::Time now) {
    if (replay_)
      replay_->Pause(pause, now);
  }
  void RewindReplay(vessel::Time now) {
    if (replay_)
      replay_->Seek(vessel::Duration{}, now);
  }
  std::optional<ReplayView> ReadReplay(vessel::Time now) const;
  application::Settings ReplayAssumptions() const;
  bool Replaying() const { return bool(replay_); }
  bool AllowsHardwareControl() const { return !replay_; }
  const std::filesystem::path &Directory() const { return root_; }

private:
  std::filesystem::path root_;
  std::function<std::string()> replay_preflight_;
  std::unique_ptr<Recorder> recorder_;
  std::unique_ptr<ReplaySession> replay_;
  std::optional<bool> recorded_simulation_;
};
} // namespace opennav::diagnostics

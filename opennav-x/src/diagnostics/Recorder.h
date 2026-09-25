#pragma once
#include "diagnostics/Recording.h"
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
namespace opennav::diagnostics {
struct RecorderStatus {
  bool active = false;
  std::size_t captured = 0, published = 0, segments = 0;
  std::filesystem::path directory;
  std::string error;
};
// GUI-thread capture at <=1 Hz; bounded queue, serialization/IO on one worker.
// A session retains at most three 8 MiB / one-hour segments. Checkpoint every
// ten frames and on Stop. Previously committed checkpoints survive crashes.
struct RecorderLimits {
  std::size_t frames_per_segment = RecordingFrameLimit, retained_segments = 3,
              checkpoint_frames = 10;
};
class Recorder {
public:
  Recorder(const std::filesystem::path &root, application::Settings assumptions,
           bool navigation_included, vessel::Time start,
           RecorderLimits limits = {});
  ~Recorder();
  bool Capture(const vessel::VesselState &, vessel::Time now);
  void Stop();
  RecorderStatus Status() const;
  Recorder(const Recorder &) = delete;
  Recorder &operator=(const Recorder &) = delete;

private:
  void Work();
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<RecordedFrame> pending_;
  std::thread worker_;
  RecorderStatus status_;
  application::Settings assumptions_;
  RecorderLimits limits_;
  bool navigation_ = false, stopping_ = false;
  vessel::Time start_, last_{};
  bool have_last_ = false;
};
Recording LoadRecording(const std::filesystem::path &file);
} // namespace opennav::diagnostics

#include "diagnostics/Replay.h"
#include <algorithm>
#include <stdexcept>
namespace opennav::diagnostics {
ReplaySession::ReplaySession(Recording r, vessel::Time now)
    : recording_(std::move(r)), origin_(now), wall_start_(now) {
  // Same validation for in-memory fixtures as externally loaded sessions.
  (void)EncodeRecording(recording_);
  (void)ReplayFrame(recording_.frames.front(), origin_);
  offset_ = recording_.frames.front().elapsed;
}
vessel::Duration ReplaySession::Elapsed(vessel::Time now) const {
  if (now < wall_start_)
    throw std::invalid_argument("Replay wall clock moved backwards");
  // Continue aging beyond the final observation, withholding stale estimates.
  const auto extra =
      paused_ ? vessel::Duration{}
              : std::chrono::duration_cast<vessel::Duration>(now - wall_start_);
  return std::min(offset_ +
                      std::min(extra, vessel::Duration(std::chrono::hours(24))),
                  vessel::Duration(std::chrono::hours(48)));
}
ReplayView ReplaySession::Read(vessel::Time now) const {
  const auto elapsed = Elapsed(now);
  auto it = std::upper_bound(
      recording_.frames.begin(), recording_.frames.end(), elapsed,
      [](auto t, const RecordedFrame &f) { return t < f.elapsed; });
  ReplayView out;
  out.now = origin_ + elapsed;
  out.elapsed = elapsed - recording_.frames.front().elapsed;
  out.duration =
      recording_.frames.back().elapsed - recording_.frames.front().elapsed;
  out.paused = paused_;
  out.ended = elapsed >= recording_.frames.back().elapsed;
  if (it != recording_.frames.begin())
    out.state = ReplayFrame(*std::prev(it), origin_);
  out.state.simulated = recording_.frames.front().state.simulated;
  out.state.replayed = true;
  return out;
}
void ReplaySession::Pause(bool pause, vessel::Time now) {
  offset_ = Elapsed(now);
  wall_start_ = now;
  paused_ = pause;
}
void ReplaySession::Seek(vessel::Duration elapsed, vessel::Time now) {
  if (elapsed.count() < 0 || elapsed > recording_.frames.back().elapsed -
                                           recording_.frames.front().elapsed)
    throw std::invalid_argument("Replay seek outside recording");
  offset_ = recording_.frames.front().elapsed + elapsed;
  wall_start_ = now;
}
} // namespace opennav::diagnostics

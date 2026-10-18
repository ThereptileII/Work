#pragma once

class Track;

namespace opennav::integration {
// Model-only consumers have no dependency on the application executable. The
// executable registers these callbacks on its main thread before XNav starts
// recording; an unregistered standalone model retains stock OpenCPN behavior.
struct TrackRecordingHooks {
  bool (*sample_allowed)() = nullptr;
  void (*storage_result)(Track *, bool) = nullptr;
};
inline TrackRecordingHooks &NativeTrackHooks() {
  static TrackRecordingHooks hooks;
  return hooks;
}
inline bool NativeTrackSampleAllowed() {
  const auto callback = NativeTrackHooks().sample_allowed;
  return !callback || callback();
}
inline void NativeTrackStorageResult(Track *track, bool ok) {
  if (const auto callback = NativeTrackHooks().storage_result)
    callback(track, ok);
}
} // namespace opennav::integration

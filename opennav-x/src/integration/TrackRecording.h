#pragma once

#include "application/NavigationObjects.h"
#include <functional>

class MyFrame;

namespace opennav::integration {
// The callback must return the live selected navigation, never demo/replay data.
void ConfigureTrackRecording(std::function<vessel::Navigation()> live_position);
// Clear the owner callback before its source is destroyed. Installed model gates
// remain fail-closed in XNav during shutdown, including the final Stop sample.
void ClearTrackRecordingBridge();
bool NativeTrackStartAllowed();
application::TrackRecordingState CopyTrackRecording();
application::TrackCatalog CopyTrackCatalog();
std::optional<application::TrackSummary> CopyTrack(const std::string &id);
application::CommandResult StartTrack(MyFrame &frame);
application::CommandResult StopTrack(MyFrame &frame,
                                     const std::string &expected_active_id);
application::CommandResult RenameTrack(const application::TrackSummary &selected,
                                       const std::string &name);
application::CommandResult SetTrackVisible(
    const application::TrackSummary &selected, bool visible);
application::CommandResult ViewTrack(MyFrame &frame, const std::string &id);
} // namespace opennav::integration

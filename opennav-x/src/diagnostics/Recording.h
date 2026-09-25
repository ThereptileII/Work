#pragma once
#include "application/Settings.h"
#include "vessel/RouteProgress.h"
#include <filesystem>
#include <fstream>
namespace opennav::diagnostics {
// Normalized state only. No transport, OpenCPN objects or hardware commands.
struct RecordedFrame {
  vessel::Duration elapsed{};
  vessel::VesselState state; // observation stamps are session-relative
};
struct Recording {
  bool navigation_included = false;
  application::Settings assumptions;
  std::vector<RecordedFrame> frames;
};
constexpr std::size_t RecordingByteLimit = 8 * 1024 * 1024;
constexpr std::size_t RecordingFrameLimit = 3600;
constexpr std::size_t FrameByteLimit = 128 * 1024;
// Strict bounded UTF-8 text/hex format, canonical decimal units. These
// functions throw on malformed, unsupported, nonfinite or out-of-order data.
std::string EncodeRecording(const Recording &recording);
Recording DecodeRecording(const std::string &bytes);
RecordedFrame CaptureFrame(const vessel::VesselState &state, vessel::Time now,
                           vessel::Time start, bool navigation_included);
// Copies and re-bases once against the replay clock. Does not write to OpenCPN.
vessel::VesselState ReplayFrame(const RecordedFrame &frame,
                                vessel::Time replay_origin);
application::Settings
RecordingAssumptions(const application::Settings &settings);
} // namespace opennav::diagnostics

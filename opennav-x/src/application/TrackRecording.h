#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace opennav::application {
// Owned copies only. OpenCPN remains the recorder and navigation-object store.
struct TrackSummary {
  std::string id, revision, name;
  std::optional<std::chrono::system_clock::time_point> started_at, ended_at;
  std::optional<double> distance_nm;
  std::size_t point_count = 0;
  bool recording = false, visible = false, editable = false, in_layer = false;
};
struct TrackCatalog {
  std::vector<TrackSummary> tracks;
  bool available = false, truncated = false;
  std::string reason;
};
struct TrackRecordingState {
  bool available = false, recording = false, pending_start = false;
  bool position_fresh = false, can_start = false, can_stop = false;
  bool layer_visible = true;
  std::optional<TrackSummary> active;
  std::string reason;
};
} // namespace opennav::application

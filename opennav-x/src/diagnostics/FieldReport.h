#pragma once
#include "adapters/Autopilot.h"
#include "adapters/Radar.h"
#include "diagnostics/Recording.h"
#include "smartnav/Advisories.h"
#include <deque>
#include <map>
namespace opennav::diagnostics {
struct FieldSnapshot {
  vessel::VesselState vessel;
  application::Settings settings;
  std::vector<vessel::SourceHealth> sources;
  smartnav::EnergyPrediction energy;
  smartnav::NavigationAdvice advice;
  adapters::PilotView pilot;
  adapters::RadarState radar;
  bool recording = false, recording_error = false;
  vessel::Time now{};
};
struct FieldEnvironment {
  // Integration supplies build/plugin metadata only, never a profile path.
  std::vector<std::string> build, plugins;
  bool startup_recovery_required = false;
  unsigned startup_failures_observed = 0;
  bool previous_launch_unfinished = false;
};
struct BundleEntry {
  std::string name, bytes;
};
// Bounded transition journal. No raw source strings, positions or object names.
// Capture happens on the application thread; no disk IO in Observe.
class FieldJournal {
public:
  void Observe(const FieldSnapshot &snapshot, vessel::Time wall_now);
  std::string Export(vessel::Time wall_now) const;
  std::size_t Size() const { return entries_.size(); }

private:
  struct Entry {
    vessel::Time at;
    std::string text;
  };
  std::deque<Entry> entries_;
  std::map<std::string, std::string> previous_;
};
// Whitelist report, not an archive of user directories. Optional recording is
// supplied only after explicit selection/consent, and is decoded/re-encoded.
std::vector<BundleEntry>
BuildFieldReport(const FieldSnapshot &, const FieldEnvironment &,
                 const FieldJournal &, vessel::Time wall_now,
                 const std::optional<std::string> &recording = {});
} // namespace opennav::diagnostics

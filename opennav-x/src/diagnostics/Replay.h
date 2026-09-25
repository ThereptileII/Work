#pragma once
#include "diagnostics/Recording.h"
namespace opennav::diagnostics {
struct ReplayView {
  vessel::VesselState state;
  vessel::Time now{}; // Explicit replay clock, never the selected live clock.
  vessel::Duration elapsed{}, duration{};
  bool paused = false, ended = false;
};
class ReplaySession {
public:
  ReplaySession(Recording recording, vessel::Time wall_start);
  ReplayView Read(vessel::Time wall_now) const;
  void Pause(bool pause, vessel::Time wall_now);
  void Seek(vessel::Duration elapsed, vessel::Time wall_now);
  const application::Settings &Assumptions() const {
    return recording_.assumptions;
  }

private:
  vessel::Duration Elapsed(vessel::Time now) const;
  Recording recording_;
  vessel::Time origin_, wall_start_;
  vessel::Duration offset_{};
  bool paused_ = false;
};
} // namespace opennav::diagnostics

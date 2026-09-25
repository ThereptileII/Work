#pragma once
#include "integration/StartupRecovery.h"
#include "vessel/VesselState.h"
#include <wx/string.h>
namespace opennav::integration {
class RecoveryStore {
public:
  explicit RecoveryStore(const wxString &directory);
  bool RequiresSafe() const { return failed_ || RecoveryRequired(record_); }
  bool BeginXNav();
  void ObserveHealthy(bool deferred_ready, vessel::Time now);
  void CleanClose();
  bool Retry(); // Explicit human action only; never an automatic restart loop.
  const std::string &Reason() const { return reason_; }
  // Preserve startup evidence after a healthy launch clears the durable count.
  unsigned FailuresObservedAtLaunch() const { return launch_failures_; }
  bool PreviousLaunchUnfinished() const { return previous_unfinished_; }

private:
  bool Save();
  wxString path_;
  RecoveryRecord record_;
  bool failed_ = false, attempt_ = false;
  unsigned launch_failures_ = 0;
  bool previous_unfinished_ = false;
  std::optional<vessel::Time> healthy_since_;
  std::string reason_;
};
} // namespace opennav::integration

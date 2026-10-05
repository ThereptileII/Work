#pragma once

#include <optional>
#include <string>

namespace opennav::application {

// Identity, never a URL, local package path or command line. Only the trusted
// updater bridge may supply this after authenticated release policy AND exact
// installed-host compatibility/upgrade checks. Shape validation below is not
// signature, freshness, compatibility or package verification.
struct StartupUpdateCandidate {
  std::string policy_sha256;
  std::string version;
  std::string commit;
};

enum class StartupUpdateCheck { NoUpdate, UpdateAvailable, CheckFailed, Offline };
enum class StartupUpdateState { Checking, Prompt, Continue, HandoffRequested };

// One instance per process startup, used only on the application thread. It
// owns no transport, file, process or hardware capability. The bridge imposes
// a bounded check deadline and supplies CheckFailed on timeout.
class StartupUpdate final {
 public:
  StartupUpdateState State() const { return state_; }
  const std::optional<StartupUpdateCandidate>& Candidate() const {
    return candidate_;
  }

  // First completion wins. A malformed/missing identity continues normally;
  // late responses cannot reopen a dismissed prompt or change a chosen release.
  void CompleteCheck(StartupUpdateCheck result,
                     std::optional<StartupUpdateCandidate> candidate = {});
  // Also used for Escape/window close. Suppresses any further startup prompt
  // in this process, including an outstanding check completing afterwards.
  void Later();
  // Consumes consent exactly once. The bridge must revalidate the bound policy
  // and package before execution; this value is not installation authority.
  std::optional<StartupUpdateCandidate> UpdateNow();
  // Failure to start the trusted handoff never prevents ordinary startup.
  void HandoffFailed();

 private:
  StartupUpdateState state_ = StartupUpdateState::Checking;
  std::optional<StartupUpdateCandidate> candidate_;
};
}  // namespace opennav::application

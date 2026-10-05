#include "application/StartupUpdate.h"

#include <iostream>
#include <stdexcept>

using namespace opennav::application;
namespace {
void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
StartupUpdateCandidate Candidate() {
  return {std::string(64, 'a'), "0.6.0-beta.1", std::string(40, 'b')};
}
void CannotReopen(StartupUpdate& model) {
  model.CompleteCheck(StartupUpdateCheck::UpdateAvailable, Candidate());
  Require(model.State() == StartupUpdateState::Continue,
          "Late check must not reopen startup prompt");
  Require(!model.UpdateNow(), "No consent after startup continued");
  Require(!model.Candidate(), "Do not retain stale candidate");
}
}  // namespace

int main() {
  try {
    for (const auto status : {StartupUpdateCheck::NoUpdate,
                             StartupUpdateCheck::CheckFailed,
                             StartupUpdateCheck::Offline,
                             static_cast<StartupUpdateCheck>(99)}) {
      StartupUpdate model;
      Require(!model.UpdateNow(), "Cannot install during check");
      model.CompleteCheck(status, Candidate());
      CannotReopen(model);
    }
    StartupUpdate missing;
    missing.CompleteCheck(StartupUpdateCheck::UpdateAvailable);
    CannotReopen(missing);

    for (int bad = 0; bad != 8; ++bad) {
      auto candidate = Candidate();
      switch (bad) {
        case 0: candidate.policy_sha256.clear(); break;
        case 1: candidate.policy_sha256[0] = 'A'; break;
        case 2: candidate.commit.pop_back(); break;
        case 3: candidate.commit[0] = 'g'; break;
        case 4: candidate.version.clear(); break;
        case 5: candidate.version = std::string(129, '1'); break;
        case 6: candidate.version = "1.0.0\nUPDATE NOW"; break;
        case 7: candidate.version = "https://example.com/setup.exe"; break;
      }
      StartupUpdate model;
      model.CompleteCheck(StartupUpdateCheck::UpdateAvailable, candidate);
      CannotReopen(model);
    }
    StartupUpdate cancelled_check;
    cancelled_check.Later();
    CannotReopen(cancelled_check);

    StartupUpdate later;
    later.CompleteCheck(StartupUpdateCheck::UpdateAvailable, Candidate());
    Require(later.State() == StartupUpdateState::Prompt, "Verified identity prompts");
    later.Later();
    later.Later();
    CannotReopen(later);

    StartupUpdate accept;
    accept.CompleteCheck(StartupUpdateCheck::UpdateAvailable, Candidate());
    auto other = Candidate();
    other.policy_sha256 = std::string(64, 'c');
    accept.CompleteCheck(StartupUpdateCheck::UpdateAvailable, other);
    accept.HandoffFailed();
    Require(accept.State() == StartupUpdateState::Prompt,
            "Spurious handoff failure must not mutate prompt");
    const auto selected = accept.UpdateNow();
    Require(selected && selected->policy_sha256 == Candidate().policy_sha256 &&
                selected->version == Candidate().version &&
                selected->commit == Candidate().commit,
            "Consent must retain exact presented candidate identity");
    Require(!accept.UpdateNow(), "Repeated activation must not launch twice");
    accept.Later();
    accept.CompleteCheck(StartupUpdateCheck::NoUpdate);
    Require(accept.State() == StartupUpdateState::HandoffRequested,
            "Late UI/check events cannot undo a requested handoff");
    accept.HandoffFailed();
    CannotReopen(accept);

    StartupUpdate next_session;
    next_session.CompleteCheck(StartupUpdateCheck::UpdateAvailable, Candidate());
    Require(next_session.State() == StartupUpdateState::Prompt,
            "Later suppression must not persist into a new process session");
    std::cout << "Startup update state contract passed.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

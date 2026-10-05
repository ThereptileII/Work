#include "application/StartupUpdate.h"

#include <algorithm>
#include <utility>

namespace opennav::application {
namespace {
bool HexIdentity(const std::string& text, std::size_t length) {
  return text.size() == length &&
         std::all_of(text.begin(), text.end(), [](char ch) {
           return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f');
         });
}
bool DisplayVersion(const std::string& text) {
  // SemVer itself is already checked by the trusted release-policy evaluator.
  // This is a bounded display/identity alphabet, not a second SemVer parser.
  return !text.empty() && text.size() <= 128 &&
         std::all_of(text.begin(), text.end(), [](char ch) {
           return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'z') ||
                  (ch >= 'A' && ch <= 'Z') || ch == '.' || ch == '-' || ch == '+';
         });
}
bool ValidIdentity(const StartupUpdateCandidate& candidate) {
  return HexIdentity(candidate.policy_sha256, 64) &&
         HexIdentity(candidate.commit, 40) && DisplayVersion(candidate.version);
}
}  // namespace

void StartupUpdate::CompleteCheck(
    StartupUpdateCheck result, std::optional<StartupUpdateCandidate> candidate) {
  if (state_ != StartupUpdateState::Checking) return;
  if (result == StartupUpdateCheck::UpdateAvailable && candidate &&
      ValidIdentity(*candidate)) {
    candidate_ = std::move(candidate);
    state_ = StartupUpdateState::Prompt;
    return;
  }
  candidate_.reset();
  state_ = StartupUpdateState::Continue;
}

void StartupUpdate::Later() {
  if (state_ != StartupUpdateState::Checking &&
      state_ != StartupUpdateState::Prompt) return;
  candidate_.reset();
  state_ = StartupUpdateState::Continue;
}

std::optional<StartupUpdateCandidate> StartupUpdate::UpdateNow() {
  if (state_ != StartupUpdateState::Prompt || !candidate_) return {};
  auto selected = std::move(candidate_);
  candidate_.reset();
  state_ = StartupUpdateState::HandoffRequested;
  return selected;
}

void StartupUpdate::HandoffFailed() {
  if (state_ == StartupUpdateState::HandoffRequested)
    state_ = StartupUpdateState::Continue;
}
}  // namespace opennav::application

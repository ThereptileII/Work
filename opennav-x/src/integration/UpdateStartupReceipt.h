#pragma once

#include <chrono>
#include <optional>
#include <string>

namespace opennav::integration {
struct UpdateStartupReceiptEnvelope {
  std::string pipe;
  std::string message;
};

// Pure policy; the platform entry point supplies the compiled commit, never an
// environment-provided commit. The receiver separately authenticates the
// process, executable hash, generation and session challenge.
class UpdateStartupReceiptState final {
 public:
  using Time = std::chrono::steady_clock::time_point;
  void Capture(const std::string& pipe, const std::string& generation,
               const std::string& challenge, const std::string& compiled_commit);
  void ObserveReady(bool xnav_ui_ready, Time now);
  void RecoveryCheckpointReached();
  std::optional<UpdateStartupReceiptEnvelope> TakeReady();

 private:
  bool captured_ = false;
  bool checkpoint_ = false;
  std::optional<UpdateStartupReceiptEnvelope> envelope_;
  std::optional<Time> ready_since_, last_observed_;
};

// Application thread only. Capture promptly at command-line parsing, before
// plugins/child launches; all three environment fields are removed even when
// malformed or the eventual mode is Legacy/Safe. Non-Windows entry points no-op.
void CaptureUpdateStartupReceipt() noexcept;
// Feed only from the actual XNav event-loop health path. A false observation
// resets this receipt's independent continuous 30-second readiness period.
void ObserveUpdateStartupHealth(bool xnav_ui_ready,
                               UpdateStartupReceiptState::Time now) noexcept;
// Call only after RecoveryStore's 30-second healthy journal Save succeeds,
// never on CleanClose, Retry or file absence. Sending is best effort once;
// only the transaction receiver decides whether startup was accepted.
void NotifyUpdateStartupHealthy() noexcept;
}  // namespace opennav::integration

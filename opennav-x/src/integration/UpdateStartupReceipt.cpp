#include "integration/UpdateStartupReceipt.h"

#include <algorithm>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <process.h>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <deque>
#include "OpenNavBuild.h"
#endif

namespace opennav::integration {
namespace {
bool Hex(const std::string& value, std::size_t count) {
  return value.size() == count &&
         std::all_of(value.begin(), value.end(), [](char ch) {
           return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f');
         });
}
}  // namespace

void UpdateStartupReceiptState::Capture(
    const std::string& pipe, const std::string& generation,
    const std::string& challenge, const std::string& compiled_commit) {
  if (captured_) return;
  captured_ = true;
  constexpr char prefix[] = "Skager.Update.";
  constexpr auto prefix_size = sizeof(prefix) - 1;
  if (pipe.size() != prefix_size + 32 || pipe.compare(0, prefix_size, prefix) ||
      !Hex(pipe.substr(prefix_size), 32) || !Hex(generation, 32) ||
      !Hex(challenge, 64) || !Hex(compiled_commit, 40)) return;
  envelope_ = UpdateStartupReceiptEnvelope{
      pipe, "SKAGER-UPDATE-READY/1 " + generation + " " + compiled_commit +
                " " + challenge + "\n"};
}

std::optional<UpdateStartupReceiptEnvelope>
UpdateStartupReceiptState::NavigationWarning(bool waiting, bool accepted) {
  ready_since_.reset();
  last_observed_.reset();
  checkpoint_ = false;
  if (!envelope_) return {};
  if ((waiting && (accepted || warning_ != Warning::None)) ||
      (!waiting && warning_ != Warning::Waiting)) {
    warning_ = Warning::Invalid;
    envelope_.reset();
    return {};
  }
  warning_ = waiting ? Warning::Waiting :
      accepted ? Warning::Accepted : Warning::Cancelled;
  auto phase = *envelope_;
  const auto identity = phase.message.substr(std::string("SKAGER-UPDATE-READY/1").size());
  phase.message = std::string(waiting ? "SKAGER-UPDATE-WAIT/1" :
      accepted ? "SKAGER-UPDATE-CONTINUE/1" : "SKAGER-UPDATE-CANCEL/1") + identity;
  phase.terminal = !waiting && !accepted;
  if (phase.terminal) envelope_.reset();
  return phase;
}

void UpdateStartupReceiptState::ObserveReady(bool ready, Time now) {
  if (!ready || warning_ == Warning::Waiting || warning_ == Warning::Cancelled ||
      warning_ == Warning::Invalid) {
    ready_since_.reset();
    last_observed_.reset();
    return;
  }
  if (!ready_since_ || (last_observed_ && now < *last_observed_))
    ready_since_ = now;
  last_observed_ = now;
}

void UpdateStartupReceiptState::RecoveryCheckpointReached() { checkpoint_ = true; }

std::optional<UpdateStartupReceiptEnvelope> UpdateStartupReceiptState::TakeReady() {
  if (warning_ == Warning::Waiting || warning_ == Warning::Cancelled ||
      warning_ == Warning::Invalid || !checkpoint_ || !ready_since_ || !last_observed_ ||
      *last_observed_ - *ready_since_ < std::chrono::seconds(30)) return {};
  auto result = std::move(envelope_);
  envelope_.reset();
  return result;
}

#ifdef _WIN32
namespace {
UpdateStartupReceiptState& Receipt() {
  static UpdateStartupReceiptState state;
  return state;
}
struct Handle {
  HANDLE value;
  ~Handle() {
    if (value && value != INVALID_HANDLE_VALUE) CloseHandle(value);
  }
};

// One bounded FIFO and one worker for the entire supervised startup. This
// preserves WAIT -> CONTINUE -> READY ordering even when the user accepts at
// once. Only owned bytes cross threads; no wx object or mutable UI is retained.
struct StartupTransport {
  std::mutex mutex;
  std::condition_variable changed;
  std::deque<UpdateStartupReceiptEnvelope> queued;
  unsigned submitted = 0;
  bool started = false;
  bool closed = false;
};

bool WriteEnvelope(HANDLE pipe, const std::string& message) {
  const Handle event{CreateEventW(nullptr, TRUE, FALSE, nullptr)};
  if (!event.value) return false;
  OVERLAPPED operation{};
  operation.hEvent = event.value;
  DWORD written = 0;
  BOOL sent = WriteFile(pipe, message.data(), static_cast<DWORD>(message.size()),
                       &written, &operation);
  if (!sent && GetLastError() == ERROR_IO_PENDING) {
    if (WaitForSingleObject(event.value, 250) != WAIT_OBJECT_0) {
      CancelIoEx(pipe, &operation);
      // Retain the OVERLAPPED and bytes until cancellation completes. This is
      // the isolated worker, never the application/UI thread.
      WaitForSingleObject(event.value, INFINITE);
      GetOverlappedResult(pipe, &operation, &written, FALSE);
      return false;
    }
    sent = GetOverlappedResult(pipe, &operation, &written, FALSE);
  }
  return sent && written == message.size();
}

unsigned __stdcall SendReceipt(void* argument) noexcept {
  const std::unique_ptr<std::shared_ptr<StartupTransport>> owned(
      static_cast<std::shared_ptr<StartupTransport>*>(argument));
  const auto state = *owned;
  try {
    std::unique_lock<std::mutex> lock(state->mutex);
    const auto name = state->queued.front().pipe;
    lock.unlock();
    const auto pipe_name = std::wstring(L"\\\\.\\pipe\\") +
        std::wstring(name.begin(), name.end());
    const Handle pipe{CreateFileW(pipe_name.c_str(), GENERIC_WRITE, 0, nullptr,
                                 OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr)};
    if (pipe.value != INVALID_HANDLE_VALUE) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::minutes(8);
      for (;;) {
        lock.lock();
        if (!state->changed.wait_until(lock, deadline, [&] {
              return !state->queued.empty() || state->closed;
            }) || state->closed || std::chrono::steady_clock::now() >= deadline) break;
        auto envelope = std::move(state->queued.front());
        state->queued.pop_front();
        lock.unlock();
        if (envelope.pipe != name || !WriteEnvelope(pipe.value, envelope.message) ||
            envelope.terminal) break;
      }
      if (lock.owns_lock()) lock.unlock();
    }
    // Final close sends EOF. No FlushFileBuffers, retry or acceptance inference.
  } catch (...) {}
  { std::lock_guard<std::mutex> lock(state->mutex);
    state->closed = true;
    state->queued.clear(); }
  return 0;
}

void SendEnvelope(std::optional<UpdateStartupReceiptEnvelope> envelope) {
  if (!envelope) return;
  static const auto state = std::make_shared<StartupTransport>();
  std::lock_guard<std::mutex> lock(state->mutex);
  if (state->closed || state->submitted >= 3) return;
  state->queued.push_back(std::move(*envelope));
  ++state->submitted;
  if (!state->started) {
    auto owned = std::make_unique<std::shared_ptr<StartupTransport>>(state);
    const auto thread = _beginthreadex(nullptr, 0, &SendReceipt, owned.get(), 0, nullptr);
    if (!thread) { state->closed = true; state->queued.clear(); return; }
    state->started = true;
    owned.release();
    CloseHandle(reinterpret_cast<HANDLE>(thread));
  }
  state->changed.notify_one();
}

void SendReadyReceipt() { SendEnvelope(Receipt().TakeReady()); }

struct EnvironmentField {
  wchar_t value[128]{};
  bool valid = false;
};
EnvironmentField ReadEnvironment(const wchar_t* name) {
  EnvironmentField result;
  const auto size = GetEnvironmentVariableW(name, result.value, 128);
  result.valid = size > 0 && size < 128;
  return result;
}
bool ClearEnvironment(const wchar_t* name) {
  // Clear CRT getenv consumers as well as the OS environment inherited by a
  // child process. No environment-backed pointer is retained.
  const auto crt = _wputenv_s(name, L"");
  const auto os = SetEnvironmentVariableW(name, nullptr);
  return crt == 0 && (os || GetLastError() == ERROR_ENVVAR_NOT_FOUND);
}
std::string Ascii(const EnvironmentField& field) {
  if (!field.valid) return {};
  std::string result;
  for (auto ch : field.value) {
    if (!ch) break;
    if (ch > 127) return {};
    result.push_back(static_cast<char>(ch));
  }
  return result;
}
}  // namespace
#endif

void CaptureUpdateStartupReceipt() noexcept {
#ifdef _WIN32
  const auto pipe = ReadEnvironment(L"SKAGER_UPDATE_PIPE");
  const auto generation = ReadEnvironment(L"SKAGER_UPDATE_GENERATION");
  const auto challenge = ReadEnvironment(L"SKAGER_UPDATE_CHALLENGE");
  // All removals run even if any field is invalid or a preceding removal fails.
  const bool pipe_cleared = ClearEnvironment(L"SKAGER_UPDATE_PIPE");
  const bool generation_cleared = ClearEnvironment(L"SKAGER_UPDATE_GENERATION");
  const bool challenge_cleared = ClearEnvironment(L"SKAGER_UPDATE_CHALLENGE");
  try {
    if (pipe_cleared && generation_cleared && challenge_cleared)
      Receipt().Capture(Ascii(pipe), Ascii(generation), Ascii(challenge), OPENNAV_BUILD_COMMIT);
    else
      Receipt().Capture({}, {}, {}, {});
  } catch (...) {}
#endif
}

void NotifyUpdateNavigationWarning(bool waiting, bool accepted) noexcept {
#ifdef _WIN32
  try { SendEnvelope(Receipt().NavigationWarning(waiting, accepted)); } catch (...) {}
#else
  (void)waiting;
  (void)accepted;
#endif
}

void ObserveUpdateStartupHealth(bool ready, UpdateStartupReceiptState::Time now) noexcept {
#ifdef _WIN32
  try {
    Receipt().ObserveReady(ready, now);
    SendReadyReceipt();
  } catch (...) {}
#else
  (void)ready;
  (void)now;
#endif
}

void NotifyUpdateStartupHealthy() noexcept {
#ifdef _WIN32
  try {
    Receipt().RecoveryCheckpointReached();
    SendReadyReceipt();
  } catch (...) {}
#endif
}
}  // namespace opennav::integration

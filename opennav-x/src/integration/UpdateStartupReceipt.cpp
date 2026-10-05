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

void UpdateStartupReceiptState::ObserveReady(bool ready, Time now) {
  if (!ready) {
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
  if (!checkpoint_ || !ready_since_ || !last_observed_ ||
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

unsigned __stdcall SendReceipt(void* argument) noexcept {
  // Sole ownership belongs to this worker: no wx objects, UI pointers or
  // mutable application state survive across the asynchronous boundary.
  const std::unique_ptr<UpdateStartupReceiptEnvelope> envelope(
      static_cast<UpdateStartupReceiptEnvelope*>(argument));
  try {
    const auto pipe_name = std::wstring(L"\\\\.\\pipe\\") +
        std::wstring(envelope->pipe.begin(), envelope->pipe.end());
    const Handle pipe{CreateFileW(pipe_name.c_str(), GENERIC_WRITE, 0, nullptr,
                                 OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr)};
    if (pipe.value == INVALID_HANDLE_VALUE) return 0;
    const Handle event{CreateEventW(nullptr, TRUE, FALSE, nullptr)};
    if (!event.value) return 0;
    OVERLAPPED operation{};
    operation.hEvent = event.value;
    DWORD written = 0;
    const BOOL sent = WriteFile(pipe.value, envelope->message.data(),
        static_cast<DWORD>(envelope->message.size()), &written, &operation);
    if (!sent && GetLastError() == ERROR_IO_PENDING) {
      if (WaitForSingleObject(event.value, 250) != WAIT_OBJECT_0) {
        CancelIoEx(pipe.value, &operation);
        // Cancellation completion must retain the OVERLAPPED and its buffer.
        // It is waited out only on this independent worker, never the UI.
        WaitForSingleObject(event.value, INFINITE);
      }
      GetOverlappedResult(pipe.value, &operation, &written, FALSE);
    }
    // Closing sends EOF. Do not FlushFileBuffers (which waits for the peer),
    // retry, infer acceptance, alter recovery state or launch another process.
  } catch (...) {}
  return 0;
}

void SendReadyReceipt() {
  auto envelope = Receipt().TakeReady();
  if (!envelope) return;
  auto owned = std::make_unique<UpdateStartupReceiptEnvelope>(std::move(*envelope));
  const auto thread = _beginthreadex(nullptr, 0, &SendReceipt, owned.get(), 0, nullptr);
  if (thread) {
    owned.release();
    CloseHandle(reinterpret_cast<HANDLE>(thread));
  }
}

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

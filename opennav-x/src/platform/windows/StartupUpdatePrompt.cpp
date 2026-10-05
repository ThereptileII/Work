// Standalone installed consent UI. The launcher verifies release metadata and
// compatibility, sends only its candidate identity and interprets exit code 10.
#include "application/StartupUpdate.h"
#include "ui/StartupUpdateDialog.h"

#include <wx/app.h>
#include <wx/log.h>
#include <wx/msw/wrapwin.h>

#include <algorithm>
#include <optional>
#include <string>

namespace {
std::optional<opennav::application::StartupUpdateCandidate> ReadCandidate() {
  const auto input = GetStdHandle(STD_INPUT_HANDLE);
  if (!input || input == INVALID_HANDLE_VALUE || GetFileType(input) != FILE_TYPE_PIPE)
    return {};
  std::string body;
  const auto deadline = GetTickCount64() + 3000;
  for (;;) {
    if (GetTickCount64() >= deadline) return {};
    DWORD available = 0;
    if (!PeekNamedPipe(input, nullptr, 0, nullptr, &available, nullptr)) {
      if (GetLastError() == ERROR_BROKEN_PIPE) break;
      return {};
    }
    if (!available) { Sleep(5); continue; }
    char bytes[257];
    DWORD read = 0;
    const auto wanted = (std::min)(available, static_cast<DWORD>(sizeof(bytes)));
    if (!ReadFile(input, bytes, wanted, &read, nullptr) || !read) return {};
    if (body.size() + read > 256) return {};
    for (DWORD i = 0; i < read; ++i)
      if (bytes[i] != '\n' && (bytes[i] < 32 || bytes[i] > 126)) return {};
    body.append(bytes, read);
  }
  const auto first = body.find('\n');
  if (first == std::string::npos) return {};
  const auto second = body.find('\n', first + 1);
  if (second == std::string::npos) return {};
  const auto third = body.find('\n', second + 1);
  if (third == std::string::npos || third != body.size() - 1) return {};
  return opennav::application::StartupUpdateCandidate{
      body.substr(0, first), body.substr(first + 1, second - first - 1),
      body.substr(second + 1, third - second - 1)};
}

class StartupUpdatePrompt final : public wxApp {
 public:
  bool OnInit() override {
    // A malformed request must never open a wx diagnostic message box or
    // require interaction. No command-line candidate or alternate source.
    wxLog::SetActiveTarget(new wxLogStderr());
    SetExitOnFrameDelete(false);
    if (argc == 1) {
      const auto candidate = ReadCandidate();
      if (candidate)
        update_.CompleteCheck(opennav::application::StartupUpdateCheck::UpdateAvailable,
                              candidate);
    }
    CallAfter([this] {
      if (update_.State() == opennav::application::StartupUpdateState::Prompt)
        result_ = opennav::ui::ShowStartupUpdateDialog(nullptr, update_) ? 10 : 0;
      ExitMainLoop();
    });
    return true;
  }
  int OnRun() override { wxApp::OnRun(); return result_; }
 private:
  int result_ = 2;
  opennav::application::StartupUpdate update_;
};
}  // namespace
wxIMPLEMENT_APP(StartupUpdatePrompt);

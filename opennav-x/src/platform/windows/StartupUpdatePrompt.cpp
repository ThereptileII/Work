// Standalone installed consent and download UI. The launcher verifies release
// metadata, sends only candidate identity for consent, and owns all preparation.
#include "application/StartupUpdate.h"
#include "ui/Controls.h"
#include "ui/StartupUpdateDialog.h"

#include <wx/app.h>
#include <wx/log.h>
#include <wx/msw/wrapwin.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/thread.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace {
wxDEFINE_EVENT(DownloadPipeFinished, wxThreadEvent);

// A worker owns the duplicate pipe and only addresses the application event
// handler, never a dialog. Disabling its target precedes bounded shutdown, so
// even a delayed OS operation cannot queue to a destroyed handler.
class DownloadPipeReader {
 public:
  bool Start(wxEvtHandler* application, HANDLE input) {
    state_ = std::make_shared<State>();
    state_->target = application;
    state_->stop = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    if (!state_->stop || !DuplicateHandle(GetCurrentProcess(), input,
        GetCurrentProcess(), &state_->input, 0, FALSE, DUPLICATE_SAME_ACCESS)) return false;
    try {
      worker_ = std::thread([state = state_] {
        const auto deadline = GetTickCount64() + 30ULL * 60 * 1000;
        int result = 2;
        while (WaitForSingleObject(state->stop, 20) == WAIT_TIMEOUT) {
          if (GetTickCount64() >= deadline) break;
          DWORD available = 0;
          // No blocking read is necessary: any content violates this protocol.
          // This worker is the sole pipe consumer; an empty open pipe waits.
          if (!PeekNamedPipe(state->input, nullptr, 0, nullptr, &available, nullptr)) {
            if (GetLastError() == ERROR_BROKEN_PIPE) result = 0;
            break;
          }
          if (available) break;
        }
        std::lock_guard<std::mutex> guard(state->target_mutex);
        if (state->target) {
          auto* event = new wxThreadEvent(DownloadPipeFinished);
          event->SetInt(result);
          wxQueueEvent(state->target, event);
        }
      });
    } catch (...) { return false; }
    return true;
  }
  ~DownloadPipeReader() { Stop(); }
  void Stop() {
    if (!state_) return;
    {
      std::lock_guard<std::mutex> guard(state_->target_mutex);
      state_->target = nullptr;
    }
    if (state_->stop) SetEvent(state_->stop);
    if (worker_.joinable()) {
      const auto thread = static_cast<HANDLE>(worker_.native_handle());
      CancelSynchronousIo(thread);
      if (WaitForSingleObject(thread, 250) == WAIT_OBJECT_0) worker_.join();
      else worker_.detach();  // Worker retains only its own handles/state.
    }
    state_.reset();
  }
 private:
  struct State {
    HANDLE input = nullptr;
    HANDLE stop = nullptr;
    std::mutex target_mutex;
    wxEvtHandler* target = nullptr;
    ~State() {
      if (input) CloseHandle(input);
      if (stop) CloseHandle(stop);
    }
  };
  std::shared_ptr<State> state_;
  std::thread worker_;
};

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
    const bool progress = argc == 2 && wxString(argv[1]) == "--download-progress";
    Bind(DownloadPipeFinished, [this](wxThreadEvent& event) {
      if (progress_finish_) progress_finish_(event.GetInt());
    });
    if (argc == 1) {
      const auto candidate = ReadCandidate();
      if (candidate)
        update_.CompleteCheck(opennav::application::StartupUpdateCheck::UpdateAvailable,
                              candidate);
    }
    CallAfter([this, progress] {
      if (progress) result_ = ShowDownloadProgress();
      else if (update_.State() == opennav::application::StartupUpdateState::Prompt)
        result_ = opennav::ui::ShowStartupUpdateDialog(nullptr, update_) ? 10 : 0;
      ExitMainLoop();
    });
    return true;
  }
  int OnRun() override { wxApp::OnRun(); return result_; }
 private:
  int ShowDownloadProgress() {
    const auto input = GetStdHandle(STD_INPUT_HANDLE);
    if (!input || input == INVALID_HANDLE_VALUE || GetFileType(input) != FILE_TYPE_PIPE)
      return 2;
    using namespace opennav::ui;
    wxDialog dialog(nullptr, wxID_ANY, "SKAGER update download", wxDefaultPosition,
                    wxDefaultSize, wxBORDER_NONE | wxTAB_TRAVERSAL);
    dialog.SetName("SKAGER update download");
    const auto palette = Theme(LightMode::Day);
    dialog.SetBackgroundColour(Colour(palette.elevated));
    auto* body = new wxBoxSizer(wxVERTICAL);
    auto* heading = new wxStaticText(&dialog, wxID_ANY, "Downloading and checking update");
    heading->SetFont(UiFont(dialog, 24, true));
    heading->SetForegroundColour(Colour(palette.primary));
    heading->Wrap(dialog.FromDIP(430));
    body->Add(heading, 0, wxEXPAND | wxBOTTOM, dialog.FromDIP(16));
    auto* detail = new wxStaticText(&dialog, wxID_ANY,
        "This may take a few minutes. You can cancel and continue with your current version.");
    detail->SetFont(UiFont(dialog, 12));
    detail->SetForegroundColour(Colour(palette.secondary));
    detail->Wrap(dialog.FromDIP(430));
    body->Add(detail, 0, wxEXPAND | wxBOTTOM, dialog.FromDIP(20));
    auto* actions = new wxBoxSizer(wxHORIZONTAL);
    actions->AddStretchSpacer();
    auto* cancel = new XNavButton(&dialog, wxID_ANY, "CANCEL", "Cancel update download");
    cancel->SetLightMode(LightMode::Day);
    cancel->SetRole(ButtonRole::Quiet);
    cancel->SetMinSize(dialog.FromDIP(wxSize(136, 48)));
    actions->Add(cancel);
    body->Add(actions, 0, wxEXPAND);
    auto* outer = new wxBoxSizer(wxVERTICAL);
    outer->Add(body, 1, wxALL | wxEXPAND, dialog.FromDIP(20));
    dialog.SetSizerAndFit(outer);
    dialog.SetMinSize(dialog.GetSize());
    dialog.Centre();
    std::optional<int> first_choice;
    progress_finish_ = [&dialog, &first_choice](int choice) {
      if (first_choice || !dialog.IsModal()) return;
      first_choice = choice;
      dialog.EndModal(choice);
    };
    cancel->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { progress_finish_(1); });
    dialog.Bind(wxEVT_CHAR_HOOK, [this](wxKeyEvent& event) {
      if (event.GetKeyCode() == WXK_ESCAPE) progress_finish_(1);
      else event.Skip();
    });
    dialog.Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent&) { progress_finish_(1); });
    DownloadPipeReader reader;
    if (reader.Start(this, input)) {
      cancel->SetFocus();
      dialog.ShowModal();
    }
    reader.Stop();
    progress_finish_ = {};
    return first_choice.value_or(2);
  }
  int result_ = 2;
  std::function<void(int)> progress_finish_;
  opennav::application::StartupUpdate update_;
};
}  // namespace
wxIMPLEMENT_APP(StartupUpdatePrompt);

// Patched SKAGER/stock modals and exact call-site boundary only. This process never
// starts OpenCPN, loads plugins, opens connections or qualifies installed health.
#include <wx/wx.h>
#include <wx/fileconf.h>
#include <wx/button.h>
#include <wx/textctrl.h>
#include <wx/timer.h>
#include <windows.h>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include "dialog_alert.h"
#include "integration/UpdateStartupReceipt.h"
#include "ui/StartupNavigationNotice.h"
#include "ui/Controls.h"

#define VERSION_FULL "5.12.4 fixture"
#define OPENNAV_X 1
wxFrame* gFrame = nullptr;
wxFileConfig* pConfig = nullptr;
int n_NavMessageShown = 0;
wxString vs = VERSION_FULL;
wxString g_config_version_string;
bool xnav_fixture = true;
namespace opennav { bool IsXNav() { return xnav_fixture; } }
#include "exact_warning.inc"
#include "exact_warning_text.inc"
bool RunPatchedWarning() {
#include "exact_warning_call.inc"
  return true;
}

// Compare every word of the visible notice with the independently extracted
// pinned upstream text, ignoring only leading/line-wrap whitespace.
std::string NoticeWords(const wxString& value) {
  std::istringstream input(value.ToStdString(wxConvUTF8));
  std::string result, word;
  while (input >> word) {
    if (!result.empty()) result += ' ';
    result += word;
  }
  return result;
}

class WarningFixture final : public wxApp {
 public:
  bool OnInit() override {
    using namespace opennav::integration;
    const auto raw = std::getenv("SKAGER_WARNING_FIXTURE_MODE");
    mode_ = raw ? raw : "";
    if (mode_ == "legacy-agree" || mode_ == "legacy-cancel") {
      xnav_fixture = false;
      mode_.erase(0, 7);
    }
    if (mode_ != "agree" && mode_ != "cancel" && mode_ != "escape" &&
        mode_ != "close" && mode_ != "timeout" && mode_ != "fast")
      return false;
    if (xnav_fixture) CaptureUpdateStartupReceipt();
    for (const auto key : {L"SKAGER_UPDATE_PIPE", L"SKAGER_UPDATE_GENERATION",
                           L"SKAGER_UPDATE_CHALLENGE"}) {
      wchar_t value[128];
      if (GetEnvironmentVariableW(key, value, 128) || _wgetenv(key)) {
        std::cerr << "FAIL updater environment retained\n" << std::flush;
        return false;
      }
    }
    const auto profile = std::getenv("SKAGER_WARNING_FIXTURE_PROFILE");
    if (!profile || !*profile) return false;
    config_ = std::make_unique<wxFileConfig>("", "", wxString::FromUTF8(profile));
    pConfig = config_.get();
    gFrame = new wxFrame(nullptr, wxID_ANY, "Isolated navigation warning fixture");
    gFrame->Show();
    timer_.SetOwner(this);
    Bind(wxEVT_TIMER, &WarningFixture::OnTimer, this);
    timer_.Start(25);
    CallAfter([this] {
      const bool accepted = RunPatchedWarning();
      timer_.Stop();
      if (!seen_ || !clicked_ || accepted != (mode_ == "agree" || mode_ == "fast") ||
          n_NavMessageShown != (accepted ? 1 : 0)) {
        std::cerr << "FAIL modal result or original persistence branch\n" << std::flush;
        ExitMainLoop();
        return;
      }
      std::cout << "PASS exact dialog result " << (accepted ? "agree" : "cancel")
                << "; original nav-message branch preserved\n" << std::flush;
      if (!xnav_fixture) {
        ExitMainLoop();
        return;
      }
      // Deliberately synthetic policy observations. This proves the production
      // sender boundary, NOT the installed app's real 30-second health path.
      using namespace opennav::integration;
      using namespace std::chrono_literals;
      const UpdateStartupReceiptState::Time now{100s};
      ObserveUpdateStartupHealth(true, now + 60s);
      NotifyUpdateStartupHealthy();
      ObserveUpdateStartupHealth(true, now + 90s);
      std::cout << "PASS synthetic post-dialog checkpoint probe; not installed health\n" << std::flush;
      // Stay alive with the normal wx loop until the authenticated supervisor
      // fixture checks EOF and terminates only this owned fixture process.
    });
    return true;
  }

 private:
  void OnTimer(wxTimerEvent&) {
    wxDialog* dialog = nullptr;
    for (auto node = wxTopLevelWindows.GetFirst(); node; node = node->GetNext()) {
      auto* candidate = dynamic_cast<wxDialog*>(node->GetData());
      if (candidate && candidate->IsShown() && candidate->IsModal()) dialog = candidate;
    }
    if (!dialog) return;
    const auto now = std::chrono::steady_clock::now();
    if (!seen_) {
      auto* agree = dynamic_cast<wxControl*>(dialog->FindWindow(wxID_OK));
      auto* cancel = dynamic_cast<wxControl*>(dialog->FindWindow(wxID_CANCEL));
      if (n_NavMessageShown != 0 || !agree || !cancel ||
          agree->GetLabel() != "Agree" || cancel->GetLabel() != "Cancel") {
        Fail(*dialog, "notice consent changed before a choice or action labels changed");
        return;
      }
      if (xnav_fixture) {
        auto* reader = dynamic_cast<wxTextCtrl*>(wxWindow::FindWindowByName(
            "Complete navigation safety notice", dialog));
        if (dynamic_cast<AlertDialog*>(dialog) ||
            dialog->GetTitle() != "Welcome to SKAGER" ||
            !dynamic_cast<opennav::ui::XNavButton*>(agree) ||
            !dynamic_cast<opennav::ui::XNavButton*>(cancel) ||
            !reader || reader->IsEditable() ||
            NoticeWords(reader->GetValue()) != NoticeWords(ExactUpstreamWarningText()) ||
            wxWindow::FindFocus() != cancel) {
          Fail(*dialog, "SKAGER notice text, native controls or cancellation focus changed");
          return;
        }
        std::cout << "PASS complete pinned safety notice and explicit Agree/Cancel controls\n"
                  << std::flush;
      } else if (!dynamic_cast<AlertDialog*>(dialog) ||
                 dialog->GetTitle() != "Welcome to OpenCPN" ||
                 !dynamic_cast<wxButton*>(agree) || !dynamic_cast<wxButton*>(cancel)) {
        Fail(*dialog, "Legacy no longer uses the stock warning");
        return;
      }
      seen_ = true;
      shown_ = now;
      std::cout << "PASS exact " << (xnav_fixture ? "SKAGER" : "stock")
                << " modal visible; no consent yet\n" << std::flush;
      if (xnav_fixture) {
        using namespace opennav::integration;
        using namespace std::chrono_literals;
        const UpdateStartupReceiptState::Time synthetic{100s};
        ObserveUpdateStartupHealth(true, synthetic);
        NotifyUpdateStartupHealthy();
        ObserveUpdateStartupHealth(true, synthetic + 31s);
      }
    }
    if (mode_ == "timeout" || clicked_) return;
    const auto delay = mode_ == "fast" ? 0 : 3500;
    if (now - shown_ < std::chrono::milliseconds(delay)) return;
    clicked_ = true;
    if (mode_ == "escape") {
      wxKeyEvent escape(wxEVT_CHAR_HOOK);
      escape.m_keyCode = WXK_ESCAPE;
      escape.SetEventObject(dialog);
      dialog->ProcessWindowEvent(escape);
      return;
    }
    if (mode_ == "close") {
      dialog->Close();
      return;
    }
    const int id = mode_ == "cancel" ? wxID_CANCEL : wxID_OK;
    auto* button = dialog->FindWindow(id);
    if (!button) {
      Fail(*dialog, "actual warning button missing");
      return;
    }
    std::cout << "PASS fixture selecting actual " << (id == wxID_OK ? "Agree" : "Cancel")
              << " button after "
              << std::chrono::duration_cast<std::chrono::milliseconds>(now - shown_).count()
              << " ms\n" << std::flush;
    wxCommandEvent press(wxEVT_BUTTON, id);
    press.SetEventObject(button);
    button->GetEventHandler()->ProcessEvent(press);
  }
  void Fail(wxDialog& dialog, const char* reason) {
    std::cerr << "FAIL " << reason << '\n' << std::flush;
    timer_.Stop();
    dialog.EndModal(wxID_CANCEL);
  }
  std::string mode_;
  std::unique_ptr<wxFileConfig> config_;
  wxTimer timer_;
  bool seen_ = false, clicked_ = false;
  std::chrono::steady_clock::time_point shown_;
};
wxIMPLEMENT_APP_NO_MAIN(WarningFixture);
int main(int argc, char** argv) { return wxEntry(argc, argv); }

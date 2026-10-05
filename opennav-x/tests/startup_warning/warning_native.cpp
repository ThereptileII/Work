// Exact upstream modal and patched call-site boundary only. This process never
// starts OpenCPN, loads plugins, opens connections or qualifies installed health.
#include <wx/wx.h>
#include <wx/fileconf.h>
#include <wx/button.h>
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

#define VERSION_FULL "5.12.4 fixture"
#define OPENNAV_X 1
wxFrame* gFrame = nullptr;
wxFileConfig* pConfig = nullptr;
int n_NavMessageShown = 0;
wxString vs = VERSION_FULL;
wxString g_config_version_string;
namespace opennav { bool IsXNav() { return true; } }
#include "exact_warning.inc"
bool RunPatchedWarning() {
#include "exact_warning_call.inc"
  return true;
}

class WarningFixture final : public wxApp {
 public:
  bool OnInit() override {
    using namespace opennav::integration;
    const auto raw = std::getenv("SKAGER_WARNING_FIXTURE_MODE");
    mode_ = raw ? raw : "";
    if (mode_ != "agree" && mode_ != "cancel" && mode_ != "timeout" && mode_ != "fast")
      return false;
    CaptureUpdateStartupReceipt();
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
    gFrame = new wxFrame(nullptr, wxID_ANY, "Isolated upstream warning fixture");
    gFrame->Show();
    timer_.SetOwner(this);
    Bind(wxEVT_TIMER, &WarningFixture::OnTimer, this);
    timer_.Start(25);
    CallAfter([this] {
      const bool accepted = RunPatchedWarning();
      timer_.Stop();
      if (!seen_ || accepted != (mode_ == "agree" || mode_ == "fast") ||
          n_NavMessageShown != (accepted ? 1 : 0)) {
        std::cerr << "FAIL modal result or original persistence branch\n" << std::flush;
        ExitMainLoop();
        return;
      }
      std::cout << "PASS exact dialog result " << (accepted ? "agree" : "cancel")
                << "; original nav-message branch preserved\n" << std::flush;
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
    AlertDialog* dialog = nullptr;
    for (auto node = wxTopLevelWindows.GetFirst(); node; node = node->GetNext()) {
      auto* candidate = dynamic_cast<AlertDialog*>(node->GetData());
      if (candidate && candidate->IsShown() && candidate->IsModal()) dialog = candidate;
    }
    if (!dialog) return;
    const auto now = std::chrono::steady_clock::now();
    if (!seen_) {
      seen_ = true;
      shown_ = now;
      std::cout << "PASS exact upstream modal visible; no consent yet\n" << std::flush;
      using namespace opennav::integration;
      using namespace std::chrono_literals;
      const UpdateStartupReceiptState::Time synthetic{100s};
      ObserveUpdateStartupHealth(true, synthetic);
      NotifyUpdateStartupHealthy();
      ObserveUpdateStartupHealth(true, synthetic + 31s);
    }
    if (mode_ == "timeout" || clicked_) return;
    const auto delay = mode_ == "fast" ? 0 : 3500;
    if (now - shown_ < std::chrono::milliseconds(delay)) return;
    const int id = mode_ == "cancel" ? wxID_CANCEL : wxID_OK;
    auto* button = dynamic_cast<wxButton*>(dialog->FindWindow(id));
    if (!button) {
      std::cerr << "FAIL exact upstream button missing\n" << std::flush;
      return;
    }
    clicked_ = true;
    std::cout << "PASS fixture selecting actual " << (id == wxID_OK ? "Agree" : "Cancel")
              << " button after "
              << std::chrono::duration_cast<std::chrono::milliseconds>(now - shown_).count()
              << " ms\n" << std::flush;
    wxCommandEvent press(wxEVT_BUTTON, id);
    press.SetEventObject(button);
    button->GetEventHandler()->ProcessEvent(press);
  }
  std::string mode_;
  std::unique_ptr<wxFileConfig> config_;
  wxTimer timer_;
  bool seen_ = false, clicked_ = false;
  std::chrono::steady_clock::time_point shown_;
};
wxIMPLEMENT_APP_NO_MAIN(WarningFixture);
int main(int argc, char** argv) { return wxEntry(argc, argv); }

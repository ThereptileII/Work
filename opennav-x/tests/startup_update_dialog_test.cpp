// The production native popup, with in-memory authenticated-identity fixtures.
// No verifier/installer, OpenCPN process, network, profile or hardware is used.
#include "ui/StartupUpdateDialog.h"
#include "ui/Controls.h"

#include <wx/app.h>
#include <wx/dialog.h>
#include <wx/log.h>
#include <wx/timer.h>
#include <wx/uiaction.h>

#include <cstdlib>
#include <functional>
#include <iostream>
#include <stdexcept>

using namespace opennav;
namespace {
application::StartupUpdateCandidate Candidate() {
  return {std::string(64, 'a'), "0.6.0-beta.1", std::string(40, 'b')};
}

class Test final : public wxApp {
 public:
  bool OnInit() override {
    std::cout.setf(std::ios::unitbuf);
    wxLog::SetActiveTarget(new wxLogStderr());
    wxSetAssertHandler([](const wxString& file, int line, const wxString&,
                         const wxString& condition, const wxString&) {
      std::cerr << "WX ASSERT " << file << ':' << line << ' ' << condition << '\n';
      std::abort();
    });
    SetExitOnFrameDelete(false);
    timer_.SetOwner(this);
    Bind(wxEVT_TIMER, [this](wxTimerEvent&) { Interact(); });
    CallAfter([this] {
      try { Run(); }
      catch (const std::exception& error) {
        result_ = 1;
        std::cerr << "FAIL " << error.what() << '\n';
      }
      timer_.Stop();
      ExitMainLoop();
    });
    return true;
  }
  int OnRun() override { wxApp::OnRun(); return result_; }

 private:
  void Check(bool passed, const char* message) {
    if (!passed) throw std::runtime_error(message);
    ++checks_;
    std::cout << "PASS " << message << '\n';
  }
  ui::XNavButton& Button(wxDialog& dialog, const wxString& label) {
    for (auto* child : dialog.GetChildren())
      if (auto* button = dynamic_cast<ui::XNavButton*>(child);
          button && button->GetLabel() == label) return *button;
    throw std::runtime_error("Startup button missing");
  }
  void MouseClick(wxWindow& target) {
    for (auto type : {wxEVT_LEFT_DOWN, wxEVT_LEFT_UP}) {
      wxMouseEvent event(type);
      event.SetEventObject(&target);
      event.SetPosition({20, 20});
      target.GetEventHandler()->ProcessEvent(event);
    }
  }
  void Key(wxWindow& target, wxEventType type, int code) {
    wxKeyEvent event(type);
    event.SetEventObject(&target);
    event.m_keyCode = code;
    target.GetEventHandler()->ProcessEvent(event);
  }
  void Command(wxWindow& target) {
    wxCommandEvent event(wxEVT_BUTTON, target.GetId());
    event.SetEventObject(&target);
    target.GetEventHandler()->ProcessEvent(event);
  }
  void Interact() {
    wxDialog* popup = nullptr;
    for (auto* window : wxTopLevelWindows)
      if (window->GetName() == "SKAGER startup update")
        popup = dynamic_cast<wxDialog*>(window);
    try {
      Check(popup && popup->IsModal(), "real startup dialog owns a modal loop");
      Check(popup->GetParent() == nullptr, "startup popup needs no application frame");
      auto action = std::move(action_);
      action_ = {};
      Check(bool(action), "interaction is consumed once");
      action(*popup);
    } catch (const std::exception& error) {
      interaction_error_ = error.what();
      if (popup && popup->IsModal()) popup->EndModal(wxID_CANCEL);
      else ExitMainLoop();
    }
  }
  void Scenario(const char* name, bool accept,
                std::function<void(wxDialog&)> action) {
    std::cout << "SCENARIO " << name << '\n';
    application::StartupUpdate model;
    model.CompleteCheck(application::StartupUpdateCheck::UpdateAvailable, Candidate());
    interaction_error_.clear();
    action_ = std::move(action);
    timer_.StartOnce(150);
    const auto selected = ui::ShowStartupUpdateDialog(nullptr, model);
    timer_.Stop();
    if (!interaction_error_.empty()) throw std::runtime_error(interaction_error_);
    Check(!action_, "dialog received its scheduled input");
    Check(bool(selected) == accept, "dialog returns expected consent choice");
    if (selected) {
      Check(selected->policy_sha256 == Candidate().policy_sha256 &&
                selected->version == Candidate().version &&
                selected->commit == Candidate().commit,
            "accepted choice preserves exact presented identity");
    }
    Check(model.State() == (accept ? application::StartupUpdateState::HandoffRequested
                                  : application::StartupUpdateState::Continue),
          "native choice produces expected application state");
    Check(!model.UpdateNow(), "repeated model activation cannot create another choice");
    Check(!ui::ShowStartupUpdateDialog(nullptr, model),
          "same session cannot reopen a completed popup");
    model.CompleteCheck(application::StartupUpdateCheck::UpdateAvailable, Candidate());
    Check(!ui::ShowStartupUpdateDialog(nullptr, model),
          "late check cannot reopen a completed popup");
  }
  void Run() {
    Scenario("Later", false, [this](wxDialog& dialog) {
      MouseClick(Button(dialog, "LATER"));
    });
    Scenario("Update Now", true, [this](wxDialog& dialog) {
      MouseClick(Button(dialog, "UPDATE NOW"));
    });
    Scenario("Escape", false, [this](wxDialog& dialog) {
      Key(dialog, wxEVT_CHAR_HOOK, WXK_ESCAPE);
    });
    Scenario("Window close", false, [](wxDialog& dialog) { dialog.Close(); });
    Scenario("Initial Return is Later", false, [this](wxDialog& dialog) {
      auto& later = Button(dialog, "LATER");
      Check(wxWindow::FindFocus() == &later, "initial focus is the safe Later choice");
      Key(later, wxEVT_CHAR_HOOK, WXK_RETURN);
      Key(later, wxEVT_KEY_DOWN, WXK_RETURN);
      Key(later, wxEVT_KEY_UP, WXK_RETURN);
    });
    Scenario("Queued repeated Update Now", true, [this](wxDialog& dialog) {
      auto& update = Button(dialog, "UPDATE NOW");
      MouseClick(update);
      MouseClick(update);
    });
    Scenario("First accepted command wins", true, [this](wxDialog& dialog) {
      Command(Button(dialog, "UPDATE NOW"));
      Command(Button(dialog, "UPDATE NOW"));
      Command(Button(dialog, "LATER"));
    });
    Scenario("First Later command wins", false, [this](wxDialog& dialog) {
      Command(Button(dialog, "LATER"));
      Command(Button(dialog, "UPDATE NOW"));
      Key(dialog, wxEVT_CHAR_HOOK, WXK_ESCAPE);
    });
#ifdef __WXMSW__
    // Native Windows is authoritative for OS input. Headless GTK runs retain
    // the same production controls with dispatched mouse/key regressions.
    Scenario("Native pointer Update Now", true, [this](wxDialog& dialog) {
      auto& update = Button(dialog, "UPDATE NOW");
      dialog.Raise();
      update.SetFocus();
      wxUIActionSimulator input;
      const auto size = update.GetClientSize();
      const auto point = update.ClientToScreen({size.x / 2, size.y / 2});
      Check(input.MouseMove(point), "OS pointer move injected");
      // Let the platform deliver motion/focus before pressing, then deliver
      // release separately, as a real pointer does across native event turns.
      action_ = [this](wxDialog&) {
        wxUIActionSimulator press;
        Check(press.MouseDown(), "OS pointer press injected");
        action_ = [this](wxDialog& popup) {
          const bool captured = Button(popup, "UPDATE NOW").HasCapture();
          wxUIActionSimulator release;
          Check(release.MouseUp(), "OS pointer release injected");
          Check(captured, "OS pointer press reached the actual Update Now button");
        };
        timer_.StartOnce(150);
      };
      timer_.StartOnce(150);
    });
#endif
    std::cout << "PASS " << checks_ << " startup update interaction checks\n";
  }

  wxTimer timer_;
  std::function<void(wxDialog&)> action_;
  std::string interaction_error_;
  int result_ = 0;
  int checks_ = 0;
};
}  // namespace
wxIMPLEMENT_APP_NO_MAIN(Test);
int main(int argc, char** argv) { return wxEntry(argc, argv); }

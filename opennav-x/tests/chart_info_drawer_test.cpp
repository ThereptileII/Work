// Offline native widget fixture: no charts, network, files or equipment.
#include "ui/ChartInfoDrawer.h"
#include <iostream>
#include <stdexcept>
#include <wx/app.h>
#include <wx/log.h>

using namespace opennav;
namespace {
void Check(bool value, const char *why) {
  if (!value) throw std::runtime_error(why);
}
template <typename T> T *Find(wxWindow *owner, const wxString &label) {
  for (auto *child : owner->GetChildren()) {
    if (auto *value = dynamic_cast<T *>(child); value && value->GetLabel() == label) return value;
    if (auto *value = Find<T>(child, label)) return value;
  }
  return nullptr;
}
void Click(wxWindow &window) {
  wxCommandEvent event(wxEVT_BUTTON, window.GetId());
  event.SetEventObject(&window);
  window.ProcessWindowEvent(event);
}
class TestApp final : public wxApp {
public:
  bool OnInit() override {
    wxLog::SetActiveTarget(new wxLogStderr());
    frame_ = new wxFrame(nullptr, wxID_ANY, "Offline chart information fixture",
                          {0, 0}, {1280, 800});
    frame_->Show();
    drawer_ = new ui::XNavChartInfoDrawer(*frame_);
    drawer_->on_dismiss = [this] { ++dismissed_; };
    CallAfter([this] { Run(); });
    return true;
  }
  int OnRun() override { wxApp::OnRun(); return result_; }
  int OnExit() override { return result_; }
private:
  void Run() {
    try {
      const auto info = application::ParseChartInfo(
          "<b>Fixture buoy</b><br>Red lateral mark<hr>"
          "<b>Fixture obstruction</b><br>Unknown clearance<hr>"
          "<b>Fixture light</b><br>Fl W 6 s 8 NM", 57, 16);
      for (const auto mode : {ui::LightMode::Day, ui::LightMode::Dusk, ui::LightMode::Night}) {
        drawer_->Open(info, frame_->GetScreenRect(), mode);
        Check(drawer_->IsShown() && drawer_->View().objects.size() == 3,
              "Every overlapping information section is retained in the drawer");
        auto *summary = Find<wxStaticText>(drawer_, "Red lateral mark");
        Check(summary && summary->IsShown(), "Human-readable information is initially visible");
        Check(summary->GetForegroundColour() == ui::Colour(ui::Theme(mode).primary),
              "Chart information uses the current Day/Dusk/Night ink");
        auto *details = Find<wxStaticText>(drawer_, "Fixture buoy\nRed lateral mark");
        Check(details && !details->IsShown(), "Complete technical details are initially secondary");
        auto *toggle = Find<ui::XNavButton>(drawer_, "Show all chart details");
        Check(toggle && toggle->GetMinSize().y >= drawer_->FromDIP(48),
              "Details disclosure has a touch-sized target");
        Click(*toggle);
        Check(details->IsShown(), "Touch disclosure exposes the complete source section");
        Check(toggle->GetLabel() == "Hide chart details", "Disclosure describes its current action");
        Click(*toggle);
        Check(!details->IsShown(), "Details collapse without removing the object");
      }
      wxKeyEvent escape(wxEVT_CHAR_HOOK);
      escape.m_keyCode = WXK_ESCAPE;
      Check(drawer_->FilterEvent(escape) == wxEventFilter::Event_Processed &&
                !drawer_->IsShown() && dismissed_ == 1,
            "Escape dismisses chart information");
      drawer_->Open(info, frame_->GetScreenRect(), ui::LightMode::Day);
      auto *close = Find<ui::XNavButton>(drawer_, "Close");
      Check(close, "A visible close action is present");
      Click(*close);
      Check(!drawer_->IsShown() && dismissed_ == 2, "Touch close dismisses chart information");
      std::cout << "Chart information drawer behavior passed\n";
    } catch (const std::exception &error) {
      result_ = 1;
      std::cerr << error.what() << '\n';
    }
    frame_->Destroy();
    ExitMainLoop();
  }
  wxFrame *frame_ = nullptr;
  ui::XNavChartInfoDrawer *drawer_ = nullptr;
  int result_ = 0, dismissed_ = 0;
};
}
wxIMPLEMENT_APP(TestApp);

// Offline interaction regression: production drawer and controls; no chart,
// network, stored credentials, OpenCPN process, screenshots or hardware.
#include "ui/AisDrawer.h"
#include <wx/app.h>
#include <wx/dialog.h>
#include <wx/log.h>
#include <wx/timer.h>
#include <wx/uiaction.h>
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace std::chrono_literals;
namespace {
class Test final : public wxApp {
public:
  bool OnInit() override {
    std::cout.setf(std::ios::unitbuf);
    wxLog::SetActiveTarget(new wxLogStderr());
    wxSetAssertHandler([](const wxString &file, int line, const wxString &,
                          const wxString &condition, const wxString &) {
      std::cerr << "WX ASSERT " << file << ':' << line << ' ' << condition << '\n';
      std::abort();
    });
    owner_ = new wxFrame(nullptr, wxID_ANY, "AIS scroll test", {0, 0}, {1280, 800});
    owner_->Bind(wxEVT_MOUSEWHEEL, [this](wxMouseEvent &) { ++outside_wheels_; });
    owner_->Bind(wxEVT_GESTURE_PAN, [this](wxPanGestureEvent &) { ++outside_pans_; });
    owner_->Bind(wxEVT_MOTION, [this](wxMouseEvent &event) {
      // wxMSW may deliver real owner motion after showing windows or changing
      // capture. It is not one of the fixture's explicitly dispatched gestures.
      // Count all synchronous fixture delivery, including forwarded events,
      // and any child-origin motion reaching the owner outside that dispatch.
      if (dispatch_origin_ || event.GetEventObject() != owner_) ++outside_moves_;
      else ++incidental_owner_moves_;
    });
    owner_->Show();
    application::OnlineAisActions online_actions;
    online_actions.read = [this](vessel::Time) { return online_; };
    online_actions.set_radius_nm = [this](int value) {
      ++radius_saves_;
      if (fail_radius_save_) return application::CommandResult{false,"Radius save failed",{}};
      online_.radius_nm = value;
      return application::CommandResult{true,"Radius saved",{}};
    };
    drawer_ = new ui::XNavAisDrawer(*owner_, online_actions, [this](int) { ++actions_; });
    drawer_->on_select = [this](int id) { selected_ = id; };
    const vessel::Time now{1000s};
    state_.available = true;
    for (int i = 0; i < 40; ++i) {
      vessel::AisTarget t;
      t.mmsi = 265000001 + i; t.name = "Fixture vessel " + std::to_string(i + 1);
      t.active = true; t.observed_at = now;
      t.latitude_deg = {58.3, "test", now, vessel::Validity::Measured, {15s, 60s}};
      t.longitude_deg = {16.8, "test", now, vessel::Validity::Measured, {15s, 60s}};
      state_.targets.push_back(t);
    }
    drawer_->Update(state_, {}, now, ui::LightMode::Day);
    drawer_->Target(265000001);
    drawer_->Present(Workspace());
    for (auto *child : drawer_->GetChildren())
      if (auto *scroll = dynamic_cast<ui::XNavScroll *>(child)) body_ = scroll;
    steps_ = {
      [this] {
        Check(body_ && Maximum() > 0, "target details exceed the viewport");
        Wheel(*Visual(), -120);
        Check(Offset() > 0, "wheel over painted target details scrolls the body");
        const auto previous = Offset();
        Wheel(*Visual(), -15);
        Check(Offset() > previous, "fractional wheel movement is retained");
        Wheel(*Visual(), -120000);
        Check(Offset() == Maximum(), "wheel clamps at the last content pixel");
        Wheel(*Visual(), -120);
        Check(Offset() == Maximum() && outside_wheels_ == 0,
              "wheel at lower bound remains owned by the panel");
        Wheel(*Visual(), 120000);
        Check(Offset() == 0, "wheel clamps at the top");
        Wheel(*Visual(), 120);
        Check(Offset() == 0 && outside_wheels_ == 0,
              "wheel at upper bound remains owned by the panel");
        Pan(*Visual(), -37);
        Check(Offset() == 37, "native touch pan over details moves exact pixels");
        Pan(*Visual(), -100000);
        Check(Offset() == Maximum(), "native pan clamps at lower bound");
        Pan(*Visual(), 100000);
        Check(Offset() == 0, "native pan clamps at upper bound");
        Drag(*Visual(), -90);
        Check(Offset() == 90, "mouse-compatible touch drag scrolls the panel");
        const auto previous_scroll = Offset();
        drawer_->Update(state_, {}, vessel::Time{1001s}, ui::LightMode::Day);
        drawer_->Present(Workspace());
        Check(Offset() == previous_scroll, "live data/layout refresh preserves position");
        drawer_->Update(state_, {}, vessel::Time{1001s}, ui::LightMode::Night);
        Check(Offset() == previous_scroll, "theme rebuild preserves position");
        drawer_->Dismiss(); drawer_->Present(Workspace());
      },
      [this] {
        Check(Offset() == 0, "reopening resets the same panel to the top");
        Wheel(*Visual(), -120000);
        auto *button = FindButton("Show on chart");
        Check(button && button->IsEnabled(), "current fixture enables chart action");
        const int previous = Offset();
        Drag(*button, 80);
        Check(Offset() == previous - 80, "drag starting on an action scrolls upward");
        Check(!wxWindow::GetCapture(), "drag release clears mouse capture");
      },
      [this] {
        Check(actions_ == 0, "drag release never activates its original button");
        Wheel(*body_, -120000);
        auto *button = FindButton("Show on chart");
        Mouse(*button, wxEVT_LEFT_DOWN, {40, 20});
        Mouse(*button, wxEVT_LEFT_UP, {40, 20});
      },
      [this] {
        Check(actions_ == 1, "ordinary click still activates exactly once");
        // A native touch recognizer can begin while a button holds capture.
        auto *button = FindButton("Show on chart");
        Mouse(*button, wxEVT_LEFT_DOWN, {40, 20});
        Pan(*button, 50);
        Mouse(*button, wxEVT_LEFT_UP, {40, 20});
      },
      [this] {
        Check(actions_ == 1 && !wxWindow::GetCapture(),
              "native touch pan cancels button press and capture");
        OwnerInputCounts("before explicit outside input");
        const int wheels = outside_wheels_, pans = outside_pans_, moves = outside_moves_;
        Wheel(*owner_, -120);
        OwnerInputCounts("after outside wheel");
        Check(outside_wheels_ == wheels + 1, "outside wheel reaches owner exactly once");
        Pan(*owner_, -60);
        OwnerInputCounts("after outside pan");
        Check(outside_pans_ == pans + 1, "outside pan reaches owner exactly once");
        Mouse(*owner_, wxEVT_MOTION, {10, 10}, true);
        OwnerInputCounts("after outside motion");
        Check(outside_moves_ == moves + 1, "outside motion reaches owner exactly once");
        Check(outside_wheels_ == 1 && outside_pans_ == 1 && outside_moves_ == 1,
              "outside chart-owner input is unaffected");
        drawer_->List();
      },
      [this] {
        Check(Offset() == 0, "page navigation resets body position");
        auto *list = List();
        Wheel(*list, -120);
        Check(Offset() == 0, "vessel list wheel owns its nested viewport");
        Mouse(*list, wxEVT_LEFT_DOWN, {90, 30});
        Mouse(*list, wxEVT_LEFT_UP, {90, 30});
      },
      [this] {
        Check(selected_ == 265000002, "wheel reaches the next vessel row");
        drawer_->List();
        selected_ = 0;
        Drag(*List(), -142);
      },
      [this] {
        Check(selected_ == 0 && drawer_->PageTitle() == "AIS targets",
              "dragging the vessel list never selects a row");
        Mouse(*List(), wxEVT_LEFT_DOWN, {90, 30});
        Mouse(*List(), wxEVT_LEFT_UP, {90, 30});
      },
      [this] {
        Check(selected_ == 265000003, "touch drag scrolls the vessel list two rows");
        Wheel(*Visual(), -120000);
        drawer_->Present({80, 68, 1014, 450});
      },
      [this] {
        Wheel(*Visual(), -120000);
        Check(Offset() == Maximum(), "resized target page retains reachable bottom");
        drawer_->ShowSettings();
        Check(Offset() == 0, "settings opens at the top after a scrolled target");
        // Disabled buttons and static labels also remain scroll surfaces.
        Wheel(*body_->GetChildren().front(), -120);
        Check(Maximum() == 0 || Offset() > 0, "settings text forwards wheel input");
        drawer_->Target(265000001);
        Wheel(*Visual(), -120000);
        const int before = Offset();
        wxDialog modal(drawer_, wxID_ANY, "modal input");
        bool modal_owned_input = false;
        timer_.Stop();
        modal.CallAfter([this, &modal, before, &modal_owned_input] {
          Wheel(modal, -120);
          modal_owned_input = Offset() == before;
          modal.EndModal(wxID_CANCEL);
        });
        modal.ShowModal();
        Check(modal_owned_input, "separate owned modal cannot scroll underlying panel");
        timer_.Start(400);
      },
      [this] {
        Check(outside_wheels_ == 1 && outside_pans_ == 1 && outside_moves_ == 1,
              "all panel gestures leave chart-owner input counters unchanged");
        drawer_->ShowSettings();
        auto *radius = Radius();
        Check(radius && radius->GetValue() == 25, "radius slider shows persisted default");
        wxKeyEvent end(wxEVT_KEY_DOWN); end.m_keyCode = WXK_END;
        end.SetEventObject(radius); radius->GetEventHandler()->ProcessEvent(end);
        Check(radius->GetValue() == 200 && radius_saves_ == 0 && online_.radius_nm == 25,
              "maximum slider draft does not save or update provider during adjustment");
        auto *apply = FindButton("Apply radius");
        Check(apply && apply->IsEnabled(), "changed radius has explicit apply action");
        wxCommandEvent event(wxEVT_BUTTON,apply->GetId()); event.SetEventObject(apply);
        apply->GetEventHandler()->ProcessEvent(event);
      },
      [this] {
        Check(radius_saves_ == 1 && online_.radius_nm == 200 && Radius()->GetValue() == 200,
              "applying maximum persists once and displays read-back value");
        wxKeyEvent home(wxEVT_KEY_DOWN); home.m_keyCode = WXK_HOME;
        home.SetEventObject(Radius()); Radius()->GetEventHandler()->ProcessEvent(home);
        Check(Radius()->GetValue() == 1, "radius slider exposes supported minimum");
        fail_radius_save_ = true;
        auto *apply = FindButton("Apply radius");
        wxCommandEvent event(wxEVT_BUTTON,apply->GetId()); event.SetEventObject(apply);
        apply->GetEventHandler()->ProcessEvent(event);
      },
      [this] {
        Check(radius_saves_ == 2 && online_.radius_nm == 200 && Radius()->GetValue() == 200,
              "failed radius save restores persisted value rather than claiming success");
        drawer_->Target(265000001);
        drawer_->Present(Workspace());
        drawer_->Raise();
      },
      [this] {
        // One OS-level pointer smoke in addition to dispatched wheel/touch
        // regressions. No screenshots or design acceptance are involved.
        const auto point = body_->ClientToScreen({100, 220});
        wxUIActionSimulator input;
        Check(input.MouseMove(point) && input.MouseDown(), "native pointer press injected");
        native_start_ = point;
      },
      [this] {
        wxUIActionSimulator input;
        Check(input.MouseMove(native_start_ - wxPoint(0, 80)), "native pointer drag injected");
      },
      [this] {
        wxUIActionSimulator input;
        Check(input.MouseUp(), "native pointer release injected");
      },
      [this] {
        Check(Offset() == 80 && !wxWindow::GetCapture(),
              "OS pointer drag scrolls the panel and releases capture");
        std::cout << "PASS " << checks_ << " AIS scroll interaction checks\n";
        Finish();
      }
    };
    timer_.SetOwner(this);
    Bind(wxEVT_TIMER, [this](wxTimerEvent &) {
      try { if (step_ < steps_.size()) steps_[step_++](); }
      catch (const std::exception &e) {
        std::cerr << "FAIL " << e.what() << '\n'; result_ = 1; Finish();
      }
    });
    timer_.Start(400);
    return true;
  }
  int OnRun() override { wxApp::OnRun(); return result_; }
private:
  wxRect Workspace() const { return {80, 68, 1014, 698}; }
  int Offset() const { return body_->GetViewStart().y; }
  int Maximum() const { return std::max(0, body_->GetVirtualSize().y - body_->GetClientSize().y); }
  wxWindow *Visual() const { return body_->GetChildren().front(); }
  ui::XNavListView *List() const {
    for (auto *child : body_->GetChildren())
      if (auto *list = dynamic_cast<ui::XNavListView *>(child)) return list;
    throw std::runtime_error("vessel list missing");
  }
  ui::XNavButton *FindButton(const wxString &name) const {
    for (auto *child : body_->GetChildren())
      if (auto *button = dynamic_cast<ui::XNavButton *>(child); button && button->GetName() == name)
        return button;
    return nullptr;
  }
  ui::XNavRange *Radius() const {
    for (auto *child : body_->GetChildren())
      if (auto *range = dynamic_cast<ui::XNavRange *>(child)) return range;
    return nullptr;
  }
  void Check(bool value, const char *message) {
    if (!value) throw std::runtime_error(message);
    ++checks_; std::cout << "PASS " << message << '\n';
  }
  void OwnerInputCounts(const char *stage) const {
    std::cout << "OWNER INPUT " << stage << " wheel=" << outside_wheels_
              << " pan=" << outside_pans_ << " motion=" << outside_moves_
              << " incidental-motion=" << incidental_owner_moves_ << '\n';
  }
  void Dispatch(wxWindow &target, wxEvent &event) {
    struct RestoreOrigin {
      wxWindow *&slot;
      wxWindow *previous;
      ~RestoreOrigin() { slot = previous; }
    } restore{dispatch_origin_, dispatch_origin_};
    dispatch_origin_ = &target;
    target.GetEventHandler()->ProcessEvent(event);
  }
  void Wheel(wxWindow &target, int rotation) {
    wxMouseEvent wheel(wxEVT_MOUSEWHEEL);
    wheel.SetEventObject(&target); wheel.SetPosition({30, 20});
    wheel.m_wheelRotation = rotation; wheel.m_wheelDelta = 120; wheel.m_linesPerAction = 3;
    Dispatch(target, wheel);
  }
  void Pan(wxWindow &target, int y) {
    wxPanGestureEvent pan;
    pan.SetEventObject(&target); pan.SetGestureStart(); pan.SetDelta({0, y});
    Dispatch(target, pan);
  }
  void Mouse(wxWindow &target, wxEventType type, wxPoint point, bool down = false) {
    wxMouseEvent event(type);
    event.SetEventObject(&target); event.SetPosition(point); event.m_leftDown = down;
    Dispatch(target, event);
  }
  void Drag(wxWindow &target, int dy) {
    const auto begin = target.ClientToScreen({40, 20});
    Mouse(target, wxEVT_LEFT_DOWN, {40, 20});
    Mouse(target, wxEVT_MOTION, {40, 20 + dy / 2}, true);
    auto *capture = wxWindow::GetCapture();
    Check(capture == body_, "body captures an active drag");
    Mouse(*capture, wxEVT_MOTION, capture->ScreenToClient(begin + wxPoint(0, dy)), true);
    Mouse(*capture, wxEVT_LEFT_UP, capture->ScreenToClient(begin + wxPoint(0, dy)));
  }
  void Finish() {
    timer_.Stop();
    drawer_->Destroy(); owner_->Destroy(); ExitMainLoop();
  }
  wxFrame *owner_ = nullptr;
  ui::XNavAisDrawer *drawer_ = nullptr;
  ui::XNavScroll *body_ = nullptr;
  vessel::AisState state_;
  application::OnlineAisState online_;
  int radius_saves_ = 0;
  bool fail_radius_save_ = false;
  wxTimer timer_;
  wxPoint native_start_;
  std::vector<std::function<void()>> steps_;
  std::size_t step_ = 0;
  int result_ = 0, checks_ = 0, actions_ = 0, selected_ = 0;
  int outside_wheels_ = 0, outside_pans_ = 0, outside_moves_ = 0;
  int incidental_owner_moves_ = 0;
  wxWindow *dispatch_origin_ = nullptr;
};
}
wxIMPLEMENT_APP_NO_MAIN(Test);
int main(int argc, char **argv) { return wxEntry(argc, argv); }

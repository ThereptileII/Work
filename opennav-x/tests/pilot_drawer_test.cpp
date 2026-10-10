// Dedicated offline widgets. Never installed or linked into the product.
#include "ui/PilotDrawer.h"
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <wx/app.h>
#include <wx/dcbuffer.h>
#include <wx/dcscreen.h>
#include <wx/dialog.h>
#include <wx/filename.h>
#include <wx/log.h>
#include <wx/stattext.h>
#include <wx/timer.h>
#ifdef __WXGTK__
#include <gtk/gtk.h>
#endif
using namespace opennav;
using namespace std::chrono_literals;
namespace {
const vessel::Time stamp{100s};
class TestApp final : public wxApp {
public:
  bool OnInit() override {
    wxLog::SetActiveTarget(new wxLogStderr());
    wxSetAssertHandler([](const wxString &, int, const wxString &,
                          const wxString &s, const wxString &) {
      std::cerr << s << std::endl;
      std::abort();
    });
    if (argc != 2)
      return false;
    output_ = argv[1];
    if (!wxFileName::Mkdir(output_, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL))
      return false;
    wxInitAllImageHandlers();
    frame_ = new wxFrame(nullptr, wxID_ANY, "TEST ONLY - Pilot component",
                         {0, 0}, {1280, 800}, wxBORDER_NONE);
    frame_->SetClientSize(1280, 800);
    frame_->SetBackgroundStyle(wxBG_STYLE_PAINT);
    frame_->Bind(wxEVT_PAINT, [this](wxPaintEvent &) {
      wxAutoBufferedPaintDC dc(frame_);
      ui::XNavPainter p(*frame_, dc, light_);
      dc.SetBackground(wxBrush(ui::Colour(p.c.background)));
      dc.Clear();
      p.Text("OFFLINE COMPONENT TEST", 80, 80, 22, p.c.attention);
      p.Text("Fake pilot / no chart, network or equipment", 80, 126, 12,
             p.c.secondary);
    });
    ui::PilotDrawerActions actions;
    actions.command = [this](adapters::PilotAction action, double delta) {
      ++commands_;
      pilot_.command.request = {std::uint64_t(commands_), action, delta, stamp};
      pilot_.command.state = adapters::CommandState::Pending;
      Feed();
    };
    actions.enable = [this](bool enabled) {
      ++enables_;
      pilot_.enabled = enabled;
      Feed();
    };
    // AutoTrack-style take-over: bind + permit behind the one enable sheet.
    actions.take_control = [this] {
      ++takes_;
      permit_ = take_ok_;
      application::CommandResult result;
      result.ok = take_ok_;
      result.message = take_ok_ ? "Bound" : "No single live pilot found";
      return result;
    };
    drawer_ = new ui::XNavPilotDrawer(*frame_, std::move(actions));
    pilot_.capabilities = {false, true, true, false, false, true, true};
    pilot_.fresh = true;
    pilot_.feedback.mode = adapters::PilotMode::Standby;
    pilot_.feedback.sequence = 1;
    pilot_.feedback.source = "OFFLINE TEST ONLY";
    pilot_.feedback.observed_at = stamp;
    pilot_.feedback.heading_magnetic_deg = {41., "OFFLINE TEST ONLY", stamp,
                                            vessel::Validity::Measured};
    state_.rudder.angle_deg = {2., "OFFLINE TEST ONLY", stamp,
                               vessel::Validity::Measured};
    frame_->Show();
    timer_.SetOwner(this);
    Bind(wxEVT_TIMER, &TestApp::Step, this);
    timer_.Start(400);
    return true;
  }
  int OnRun() override {
    wxApp::OnRun();
    return failed_ ? 1 : 0;
  }

private:
  void Check(bool ok, const char *name) {
    ++checks_;
    if (!ok)
      throw std::runtime_error(name);
  }
  void Feed(vessel::Time now = stamp) {
    drawer_->Update(pilot_, now, state_, now, permit_, light_);
    drawer_->Present(wxRect(80, 68, 1014, 698));
    frame_->Refresh(false);
  }
  void Capture(const char *name) {
    Check(drawer_->GetScreenRect() == wxRect(682, 80, 398, 674),
          "canonical pilot sheet geometry");
    Check(frame_->GetClientSize() == wxSize(1280, 800),
          "canonical capture size");
    const auto origin = frame_->ClientToScreen({0, 0});
#ifdef __WXGTK__
    // As with the AIS fixture, GTK ScreenDC may return cached pixels. Capture
    // the actual root window; never redraw a widget into a synthetic image.
    auto *pixels = gdk_pixbuf_get_from_window(gdk_get_default_root_window(),
                                              origin.x, origin.y, 1280, 800);
    Check(pixels != nullptr, "capture screen pixels");
    const auto file = output_ + "/" + name + ".png";
    const bool saved =
        gdk_pixbuf_save(pixels, file.utf8_str(), "png", nullptr, nullptr);
    g_object_unref(pixels);
    Check(saved, "write capture");
#else
    wxScreenDC screen;
    wxBitmap bitmap(1280, 800);
    wxMemoryDC memory(bitmap);
    Check(memory.Blit(0, 0, 1280, 800, &screen, origin.x, origin.y),
          "capture screen pixels");
    memory.SelectObject(wxNullBitmap);
    Check(bitmap.SaveFile(output_ + "/" + name + ".png", wxBITMAP_TYPE_PNG),
          "write capture");
#endif
    names_.push_back(name);
  }
  template <class T> T *Find(const wxString &name) {
    return dynamic_cast<T *>(wxWindow::FindWindowByName(name, drawer_));
  }
  ui::XNavButton *Button(const wxString &label) {
    const auto walk = [&](const auto &self,
                          wxWindow *parent) -> ui::XNavButton * {
      if (auto *b = dynamic_cast<ui::XNavButton *>(parent);
          b && b->GetLabel() == label)
        return b;
      for (auto *c : parent->GetChildren())
        if (auto *b = self(self, c))
          return b;
      return nullptr;
    };
    return walk(walk, drawer_);
  }
  void Click(const wxString &label) {
    auto *button = Button(label);
    Check(button && button->IsEnabled(), "requested action enabled");
    Dispatch(label);
  }
  void Dispatch(const wxString &label) {
    auto *button = Button(label);
    Check(button != nullptr, "requested button exists");
    wxCommandEvent click(wxEVT_BUTTON, button->GetId());
    click.SetEventObject(button);
    button->GetEventHandler()->ProcessEvent(click);
  }
  void Confirm(const wxString &name, bool accept) {
    auto *dialog = dynamic_cast<wxDialog *>(wxWindow::FindWindowByName(name));
    Check(dialog && dialog->IsModal(), "explicit confirmation shown");
    Check(dialog->GetClientSize().x == dialog->FromDIP(520),
          "safety confirmation uses application width, not the narrow drawer");
    Check(frame_->GetScreenRect().Contains(dialog->GetScreenRect()),
          "confirmation remains inside application bounds");
    const auto check_text = [this](const auto &self, wxWindow *parent) -> void {
      for (auto *child : parent->GetChildren()) {
        if (auto *label = dynamic_cast<wxStaticText *>(child))
          Check(label->GetBestSize().x <= parent->GetClientSize().x,
                "full confirmation text wraps within available width");
        self(self, child);
      }
    };
    check_text(check_text, dialog);
    dialog->EndModal(accept ? wxID_OK : wxID_CANCEL);
  }
  void Step(wxTimerEvent &) {
    try {
      switch (step_++) {
      case 0:
        Feed();
        break;
      case 1:
        Feed();
        break;
      case 2:
        Check(drawer_->View().heading_magnetic_deg == 41.,
              "fresh actual magnetic heading");
        Check(!Button("Auto")->IsEnabled() && !Button("Standby")->IsEnabled(),
              "all commands off initially");
        Capture("autopilot-day");
        light_ = ui::LightMode::Dusk;
        Feed();
        break;
      case 3:
        Capture("autopilot-dusk");
        light_ = ui::LightMode::Night;
        Feed();
        break;
      case 4:
        Capture("autopilot-night");
        light_ = ui::LightMode::Day;
        Feed();
        Click("Enable control");
        break;
      case 5:
        Check(enables_ == 0 && !Button("Enable control")->IsSelected(),
              "no optimistic enable");
        Confirm("Enable physical pilot control?", false);
        break;
      case 6:
        Check(enables_ == 0, "cancel does not enable");
        Click("Enable control");
        break;
      case 7:
        Confirm("Enable physical pilot control?", true);
        break;
      case 8:
        Check(enables_ == 1 && Button("Enable control")->IsSelected(),
              "explicit session enable");
        Check(!Button("Track")->IsEnabled() && !Button("Wind")->IsEnabled(),
              "unsupported modes remain disabled");
        Dispatch("Track");
        Dispatch("Wind");
        Click("Auto");
        break;
      case 9:
        Check(commands_ == 0, "no mode command before confirmation");
        Confirm("Request AUTO", false);
        break;
      case 10:
        Check(commands_ == 0, "cancel does not command");
        Click("Auto");
        break;
      case 11:
        Confirm("Request AUTO", true);
        break;
      case 12:
        Check(commands_ == 1 && drawer_->View().pending,
              "manual command pending");
        Check(drawer_->View().mode == adapters::PilotMode::Standby,
              "transmission cannot change displayed mode");
        Check(!Button("Auto")->IsEnabled() &&
                  !Button(wxString::FromUTF8("+1°"))->IsEnabled(),
              "pending command prevents storm");
        Check(Button("Standby")->IsEnabled(),
              "Standby remains reachable while pending");
        Capture("autopilot-pending-day");
        Click("Standby");
        break;
      case 13:
        Check(commands_ == 2 && pilot_.command.request.action ==
                                    adapters::PilotAction::Standby,
              "Standby direct manual request");
        pilot_.feedback.mode = adapters::PilotMode::Auto;
        pilot_.feedback.locked_heading_magnetic_deg = {
            145., "OFFLINE TEST ONLY", stamp, vessel::Validity::Measured};
        pilot_.command.state = adapters::CommandState::Confirmed;
        Feed();
        Click(wxString::FromUTF8("+1°"));
        Click(wxString::FromUTF8("+1°"));
        break;
      case 14:
        Check(commands_ == 3 &&
                  pilot_.command.request.action == adapters::PilotAction::AlterCourse &&
                  pilot_.command.request.delta_deg == 1.,
              "duplicate queued input sends once");
        Check(drawer_->View().heading_magnetic_deg == 145.,
              "pending increment cannot invent heading");
        pilot_.feedback.locked_heading_magnetic_deg.value = 146.;
        ++pilot_.feedback.sequence;
        pilot_.command.state = adapters::CommandState::Confirmed;
        Feed();
        break;
      case 15:
        Check(drawer_->View().commanded &&
                  drawer_->View().heading_magnetic_deg == 146.,
              "measured confirmed heading is displayed");
        Capture("autopilot-auto-day");
        Click(wxString::FromUTF8("−10°"));
        Click(wxString::FromUTF8("−10°"));
        break;
      case 16:
      case 18:
      case 20: {
        const int index = (step_ - 17) / 2;
        const double deltas[] = {-10., -1., 10.};
        const double headings[] = {146., 136., 135.};
        Check(commands_ == 4 + index &&
                  pilot_.command.request.action == adapters::PilotAction::AlterCourse &&
                  pilot_.command.request.delta_deg == deltas[index],
              "each labeled course button dispatches its exact delta once");
        Check(drawer_->View().heading_magnetic_deg == headings[index],
              "every pending course request retains the observed heading");
        for (const auto *label : {"−10°", "−1°", "+1°", "+10°"}) {
          const auto text = wxString::FromUTF8(label);
          Check(!Button(text)->IsEnabled(), "all course buttons disabled while pending");
          Dispatch(text);
        }
        Check(commands_ == 4 + index,
              "synthetic disabled course events cannot bypass pending guard");
        pilot_.feedback.locked_heading_magnetic_deg.value =
            headings[index] + deltas[index];
        ++pilot_.feedback.sequence;
        pilot_.command.state = adapters::CommandState::Confirmed;
        Feed();
        break;
      }
      case 17:
        Click(wxString::FromUTF8("−1°"));
        Click(wxString::FromUTF8("−1°"));
        break;
      case 19:
        Click(wxString::FromUTF8("+10°"));
        Click(wxString::FromUTF8("+10°"));
        break;
      case 21:
        Check(drawer_->View().heading_magnetic_deg == 145.,
              "all course button results follow supplied feedback");
        pilot_.command.state = adapters::CommandState::TimedOut;
        Feed(stamp + 4s);
        break;
      case 22:
        Check(!drawer_->View().heading_magnetic_deg &&
                  !Button("Auto")->IsEnabled(),
              "loss of feedback suppresses dial and Auto");
        Check(Button("Standby")->IsEnabled(),
              "manual Standby remains available after loss");
        Check(drawer_->View().note.find("No confirmation") != std::string::npos,
              "timeout is explicit");
        Capture("autopilot-stale-day");
        state_.replayed = true;
        Feed();
        break;
      case 23:
        Check(!Button("Enable control")->IsEnabled() &&
                  !Button("Standby")->IsEnabled(),
              "replay disables all hardware actions");
        Check(!drawer_->View().heading_magnetic_deg,
              "replay cannot masquerade as live pilot");
        Capture("autopilot-replay-day");
        state_.replayed = false;
        pilot_.output_unavailable = true;
        Feed();
        break;
      case 24: {
        Check(drawer_->View().output_unavailable && !drawer_->View().can_toggle,
              "product output restriction is explicit in owned presentation");
        Check(Button("Control unavailable") && !Button("Enable control"),
              "unavailable control cannot promise an enable path");
        Check(!Button("Control unavailable")->IsEnabled(),
              "old enabled permission cannot activate product toggle");
        Check(drawer_->View().heading_magnetic_deg == 145.,
              "status-only product retains measured pilot heading");
        for (const auto *label : {"Standby", "Auto", "Track", "Wind",
                                  "−10°", "−1°", "+1°", "+10°"}) {
          Check(!Button(wxString::FromUTF8(label))->IsEnabled(),
                "all product commands unavailable");
          Dispatch(wxString::FromUTF8(label));
        }
        auto *toggle = Button("Control unavailable");
        wxCommandEvent event(wxEVT_BUTTON, toggle->GetId());
        event.SetEventObject(toggle);
        toggle->GetEventHandler()->ProcessEvent(event);
        Capture("autopilot-status-only-day");
        light_ = ui::LightMode::Night;
        Feed();
        break;
      }
      case 25:
        Check(enables_ == 1 && commands_ == 6,
              "disabled product event cannot invoke an enable or command callback");
        Capture("autopilot-status-only-night");
        // A live but never-configured pilot: the switch itself must offer the
        // take-over (owner feedback: no separate settings page).
        light_ = ui::LightMode::Day;
        pilot_.output_unavailable = false;
        pilot_.enabled = false;
        pilot_.command.state = adapters::CommandState::Confirmed;
        pilot_.feedback.mode = adapters::PilotMode::Standby;
        pilot_.feedback.observed_at = stamp;
        permit_ = false;
        Feed();
        Check(!drawer_->View().can_toggle, "unconfigured pilot is not yet authorized");
        Click("Enable control");
        break;
      case 26:
        Check(takes_ == 0, "no binding before confirmation");
        Confirm("Enable physical pilot control?", true);
        break;
      case 27:
        Check(takes_ == 1 && permit_, "one confirmation binds and permits");
        Check(enables_ == 1, "session waits for the adapter to verify the binding");
        Feed();
        Check(enables_ == 2 && drawer_->View().enabled &&
                  Button("Enable control")->IsSelected(),
              "verified binding completes the confirmed enable");
        Click("Enable control");
        break;
      case 28:
        Check(enables_ == 3 && !drawer_->View().enabled, "switching off needs no sheet");
        permit_ = false;
        take_ok_ = false;
        Feed();
        Click("Enable control");
        break;
      case 29:
        Confirm("Enable physical pilot control?", true);
        break;
      case 30:
        Feed();
        Check(takes_ == 2 && enables_ == 3 && !Button("Enable control")->IsSelected(),
              "failed take-over never enables");
        Capture("autopilot-takeover-failed-day");
        drawer_->Dismiss();
        Check(!drawer_->IsShown(), "close restores owner");
        Finish();
        break;
      }
    } catch (const std::exception &e) {
      failed_ = true;
      std::cerr << e.what() << std::endl;
      Finish();
    }
  }
  void Finish() {
    timer_.Stop();
    std::ofstream f((output_ + "/result.json").ToStdString());
    f << "{\"passed\":" << (failed_ ? "false" : "true")
      << ",\"checks\":" << checks_ << ",\"captures\":[";
    for (std::size_t i = 0; i < names_.size(); ++i) {
      if (i)
        f << ',';
      f << '"' << names_[i] << '"';
    }
    f << "]}\n";
    f.close();
    drawer_->Destroy();
    frame_->Destroy();
    ExitMainLoop();
  }
  wxString output_;
  wxFrame *frame_ = nullptr;
  ui::XNavPilotDrawer *drawer_ = nullptr;
  wxTimer timer_;
  vessel::VesselState state_;
  adapters::PilotView pilot_;
  int enables_ = 0, takes_ = 0;
  bool permit_ = true, take_ok_ = true;
  ui::LightMode light_ = ui::LightMode::Day;
  std::vector<std::string> names_;
  int step_ = 0, checks_ = 0, commands_ = 0;
  bool failed_ = false;
};
} // namespace
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc, char **argv) { return wxEntry(argc, argv); }

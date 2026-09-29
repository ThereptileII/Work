// Dedicated offline widgets. Never installed or linked into the product.
#include "integration/RouteProgressInput.h"
#include "smartnav/VesselEnergy.h"
#include "ui/AnchorDrawer.h"
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
    frame_ = new wxFrame(nullptr, wxID_ANY, "TEST ONLY - Anchor component",
                         {0, 0}, {1280, 800}, wxBORDER_NONE);
    frame_->SetClientSize(1280, 800);
    frame_->SetBackgroundStyle(wxBG_STYLE_PAINT);
    frame_->Bind(wxEVT_PAINT, [this](wxPaintEvent &) {
      wxAutoBufferedPaintDC dc(frame_);
      ui::XNavPainter p(*frame_, dc, light_);
      dc.SetBackground(wxBrush(ui::Colour(p.c.background)));
      dc.Clear();
      p.Text("OFFLINE COMPONENT TEST", 80, 80, 22, p.c.attention);
      p.Text("Synthetic watch / no chart, network or equipment", 80, 126, 12,
             p.c.secondary);
    });
    application::NavigationActions actions;
    actions.start_anchor = [this](double radius) {
      ++commands_;
      last_radius_ = radius;
      return application::CommandResult{true, "test only", {}};
    };
    actions.clear_anchor = [this](const std::string &) {
      ++commands_;
      return application::CommandResult{true, "test only", {}};
    };
    drawer_ = new ui::XNavAnchorDrawer(*frame_, std::move(actions));
    const auto sample = [](double n) {
      return vessel::Sample{n, "TEST ONLY", stamp, vessel::Validity::Measured};
    };
    state_.navigation.latitude_deg = sample(58.);
    state_.navigation.longitude_deg = sample(16.);
    state_.environment.depth_below_transducer_m = sample(5.8);
    state_.wind.true_speed_kn = sample(14.3);
    state_.battery.soc_percent = sample(68);
    watch_.waypoint_id = "test-watch";
    watch_.source = "test integration";
    watch_.anchor = application::Coordinate{58., 16.};
    watch_.radius_m = 50;
    watch_.observed_at = stamp;
    watch_.state = "OpenCPN watch / test input";
    watch_.distance_m = sample(18.);
    watch_.vessel_position = application::AnchorFix{
        {58., 16.}, stamp, 12., std::sqrt(180.), "TEST ONLY"};
    for (int i = 0; i < 20; ++i)
      watch_.recent_positions.push_back({{58., 16.},
                                         stamp - (20 - i) * 1s,
                                         12. * std::sin(i * .15),
                                         14. * std::cos(i * .15)});
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
    drawer_->Update(watch_, state_, now, light_);
    drawer_->Present(wxRect(80, 68, 1014, 698));
    frame_->Refresh(false);
  }
  void Capture(const char *name) {
    Check(drawer_->GetScreenRect() == wxRect(682, 80, 398, 674),
          "canonical anchor sheet geometry");
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
        Check(drawer_->View().distance_m == 18.,
              "current owned watch distance");
        Check(!Find<ui::XNavRange>("Alarm radius")->IsEnabled(),
              "active watch radius cannot be silently edited");
        Capture("anchor-day");
        light_ = ui::LightMode::Dusk;
        Feed();
        break;
      case 3:
        Capture("anchor-dusk");
        light_ = ui::LightMode::Night;
        Feed();
        break;
      case 4:
        Capture("anchor-night");
        light_ = ui::LightMode::Day;
        watch_.alarm = true;
        Feed(stamp + 6s);
        break;
      case 5:
        Check(drawer_->View().alarm && !drawer_->View().distance_m &&
                  !drawer_->View().vessel_position,
              "stale GPS withholds position but retains upstream alarm");
        Capture("anchor-stale-day");
        watch_ = {};
        Feed();
        break;
      case 6: {
        Check(!drawer_->View().active && drawer_->View().can_start,
              "inactive watch can be armed with current GPS");
        auto *range = Find<ui::XNavRange>("Alarm radius");
        Check(range && range->IsEnabled(), "inactive radius editable");
        wxKeyEvent key(wxEVT_KEY_DOWN);
        key.m_keyCode = WXK_RIGHT;
        range->GetEventHandler()->ProcessEvent(key);
        Check(range->GetValue() == 55,
              "keyboard changes radius by prototype step");
        Check(commands_ == 0, "changing planned radius does not issue command");
        Capture("anchor-inactive-day");
        auto *button = Button("Set anchor & start watch");
        Check(button && button->IsEnabled(), "explicit set action available");
        wxCommandEvent click(wxEVT_BUTTON, button->GetId());
        click.SetEventObject(button);
        button->GetEventHandler()->ProcessEvent(click);
        break;
      }
      case 7: {
        auto *dialog = dynamic_cast<wxDialog *>(
            wxWindow::FindWindowByName("Set anchor watch"));
        Check(dialog && dialog->IsModal(),
              "anchor command requires confirmation");
        Check(commands_ == 0, "no command before confirmation");
        dialog->EndModal(wxID_CANCEL);
        break;
      }
      case 8:
        Check(commands_ == 0, "cancel leaves watch unchanged");
        state_.replayed = true;
        Feed();
        break;
      case 9:
        Check(!Button("Set anchor & start watch")->IsEnabled(),
              "replay cannot arm watch");
        Check(!Find<ui::XNavRange>("Alarm radius")->IsEnabled(),
              "replay radius unavailable");
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
  ui::XNavAnchorDrawer *drawer_ = nullptr;
  wxTimer timer_;
  vessel::VesselState state_;
  application::AnchorState watch_;
  double last_radius_ = 0;
  ui::LightMode light_ = ui::LightMode::Day;
  std::vector<std::string> names_;
  int step_ = 0, checks_ = 0, commands_ = 0;
  bool failed_ = false;
};
} // namespace
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc, char **argv) { return wxEntry(argc, argv); }

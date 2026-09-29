// Dedicated offline widgets. Never installed or linked into the product.
#include "ui/AlertDrawer.h"
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
    frame_ = new wxFrame(nullptr, wxID_ANY, "TEST ONLY - Alerts component",
                         {0, 0}, {1280, 800}, wxBORDER_NONE);
    frame_->SetClientSize(1280, 800);
    frame_->SetBackgroundStyle(wxBG_STYLE_PAINT);
    frame_->Bind(wxEVT_PAINT, [this](wxPaintEvent &) {
      wxAutoBufferedPaintDC dc(frame_);
      ui::XNavPainter p(*frame_, dc, light_);
      dc.SetBackground(wxBrush(ui::Colour(p.c.background)));
      dc.Clear();
      p.Text("OFFLINE COMPONENT TEST", 80, 80, 22, p.c.attention);
      p.Text("Fake alerts / no chart, network or equipment", 80, 126, 12,
             p.c.secondary);
    });
    ui::AlertDrawerActions actions;
    actions.inspect = [this](application::AlertArea area) {
      ++inspected_;
      last_area_ = area;
    };
    actions.acknowledge = [this](const std::string &id, std::uint64_t episode) {
      ++requested_;
      last_episode_ = episode;
      last_id_ = id;
      if (id == alert_.id && episode == alert_.episode)
        alert_.acknowledged = true;
    };
    drawer_ = new ui::XNavAlertDrawer(*frame_, std::move(actions));
    alert_ = {
        "test-gps",
        "Position unavailable or stale",
        "Check the GPS source. Position and route predictions are unavailable.",
        "OFFLINE TEST ONLY",
        application::AlertLevel::Critical,
        application::AlertArea::Sources,
        stamp,
        1,
        false};
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
  void Feed() {
    drawer_->Present(wxRect(80, 68, 1014, 698));
    drawer_->Update(empty_ ? std::vector<application::Alert>{}
                           : std::vector<application::Alert>{alert_},
                    replay_, light_);
    frame_->Refresh(false);
  }
  void Capture(const char *name) {
    Check(drawer_->GetScreenRect() == wxRect(682, 80, 398, 674),
          "canonical alert sheet geometry");
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
    wxCommandEvent click(wxEVT_BUTTON, button->GetId());
    click.SetEventObject(button);
    button->GetEventHandler()->ProcessEvent(click);
  }
  void Step(wxTimerEvent &) {
    try {
      switch (step_++) {
      case 0:
        Feed();
        break;
      case 1:
        Check(requested_ == 0 && inspected_ == 0,
              "paint never acknowledges or inspects");
        Check(Button("Acknowledge")->IsEnabled(),
              "new critical episode can be acknowledged");
        Check(Button("View source health")->GetSize().y == 48,
              "touch inspect target");
        Capture("alerts-day");
        light_ = ui::LightMode::Dusk;
        Feed();
        break;
      case 2:
        Capture("alerts-dusk");
        light_ = ui::LightMode::Night;
        Feed();
        break;
      case 3:
        Capture("alerts-night");
        light_ = ui::LightMode::Day;
        Feed();
        Click("Acknowledge");
        Click("Acknowledge");
        break;
      case 4:
        Check(requested_ == 1, "queued repeated tap only acknowledges once");
        Check(alert_.acknowledged, "original episode acknowledged");
        Check(last_id_ == "test-gps" && last_episode_ == 1,
              "original identity delivered");
        Feed();
        break;
      case 5:
        Check(!Button("Acknowledge")->IsEnabled(),
              "acknowledged action disabled");
        Check(Find<wxPanel>("Alert Position unavailable or stale") != nullptr,
              "critical condition still displayed");
        Capture("alerts-acknowledged-day");
        Click("View source health");
        break;
      case 6:
        Check(inspected_ == 1 && last_area_ == application::AlertArea::Sources,
              "inspect preserves owning source");
        empty_ = true;
        Feed();
        break;
      case 7:
        Check(Button("Acknowledge") == nullptr, "recovered condition removed");
        Check(Find<wxPanel>("No current alerts") != nullptr,
              "honest empty state");
        Capture("alerts-empty-day");
        empty_ = false;
        alert_.episode = 2;
        alert_.acknowledged = false;
        Feed();
        Click("Acknowledge");
        alert_.episode = 3;
        Feed();
        break;
      case 8:
        Check(requested_ == 2 && last_episode_ == 2,
              "delayed request retains old episode");
        Check(!alert_.acknowledged,
              "delayed request cannot acknowledge recurrence");
        Check(Button("Acknowledge")->IsEnabled(),
              "new recurrence remains actionable");
        Capture("alerts-recurrence-day");
        replay_ = true;
        Feed();
        break;
      case 9:
        Check(!Button("Acknowledge")->IsEnabled(),
              "historical condition cannot acknowledge live state");
        Check(!Button("View source health")->IsEnabled(),
              "historical source cannot impersonate live condition");
        Capture("alerts-replay-day");
        replay_ = false;
        alert_.area = application::AlertArea::Ais;
        Feed();
        Click("View traffic");
        break;
      case 10:
        Check(inspected_ == 2 && last_area_ == application::AlertArea::Ais,
              "AIS inspect uses copied area");
        Click("Close");
        Check(!drawer_->IsShown(),
              "close hides sheet without clearing condition");
        Check(!alert_.acknowledged, "dismissal never acknowledges");
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
  ui::XNavAlertDrawer *drawer_ = nullptr;
  wxTimer timer_;
  application::Alert alert_;
  application::AlertArea last_area_ = application::AlertArea::Sources;
  ui::LightMode light_ = ui::LightMode::Day;
  std::vector<std::string> names_;
  std::string last_id_;
  std::uint64_t last_episode_ = 0;
  int step_ = 0, checks_ = 0, requested_ = 0, inspected_ = 0;
  bool failed_ = false, replay_ = false, empty_ = false;
};
} // namespace
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc, char **argv) { return wxEntry(argc, argv); }

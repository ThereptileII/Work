// Dedicated offline widgets. Never installed or linked into the product.
#include "integration/RouteProgressInput.h"
#include "smartnav/VesselEnergy.h"
#include "ui/PassageDrawer.h"
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <wx/app.h>
#include <wx/dcbuffer.h>
#include <wx/dcscreen.h>
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
    frame_ = new wxFrame(nullptr, wxID_ANY, "TEST ONLY - Passage component",
                         {0, 0}, {1280, 800}, wxBORDER_NONE);
    frame_->SetClientSize(1280, 800);
    frame_->SetBackgroundStyle(wxBG_STYLE_PAINT);
    frame_->Bind(wxEVT_PAINT, [this](wxPaintEvent &) {
      wxAutoBufferedPaintDC dc(frame_);
      ui::XNavPainter p(*frame_, dc, light_);
      dc.SetBackground(wxBrush(ui::Colour(p.c.background)));
      dc.Clear();
      p.Text("OFFLINE COMPONENT TEST", 80, 80, 22, p.c.attention);
      p.Text("Synthetic route / no chart, network or equipment", 80, 126, 12,
             p.c.secondary);
    });
    drawer_ = new ui::XNavPassageDrawer(*frame_, {});
    drawer_->on_library = [this] { ++library_; };
    drawer_->on_plot = [this] { ++plot_; };
    auto sample = [](double n) {
      return vessel::Sample{n, "TEST ONLY", stamp, vessel::Validity::Measured};
    };
    integration::RouteRead r;
    r.route.active = true;
    r.route.id = "fixture";
    r.route.name = "Fixture passage";
    r.route.active_index = 0;
    r.route.active_point_id = "a";
    r.route.points = {{"a", 58, 16.1, 0, "First waypoint", {}},
                      {"b", 58.1, 16.2, 5.1, "Fairway", 9},
                      {"c", 58.2, 16.3, 6.6, "Approach", 76},
                      {"d", 58.3, 16.4, 5.8, "Destination", 76}};
    r.position.latitude_deg = sample(58);
    r.position.longitude_deg = sample(16);
    r.upstream_position_valid = true;
    r.upstream_latitude_deg = 58;
    r.upstream_longitude_deg = 16;
    r.range_to_active_nm = .7;
    r.bearing_to_active_true_deg = 43;
    integration::RouteProgressInput input("offline fixture");
    input.Complete(r, r, stamp);
    state_.navigation = r.position;
    state_.navigation.route = input.Current();
    state_.navigation.sog_kn = sample(6.3);
    state_.navigation.cog_deg = sample(43);
    state_.battery.soc_percent = sample(68);
    state_.battery.net_discharge_kw = sample(2.15);
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
    auto energy =
        smartnav::PredictVesselEnergy({25, 25, .5, "test only"}, state_, now);
    auto advice = smartnav::Advise(state_, energy, {}, now);
    drawer_->Update(state_, advice, energy, now, light_,
                    wxDateTime(28, wxDateTime::Sep, 2026, 11, 49, 0));
    drawer_->Present(wxRect(80, 68, 1014, 698));
    frame_->Refresh(false);
  }
  void Capture(const char *name) {
    Check(drawer_->GetScreenRect() == wxRect(682, 80, 398, 674),
          "canonical passage sheet geometry");
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
        Check(drawer_->View().points.size() == 4, "owned route points shown");
        Check(drawer_->View().distance_nm &&
                  std::abs(*drawer_->View().distance_nm - 18.2) < 1e-8,
              "route total preserved");
        Capture("passage-day");
        light_ = ui::LightMode::Dusk;
        Feed();
        break;
      case 3:
        Capture("passage-dusk");
        light_ = ui::LightMode::Night;
        Feed();
        break;
      case 4:
        Capture("passage-night");
        light_ = ui::LightMode::Day;
        Feed(stamp + 20s);
        break;
      case 5:
        Check(!drawer_->View().distance_nm && !drawer_->View().arrival_soc,
              "stale dependencies hidden");
        Capture("passage-stale-day");
        state_.navigation.route.reset();
        Feed();
        break;
      case 6:
        Check(!drawer_->View().active && !drawer_->View().distance_nm,
              "deleted route never zero");
        Capture("passage-inactive-day");
        drawer_->Dismiss();
        break;
      case 7:
        Check(!drawer_->IsShown(), "close restores owner");
        Check(!library_ && !plot_, "reading never issues navigation actions");
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
  ui::XNavPassageDrawer *drawer_ = nullptr;
  wxTimer timer_;
  vessel::VesselState state_;
  ui::LightMode light_ = ui::LightMode::Day;
  std::vector<std::string> names_;
  int step_ = 0, checks_ = 0, library_ = 0, plot_ = 0;
  bool failed_ = false;
};
} // namespace
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc, char **argv) { return wxEntry(argc, argv); }

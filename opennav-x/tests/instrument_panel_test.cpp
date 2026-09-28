// Non-installed, offline native widget process. No profile, network or devices.
#include "application/Settings.h"
#include "ui/InstrumentPanel.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <wx/app.h>
#include <wx/dcbuffer.h>
#include <wx/dcscreen.h>
#include <wx/filename.h>
#include <wx/frame.h>
#include <wx/log.h>
#include <wx/sizer.h>
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
    frame_ = new wxFrame(nullptr, wxID_ANY, "TEST ONLY - Instruments", {0, 0},
                         {1280, 800}, wxBORDER_NONE);
    frame_->SetClientSize(1280, 800);
    auto *host = new wxPanel(frame_, wxID_ANY);
    // wxFrame's implicit single-child sizing differs by backend. Give the
    // offline host an explicit layout so children cannot paint outside a tiny
    // default native panel while still reporting the expected screen origin.
    auto *frame_sizer = new wxBoxSizer(wxVERTICAL);
    frame_sizer->Add(host, 1, wxEXPAND);
    frame_->SetSizer(frame_sizer);
    frame_->Layout();
    host->SetBackgroundStyle(wxBG_STYLE_PAINT);
    host->Bind(wxEVT_PAINT, [this, host](wxPaintEvent &) {
      wxAutoBufferedPaintDC dc(host);
      ui::XNavPainter p(*host, dc, light_);
      dc.SetBackground(wxBrush(ui::Colour(p.c.background)));
      dc.Clear();
      p.Text("OFFLINE COMPONENT TEST / No chart, network, profile or equipment",
             80, 20, 16, p.c.attention);
      p.Text("HORIZON RESERVED / All readings in this separate executable are "
             "synthetic",
             100, 680, 14, p.c.attention);
    });
    scroll_ = new ui::XNavScroll(host);
    scroll_->SetSize(80, 68, 1014, 566);
    panel_ = new ui::XNavInstrumentPanel(scroll_);
    panel_->on_close = [this] { ++closed_; };
    panel_->on_rail = [this] { ++rail_; };
    panel_->on_health = [this] { ++health_; };
    panel_->on_configure = [this] { ++configured_; };
    auto *sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(panel_, 0, wxEXPAND);
    scroll_->SetSizer(sizer);
    auto sample = [](double n) {
      return vessel::Sample{n, "TEST ONLY", stamp, vessel::Validity::Measured};
    };
    state_.navigation.heading_true_deg = sample(41);
    state_.navigation.cog_deg = sample(43);
    state_.navigation.sog_kn = sample(6.3);
    state_.navigation.stw_kn = sample(6.1);
    state_.wind.true_speed_kn = sample(14.3);
    state_.wind.true_angle_deg = sample(-72);
    state_.wind.apparent_speed_kn = sample(17.1);
    state_.wind.apparent_angle_deg = sample(-54);
    state_.environment.depth_below_transducer_m = sample(8.4);
    state_.environment.water_temperature_c = sample(16.8);
    state_.rudder.heel_deg = sample(8.2);
    frame_->Show();
    timer_.SetOwner(this);
    Bind(wxEVT_TIMER, &TestApp::Step, this);
    timer_.Start(350);
    return true;
  }
  int OnRun() override {
    wxApp::OnRun();
    return failed_ ? 1 : 0;
  }

private:
  void Check(bool ok, const char *message) {
    ++checks_;
    if (!ok)
      throw std::runtime_error(message);
  }
  void Feed(vessel::Time now = stamp) {
    panel_->Update(state_, application::Settings{}.instruments, now, light_);
    scroll_->SetBackgroundColour(ui::Colour(ui::Theme(light_).background));
    scroll_->Layout();
    scroll_->FitInside();
    frame_->Refresh(false);
  }
  void Capture(const char *name) {
    Check(scroll_->GetScreenRect() == wxRect(80, 68, 1014, 566),
          "full-view preserves horizon space");
    Check(frame_->GetClientSize() == wxSize(1280, 800),
          "canonical capture size");
    Check(scroll_->GetParent()->GetClientSize() == frame_->GetClientSize() &&
              scroll_->GetParent()->GetScreenRect().Contains(scroll_->GetScreenRect()),
          "offline host fully contains the actual painted viewport");
    const auto origin = frame_->ClientToScreen({0, 0});
#ifdef __WXGTK__
    auto *pixels = gdk_pixbuf_get_from_window(gdk_get_default_root_window(),
                                              origin.x, origin.y, 1280, 800);
    Check(pixels != nullptr, "actual screen pixels");
    const bool saved =
        gdk_pixbuf_save(pixels, (output_ + "/" + name + ".png").utf8_str(),
                        "png", nullptr, nullptr);
    g_object_unref(pixels);
    Check(saved, "write capture");
#else
    wxScreenDC screen;
    wxBitmap bitmap(1280, 800);
    wxMemoryDC memory(bitmap);
    Check(memory.Blit(0, 0, 1280, 800, &screen, origin.x, origin.y),
          "actual screen pixels");
    memory.SelectObject(wxNullBitmap);
    Check(bitmap.SaveFile(output_ + "/" + name + ".png", wxBITMAP_TYPE_PNG),
          "write capture");
#endif
    names_.push_back(name);
  }
  void Click(const char *name) {
    for (auto *b : panel_->Controls())
      if (b->GetLabel() == name) {
        Check(b->IsEnabled(), "action enabled only with callback");
        wxCommandEvent e(wxEVT_BUTTON, b->GetId());
        e.SetEventObject(b);
        b->GetEventHandler()->ProcessEvent(e);
        return;
      }
    throw std::runtime_error("missing action");
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
      case 2: {
        const auto regions = panel_->Regions();
        Check(regions[0].second == wxRect(32, 146, 488, 540),
              "wind card matches exact prototype geometry");
        Check(regions[1].second == wxRect(538, 146, 216, 126),
              "first numeric tile geometry");
        Check(regions[2].second == wxRect(766, 146, 216, 126),
              "second numeric tile geometry");
        Check(panel_->View().wind_bearing_true_deg == 329,
              "heading plus signed relative angle");
        Capture("instruments-day");
        light_ = ui::LightMode::Dusk;
        Feed();
        break;
      }
      case 3:
        Capture("instruments-dusk");
        light_ = ui::LightMode::Night;
        Feed();
        break;
      case 4:
        Capture("instruments-night");
        light_ = ui::LightMode::Day;
        Feed(stamp + 6s);
        break;
      case 5:
        Check(!panel_->View().heading.value &&
                  !panel_->View().wind_bearing_true_deg,
              "stale direction withheld");
        Check(panel_->View().heading.quality == vessel::Quality::Stale,
              "stale quality retained");
        Capture("instruments-stale-day");
        state_.navigation.heading_true_deg = {};
        Feed();
        break;
      case 6:
        Check(!panel_->View().heading.value &&
                  !panel_->View().wind_bearing_true_deg &&
                  panel_->View().true_angle.value == -72,
              "missing heading is not replaced by COG");
        Capture("instruments-heading-unavailable-day");
        Check(scroll_->CanScroll(1), "lower configured readings reachable");
        scroll_->Scroll(0, 30);
        break;
      case 7:
        Check(scroll_->GetViewStart().y > 0, "native scrolling moves body");
        Click("Configure data rail");
        Click("Inspect source quality");
        Click("Configure instruments");
        break;
      case 8:
        Check(rail_ == 1 && health_ == 1 && configured_ == 1,
              "all settings callbacks routed once");
        scroll_->Scroll(0, 0);
        Click("Close");
        break;
      case 9:
        Check(closed_ == 1, "close routes once");
        Check(scroll_->GetViewStart().y == 0, "return to top");
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
    frame_->Destroy();
    ExitMainLoop();
  }
  wxString output_;
  wxFrame *frame_ = nullptr;
  ui::XNavScroll *scroll_ = nullptr;
  ui::XNavInstrumentPanel *panel_ = nullptr;
  wxTimer timer_;
  vessel::VesselState state_;
  ui::LightMode light_ = ui::LightMode::Day;
  std::vector<std::string> names_;
  int step_ = 0, checks_ = 0, closed_ = 0, rail_ = 0, health_ = 0,
      configured_ = 0;
  bool failed_ = false;
};
} // namespace
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc, char **argv) { return wxEntry(argc, argv); }

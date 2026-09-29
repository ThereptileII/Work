// Dedicated offline widgets. Never installed or linked into the product.
#include "ui/HealthDrawer.h"
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
    frame_ = new wxFrame(nullptr, wxID_ANY, "TEST ONLY - Health component",
                         {0, 0}, {1280, 800}, wxBORDER_NONE);
    frame_->SetClientSize(1280, 800);
    frame_->SetBackgroundStyle(wxBG_STYLE_PAINT);
    frame_->Bind(wxEVT_PAINT, [this](wxPaintEvent &) {
      wxAutoBufferedPaintDC dc(frame_);
      ui::XNavPainter p(*frame_, dc, light_);
      dc.SetBackground(wxBrush(ui::Colour(p.c.background)));
      dc.Clear();
      p.Text("OFFLINE COMPONENT TEST", 80, 80, 22, p.c.attention);
      p.Text("Fake source reports / no chart, network or equipment", 80, 126, 12,
             p.c.secondary);
    });
    ui::HealthDrawerActions actions;
    actions.configure = [this](const application::HealthSignal &signal) {
      ++configured_; last_id_ = signal.id;
    };
    actions.manage = [this] { ++managed_; };
    actions.diagnostics = [this] { ++exported_; };
    drawer_ = new ui::XNavHealthDrawer(*frame_, std::move(actions));
    vessel_.navigation.latitude_deg = {57., "OFFLINE TEST GPS", stamp, vessel::Validity::Measured};
    vessel_.navigation.longitude_deg = {16., "OFFLINE TEST GPS", stamp, vessel::Validity::Measured};
    vessel_.environment.depth_below_transducer_m = {8.4, "OFFLINE TEST N2K / PGN 128267", stamp, vessel::Validity::Measured};
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
    drawer_->Update(application::PresentSourceHealth(vessel_, {}, {}, {}, {}, now_),light_);
    frame_->Refresh(false);
  }
  void Capture(const char *name) {
    Check(drawer_->GetScreenRect() == wxRect(682, 80, 398, 674),
          "canonical health sheet geometry");
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
      case 0: Feed(); break;
      case 1:
        Check(configured_==0 && exported_==0,"display cannot configure or export automatically");
        Check(Button("GPS")->GetSize().y==69,"prototype 69px disclosure target");
        Check(Button("Depth")->GetSize().y==70,"fractional prototype row height retained");
        {
          const char *labels[]={"GPS","Heading","Depth","Wind","Motor","Battery","Rudder"};
          const int positions[]={236,313,390,468,545,622,699};
          for(int i=0;i<7;++i)
            Check(Button(labels[i])->GetScreenPosition()==wxPoint(705,positions[i]),
                  "canonical cumulative disclosure positions");
        }
        Check(!Find<wxPanel>("Source detail gps")->IsShown(),"details initially collapsed");
        Capture("health-day"); light_=ui::LightMode::Dusk;Feed();break;
      case 2: Capture("health-dusk");light_=ui::LightMode::Night;Feed();break;
      case 3: Capture("health-night");light_=ui::LightMode::Day;Feed();Click("GPS");break;
      case 4:
        Check(Find<wxPanel>("Source detail gps")->IsShown(),"disclosure expands selected signal");
        Check(!Find<wxPanel>("Source detail depth")->IsShown(),"opening GPS does not expand another signal");
        Capture("health-gps-day");
        Click("Configure this sensor");break;
      case 5:
        Check(configured_==1 && last_id_=="gps","configuration retains copied signal identity");
        now_=stamp+6s;Feed();break;
      case 6:
        Check(Find<wxPanel>("Source detail gps")->IsShown(),"age update preserves disclosure state");
        Capture("health-stale-day");
        vessel_.replayed=true;Feed();break;
      case 7:
        Check(!Button("Configure this sensor")->IsEnabled(),"historical data cannot configure live source");
        Check(!Button("Manage all sensors")->IsEnabled(),"historical source setup disabled");
        Check(Button("Export diagnostics")->IsEnabled(),"historical diagnostics available");
        {
          // A queued event from before replay must also fail closed, even
          // though ordinary pointer/keyboard dispatch is now disabled.
          auto *button=Button("Configure this sensor");
          wxCommandEvent queued(wxEVT_BUTTON,button->GetId());
          queued.SetEventObject(button);
          button->GetEventHandler()->ProcessEvent(queued);
        }
        break;
      case 8:
        Check(configured_==1,"queued disabled configuration refused");
        Capture("health-replay-day");
        Click("GPS");break;
      case 9:
        Check(!Find<wxPanel>("Source detail gps")->IsShown(),"disclosure collapses while historical");
        vessel_.replayed=false;now_=stamp;Feed();
        Click("Close");Check(!drawer_->IsShown(),"close returns to chart");Finish();break;
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
  ui::XNavHealthDrawer *drawer_ = nullptr;
  wxTimer timer_;
  vessel::VesselState vessel_;
  vessel::Time now_=stamp;
  ui::LightMode light_ = ui::LightMode::Day;
  std::vector<std::string> names_;
  std::string last_id_;
  int step_=0,checks_=0,configured_=0,managed_=0,exported_=0;
  bool failed_=false;

};
} // namespace
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc, char **argv) { return wxEntry(argc, argv); }

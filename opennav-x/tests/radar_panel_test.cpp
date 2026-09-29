// Non-installed, offline native widget process. No profile, network or devices.
#include "ui/RadarPanel.h"
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
    frame_ = new wxFrame(nullptr, wxID_ANY, "TEST ONLY - Radar status", {0, 0},
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
    });
    scroll_ = new ui::XNavScroll(host);
    scroll_->SetSize(80, 68, 1014, 698);
    panel_ = new ui::XNavRadarPanel(scroll_);
    panel_->on_close = [this] { ++closed_; };
    panel_->on_plugins = [this] { ++plugins_; };
    auto *sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(panel_, 0, wxEXPAND);
    scroll_->SetSizer(sizer);
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
  void Feed() {
    panel_->Update(state_, now_, replay_, light_);
    scroll_->Layout();
    scroll_->FitInside();
    frame_->Refresh(false);
  }
  void Capture(const char *name) {
    Check(scroll_->GetScreenRect() == wxRect(80, 68, 1014, 698),
          "radar full view occupies exact workspace");
    Check(frame_->GetClientSize() == wxSize(1280, 800),
          "canonical capture size");
    Check(scroll_->GetParent()->GetClientSize() == frame_->GetClientSize() &&
              scroll_->GetParent()->GetScreenRect().Contains(
                  scroll_->GetScreenRect()),
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
  void CheckControls() {
    for (auto *b : panel_->Controls())
      if (b->GetLabel() == "Radar active" || b->GetLabel() == "Guard zone" ||
          b->GetLabel() == "Pause sweep") {
        Check(!b->IsEnabled(), "unverified scanner command unavailable");
        Check(!b->IsSelected(), "no invented activation or guard state");
      }
  }
  void Step(wxTimerEvent &) {
    try {
      switch (step_++) {
      case 0:
        scroll_->SetSize(80, 68, 944, 698);
        Feed();
        break;
      case 1:
        scroll_->SetSize(80, 68, 1014, 698);
        Feed();
        break;
      case 2:
        Check(panel_->Regions()[0].second == wxRect(32, 144, 657, 508),
              "exact radar display geometry");
        Check(panel_->Regions()[1].second == wxRect(717, 144, 265, 508),
              "exact radar control column");
        CheckControls();
        Check(!scroll_->CanScroll(1),
              "primary radar surface fits without whole-page scrolling");
        Capture("radar-day");
        light_ = ui::LightMode::Dusk;
        Feed();
        break;
      case 3:
        Capture("radar-dusk");
        light_ = ui::LightMode::Night;
        Feed();
        break;
      case 4:
        Capture("radar-night");
        light_ = ui::LightMode::Day;
        state_.available = true;
        state_.source = "OFFLINE CAPABILITY TEST";
        state_.observed_at = stamp;
        state_.capabilities = {true, true, true, true, true};
        state_.presentation = adapters::RadarPresentation::Focus;
        Feed();
        break;
      case 5:
        CheckControls();
        Capture("radar-status-only-day");
        now_ = stamp + 4s;
        Feed();
        break;
      case 6:
        CheckControls();
        Capture("radar-stale-day");
        replay_ = true;
        Feed();
        break;
      case 7:
        CheckControls();
        Check(!panel_->Controls().back()->IsEnabled(),
              "historical source cannot open live plugin interface");
        Capture("radar-replay-day");
        replay_ = false;
        Feed();
        // Only the right-hand column scrolls to advanced plugin/status detail.
        for (auto *child : panel_->GetChildren())
          if (auto *column = dynamic_cast<ui::XNavScroll *>(child)) {
            Check(column->CanScroll(1), "advanced controls have their own scroll");
            column->Scroll(0, 1000);
          }
        break;
      case 8:
        Check(panel_->Controls().back()->GetParent()->GetParent()->GetScreenRect()
                  .Contains(panel_->Controls().back()->GetScreenRect()),
              "plugin action becomes fully visible in its column");
        Click("OpenCPN radar plugins");
        break;
      case 9:
        Check(plugins_ == 1, "plugin entry routed to supplied callback only");
        Click("Close");
        break;
      case 10:
        Check(closed_ == 1, "close returns to chart via owner");
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
  ui::XNavRadarPanel *panel_ = nullptr;
  wxTimer timer_;
  adapters::RadarState state_;
  vessel::Time now_ = stamp;
  bool replay_ = false;
  ui::LightMode light_ = ui::LightMode::Day;
  std::vector<std::string> names_;
  int step_ = 0, checks_ = 0, closed_ = 0, plugins_ = 0;
  bool failed_ = false;
};
} // namespace
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc, char **argv) { return wxEntry(argc, argv); }

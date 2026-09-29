// Non-installed offline widget driver. No OpenCPN profile, network or devices.
#include "ui/SettingsDrawer.h"
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
class TestApp final : public wxApp {
public:
  bool OnInit() override {
    wxLog::SetActiveTarget(new wxLogStderr());
    wxSetAssertHandler([](const wxString &, int, const wxString &,
                          const wxString &s, const wxString &) {
      std::cerr << s << std::endl;
      std::abort();
    });
    if (argc != 2) return false;
    output_ = argv[1];
    if (!wxFileName::Mkdir(output_, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL)) return false;
    wxInitAllImageHandlers();
    frame_ = new wxFrame(nullptr, wxID_ANY, "TEST ONLY - Preferences", {0,0},
                         {1280,800}, wxBORDER_NONE);
    frame_->SetClientSize(1280,800);
    auto *host = new wxPanel(frame_,wxID_ANY);
    auto *layout = new wxBoxSizer(wxVERTICAL);
    layout->Add(host,1,wxEXPAND);
    frame_->SetSizer(layout);
    frame_->Layout();
    host->SetBackgroundStyle(wxBG_STYLE_PAINT);
    host->Bind(wxEVT_PAINT,[this,host](wxPaintEvent &) {
      wxAutoBufferedPaintDC dc(host);
      ui::XNavPainter p(*host,dc,light_);
      dc.SetBackground(wxBrush(ui::Colour(p.c.background)));
      dc.Clear();
      p.Text("OFFLINE COMPONENT TEST / No chart, network, profile or equipment",
              80,20,16,p.c.attention);
      p.Text("TEST DATA / All values in this separate executable are synthetic",
              100,772,12,p.c.attention);
    });
    ui::SettingsDrawerActions actions;
    actions.page=[this](ui::ProductPage p){++navigations_;last_page_=p;};
    actions.advanced=[this]{++advanced_;};
    actions.plugins=[this]{++plugins_;};
    actions.fullscreen=[this]{++fullscreen_;};
    actions.diagnostics=[this]{++diagnostics_;};
    actions.theme=[this](ui::LightMode mode){light_=mode;Feed();};
    // No process launch, network, credential, navigation mutation or actuator
    // path exists in this dedicated test executable.
    panel_ = new ui::XNavSettingsDrawer(*frame_,std::move(actions));
    panel_->on_dismiss=[this]{++closed_;};
    frame_->Show();
    timer_.SetOwner(this);
    Bind(wxEVT_TIMER,&TestApp::Step,this);
    timer_.Start(350);
    return true;
  }
  int OnRun() override { wxApp::OnRun(); return failed_?1:0; }
private:
  void Check(bool ok, const char *message) {
    ++checks_;
    if (!ok) throw std::runtime_error(message);
  }
  void Feed() {
    panel_->Update(state_,light_);
    panel_->Present(wxRect(frame_->ClientToScreen({80,68}),wxSize(1014,698)));
    frame_->Refresh(false);
  }
  wxWindow *Find(wxWindow *root,const wxString &label) {
    for(auto *child:root->GetChildren()) {
      if(dynamic_cast<ui::XNavButton *>(child) && child->GetLabel()==label)return child;
      if(auto *nested=Find(child,label))return nested;
    }
    return nullptr;
  }
  void Click(const wxString &label) {
    auto *b=Find(panel_,label);
    Check(b && b->IsShownOnScreen() && b->IsEnabled(),"visible enabled contextual action");
    wxCommandEvent event(wxEVT_BUTTON,b->GetId());event.SetEventObject(b);
    b->GetEventHandler()->ProcessEvent(event);
  }
  void Capture(const char *name) {
    Check(frame_->GetClientSize()==wxSize(1280,800),"canonical native screen");
    Check(panel_->GetScreenRect()==wxRect(frame_->ClientToScreen({648,80}),wxSize(432,674)),"wide preferences geometry preserves chart and rail");
    Check(panel_->GetParent()->GetScreenRect().Contains(panel_->GetScreenRect()),"host contains painted view");
    const auto origin=frame_->ClientToScreen({0,0});
#ifdef __WXGTK__
    auto *pixels=gdk_pixbuf_get_from_window(gdk_get_default_root_window(),origin.x,origin.y,1280,800);
    Check(pixels!=nullptr,"actual screen pixels");
    const bool saved=gdk_pixbuf_save(pixels,(output_+"/"+name+".png").utf8_str(),"png",nullptr,nullptr);
    g_object_unref(pixels);
    Check(saved,"write capture");
#else
    wxScreenDC screen;
    wxBitmap bitmap(1280,800);
    wxMemoryDC memory(bitmap);
    Check(memory.Blit(0,0,1280,800,&screen,origin.x,origin.y),"actual screen pixels");
    memory.SelectObject(wxNullBitmap);
    Check(bitmap.SaveFile(output_+"/"+name+".png",wxBITMAP_TYPE_PNG),"write capture");
#endif
    names_.push_back(name);
  }
  void Step(wxTimerEvent &) {
    try {
      switch(step_++) {
      case 0: Feed(); break;
      case 1:
        for(const auto *tab:{"Vessel","Navigation","Sensors","Autopilot","Radar","Display","System","Help"}) {
          auto *b=Find(panel_,tab);Check(b && b->IsShownOnScreen(),"all eight sections visible");
          Check(panel_->GetScreenRect().Contains(b->GetScreenRect()),"section fits drawer");
        }
        Capture("settings-day");light_=ui::LightMode::Dusk;Feed();break;
      case 2: Capture("settings-dusk");light_=ui::LightMode::Night;Feed();break;
      case 3: Capture("settings-night");light_=ui::LightMode::Day;Feed();Click("Sensors");break;
      case 4: Check(panel_->Section()==ui::SettingsSection::Sensors,"tab switches after event dispatch");
        Capture("sensors-day");light_=ui::LightMode::Dusk;Feed();break;
      case 5: Capture("sensors-dusk");light_=ui::LightMode::Night;Feed();break;
      case 6: Capture("sensors-night");Click("Manage sensors");break;
      case 7: Check(navigations_==1 && last_page_==ui::ProductPage::Sources,"existing source workflow callback only");
        Click("Add a sensor");break;
      case 8: Check(advanced_==1,"connection editing delegates to upstream callback");
        light_=ui::LightMode::Day;Feed();Click("Display");break;
      case 9:
        light_=ui::LightMode::Night;Feed();
        Check(static_cast<ui::XNavButton *>(Find(panel_,"Night"))->IsSelected() &&
              !static_cast<ui::XNavButton *>(Find(panel_,"Day"))->IsSelected(),
              "external light change updates existing display selection");
        light_=ui::LightMode::Day;Feed();
        Check(static_cast<ui::XNavButton *>(Find(panel_,"Day"))->IsSelected() &&
              !static_cast<ui::XNavButton *>(Find(panel_,"Night"))->IsSelected(),
              "external light return restores exclusive selection");
        Capture("display-day");Click("Dusk");break;
      case 10: Check(light_==ui::LightMode::Dusk,"light callback applied");Capture("display-dusk");Click("Night");break;
      case 11: Check(light_==ui::LightMode::Night,"night callback applied");Capture("display-night");Click("Toggle fullscreen");break;
      case 12: Check(fullscreen_==1,"fullscreen invokes one display callback");Click("Personalise instruments");break;
      case 13: Check(navigations_==2 && last_page_==ui::ProductPage::RailLayout,"rail configuration preserved");
        Click("System");break;
      case 14:
        Check(Find(panel_,"Legacy mode") && !Find(panel_,"Legacy mode")->IsEnabled(),"unavailable restart action disabled");
        Check(!Find(panel_,"AUTO") && !Find(panel_,"STBY"),"preferences cannot execute physical controls");
        Capture("system-night");Click("Diagnostics");break;
      case 15: Check(diagnostics_==1,"diagnostics callback once");Click("Radar");break;
      case 16: Capture("settings-radar-night");Click("Autopilot");break;
      case 17: Capture("settings-autopilot-night");Click("Close");break;
      case 18: Check(closed_==1 && !panel_->IsShown(),"close hides only preferences");Finish();break;
      }
    } catch(const std::exception &e) {
      failed_=true;std::cerr<<e.what()<<std::endl;Finish();
    }
  }
  void Finish() {
    timer_.Stop();
    std::ofstream f((output_+"/result.json").ToStdString());
    f<<"{\"passed\":"<<(failed_?"false":"true")<<",\"checks\":"<<checks_<<",\"captures\":[";
    for(std::size_t i=0;i<names_.size();++i) {if(i)f<<',';f<<'"'<<names_[i]<<'"';}
    f<<"]}\n";f.close();frame_->Destroy();ExitMainLoop();
  }
  wxString output_;
  wxFrame *frame_=nullptr;
  ui::XNavSettingsDrawer *panel_=nullptr;
  wxTimer timer_;
  ui::ProductState state_;
  ui::ProductPage last_page_=ui::ProductPage::Home;
  int navigations_=0,advanced_=0,plugins_=0,fullscreen_=0,diagnostics_=0;
  ui::LightMode light_=ui::LightMode::Day;
  std::vector<std::string> names_;
  int step_=0,checks_=0,closed_=0;
  bool failed_=false;
};
} // namespace
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc,char **argv) {return wxEntry(argc,argv);}

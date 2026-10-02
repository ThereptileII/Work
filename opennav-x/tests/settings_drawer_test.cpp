// Non-installed offline widget driver. No OpenCPN profile, network or devices.
#include "ui/SettingsDrawer.h"
#include <array>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <wx/app.h>
#include <wx/dcbuffer.h>
#include <wx/dcscreen.h>
#include <wx/filename.h>
#include <wx/frame.h>
#include <wx/graphics.h>
#include <wx/log.h>
#include <wx/popupwin.h>
#include <wx/sizer.h>
#include <wx/timer.h>
#include <wx/uiaction.h>
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
    actions.settings=[this]{return state_.settings;};
    actions.display=[this]{return display_;};
    actions.save_display=[this](const application::DisplayPreferences &next){
      ++display_saves_;
      Check(next.scale_percent==125 && next.layout==application::ChartLayout::ChartFocus,
            "both display choices submitted together");
      if(display_saves_==1)return application::CommandResult{false,"Synthetic display save failure"};
      display_=next;
      panel_->SetDisplayPreferences(next);
      panel_->Present(wxRect(frame_->ClientToScreen({80,68}),wxSize(1014,698)));
      return application::CommandResult{true,"Display preferences applied"};
    };
    actions.save_vessel=[this](const application::Settings &settings,
                               const std::string &name,double chart_depth){
      ++saves_;
      Check(name=="TEST VESSEL","typed name passed unchanged to save");
      Check(settings.energy.battery.capacity_kwh==42,
            "untouched capacity uses latest settings, not stale form text");
      Check(std::isnan(settings.energy.battery.reserve_soc_percent),
            "unconfigured reserve remains unconfigured");
      Check(std::isnan(chart_depth),"unchanged stock chart depth is not rewritten");
      if(saves_==1)return application::CommandResult{false,"Synthetic save failure"};
      state_.settings=settings;state_.vessel_name=name;
      return application::CommandResult{true,"Synthetic save success"};
    };
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
  wxTextCtrl *FindText(wxWindow *root,const wxString &name) {
    for(auto *child:root->GetChildren()) {
      if(auto *text=dynamic_cast<wxTextCtrl *>(child);text&&text->GetName()==name)return text;
      if(auto *nested=FindText(child,name))return nested;
    }
    return nullptr;
  }
  ui::XNavChoiceField *FindChoice(const wxString &name) {
    const auto find=[&](const auto &self,wxWindow *root)->ui::XNavChoiceField * {
      for(auto *child:root->GetChildren()) {
        if(auto *choice=dynamic_cast<ui::XNavChoiceField *>(child);
           choice && choice->GetName()==name)return choice;
        if(auto *nested=self(self,child))return nested;
      }
      return nullptr;
    };
    return find(find,panel_);
  }
  void Choose(const wxString &name,int index) {
    auto *choice=FindChoice(name);
    Check(choice && choice->IsShownOnScreen(),"visible owner-drawn choice field");
    choice->SetSelection(index);
    wxCommandEvent event(wxEVT_CHOICE,choice->GetId());
    event.SetEventObject(choice);event.SetInt(index);
    choice->GetEventHandler()->ProcessEvent(event);
  }
  void Click(const wxString &label) {
    auto *b=Find(panel_,label);
    Check(b && b->IsShownOnScreen() && b->IsEnabled(),"visible enabled contextual action");
    wxCommandEvent event(wxEVT_BUTTON,b->GetId());event.SetEventObject(b);
    b->GetEventHandler()->ProcessEvent(event);
  }
  void Capture(const char *name) {
    Check(frame_->GetClientSize()==wxSize(1280,800),"canonical native screen");
    const int width=display_.scale_percent==125?460:display_.scale_percent==150?480:432;
    Check(panel_->GetScreenRect()==wxRect(frame_->ClientToScreen({1080-width,80}),wxSize(width,674)),
          "wide preferences geometry follows applied interface scale");
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
        {
          std::ofstream geometry((output_+"/vessel-geometry.json").ToStdString());
          geometry<<"{\"inputs\":[";
          bool first=true;
          // Rounded rectangles from the immutable HTML's Windows capture.json
          // (settings-day, field inputs). Allow 2px for wx/GTK integer layout.
          const std::array<wxRect,5> reference{{{671,318,386,46},{671,414,185,46},
            {872,414,185,46},{671,513,386,46},{671,601,386,46}}};
          std::size_t index=0;
          for(const auto *label:{"Vessel name","Draft · metres","Safety depth · metres",
                                 "Usable battery capacity · kWh","Minimum reserve · %"}) {
            if(!first)geometry<<',';
            first=false;
            const auto rect=FindText(panel_,wxString::FromUTF8(label))->GetParent()->GetScreenRect();
            const auto expected=reference[index++];
            if (std::abs(rect.x-expected.x)>2 || std::abs(rect.y-expected.y)>2 ||
                std::abs(rect.width-expected.width)>2 || rect.height!=expected.height)
              std::cerr << label << " actual " << rect.x << ',' << rect.y << ','
                        << rect.width << ',' << rect.height << " expected " <<
                  expected.x << ',' << expected.y << ',' << expected.width << ',' << expected.height << '\n';
            Check(std::abs(rect.x-expected.x)<=2 && std::abs(rect.y-expected.y)<=2 &&
                  std::abs(rect.width-expected.width)<=2 && rect.height==expected.height,
                  "vessel input geometry follows independent Windows HTML");
            geometry<<"{\"x\":"<<rect.x<<",\"y\":"<<rect.y<<",\"width\":"<<rect.width
                    <<",\"height\":"<<rect.height<<'}';
          }
          const auto save=Find(panel_,"Save vessel profile")->GetScreenRect();
          Check(std::abs(save.x-671)<=2 && std::abs(save.y-673)<=2 &&
                std::abs(save.width-386)<=2 && save.height==48,
                "Save placement follows independent Windows HTML");
          geometry<<"],\"save\":{\"x\":"<<save.x<<",\"y\":"<<save.y
                  <<",\"width\":"<<save.width<<",\"height\":"<<save.height<<"}}\n";
        }
        for(const auto *tab:{"Vessel","Navigation","Sensors","Autopilot","Radar","Display","System","Help"}) {
          auto *b=Find(panel_,tab);Check(b && b->IsShownOnScreen(),"all eight sections visible");
          Check(panel_->GetScreenRect().Contains(b->GetScreenRect()),"section fits drawer");
        }
#ifdef __WXMSW__
        // Independently measured immutable Windows HTML: six tabs on row one.
        Check(Find(panel_,"Display")->GetPosition().y==Find(panel_,"Vessel")->GetPosition().y,
              "Windows Preferences retains six tabs on the first row");
        Check(Find(panel_,"System")->GetPosition().y>Find(panel_,"Display")->GetPosition().y,
              "System starts the second prototype row");
#if wxUSE_GRAPHICS_DIRECT2D
        {
          auto *renderer=wxGraphicsRenderer::GetDirect2DRenderer();
          Check(renderer!=nullptr,"Native tab paint renderer is available");
          std::unique_ptr<wxGraphicsContext> graphics(renderer->CreateMeasuringContext());
          graphics->SetFont(graphics->CreateFont(11.,ui::UiFontWeight(*panel_,11,400).GetFaceName()));
          const std::pair<const char *,double> text_widths[]={{"Vessel",29.640625},{"Navigation",52.90625},
              {"Sensors",37.4375},{"Autopilot",45.46875},{"Radar",28.078125},{"Display",35.109375},
              {"System",34.46875},{"Help",22.703125}};
          for(const auto &expected:text_widths) {
            double width=0,height=0;graphics->GetTextExtent(expected.first,&width,&height);
            Check(std::abs(width-expected.second)<.05,"Painted tab advance matches independent Windows HTML");
          }
        }
#else
        Check(false,"Validated Windows build must provide DirectWrite tab painting");
#endif
#endif
        Capture("settings-day");
        for(const auto *label:{"Vessel name","Draft · metres","Safety depth · metres",
                               "Usable battery capacity · kWh","Minimum reserve · %"})
          Check(FindText(panel_,wxString::FromUTF8(label))!=nullptr,"all five vessel fields visible");
        Check(saves_==0,"typing has no persistence side effect");
        Check(FindText(panel_,"Vessel name")!=nullptr,"prototype vessel name is editable");
        FindText(panel_,"Vessel name")->SetValue("TEST VESSEL");
        state_.settings.energy.battery.capacity_kwh=42;
        light_=ui::LightMode::Dusk;Feed();
        Check(FindText(panel_,"Vessel name")->GetValue()=="TEST VESSEL",
              "draft survives state and theme refresh");
        Check(saves_==0,"refresh has no persistence side effect");
        Click("Save vessel profile");break;
      case 2:
        Check(saves_==1,"first save was attempted");
        Check(FindText(panel_,"Vessel name")->GetValue()=="TEST VESSEL",
              "failed save retains editable draft");
        Capture("settings-dusk");light_=ui::LightMode::Night;Feed();
        Click("Save vessel profile");break;
      case 3:
        Check(saves_==2,"successful retry saved once");
        Capture("settings-night");light_=ui::LightMode::Day;Feed();Click("Sensors");break;
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
        Check(FindChoice("Interface scale") && FindChoice("Chart layout"),
              "both prototype select fields visible");
        {
          auto *choice=FindChoice("Interface scale");
          wxUIActionSimulator input;
          const auto rect=choice->GetScreenRect();
          const wxPoint center(rect.x+rect.width/2,rect.y+rect.height/2);
          Check(input.MouseMove(center) && input.MouseClick(),"activate Display choice with pointer");
          wxTheApp->Yield(true);
          auto popup=[choice]{for(auto *child:wxGetTopLevelParent(choice)->GetChildren())
            if(dynamic_cast<wxPopupTransientWindow *>(child))return child;
            return static_cast<wxWindow *>(nullptr);};
          Check(popup()!=nullptr,"Display choice popup opens from field activation");
          wxKeyEvent escape(wxEVT_CHAR_HOOK);escape.m_keyCode=WXK_ESCAPE;
          escape.SetEventObject(popup());
          popup()->ProcessWindowEvent(escape);
          wxTheApp->Yield(true);
          Check(panel_->IsShown(),"Escape closes choice popup without dismissing settings");
          wxMilliSleep(400); // GTK otherwise coalesces the next activation as a double-click.
          Check(input.MouseMove(center) && input.MouseClick(),"reopen Display choice with pointer");
          wxTheApp->Yield(true);
          Check(popup()!=nullptr,"Display choice popup reopens after Escape");
          wxKeyEvent second_escape(wxEVT_CHAR_HOOK);second_escape.m_keyCode=WXK_ESCAPE;
          second_escape.SetEventObject(popup());
          popup()->ProcessWindowEvent(second_escape);
          wxTheApp->Yield(true);
          Check(popup()==nullptr && panel_->IsShown(),
                "second Escape leaves a closed field and visible settings drawer");
          wxMilliSleep(250);wxTheApp->Yield(true);
        }
        Capture("display-day");
        Choose("Interface scale",1);Choose("Chart layout",1);
        Check(display_saves_==0 && display_.scale_percent==100,
              "select changes remain draft before Apply");
        light_=ui::LightMode::Night;Feed();
        Check(FindChoice("Interface scale")->GetSelection()==1 &&
              FindChoice("Chart layout")->GetSelection()==1,
              "timer and theme refresh preserve uncommitted selection");
        Click("Apply display preferences");break;
      case 10:
        Check(display_saves_==1 && display_.scale_percent==100,
              "failed display save leaves applied scale unchanged");
        Check(panel_->GetScreenRect().width==432 &&
              FindChoice("Interface scale")->GetSelection()==1,
              "failed display save retains editable draft without resizing");
        Click("Apply display preferences");break;
      case 11:
        Check(display_saves_==2 && display_.scale_percent==125 &&
              display_.layout==application::ChartLayout::ChartFocus,
              "successful Apply commits both choices");
        Check(panel_->GetScreenRect().width==460,"successful Apply resizes wide drawer immediately");
        {
          auto *apply=dynamic_cast<ui::XNavButton *>(Find(panel_,"Apply display preferences"));
          Check(apply && apply->GetMinSize().y==panel_->FromDIP(48),
                "125 percent keeps the primary action's base height");
          auto larger=display_;larger.scale_percent=150;
          display_=larger;
          panel_->SetDisplayPreferences(larger);
          panel_->Present(wxRect(frame_->ClientToScreen({80,68}),wxSize(1014,698)));
          Check(apply->GetMinSize().y==panel_->FromDIP(56),
                "150 percent raises a marked primary action to 56 logical pixels");
          Check(FindChoice("Interface scale")->GetMinSize().y==panel_->FromDIP(56),
                "150 percent raises the owner-drawn field to 56 logical pixels");
          wxTheApp->Yield(true);
          Capture("display-150-chart-night");
          display_.scale_percent=125;
          panel_->SetDisplayPreferences(display_);
          panel_->Present(wxRect(frame_->ClientToScreen({80,68}),wxSize(1014,698)));
          wxTheApp->Yield(true);
          Check(apply->GetMinSize().y==panel_->FromDIP(48),
                "returning to 125 percent restores the action's base height");
        }
        Capture("display-applied-125-chart");Click("Dusk");break;
      case 12: Check(light_==ui::LightMode::Dusk,"light callback applied");Capture("display-dusk");Click("Night");break;
      case 13: Check(light_==ui::LightMode::Night,"night callback applied");Capture("display-night");Click("Toggle fullscreen");break;
      case 14: Check(fullscreen_==1,"fullscreen invokes one display callback");Click("Personalise instruments");break;
      case 15: Check(navigations_==2 && last_page_==ui::ProductPage::RailLayout,"rail configuration preserved");
        Click("System");break;
      case 16:
        Check(Find(panel_,"Legacy mode") && !Find(panel_,"Legacy mode")->IsEnabled(),"unavailable restart action disabled");
        Check(!Find(panel_,"AUTO") && !Find(panel_,"STBY"),"preferences cannot execute physical controls");
        Capture("system-night");Click("Diagnostics");break;
      case 17: Check(diagnostics_==1,"diagnostics callback once");Click("Radar");break;
      case 18: Capture("settings-radar-night");Click("Autopilot");break;
      case 19: Capture("settings-autopilot-night");Click("Close");break;
      case 20:
        Check(closed_==1 && !panel_->IsShown(),"close hides only preferences");
        display_.scale_percent=100;
        panel_->SetDisplayPreferences(display_);
        panel_->Select(ui::SettingsSection::Vessel);
        Feed();
        break;
      case 21:
        {
          // Reproduce the retained offset after reaching Advanced battery model.
          // Scroll the real component body; no fixture-only product hooks.
          auto *body=ScrollBody();
          body->Scroll(0,body->GetVirtualSize().y);
          Check(body->GetViewStart().y>0,"Vessel body overflows at canonical size");
          Check(!body->GetScreenRect().Contains(Find(panel_,"Sensors")->GetScreenRect()),
                "scrolled Sensors tab starts outside the body viewport");
          const auto offset=body->GetViewStart();
          light_=ui::LightMode::Dusk;Feed();
          Check(body->GetViewStart()==offset,"live state and theme refresh preserve scroll");
          panel_->Dismiss();
          panel_->Open(wxRect(frame_->ClientToScreen({80,68}),wxSize(1014,698)));
        }
        break;
      case 22:
        {
          auto *body=ScrollBody();
          Check(body->GetViewStart()==wxPoint(0,0),"root Preferences reopening resets body scroll");
          Check(panel_->Section()==ui::SettingsSection::Vessel,"reopening preserves selected section");
          for(const auto *tab:{"Vessel","Navigation","Sensors","Autopilot","Radar","Display","System","Help"}) {
            auto *b=Find(panel_,tab);
            Check(b && b->IsShownOnScreen() && b->IsEnabled() &&
                  body->GetScreenRect().Contains(b->GetScreenRect()),
                  "reopened section fully visible and enabled in body viewport");
          }
          auto *sensor=Find(panel_,"Sensors");
          const auto rect=sensor->GetScreenRect();
          wxUIActionSimulator input;
          Check(input.MouseMove(rect.x+rect.width/2,rect.y+rect.height/2) && input.MouseClick(),
                "reopened Sensors section receives real pointer input");
        }
        break;
      case 23:
        Check(panel_->Section()==ui::SettingsSection::Sensors,"pointer reaches Sensors after root reopen");
        Check(saves_==2 && display_saves_==2,"reopening does not save settings");
        Finish();break;
      }
    } catch(const std::exception &e) {
      failed_=true;std::cerr<<e.what()<<std::endl;Finish();
    }
  }
  ui::XNavScroll *ScrollBody() {
    for(auto *child:panel_->GetChildren())
      if(auto *body=dynamic_cast<ui::XNavScroll *>(child))return body;
    throw std::runtime_error("Preferences scroll body missing");
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
  int saves_=0;
  int display_saves_=0;
  application::DisplayPreferences display_;
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

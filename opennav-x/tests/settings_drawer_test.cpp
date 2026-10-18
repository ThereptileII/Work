// Non-installed offline widget driver. No OpenCPN profile, network or devices.
#include "ui/SettingsDrawer.h"
#include "ui/DisplaySizing.h"
#include <array>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <cstdint>
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
#ifdef __WXMSW__
#include <wx/msw/wrapwin.h>
#endif
#ifdef __WXGTK__
#include <gtk/gtk.h>
#endif
using namespace opennav;
using namespace std::chrono_literals;
namespace {
static_assert(ui::DisplayFieldHeight(125)==58 && ui::DisplayFieldFont(125)==20 &&
              ui::DisplayActionHeight(125,48)==60 && ui::DisplayActionFont(125,12)==15,
              "125 percent display contract");
static_assert(ui::DisplayFieldHeight(150)==69 && ui::DisplayFieldFont(150)==24 &&
              ui::DisplayActionHeight(150,48)==72 && ui::DisplayActionFont(150,12)==18,
              "150 percent display contract");
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
    actions.page=[this](ui::ProductPage p){
      ++navigations_;last_page_=p;
#ifdef __WXMSW__
      if(activation_repro_ && p==ui::ProductPage::EnergySettings) {
        ActivationEvidence("battery-callback");
        // Mirror Shell::ShowProduct: leave the cached drawer and use its owner.
        panel_->Dismiss();
        ::SetForegroundWindow(static_cast<HWND>(frame_->GetHandle()));
      }
#endif
    };
    actions.advanced=[this]{++advanced_;};
    // Sensor connection editing has its own capability in the current drawer.
    actions.connections=actions.advanced;
    actions.plugins=[this]{++plugins_;};
    actions.fullscreen=[this]{++fullscreen_;};
    actions.diagnostics=[this]{++diagnostics_;};
    actions.boat_setup=[this]{++boat_setups_;};
    actions.theme=[this](ui::LightMode mode){light_=mode;Feed();};
    actions.settings=[this]{return state_.settings;};
    actions.display=[this]{return display_;};
    actions.save_display=[this](const application::DisplayPreferences &next){
      ++display_saves_;
      const std::array<application::DisplayPreferences,6> expected{{
        {125,application::ChartLayout::Balanced},
        {125,application::ChartLayout::Balanced},
        {125,application::ChartLayout::ChartFocus},
        {125,application::ChartLayout::ChartFocus},
        {150,application::ChartLayout::ChartFocus},
        {125,application::ChartLayout::ChartFocus}}};
      Check(display_saves_<=static_cast<int>(expected.size()),
            "one persistence attempt per committed display choice");
      const auto &wanted=expected[display_saves_-1];
      Check(next.scale_percent==wanted.scale_percent && next.layout==wanted.layout,
            "each display choice submits the current scale and layout together");
      if(display_saves_==1 || display_saves_==3)
        return application::CommandResult{false,"Synthetic display save failure"};
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
    timer_.StartOnce(350);
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
  void CheckDisplaySizing(int field_height,int field_font,int action_height) {
    const auto *scale=FindChoice("Interface and chart scale"),*layout=FindChoice("Chart layout");
    Check(scale && layout && scale->GetMinSize().y==panel_->FromDIP(field_height) &&
          layout->GetMinSize().y==panel_->FromDIP(field_height),
          "both live choice fields use the applied display height");
    const auto *fullscreen=Find(panel_,"Toggle fullscreen");
    Check(fullscreen && fullscreen->GetMinSize().y==panel_->FromDIP(action_height),
          "existing Display action uses the applied display height");
    const int before=display_saves_;
    panel_->Select(ui::SettingsSection::Vessel);
    for(const auto *label:{"Vessel name","Draft · metres","Safety depth · metres",
                           "Usable battery capacity · kWh","Minimum reserve · %"}) {
      const auto *field=FindText(panel_,wxString::FromUTF8(label));
      Check(field && field->GetParent()->GetMinSize().y==panel_->FromDIP(field_height),
            "all vessel field frames use the applied display height");
      Check(field->GetFont().GetNativeFontInfoDesc()==
                ui::UiFont(*panel_,field_font).GetNativeFontInfoDesc(),
            "all vessel field fonts use the applied display size");
    }
    const auto *save=Find(panel_,"Save vessel profile");
    Check(save && save->GetMinSize().y==panel_->FromDIP(action_height),
          "vessel save action uses the applied display height");
    panel_->Select(ui::SettingsSection::Display);
    Check(display_saves_==before,"rebuilding settings sections does not persist display choices");
  }
  void Capture(const char *name, bool canonical=true) {
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
    if(canonical)names_.push_back(name);
  }
#ifdef __WXMSW__
  static void JsonText(std::ostream &out,const wxString &value) {
    out<<'"';
    for(unsigned char c:value.ToStdString(wxConvUTF8)) {
      if(c=='"' || c=='\\')out<<'\\'<<c;
      else if(c<32) {const char *hex="0123456789abcdef";out<<"\\u00"<<hex[c>>4]<<hex[c&15];}
      else out<<c;
    }
    out<<'"';
  }
  static void NativeWindow(std::ostream &out,HWND window) {
    wchar_t name[256]={},kind[128]={};
    if(window) {::GetWindowTextW(window,name,256);::GetClassNameW(window,kind,128);}
    out<<"{\"hwnd\":"<<reinterpret_cast<std::uintptr_t>(window)<<",\"class\":";
    JsonText(out,kind);out<<",\"caption\":";JsonText(out,name);out<<'}';
  }
  static void Rect(std::ostream &out,const wxRect &r) {
    out<<"{\"x\":"<<r.x<<",\"y\":"<<r.y<<",\"width\":"<<r.width<<",\"height\":"<<r.height<<'}';
  }
  void ActivationEvidence(const char *phase) {
    auto *sensor=Find(panel_,"Sensors");auto *battery=Find(panel_,"Advanced battery model");
    auto *body=ScrollBody();const auto scroll=body->GetViewStart();
    const auto rect=sensor->GetScreenRect();
    const POINT point{rect.x+rect.width/2,rect.y+rect.height/2};
    const POINT cached{sensor_before_.x+sensor_before_.width/2,sensor_before_.y+sensor_before_.height/2};
    int ux=0,uy=0;body->GetScrollPixelsPerUnit(&ux,&uy);
    std::ofstream out((output_+"/activation/evidence.jsonl").ToStdString(),std::ios::app);
    out<<"{\"phase\":\""<<phase<<"\",\"step\":"<<step_-1<<",\"scroll\":["<<scroll.x<<','<<scroll.y
       <<"],\"pixels_per_unit\":["<<ux<<','<<uy<<"],\"focus\":";
    NativeWindow(out,::GetFocus());out<<",\"foreground\":";NativeWindow(out,::GetForegroundWindow());
    out<<",\"drawer\":";NativeWindow(out,static_cast<HWND>(panel_->GetHandle()));
    out<<",\"sensor\":";NativeWindow(out,static_cast<HWND>(sensor->GetHandle()));
    out<<",\"sensor_rect\":";Rect(out,rect);out<<",\"body_rect\":";Rect(out,body->GetScreenRect());
    out<<",\"battery_rect\":";Rect(out,battery?battery->GetScreenRect():wxRect{});
    out<<",\"current_sensor_hit\":";NativeWindow(out,::WindowFromPoint(point));
    out<<",\"cached_sensor_rect\":";Rect(out,sensor_before_);
    out<<",\"cached_sensor_hit\":";NativeWindow(out,::WindowFromPoint(cached));out<<"}\n";
  }
  void ActivateSurface(wxWindow *surface) {
    const auto hwnd=static_cast<HWND>(surface->GetHandle());
    ::SetForegroundWindow(hwnd);
    // Single-shot Step cannot reenter while native activation events are pumped.
    for(int i=0;i<60 && ::GetForegroundWindow()!=hwnd;++i) {
      wxMilliSleep(50);wxTheApp->Yield(true);
    }
    Check(::GetForegroundWindow()==hwnd,"activation repro foreground is the requested surface");
  }
  void NativeClick(wxWindow *target) {
    Check(target && target->IsShownOnScreen() && target->IsEnabled(),"activation repro target visible and enabled");
    const auto rect=target->GetScreenRect();
    Check(ScrollBody()->GetScreenRect().Contains(rect),"activation repro pointer target fully contained");
    const POINT point{rect.x+rect.width/2,rect.y+rect.height/2};
    const auto hwnd=static_cast<HWND>(target->GetHandle());
    Check(::GetForegroundWindow()==static_cast<HWND>(panel_->GetHandle()),"activation repro drawer owns input");
    Check(::WindowFromPoint(point)==hwnd && ::IsWindowEnabled(hwnd),"activation repro exact HWND before pointer move");
    wxUIActionSimulator input;
    Check(input.MouseMove(point.x,point.y),"activation repro real pointer move");
    Check(::WindowFromPoint(point)==hwnd && ::IsWindowEnabled(hwnd),"activation repro exact HWND before mouse-down");
    Check(input.MouseClick(),"activation repro real pointer click");
  }
#endif
  void Step(wxTimerEvent &) {
    if(finished_)return;
    const int current=step_++;
    std::cout<<"Settings step "<<current<<" begin; captures="<<names_.size()
             <<"; display saves="<<display_saves_<<std::endl;
    try {
      switch(current) {
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
        Check(FindChoice("Interface and chart scale") && FindChoice("Chart layout"),
              "both prototype select fields visible");
        Check(!Find(panel_,"Apply display preferences"),
              "display choices apply directly without a separate Apply action");
        {
          auto *choice=FindChoice("Interface and chart scale");
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
        Check(display_saves_==0,"opening and cancelling choice popups does not save");
        Choose("Interface and chart scale",1);
        Check(display_saves_==1 && display_.scale_percent==100 &&
              display_.layout==application::ChartLayout::Balanced,
              "scale selection saves immediately and failed persistence keeps stored values");
        light_=ui::LightMode::Night;Feed();
        Check(FindChoice("Interface and chart scale")->GetSelection()==0 &&
              FindChoice("Chart layout")->GetSelection()==0 && display_saves_==1,
              "failed scale selection rolls back and refresh does not retry persistence");
        break;
      case 10:
        Check(display_saves_==1 && display_.scale_percent==100,
              "failed display save leaves applied scale unchanged");
        Check(panel_->GetScreenRect().width==432 &&
              FindChoice("Interface and chart scale")->GetSelection()==0,
              "failed display save restores the applied selection without resizing");
        Choose("Interface and chart scale",1);
        Check(display_saves_==2 && display_.scale_percent==125 &&
              display_.layout==application::ChartLayout::Balanced &&
              panel_->GetScreenRect().width==460,
              "retrying scale selection persists and resizes immediately");
        Choose("Chart layout",1);
        Check(display_saves_==3 && display_.scale_percent==125 &&
              display_.layout==application::ChartLayout::Balanced &&
              FindChoice("Interface and chart scale")->GetSelection()==1 &&
              FindChoice("Chart layout")->GetSelection()==0,
              "failed layout save restores layout while preserving the applied scale");
        Feed();
        Check(display_saves_==3 && FindChoice("Chart layout")->GetSelection()==0,
              "refresh preserves layout rollback without retrying persistence");
        Choose("Chart layout",1);
        break;
      case 11:
        Check(display_saves_==4 && display_.scale_percent==125 &&
              display_.layout==application::ChartLayout::ChartFocus,
              "successful layout retry commits both current choices immediately");
        Check(panel_->GetScreenRect().width==460,"layout change retains the applied drawer width");
        CheckDisplaySizing(58,20,60);
        {
          Choose("Interface and chart scale",2);
          Check(display_saves_==5 && display_.scale_percent==150 &&
                display_.layout==application::ChartLayout::ChartFocus &&
                panel_->GetScreenRect().width==480,
                "150 percent choice persists immediately and retains chart layout");
          CheckDisplaySizing(69,24,72);
          wxTheApp->Yield(true);
          Capture("display-150-chart-night");
          Choose("Interface and chart scale",1);
          Check(display_saves_==6 && display_.scale_percent==125 &&
                display_.layout==application::ChartLayout::ChartFocus,
                "returning to 125 percent saves once and retains chart layout");
          CheckDisplaySizing(58,20,60);
          wxTheApp->Yield(true);
          Check(panel_->GetScreenRect().width==460,
                "returning to 125 percent restores the applied drawer width");
        }
        Capture("display-applied-125-chart");Click("Dusk");break;
      case 12: Check(light_==ui::LightMode::Dusk,"light callback applied");Capture("display-dusk");Click("Night");break;
      case 13: Check(light_==ui::LightMode::Night,"night callback applied");Capture("display-night");Click("Toggle fullscreen");break;
      case 14: Check(fullscreen_==1,"fullscreen invokes one display callback");Click("Personalise instruments");break;
      case 15: Check(navigations_==2 && last_page_==ui::ProductPage::RailLayout,"rail configuration preserved");
        display_.scale_percent=100;panel_->SetDisplayPreferences(display_);Feed();
        Click("System");break;
      case 16:
        {
          auto *unavailable=new ui::XNavSettingsDrawer(*frame_,{});
          unavailable->Select(ui::SettingsSection::System);
          const auto *advanced=Find(unavailable,"Advanced / Legacy Settings");
          Check(advanced && !advanced->IsEnabled(),"missing upstream settings callback disables direct action");
          auto *setup=Find(unavailable,"Run vessel setup");
          Check(setup && !setup->IsEnabled(),"missing boat setup callback disables System setup row");
          wxCommandEvent event(wxEVT_BUTTON,setup->GetId());event.SetEventObject(setup);
          setup->GetEventHandler()->ProcessEvent(event);
          wxTheApp->Yield(true);
          Check(boat_setups_==0,"disabled System setup cannot dispatch a setup action");
          unavailable->Select(ui::SettingsSection::Vessel);
          setup=Find(unavailable,"Run boat setup");
          Check(setup && !setup->IsEnabled(),"missing boat setup callback also disables Vessel setup");
          unavailable->Destroy();
        }
        Check(!Find(panel_,"AUTO") && !Find(panel_,"STBY"),"preferences cannot execute physical controls");
        for(const auto *label:{"Installation & recovery","Updates","Export settings backup","Import settings backup"}) {
          auto *unavailable=Find(panel_,label);
          Check(unavailable && !unavailable->IsEnabled(),"unavailable System capability is disabled");
          wxCommandEvent event(wxEVT_BUTTON,unavailable->GetId());event.SetEventObject(unavailable);
          unavailable->GetEventHandler()->ProcessEvent(event);
        }
        Check(Find(panel_,"About & licenses") && Find(panel_,"About & licenses")->IsEnabled(),
              "packaged About and license viewer is available");
        wxTheApp->Yield(true);
        Check(advanced_==1 && navigations_==2 && diagnostics_==0 && plugins_==0,
              "unavailable System rows cannot delegate actions");
        Capture("system-night");
        ScrollBody()->Scroll(0,1000);wxTheApp->Yield(true);
        Check(ScrollBody()->GetScreenRect().Contains(Find(panel_,"Advanced / Legacy Settings")->GetScreenRect()),
              "direct Advanced action is reachable by scrolling");
        Capture("system-bottom-night",false);
        Check(ScrollBody()->GetScreenRect().Contains(Find(panel_,"Run vessel setup")->GetScreenRect()),
              "System setup action is reachable by scrolling");
        Click("Run vessel setup");wxTheApp->Yield(true);
        Check(boat_setups_==1 && saves_==2 && display_saves_==6 && navigations_==2,
              "System setup invokes existing setup callback once without saving or navigating");
        Click("Advanced / Legacy Settings");break;
      case 17:
        Check(advanced_==2 && navigations_==2,"Advanced settings delegates directly without opening XNav preferences");
        Click("Interface & recovery");wxTheApp->Yield(true);
        Check(navigations_==3 && last_page_==ui::ProductPage::System,"real interface recovery remains accessible");
        Click("Help & guides");wxTheApp->Yield(true);
        Check(panel_->Section()==ui::SettingsSection::Help,"limited Help row opens existing Help tab");
        panel_->Select(ui::SettingsSection::System);wxTheApp->Yield(true);
        for(const bool back:{false,true}) {
          wxKeyEvent key(wxEVT_CHAR_HOOK);key.m_keyCode=back?WXK_LEFT:WXK_ESCAPE;key.m_altDown=back;
          key.SetEventObject(panel_);panel_->ProcessWindowEvent(key);wxTheApp->Yield(true);
          Check(!panel_->IsShown(),"System root respects Escape and Alt-Left dismissal");
          panel_->Open(wxRect(frame_->ClientToScreen({80,68}),wxSize(1014,698)));wxTheApp->Yield(true);
          Check(ScrollBody()->GetViewStart()==wxPoint(0,0),"System reopens at the root scroll position");
        }
        Click("Diagnostics");break;
      case 18: Check(diagnostics_==1,"diagnostics callback once");Click("Radar");break;
      case 19: Capture("settings-radar-night");Click("Autopilot");break;
      case 20: Capture("settings-autopilot-night");Click("Close");break;
      case 21:
        Check(closed_==3 && !panel_->IsShown(),"close hides only preferences");
        display_.scale_percent=100;
        panel_->SetDisplayPreferences(display_);
        panel_->Select(ui::SettingsSection::Navigation);
        Feed();
        {
          auto *alarms=Find(panel_,"Alarms & warnings");
          Check(alarms && alarms->IsEnabled() &&
                    ScrollBody()->GetScreenRect().Contains(alarms->GetScreenRect()),
                "alarm settings visible without scrolling Navigation preferences");
          const auto previous=navigations_;
          Click("Alarms & warnings");wxTheApp->Yield(true);
          Check(navigations_==previous+1 && last_page_==ui::ProductPage::AlarmSettings,
                "alarm entry opens settings overview rather than active notifications");
          Check(saves_==2 && display_saves_==6,
                "opening alarm settings never changes configured values");
        }
        panel_->Select(ui::SettingsSection::Vessel);
        Feed();
        break;
      case 22:
        {
          // Reproduce the retained offset after reaching Advanced battery model.
          // Scroll the real component body; no fixture-only product hooks.
          auto *body=ScrollBody();
          body->Scroll(0,body->GetVirtualSize().y);
          Check(body->GetViewStart().y>0,"Vessel body overflows at canonical size");
          Check(!body->GetScreenRect().Contains(Find(panel_,"Sensors")->GetScreenRect()),
                "scrolled Sensors tab starts outside the body viewport");
          Click("Run boat setup");wxTheApp->Yield(true);
          Check(boat_setups_==2 && saves_==2 && display_saves_==6,
                "Vessel setup retains the same callback without saving settings");
          const auto offset=body->GetViewStart();
          light_=ui::LightMode::Dusk;Feed();
          Check(body->GetViewStart()==offset,"live state and theme refresh preserve scroll");
          panel_->Dismiss();
          panel_->Open(wxRect(frame_->ClientToScreen({80,68}),wxSize(1014,698)));
        }
        break;
      case 23:
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
      case 24:
        Check(panel_->Section()==ui::SettingsSection::Sensors,"pointer reaches Sensors after root reopen");
        Check(saves_==2 && display_saves_==6,"reopening does not save settings");
#ifdef __WXMSW__
        Check(wxFileName::Mkdir(output_+"/activation",wxS_DIR_DEFAULT,wxPATH_MKDIR_FULL),"create separate activation evidence directory");
        activation_repro_=true;
        panel_->Select(ui::SettingsSection::Vessel);Feed();
        ActivateSurface(panel_);
        break;
      case 25:
        ScrollBody()->Scroll(0,ScrollBody()->GetVirtualSize().y);
        activation_navigations_=navigations_;
        NativeClick(Find(panel_,"Advanced battery model"));
        break;
      case 26:
        Check(navigations_==activation_navigations_+1 && last_page_==ui::ProductPage::EnergySettings,
              "physical battery click reaches owner callback once");
        Check(!panel_->IsShown(),"owner page callback dismisses cached Preferences");
        Check(::GetForegroundWindow()==static_cast<HWND>(frame_->GetHandle()),"owner active before root Preferences reopen");
        ActivationEvidence("owner-before-reopen");
        panel_->Open(wxRect(frame_->ClientToScreen({80,68}),wxSize(1014,698)));
        ActivationEvidence("reopened-before-owner-activation");
        break;
      case 27:
        // Present/Raise may foreground the drawer on Windows. Exercise the
        // actual owner-to-drawer transition explicitly instead of assuming
        // that ShowWithoutActivating kept the owner in the foreground.
        ActivateSurface(frame_);
        ActivationEvidence("owner-after-reopen");
        break;
      case 28:
        sensor_before_=Find(panel_,"Sensors")->GetScreenRect();
        ActivationEvidence("before-activation");
        Capture("activation/before-activation",false);
        Check(ScrollBody()->GetViewStart()==wxPoint(0,0),"root reopen initially resets scroll before activation");
        Check(::GetForegroundWindow()==static_cast<HWND>(frame_->GetHandle()),"explicit owner deactivation settled before pointer activation");
        Check(::WindowFromPoint(POINT{sensor_before_.x+sensor_before_.width/2,sensor_before_.y+sensor_before_.height/2})==
              static_cast<HWND>(Find(panel_,"Sensors")->GetHandle()),"Sensors initially passes exact native pointer hit");
        ActivateSurface(panel_);
        ActivationEvidence("after-activation-immediate");
        break;
      case 29:
        ActivationEvidence("after-activation-settled");
        Capture("activation/after-activation",false);
        Check(ScrollBody()->GetViewStart()==wxPoint(0,0),"drawer activation must preserve root Preferences scroll reset");
        Check(Find(panel_,"Sensors")->GetScreenRect()==sensor_before_,"Sensors must not move when reopened drawer activates");
        NativeClick(Find(panel_,"Sensors"));
        break;
      case 30:
        Check(panel_->Section()==ui::SettingsSection::Sensors,"physical Sensors click works after owner to drawer activation");
        Check(saves_==2 && display_saves_==6,"activation transition does not save settings");
        ActivationEvidence("sensors-selected");
#endif
        Finish();break;
      }
    } catch(const std::exception &e) {
      failed_=true;std::cerr<<e.what()<<std::endl;Finish();
    }
    std::cout<<"Settings step "<<current<<" end; captures="<<names_.size()
             <<"; display saves="<<display_saves_<<std::endl;
    // Display popup checks yield to real native events. A repeating timer can
    // enter the next scenario before this one's choices finish (wxMSW WM_TIMER).
    // Rearm only after the entire current scenario, keeping the same cadence.
    if(!finished_)timer_.StartOnce(350);
  }
  ui::XNavScroll *ScrollBody() {
    for(auto *child:panel_->GetChildren())
      if(auto *body=dynamic_cast<ui::XNavScroll *>(child))return body;
    throw std::runtime_error("Preferences scroll body missing");
  }
  void Finish() {
    if(finished_)return;
    finished_=true;
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
  int navigations_=0,advanced_=0,plugins_=0,fullscreen_=0,diagnostics_=0,boat_setups_=0;
  ui::LightMode light_=ui::LightMode::Day;
  std::vector<std::string> names_;
  int step_=0,checks_=0,closed_=0;
  bool failed_=false,finished_=false;
#ifdef __WXMSW__
  bool activation_repro_=false;
  int activation_navigations_=0;
  wxRect sensor_before_;
#endif
};
} // namespace
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc,char **argv) {return wxEntry(argc,argv);}

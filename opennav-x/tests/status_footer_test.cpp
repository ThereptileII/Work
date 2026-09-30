// Offline component process only. No live profile, navigation or equipment.
#include "ui/StatusFooter.h"
#include <wx/app.h>
#include <wx/dcbuffer.h>
#include <wx/dcscreen.h>
#include <wx/eventfilter.h>
#include <wx/frame.h>
#include <wx/filename.h>
#include <wx/log.h>
#include <wx/sizer.h>
#include <cstdint>
#ifdef __WXMSW__
#include <windows.h>
#endif
#include <wx/timer.h>
#include <wx/uiaction.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
#ifdef __WXGTK__
#include <gtk/gtk.h>
#endif
using namespace opennav;
using namespace std::chrono_literals;
namespace {
class TestApp final:public wxApp {
 public:
  class KeyEvidence final:public wxEventFilter {
   public:
    int FilterEvent(wxEvent &event) override {
      if(event.GetEventObject()!=target)return Event_Skip;
      auto *key=dynamic_cast<wxKeyEvent *>(&event);if(!key)return Event_Skip;
      if(event.GetEventType()==wxEVT_CHAR_HOOK){++hooks;last_phase="char-hook";}
      if(event.GetEventType()==wxEVT_KEY_DOWN){++downs;last_phase="key-down";}
      if(event.GetEventType()==wxEVT_KEY_UP){++ups;last_phase="key-up";}
      last_keycode=key->GetKeyCode();
      return Event_Skip;
    }
    wxWindow *target=nullptr;const char *last_phase="none";int last_keycode=0,hooks=0,downs=0,ups=0;
  };
  bool OnInit() override {
    wxLog::SetActiveTarget(new wxLogStderr());
    wxSetAssertHandler([](const wxString &,int,const wxString &,const wxString &s,const wxString &){std::cerr<<s<<std::endl;std::abort();});
    if(argc!=2)return false;output_=argv[1];
    if(!wxFileName::Mkdir(output_,wxS_DIR_DEFAULT,wxPATH_MKDIR_FULL))return false;
    wxInitAllImageHandlers();
    frame_=new wxFrame(nullptr,wxID_ANY,"TEST ONLY - Status footer",{0,0},{1280,800},wxBORDER_NONE);
    Resize(1280,800);frame_->SetBackgroundColour(ui::Colour(ui::Theme(light_).surface));
    host_=new wxPanel(frame_,wxID_ANY,wxDefaultPosition,wxDefaultSize,wxTAB_TRAVERSAL);
    auto *layout=new wxBoxSizer(wxVERTICAL);
    layout->Add(host_,1,wxEXPAND);frame_->SetSizer(layout);frame_->Layout();
    footer_=new ui::XNavStatusFooter(host_,[this]{++opened_;});
    keys_.target=Health();wxEvtHandler::AddFilter(&keys_);
    footer_->SetSize(0,766,1280,34);
    frame_->Show();frame_->Raise();
    timer_.SetOwner(this);Bind(wxEVT_TIMER,&TestApp::Step,this);timer_.Start(300);
    return true;
  }
  int OnRun() override {wxApp::OnRun();return failed_?1:0;}
 private:
  void Resize(int width,int height) {
    frame_->SetClientSize(width,height);frame_->Layout();
  }
  wxRect NativeRect(wxWindow *window) {
#ifdef __WXMSW__
    RECT r{};
    Check(::GetWindowRect(static_cast<HWND>(window->GetHandle()),&r)!=0,"native HWND rectangle available");
    return {static_cast<int>(r.left),static_cast<int>(r.top),
            static_cast<int>(r.right-r.left),static_cast<int>(r.bottom-r.top)};
#else
    return window->GetScreenRect();
#endif
  }
  void RecordLayout(const char *phase,wxWindow *target=nullptr) {
    const auto client=wxRect(frame_->ClientToScreen({0,0}),frame_->GetClientSize());
    const auto frame=NativeRect(frame_),host=NativeRect(host_),component=NativeRect(footer_);
    std::ofstream out((output_+"/native-layout.jsonl").ToStdString(),std::ios::app);
    const auto rect=[&](const wxRect &r){out<<'['<<r.x<<','<<r.y<<','<<r.width<<','<<r.height<<']';};
    const auto window=[&](const char *name,wxWindow *w,const wxRect &r){
      out<<",\""<<name<<"\":{\"native_handle\":"<<reinterpret_cast<std::uintptr_t>(w->GetHandle())
         <<",\"shown\":"<<(w->IsShownOnScreen()?"true":"false")<<",\"screen\":";rect(r);out<<'}';};
    out<<"{\"phase\":\""<<phase<<"\",\"step\":"<<step_<<",\"frame_client\":";rect(client);
    window("frame",frame_,frame);window("host",host_,host);window("footer",footer_,component);
    if(target) {
      const auto bounds=NativeRect(target);window("target",target,bounds);
      auto *hit=wxFindWindowAtPoint(bounds.GetTopLeft()+wxPoint(bounds.width/2,bounds.height/2));
      out<<",\"pointer_hit_handle\":"<<(hit?reinterpret_cast<std::uintptr_t>(hit->GetHandle()):0);
    }
    out<<"}\n";out.close();
    Check(frame_->IsShownOnScreen()&&host_->IsShownOnScreen()&&footer_->IsShownOnScreen(),"actual fixture parents and component are shown");
    Check(host==client,"fixture host fills actual frame client");
    Check(host.Contains(component),"actual component is contained by visible host");
    Check(component==wxRect(host_->ClientToScreen(footer_->GetPosition()),footer_->GetSize()),"native component rectangle matches requested placement");
    Check(component==wxRect(client.x,client.y+client.height-34,client.width,34),"native footer fills actual client bottom exactly");
  }
  void Check(bool ok,const char *message){++checks_;if(!ok)throw std::runtime_error(message);}
  void Feed(vessel::Time now=stamp_) {
    footer_->Update(application::PresentFooter(state_,{},application::PresentSourceHealth(state_,{},{},{},{},now),now),light_);
    host_->SetBackgroundColour(ui::Colour(ui::Theme(light_).surface));frame_->Refresh(false);
  }
  ui::XNavButton *Health(){return dynamic_cast<ui::XNavButton *>(wxWindow::FindWindowByName("Footer source health",footer_));}
  void Geometry(int width,bool middle) {
    Check(footer_->GetSize()==wxSize(width,34),"34px footer height at canonical scale");
    Check(footer_->GetName()=="OpenNav status footer","stable native footer identity");
    Check(footer_->MiddleVisible()==middle,"prototype 1100px middle breakpoint");
    const auto left=footer_->LeftRegion(),mid=footer_->MiddleRegion();
    const auto health=Health()->GetRect();
    Check(left.x==20&&width-health.GetRight()-1==20,"prototype exact outer 20px padding");
    Check(health.y>=0&&health.GetBottom()<34&&health.height>=10&&health.height<=20,"natural text-sized health link fits footer");
    Check(left.GetRight()<health.x,"position/health never overlap at supported width");
    Check(Health()->GetLabel()=="Source health"&&Health()->IsEnabled(),"stable real input target");
    if(middle) {
      Check(left.GetRight()<mid.x&&mid.GetRight()<health.x,"three groups do not overlap");
      Check(std::abs((mid.x-left.GetRight()-1)-(health.x-mid.GetRight()-1))<=2,"CSS space-between uses equal free gaps");
    } else Check(mid.width==0,"hidden middle contributes no visible region");
  }
  void Capture(const char *name) {
    RecordLayout(name);
    const auto size=frame_->GetClientSize();const auto origin=frame_->ClientToScreen({0,0});
#ifdef __WXGTK__
    auto *pixels=gdk_pixbuf_get_from_window(gdk_get_default_root_window(),origin.x,origin.y,size.x,size.y);
    Check(pixels!=nullptr,"actual root-window capture");
    const bool saved=gdk_pixbuf_save(pixels,(output_+"/"+name+".png").utf8_str(),"png",nullptr,nullptr);
    g_object_unref(pixels);Check(saved,"capture saved");
#else
    wxScreenDC screen;wxBitmap image(size.x,size.y);wxMemoryDC memory(image);
    Check(memory.Blit(0,0,size.x,size.y,&screen,origin.x,origin.y),"actual screen capture");memory.SelectObject(wxNullBitmap);
    Check(image.SaveFile(output_+"/"+name+".png",wxBITMAP_TYPE_PNG),"capture saved");
#endif
    captures_.push_back(name);
  }
  void Prototype(ui::LightMode mode) {
    // Literal illustrative HTML content, confined to this test process. The
    // product model explicitly cannot produce this XTE or claim these sources.
    Resize(1280,800);footer_->SetSize(0,766,1280,34);
    application::FooterView v;
    v.navigation_state="UNDERWAY";v.position="58° 20.462′ N   016° 48.218′ E";
    v.cog="043°";v.xte="0.02 nm";v.health_source="NMEA 2000";v.health_summary="9 of 10 sources";
    v.position_state=v.cog_state=v.health_state=application::SignalState::Current;
    light_=mode;footer_->Update(v,mode);frame_->SetFocus();
    const auto outside=frame_->ClientToScreen({1279,799});
    wxUIActionSimulator input;input.MouseMove(outside.x,outside.y);
  }
  void Step(wxTimerEvent &) {
    try {
      switch(step_++) {
      case 0:Feed();break;
      case 1:
        Geometry(1280,true);Check(opened_==0,"observation never opens health");Capture("unavailable-day");
        state_.navigation.latitude_deg={58.34103333333,"TEST ONLY GPS",stamp_,vessel::Validity::Measured};
        state_.navigation.longitude_deg={16.80363333333,"TEST ONLY GPS",stamp_,vessel::Validity::Measured};
        state_.navigation.cog_deg={43.,"TEST ONLY GPS",stamp_,vessel::Validity::Measured};Feed();break;
      case 2:{
        Geometry(1280,true);Capture("measured-day");
        auto *button=Health();RecordLayout("source-health-pointer",button);const auto rect=button->GetScreenRect();
        Check(wxFindWindowAtPoint(rect.GetTopLeft()+wxPoint(rect.width/2,rect.height/2))==button,"pointer location hits actual footer button");
        wxUIActionSimulator input;Check(input.MouseMove(rect.x+rect.width/2,rect.y+rect.height/2)&&input.MouseClick(),"native pointer input injected");break;
      }
      case 3:Check(opened_==1,"actual pointer click opens source health once");light_=ui::LightMode::Dusk;Feed();break;
      case 4:Geometry(1280,true);Capture("measured-dusk");light_=ui::LightMode::Night;Feed();break;
      case 5:Capture("measured-night");Feed(stamp_+5s);break;
      case 6:
        Check(footer_->View().position_state==application::SignalState::Stale&&footer_->View().cog=="STALE","same retained observation visibly expires");
        Capture("stale-night");Resize(1024,640);footer_->SetSize(0,606,1024,34);Feed();break;
      case 7:Geometry(1024,false);Capture("responsive-125-equivalent");Resize(853,533);footer_->SetSize(0,499,853,34);Feed();break;
      case 8:Geometry(853,false);Capture("responsive-150-equivalent");Resize(1100,640);footer_->SetSize(0,606,1100,34);Feed();break;
      case 9:Geometry(1100,false);Resize(1101,640);footer_->SetSize(0,606,1101,34);Feed();break;
      case 10:Geometry(1101,true);Prototype(ui::LightMode::Day);break;
      case 11:Geometry(1280,true);Capture("prototype-fixture-day");Prototype(ui::LightMode::Dusk);break;
      case 12:Capture("prototype-fixture-dusk");Prototype(ui::LightMode::Night);break;
      case 13:{Capture("prototype-fixture-night");Health()->SetFocus();auto *focus=wxWindow::FindFocus();Check(focus==Health(),"health owns native keyboard focus");focus_handle_=reinterpret_cast<std::uintptr_t>(focus->GetHandle());target_handle_=reinterpret_cast<std::uintptr_t>(Health()->GetHandle());wxUIActionSimulator input;Check(input.Char(WXK_RETURN),"native Enter char input");break;}
      case 14:{Check(opened_==2,"normal Enter opens health exactly once");wxUIActionSimulator input;Check(input.KeyDown(WXK_RETURN),"native Enter down input");break;}
      case 15:{Check(opened_==2,"Enter down does not activate before release");wxUIActionSimulator input;Check(input.KeyDown(WXK_RETURN)&&input.KeyUp(WXK_RETURN),"native held Enter repeat and up input");break;}
      case 16:{Check(opened_==3,"held Enter opens health exactly once on release");wxUIActionSimulator input;Check(input.Char(WXK_SPACE),"native Space input");break;}
      case 17:{Check(opened_==4,"Space opens health once");Health()->Disable();wxUIActionSimulator input;Check(input.Char(WXK_RETURN),"disabled native Enter input");break;}
      case 18:{Check(opened_==4,"disabled Enter does not open health");Health()->Enable();tab_=new ui::XNavButton(host_,wxID_ANY,"Tab destination","Tab destination");tab_->SetSize(0,0,48,48);Health()->SetFocus();wxUIActionSimulator input;Check(input.Char(WXK_TAB),"native Tab input");break;}
      case 19:Check(wxWindow::FindFocus()==tab_,"Tab reaches actual destination control");Check(opened_==4,"Tab does not open health");Check(keys_.hooks>=4&&keys_.downs>=1&&keys_.ups>=3,"actual hook/down/up keyboard evidence observed");Finish();break;
      }
    }catch(const std::exception &e){failed_=true;std::cerr<<e.what()<<'\n';Finish();}
  }
  void Finish(){
    timer_.Stop();wxEvtHandler::RemoveFilter(&keys_);std::ofstream out((output_+"/result.json").ToStdString());
    out<<"{\"passed\":"<<(failed_?"false":"true")<<",\"checks\":"<<checks_<<",\"native_dpi_qualification\":false,\"captures\":[";
    for(size_t i=0;i<captures_.size();++i)out<<(i?",":"")<<"\""<<captures_[i]<<"\"";
    const auto left=footer_->LeftRegion(),middle=footer_->MiddleRegion(),health=Health()->GetRect();
    out<<"],\"keyboard\":{\"focused_handle_before_input\":"<<focus_handle_<<",\"target_handle\":"<<target_handle_<<",\"last_phase\":\""<<keys_.last_phase<<"\",\"last_keycode\":"<<keys_.last_keycode<<",\"char_hooks\":"<<keys_.hooks<<",\"key_downs\":"<<keys_.downs<<",\"key_ups\":"<<keys_.ups<<",\"activations\":"<<opened_<<",\"tab_destination_handle\":"<<(tab_?reinterpret_cast<std::uintptr_t>(tab_->GetHandle()):0)<<",\"tab_on_destination\":"<<(tab_&&wxWindow::FindFocus()==tab_?"true":"false")<<"},\"canonical_geometry\":{\"left\":["<<left.x<<","<<left.width<<"],\"middle\":["<<middle.x<<","<<middle.width<<"],\"health\":["<<health.x<<","<<health.y<<","<<health.width<<","<<health.height<<"]}}\n";
    std::cout<<(failed_?"FAIL ":"PASS ")<<checks_<<" status footer component checks\n";frame_->Destroy();ExitMainLoop();
  }
  static inline const vessel::Time stamp_{100s};
  wxString output_;wxFrame *frame_=nullptr;wxPanel *host_=nullptr;ui::XNavStatusFooter *footer_=nullptr;ui::XNavButton *tab_=nullptr;
  vessel::VesselState state_;ui::LightMode light_=ui::LightMode::Day;
  std::vector<std::string> captures_;
  wxTimer timer_;KeyEvidence keys_;std::uintptr_t focus_handle_=0,target_handle_=0;int step_=0,opened_=0,checks_=0;bool failed_=false;
};
}
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc, char **argv) { return wxEntry(argc, argv); }

// Disposable native component host: actual production widgets, no OpenCPN/profile/network.
#include "ui/SettingsDrawer.h"
#include "ui/PrototypeGeometry.h"
#include <wx/app.h>
#include <wx/filename.h>
#include <wx/log.h>
#include <wx/timer.h>
#include <wx/msw/wrapwin.h>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <commctrl.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace opennav;
namespace {
std::string Quote(const wxString &value) {
  std::string out="\"";
  for (unsigned char c:value.ToStdString(wxConvUTF8)) {
    if(c=='"'||c=='\\'){out+='\\';out+=char(c);}
    else if(c<32){const char *hex="0123456789abcdef";out+="\\u00";out+=hex[c>>4];out+=hex[c&15];}
    else out+=char(c);
  }
  return out+'"';
}
void Rect(std::ostream &out,const wxRect &r) {
  out<<"\"x\":"<<r.x<<",\"y\":"<<r.y<<",\"width\":"<<r.width<<",\"height\":"<<r.height;
}
class App final:public wxApp {
public:
  bool OnInit() override {
    if(argc!=2)return false;
    output_=argv[1];
    if(!wxDirExists(output_))return false;
    wxLog::SetActiveTarget(new wxLogStderr());
    wxSetAssertHandler([](const wxString &,int,const wxString &,const wxString &s,const wxString &){std::cerr<<s<<'\n';std::abort();});
    frame_=new wxFrame(nullptr,wxID_ANY,"TEST ONLY - Preferences touch",{0,0},{1280,800},wxDEFAULT_FRAME_STYLE|wxWANTS_CHARS);
    frame_->SetBackgroundColour(ui::Colour(ui::Theme(ui::LightMode::Night).surface));
    ui::SettingsDrawerActions actions;
    actions.settings=[this]{return state_.settings;};
    actions.save_vessel=[this](const application::Settings &,const std::string &,double){
      ++saves_;return application::CommandResult{false,"Offline proof must not save"};};
    actions.page=[this](ui::ProductPage page){++actions_;battery_action_=page==ui::ProductPage::EnergySettings;};
    drawer_=new ui::XNavSettingsDrawer(*frame_,std::move(actions));
    state_.chart_safety_depth_m=3;
    drawer_->Update(state_,ui::LightMode::Night);
    frame_->Show();
    timer_.SetOwner(this);Bind(wxEVT_TIMER,&App::Tick,this);timer_.StartOnce(250);
    started_=std::chrono::steady_clock::now();return true;
  }
  int OnRun() override {wxApp::OnRun();return failed_?1:0;}
private:
  wxRect Workspace() {
    const auto size=frame_->GetClientSize(),logical=frame_->ToDIP(size);
    const auto layout=ui::prototype::DisplayDesktop(logical.x,logical.y,application::DisplayPreferences{}.layout);
    const int left=frame_->FromDIP(layout.navigation),top=frame_->FromDIP(layout.top);
    return {frame_->ClientToScreen({left,top}),wxSize(size.x-left-frame_->FromDIP(layout.rail),size.y-top-frame_->FromDIP(ui::prototype::footer))};
  }
  void Controls(std::ostream &out,wxWindow *window,bool &first) {
    auto *field=dynamic_cast<wxTextCtrl *>(window);
    if(field||dynamic_cast<ui::XNavButton *>(window)) {
      if(!first)out<<',';first=false;
      const auto r=window->GetScreenRect();bool visible=window->IsShownOnScreen();
      for(auto *p=window->GetParent();p&&!p->IsTopLevel();p=p->GetParent())visible=visible&&p->GetScreenRect().Contains(r);
      out<<'{';Rect(out,r);out<<",\"label\":"<<Quote(field?"Field: "+window->GetName():window->GetLabel())
        <<",\"accessible_name\":"<<Quote(window->GetName())<<",\"visible\":"<<(visible?"true":"false")
        <<",\"enabled\":"<<(window->IsEnabled()?"true":"false")<<",\"hwnd\":"<<reinterpret_cast<std::uintptr_t>(window->GetHandle());
      if(field)out<<",\"value\":"<<Quote(field->GetValue());
      out<<'}';
    }
    for(auto *child:window->GetChildren())Controls(out,child,first);
  }
  static LRESULT CALLBACK Input(HWND hwnd,UINT message,WPARAM w,LPARAM l,UINT_PTR id,DWORD_PTR data) {
    auto *self=reinterpret_cast<App *>(data);
    if(message==WM_GESTURE) {
      GESTUREINFO info{};info.cbSize=sizeof(info);
      if(GetGestureInfo(reinterpret_cast<HGESTUREINFO>(l),&info)&&info.dwID==GID_PAN) {
        if(hwnd==self->body_hwnd_)++self->body_pans_;else ++self->child_pans_;
      }
    }
    if(message==WM_LBUTTONDOWN&&hwnd!=self->body_hwnd_)++self->child_downs_;
    if(message==WM_NCDESTROY)RemoveWindowSubclass(hwnd,&Input,id);
    // Observe only: the real widget retains gesture ownership and cleanup.
    return DefSubclassProc(hwnd,message,w,l);
  }
  void ObserveInput(wxWindow *window) {
    if(dynamic_cast<ui::XNavScroll *>(window))body_hwnd_=static_cast<HWND>(window->GetHandle());
    if(!SetWindowSubclass(static_cast<HWND>(window->GetHandle()),&Input,1,reinterpret_cast<DWORD_PTR>(this)))
      throw std::runtime_error("Cannot observe native touch recipients");
    for(auto *child:window->GetChildren())ObserveInput(child);
  }
  void Publish() {
    ui::XNavScroll *body=nullptr;
    for(auto *child:drawer_->GetChildren())if(auto *s=dynamic_cast<ui::XNavScroll *>(child))body=s;
    if(!body)throw std::runtime_error("Missing actual Settings body");
    int ux=0,uy=0;body->GetScrollPixelsPerUnit(&ux,&uy);
    std::ostringstream out;
    out<<"{\"fixture_only\":true,\"pid\":"<<GetCurrentProcessId()<<",\"native_dpi\":"<<GetDpiForWindow(static_cast<HWND>(drawer_->GetHandle()))
      <<",\"wx_dpi\":"<<drawer_->GetDPI().x<<",\"saves\":"<<saves_<<",\"actions\":"<<actions_
      <<",\"battery_action\":"<<(battery_action_?"true":"false")<<",\"body_scroll_px\":"<<body->GetViewStart().y*uy
      <<",\"body_pan_messages\":"<<body_pans_<<",\"child_pan_messages\":"<<child_pans_<<",\"child_mouse_downs\":"<<child_downs_
      <<",\"focus_hwnd\":"<<reinterpret_cast<std::uintptr_t>(GetFocus())
      <<",\"runtime\":{\"ui_update\":{\"ticks\":"<<++ticks_<<"},\"display\":{\"drawer\":{";
    Rect(out,drawer_->GetScreenRect());out<<"},\"interaction_controls\":[";bool first=true;Controls(out,drawer_,first);out<<"]}}}";
    const auto temp=output_+"/observation.tmp",dest=output_+"/observation.json";
    {std::ofstream file(std::filesystem::path(temp.ToStdWstring()),std::ios::binary);file<<out.str();file.close();if(!file)throw std::runtime_error("Cannot write observation");}
    for(int attempt=0;;++attempt) {
      if(MoveFileExW(temp.wc_str(),dest.wc_str(),MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH))break;
      if(GetLastError()!=ERROR_SHARING_VIOLATION||attempt==4)throw std::runtime_error("Cannot publish observation");
      Sleep(5);
    }
  }
  void Tick(wxTimerEvent &) {
    if(finished_)return;
    try {
      if(std::chrono::steady_clock::now()-started_>std::chrono::seconds(85))throw std::runtime_error("Touch host deadline");
      if(!opened_){drawer_->Open(Workspace());ObserveInput(drawer_);opened_=true;}
      drawer_->Update(state_,ui::LightMode::Night);drawer_->Present(Workspace());
      Publish();
      if(wxFileExists(output_+"/stop")){Finish();return;}
    }catch(const std::exception &e){failed_=true;std::cerr<<e.what()<<'\n';Finish();return;}
    // Native input can pump events; no repeating timer or reentry.
    if(!finished_)timer_.StartOnce(250);
  }
  void Finish() {
    if(finished_)return;finished_=true;timer_.Stop();
    std::ofstream out(std::filesystem::path((output_+"/host-result.json").ToStdWstring()));
    out<<"{\"completed\":"<<(failed_?"false":"true")<<",\"saves\":"<<saves_<<",\"actions\":"<<actions_<<"}";
    out.close();drawer_->Destroy();frame_->Destroy();ExitMainLoop();
  }
  wxString output_;wxFrame *frame_=nullptr;ui::XNavSettingsDrawer *drawer_=nullptr;
  ui::ProductState state_;wxTimer timer_;std::chrono::steady_clock::time_point started_;
  HWND body_hwnd_=nullptr;int body_pans_=0,child_pans_=0,child_downs_=0;
  int ticks_=0,saves_=0,actions_=0;bool battery_action_=false,opened_=false,finished_=false,failed_=false;
};
}
wxIMPLEMENT_APP_NO_MAIN(App);
int main(int argc,char **argv) {
  const auto ci=std::getenv("GITHUB_ACTIONS"),permit=std::getenv("OPENNAV_DISPOSABLE_DESKTOP");
  if(!ci||std::string(ci)!="true"||!permit||std::string(permit)!="1")return 2;
  // Before wxEntry creates any HWND; requested DPI alone is never acceptance.
  if(!SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2)&&
     !AreDpiAwarenessContextsEqual(GetThreadDpiAwarenessContext(),DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2))return 3;
  return wxEntry(argc,argv);
}

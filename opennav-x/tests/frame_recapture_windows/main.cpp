// Isolated native Windows focus regression. No OpenCPN, profile or transport.
// Both variants use the complete production XNavButton implementation.
#include "ui/Controls.h"
#include <wx/app.h>
#include <wx/dialog.h>
#include <wx/frame.h>
#include <wx/log.h>
#include <wx/sizer.h>
#include <wx/textctrl.h>
#include <wx/timer.h>
#include <wx/weakref.h>
#include <wx/msw/wrapwin.h>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace opennav::ui {
class Shell {
 public:
  explicit Shell(wxFrame& frame):frame_(frame){}
  wxFrame& frame_;
  wxWeakRef<wxDialog> boat_setup_;
  wxWindow *context_=nullptr,*route_context_=nullptr;
  bool DrawerRegion() const {return false;}
  bool HasTransientSurface() const;
};
#include "shell-transient.inc"
}
namespace opennav {
ui::Shell* shell=nullptr;
bool IsXNav(){return true;}
#include "integration-transient.inc"
}
class MyFrame final:public wxFrame {
 public:
  MyFrame():wxFrame(nullptr,wxID_ANY,"Offline recapture owner",{40,40},{700,500}){}
  void OnRecaptureTimer(wxTimerEvent&);
  void Raise() override {++raises;wxFrame::Raise();}
  int raises=0;
};
#include "frame-recapture.inc"

namespace {
class App final:public wxApp {
 public:
  bool OnInit() override {
    std::cout.setf(std::ios::unitbuf);
    wxLog::SetActiveTarget(new wxLogStderr());
    wxSetAssertHandler([](const wxString&,int,const wxString&,const wxString& condition,const wxString&){
      std::cerr<<"WX ASSERT "<<condition<<'\n';std::abort();
    });
    SetExitOnFrameDelete(false);
    owner_=new MyFrame;
    auto* edit=new wxTextCtrl(owner_,wxID_ANY,"Offline owner focus target");
    auto* owner_layout=new wxBoxSizer(wxVERTICAL);owner_layout->Add(edit,1,wxEXPAND);
    owner_->SetSizer(owner_layout);owner_->Show();edit->SetFocus();
    setup_=new wxDialog(owner_,wxID_ANY,"Boat Setup & Sensor Check",{160,140},{400,220});
    button_=new opennav::ui::XNavButton(setup_,wxID_ANY,"Later","Later");
    button_->SetMinSize({180,60});
    auto* layout=new wxBoxSizer(wxVERTICAL);layout->AddStretchSpacer();
    layout->Add(button_,0,wxEXPAND|wxALL,20);setup_->SetSizer(layout);
    button_->Bind(wxEVT_BUTTON,[this](wxCommandEvent&){++activations_;Trace("button");});
    button_->Bind(wxEVT_LEFT_DOWN,[this](wxMouseEvent& e){++downs_;Trace("down");e.Skip();});
    button_->Bind(wxEVT_LEFT_UP,[this](wxMouseEvent& e){++ups_;Trace("up");e.Skip();});
    button_->Bind(wxEVT_KILL_FOCUS,[this](wxFocusEvent& e){++blurs_;Trace("blur");e.Skip();});
    model_=new opennav::ui::Shell(*owner_);model_->boat_setup_=setup_;opennav::shell=model_;
    setup_->Show();
    timer_.SetOwner(this);Bind(wxEVT_TIMER,&App::Step,this);
    deadline_=Clock::now()+std::chrono::seconds(5);timer_.Start(30);return true;
  }
  int OnRun() override {wxApp::OnRun();return failed_?1:0;}
 private:
  using Clock=std::chrono::steady_clock;
  void Check(bool condition,const char* why){if(!condition)throw std::runtime_error(why);}
  static HWND Native(wxWindow* window){return reinterpret_cast<HWND>(window->GetHandle());}
  void Trace(const char* stage){
    std::cout<<stage<<" foreground="<<GetForegroundWindow()<<" focus="<<GetFocus()
      <<" capture="<<GetCapture()<<" downs="<<downs_<<" ups="<<ups_
      <<" blurs="<<blurs_<<" activations="<<activations_<<'\n';
  }
  void Mouse(DWORD flags){
    INPUT input{};input.type=INPUT_MOUSE;input.mi.dwFlags=flags;
    Check(SendInput(1,&input,sizeof(input))==1,"native pointer input insertion failed");
  }
  void Next(){++stage_;deadline_=Clock::now()+std::chrono::seconds(3);}
  void Step(wxTimerEvent&) {try {
    Check(Clock::now()<deadline_,"native interaction stage timed out");
    switch(stage_) {
      case 0:
        Check(setup_->IsShownOnScreen()&&!setup_->IsModal(),"owned modeless setup required");
        Check(SetForegroundWindow(Native(setup_))!=0,"setup activation failed");
        Next();break;
      case 1: {
        if(GetForegroundWindow()!=Native(setup_))break;
        const auto rect=button_->GetScreenRect();POINT point{rect.x+rect.width/2,rect.y+rect.height/2};
        Check(SetCursorPos(point.x,point.y)!=0,"pointer positioning failed");
        Check(WindowFromPoint(point)==Native(button_),"actual button must be unobscured");
        blurs_=0; // Count only focus loss during this single press.
        Mouse(MOUSEEVENTF_LEFTDOWN);mouse_down_=true;Next();break;
      }
      case 2: {
        if(downs_!=1||GetCapture()!=Native(button_)||GetFocus()!=Native(button_))break;
        Check(blurs_==0&&activations_==0,"unexpected pre-recapture cancellation or activation");
        Trace("before-recapture");wxTimerEvent event(timer_);owner_->OnRecaptureTimer(event);
        Trace("after-recapture");Next();break;
      }
      case 3:
#if EXPECTED_SETUP_GUARD
        Check(owner_->raises==0,"guarded callback must not raise owner");
        Check(GetForegroundWindow()==Native(setup_)&&GetFocus()==Native(button_)&&blurs_==0,
              "guarded recapture must preserve pressed-button focus");
#else
        Check(owner_->raises==1,"original callback must raise owner");
        if(GetForegroundWindow()!=Native(owner_)||blurs_!=1)break;
        Check(GetFocus()!=Native(button_),"original recapture must remove button focus");
#endif
        Mouse(MOUSEEVENTF_LEFTUP);mouse_down_=false;Next();break;
      case 4:
        if(ups_!=1)break;
#if EXPECTED_SETUP_GUARD
        if(activations_!=1)break;
#else
        Check(activations_==0,"original guard must reproduce cancelled activation");
#endif
        // Give queued wxEVT_BUTTON dispatch a separate event-loop turn.
        Next();break;
      case 5:
        Check(downs_==1&&ups_==1,"one physical press/release only");
        Check(activations_==EXPECTED_SETUP_GUARD,"unexpected queued activation count");
        Check(GetCapture()!=Native(button_),"release must end mouse capture");
        Trace("complete");
        std::cout<<(EXPECTED_SETUP_GUARD?"PASS guarded activates exactly once":"PASS original reproduces focus-loss cancellation")<<'\n';
        Finish();return;
    }
  }catch(const std::exception& error){failed_=true;std::cerr<<"FAIL stage="<<stage_<<' '<<error.what()<<'\n';Trace("failure");Finish();}}
  void Finish(){
    timer_.Stop();
    if(mouse_down_){INPUT input{};input.type=INPUT_MOUSE;input.mi.dwFlags=MOUSEEVENTF_LEFTUP;SendInput(1,&input,sizeof(input));mouse_down_=false;}
    if(button_->HasCapture())button_->ReleaseMouse();
    opennav::shell=nullptr;delete model_;setup_->Destroy();owner_->Destroy();ExitMainLoop();
  }
  MyFrame* owner_=nullptr;wxDialog* setup_=nullptr;opennav::ui::XNavButton* button_=nullptr;
  opennav::ui::Shell* model_=nullptr;wxTimer timer_;Clock::time_point deadline_;
  int stage_=0,downs_=0,ups_=0,blurs_=0,activations_=0;bool failed_=false,mouse_down_=false;
};
}
wxIMPLEMENT_APP_NO_MAIN(App);
int main(int argc,char** argv){return wxEntry(argc,argv);}

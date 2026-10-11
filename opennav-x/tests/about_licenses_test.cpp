// Offline component: no actual browser, file manager, navigation or equipment.
#include "ui/SettingsDrawer.h"
#include <iostream>
#include <stdexcept>
#include <wx/app.h>
#include <wx/log.h>
#include <wx/timer.h>

using namespace opennav;
class AboutTest final:public wxApp {
 public:
  bool OnInit() override {
    wxLog::SetActiveTarget(new wxLogStderr());
    frame_=new wxFrame(nullptr,wxID_ANY,"Offline About component",{0,0},{1280,800});
    ui::SettingsDrawerActions actions;
    actions.read_notice=[this](application::AboutNotice notice) {
      ++reads_;last_notice_=notice;
      return application::AboutNoticeText{reads_==1,reads_==1?
          "Packaged notice — båt\nNo external action.":"This packaged notice is missing: docs/THIRD_PARTY_NOTICES.md"};
    };
    actions.open_about_url=[this](const std::string &url) {urls_.push_back(url);return urls_.size()==1;};
    actions.open_notices_folder=[this] {++folders_;return application::CommandResult{false,"The packaged docs/licenses folder is missing."};};
    drawer_=new ui::XNavSettingsDrawer(*frame_,std::move(actions));
    frame_->Show();drawer_->Select(ui::SettingsSection::System);
    drawer_->Open(wxRect(frame_->ClientToScreen({80,68}),wxSize(1014,698)));
    timer_.SetOwner(this);Bind(wxEVT_TIMER,&AboutTest::Step,this);timer_.StartOnce(150);
    return true;
  }
  int OnRun() override {wxApp::OnRun();return failed_?1:0;}
 private:
  void Check(bool value,const char *message) {++checks_;if(!value)throw std::runtime_error(message);}
  wxWindow *Find(wxWindow *root,const wxString &label) {
    for(auto *child:root->GetChildren()) {
      if(child->GetLabel()==label)return child;
      if(auto *nested=Find(child,label))return nested;
    }
    return nullptr;
  }
  void Click(const wxString &label,bool pointer=false) {
    auto *button=Find(drawer_,label);
    Check(button && button->IsEnabled(),"enabled XNav action exists");
    Check(dynamic_cast<ui::XNavButton *>(button)!=nullptr,"primary actions remain XNav controls");
    Check(button->GetMinSize().y>=button->FromDIP(44),"action retains minimum touch target");
    button->SetFocus();
    if(pointer) {
      for(auto type:{wxEVT_LEFT_DOWN,wxEVT_LEFT_UP}) {
        wxMouseEvent event(type);event.SetEventObject(button);event.SetPosition({12,12});
        button->ProcessWindowEvent(event);
      }
    } else {
      for(auto type:{wxEVT_KEY_DOWN,wxEVT_KEY_UP}) {
        wxKeyEvent event(type);event.SetEventObject(button);event.m_keyCode=WXK_SPACE;
        button->ProcessWindowEvent(event);
      }
    }
    // The production drawer intentionally defers rebuilds until after the
    // button handler. This one-shot timer cannot re-enter while draining them.
    wxTheApp->Yield(true);
  }
  wxTextCtrl *Reader() {
    return dynamic_cast<wxTextCtrl *>(wxWindow::FindWindowByName("Packaged notice text",drawer_));
  }
  void Back(bool alt=false) {
    wxKeyEvent key(wxEVT_CHAR_HOOK);key.m_keyCode=alt?WXK_LEFT:WXK_ESCAPE;key.m_altDown=alt;
    key.SetEventObject(Reader()?static_cast<wxWindow *>(Reader()):drawer_);
    Check(drawer_->FilterEvent(key)==wxEventFilter::Event_Processed,"drawer owns back key");
    wxTheApp->Yield(true);
  }
  void Step(wxTimerEvent &) {
    try {
      switch(step_++) {
        case 0:Click("About & licenses");break;
        case 1:
          Check(Find(drawer_,"OpenCPN project"),"About opens from System");
          Check(reads_==0 && urls_.empty() && folders_==0,"opening About has no file or external action");
          Click("Source & license information");break;
        case 2:
          Check(reads_==1 && last_notice_==application::AboutNotice::Sources,"selected packaged document read once");
          Check(Reader() && !Reader()->IsEditable(),"notice is selectable read-only text");
          Check(Reader()->GetValue().Contains(wxString::FromUTF8("båt")),"UTF-8 notice displayed");
          drawer_->Update({},ui::LightMode::Night);Back();break;
        case 3:Check(!Reader(),"Escape returns to About");Click("Third-party notices");break;
        case 4:
          Check(last_notice_==application::AboutNotice::Components && Reader()->GetValue().Contains("missing"),"missing file is explicit in reader");
          Back(true);break;
        case 5:Click("OpenCPN project",true);break;
        case 6:
          Check(urls_.size()==1 && urls_[0]=="https://opencpn.org/","project uses official URL only after activation");
          Click("OpenCPN source");break;
        case 7:
          Check(urls_.size()==2 && urls_[1]=="https://github.com/OpenCPN/OpenCPN","source uses official repository");
          Check(Reader() && Reader()->GetValue().Contains("https://github.com/OpenCPN/OpenCPN"),"browser failure retains copyable URL");
          Click("Back to About & licenses");break;
        case 8:Click("Open notices folder");break;
        case 9:Check(folders_==1 && Reader()->GetValue().Contains("missing"),"missing folder reported");Back();break;
        case 10:Back(true);break;
        case 11:
          Check(drawer_->Section()==ui::SettingsSection::System && Find(drawer_,"About & licenses"),"Alt-Left returns to originating System tab");
          drawer_->Select(ui::SettingsSection::Help);Click("About & licenses");break;
        case 12:Check(Find(drawer_,"OpenCPN source"),"Help entry opens same About page");Back();break;
        case 13:
          Check(drawer_->Section()==ui::SettingsSection::Help,"About returns to Help origin");
          Back();break;
        case 14:
          Check(!drawer_->IsShown(),"Escape at root dismisses preferences");
          std::cout<<checks_<<" About component checks passed\n";frame_->Destroy();ExitMainLoop();return;
      }
      timer_.StartOnce(30);
    } catch(const std::exception &error) {
      std::cerr<<error.what()<<" at step "<<step_-1<<'\n';failed_=true;frame_->Destroy();ExitMainLoop();
    }
  }
  wxFrame *frame_=nullptr;ui::XNavSettingsDrawer *drawer_=nullptr;wxTimer timer_;
  int step_=0,checks_=0,reads_=0,folders_=0;bool failed_=false;
  application::AboutNotice last_notice_=application::AboutNotice::Sources;
  std::vector<std::string> urls_;
};
wxIMPLEMENT_APP(AboutTest);

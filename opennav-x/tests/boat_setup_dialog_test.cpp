// Real native controls, owned configuration and offline observations only.
#include "ui/BoatSetupDialog.h"
#include "ui/Controls.h"
#include <wx/app.h>
#include <wx/dialog.h>
#include <wx/display.h>
#include <wx/frame.h>
#include <wx/log.h>
#include <wx/textctrl.h>
#include <iostream>
#include <stdexcept>
using namespace opennav;
void Check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
wxWindow* Find(wxWindow* owner,const wxString& label){
  for(auto* child:owner->GetChildren()){
    if(child->GetLabel()==label)return child;
    if(auto* result=Find(child,label))return result;
  }return nullptr;
}
void Click(wxWindow* owner,const wxString& label){
  auto* button=Find(owner,label);Check(button && button->IsEnabled(),"Expected enabled wizard action");
  wxCommandEvent event(wxEVT_BUTTON,button->GetId());event.SetEventObject(button);button->ProcessWindowEvent(event);
}
void CheckGeometry(){
  // Includes an offset taskbar and a monitor left of the primary display.
  for(const auto& work: {wxRect(0,32,1280,768), wxRect(-1280,0,1280,760)}){
    for(int scale: {100,125,150}){
      const int margin=16*scale/100;
      for(const auto& parent: {work, wxRect(work.x-400,work.y+500,800,600)}){
        const auto bounds=ui::BoatSetupDialogBounds(work,parent,{660*scale/100,620*scale/100},margin);
        auto usable=work;usable.Deflate(margin);
        Check(usable.Contains(bounds),"Scaled outer window stays within display workarea margins");
        Check(bounds.width<=660*scale/100 && bounds.height<=620*scale/100,"Preferred size never expands");
      }
    }
  }
}
void CheckActionVisible(wxDialog* dialog,const wxString& label){
  auto* action=Find(dialog,label);Check(action,"Expected action exists");
  Check(dialog->GetClientRect().Contains(action->GetRect()),"Action remains inside the dialog client area");
  Check(action->GetSize().y>=dialog->FromDIP(48),"Action retains 48 DIP target height");
}
class App final:public wxApp{
 public:
  bool OnInit()override{
    wxLog::SetActiveTarget(new wxLogStderr());frame_=new wxFrame(nullptr,wxID_ANY,"Offline boat setup fixture",{0,0},{1280,800});frame_->Show();
    CallAfter([this]{Run();});return true;
  }
  int OnRun()override{wxApp::OnRun();return result_;}
 private:
  void Run(){try{
    CheckGeometry();
    application::BoatSetupDraft draft;draft.safety_depth_m=3;
    int saves=0,checks=0;bool fail=true;
    ui::BoatSetupActions actions;
    actions.sensors=[&]{++checks;return std::vector<std::string>{"GPS: No data","Depth: Stale — offline test"};};
    actions.save=[&](const auto& value){++saves;Check(value.vessel_name=="Fixture vessel","Entered vessel retained");Check(value.safety_depth_m==3,"Blank chart depth preserves existing value");return application::CommandResult{!fail,fail?"Storage unavailable":"Saved"};};
    auto* dialog=ui::ShowBoatSetupDialog(*frame_,draft,actions,ui::LightMode::Day);
    Check(!dialog->IsModal() && dialog->IsShown(),"Setup cannot block startup health");
    const wxDisplay display(wxDisplay::GetFromWindow(frame_));
    Check(display.GetClientArea().Contains(dialog->GetScreenRect()),"Actual setup fits display workarea");
    CheckActionVisible(dialog,"Later");
    CheckActionVisible(dialog,"Continue");
    auto* name=dynamic_cast<wxTextCtrl*>(wxWindow::FindWindowByName("Vessel name",dialog));Check(name,"Vessel field accessible");name->ChangeValue("Fixture vessel");
    auto* depth=dynamic_cast<wxTextCtrl*>(wxWindow::FindWindowByName(wxString::FromUTF8("Chart safety depth · metres"),dialog));Check(depth,"Depth field accessible");depth->ChangeValue("");
    for(int i=0;i<5;++i)Click(dialog,"Continue");
    Check(checks==2,"Live inventory rechecked at summary");
    Check(saves==0 && Find(dialog,"Save & open helm"),"Review precedes every write");
    CheckActionVisible(dialog,"Save & open helm");
    Click(dialog,"Back");Click(dialog,"Continue");
    Click(dialog,"Save & open helm");Check(saves==1 && dialog->IsShown(),"Failed save leaves review open");
    fail=false;Click(dialog,"Save & open helm");Check(saves==2,"Explicit retry writes only after approval");
    auto* later=ui::ShowBoatSetupDialog(*frame_,draft,actions,ui::LightMode::Night);Click(later,"Later");Check(saves==2,"Later never saves configuration");
    std::cout<<"Boat setup native navigation, deferred save and Later passed\n";
  }catch(const std::exception& e){result_=1;std::cerr<<e.what()<<'\n';}frame_->Destroy();ExitMainLoop();}
  wxFrame* frame_=nullptr;int result_=0;
};
wxIMPLEMENT_APP_NO_MAIN(App);
int main(int argc, char** argv) { return wxEntry(argc, argv); }

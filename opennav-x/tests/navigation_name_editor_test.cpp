// Offline native input fixture; no navigation connection or chart data.
#include "ui/NameEditor.h"
#include <wx/app.h>
#include <wx/frame.h>
#include <wx/log.h>
#include <iostream>
#include <stdexcept>
using namespace opennav;
namespace {
int checks=0;
void Check(bool ok,const char*why){++checks;if(!ok)throw std::runtime_error(why);}
template<class T> T* Find(wxWindow& owner,const wxString& label) {
 for(auto*child:owner.GetChildren()) {
  if(auto*value=dynamic_cast<T*>(child);value && value->GetName()==label)return value;
  if(auto*value=Find<T>(*child,label))return value;
 } return nullptr;
}
void Click(wxWindow&window){wxCommandEvent e(wxEVT_BUTTON,window.GetId());e.SetEventObject(&window);window.ProcessWindowEvent(e);}
class TestApp final:public wxApp {
 wxFrame*frame_=nullptr;int result_=0;
 bool OnInit()override{wxLog::SetActiveTarget(new wxLogStderr());frame_=new wxFrame(nullptr,wxID_ANY,"Name editing fixture",{0,0},{800,400});frame_->Show();CallAfter([this]{Run();});return true;}
 int OnRun()override{wxApp::OnRun();return result_;}
 int OnExit()override{return result_;}
 void Run(){try{
  int saves=0;bool persist=true;std::string stored="Original";
  auto*editor=new ui::XNavNameEditor(*frame_,"Route name",stored,ui::LightMode::Day,100,true,
    [&](const std::string&name){++saves;if(persist)stored=name;return application::CommandResult{persist,"Storage result",{}};},{});
  auto*input=Find<wxTextCtrl>(*editor,"Route name");
  auto*save=Find<ui::XNavButton>(*editor,"Save route name");
  auto*cancel=Find<ui::XNavButton>(*editor,"Cancel route name");
  Check(input&&save&&cancel,"accessible native field actions");
  Check(!save->IsEnabled()&&!cancel->IsEnabled(),"untouched existing name does not save");
  input->SetValue("Draft");Check(editor->DraftChanged()&&save->IsEnabled(),"native typing owns dirty draft");
  Check(ui::XNavNameEditor::PreserveDrafts(*frame_,ui::LightMode::Night,true)&&input->GetValue()=="Draft","theme refresh preserves draft");
  Check(input->GetForegroundColour()==ui::Colour(ui::Theme(ui::LightMode::Night).primary),"theme updates while dirty");
  ui::XNavNameEditor::PreserveDrafts(*frame_,ui::LightMode::Night,false);
  Check(!input->IsEnabled()&&!save->IsEnabled()&&cancel->IsEnabled(),"replay disables save but allows cancel");
  Click(*save);Check(saves==0,"disabled save cannot dispatch");
  Click(*cancel);Check(input->GetValue()=="Original"&&!editor->DraftChanged()&&saves==0,"cancel restores without storage");
  editor->Present(ui::LightMode::Day,true);input->SetValue("   ");Check(!save->IsEnabled(),"blank name cannot save");
  input->SetValue("User override");persist=false;Click(*save);
  Check(saves==1&&stored=="Original"&&editor->DraftChanged()&&input->GetValue()=="User override","failed persistence keeps draft");
  persist=true;Click(*save);Check(saves==2&&stored=="User override"&&!editor->DraftChanged(),"successful save commits exact override");
  input->SetValue("Discard");wxKeyEvent escape(wxEVT_CHAR_HOOK);escape.m_keyCode=WXK_ESCAPE;input->ProcessWindowEvent(escape);
  Check(input->GetValue()=="User override"&&saves==2,"escape cancels to accepted name");
  auto*locked=new ui::XNavNameEditor(*frame_,"Waypoint name","Protected",ui::LightMode::Day,100,false,{},{});
  Check(!Find<wxTextCtrl>(*locked,"Waypoint name")->IsEnabled(),"protected names remain read-only");
  std::cout<<checks<<" native name editor checks passed\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';result_=1;}frame_->Destroy();ExitMainLoop();}
};
}
wxIMPLEMENT_APP(TestApp);

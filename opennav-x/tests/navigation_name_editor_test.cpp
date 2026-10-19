// Offline native input fixture; no navigation connection or chart data.
#include "ui/NameEditor.h"
#include "NavigationNameObservation.h"
#include <wx/app.h>
#include <wx/frame.h>
#include <wx/log.h>
#include <wx/utils.h>
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
#ifdef __WXGTK__
bool OriginalLabelObservation(wxWindow *window,const wxString &text) {
 if(window->IsShownOnScreen()&&window->GetLabel()==text)return true;
 for(auto*child:window->GetChildren())if(OriginalLabelObservation(child,text))return true;
 return false;
}
#endif
void CheckNameObservation(wxFrame &owner) {
 for (bool route : {true, false}) {
  const auto kind=route?ui::ProductPage::RouteDetail:ui::ProductPage::WaypointDetail;
  const wxString name=route?"Route name":"Waypoint name";
  const wxString renamed=route?"BETA TEST externally renamed route":"BETA TEST externally renamed mark";
  application::Route saved_route;saved_route.id="route";saved_route.name="Original";saved_route.editable=true;
  application::Waypoint saved_point;saved_point.id="point";saved_point.name="Original";saved_point.editable=true;
  ui::ProductActions actions;
  actions.navigation.route=[&](const std::string&){return std::optional<application::Route>(saved_route);};
  actions.navigation.waypoint_context=[&](const std::string&,vessel::Time){application::WaypointContext value;value.waypoint=saved_point;return value;};
  actions.navigation.edit_route=[](const application::Route&,const std::string&,const std::string&){return application::CommandResult{};};
  actions.navigation.edit_waypoint=[](const application::Waypoint&,const std::string&,const std::string&){return application::CommandResult{};};
  auto*page=new ui::ProductPanel(&owner,actions);page->SetSize(0,0,760,360);
  page->ShowObject(route?"route":"point",route,ui::LightMode::Day);page->Show();page->Layout();
  auto*field=Find<wxTextCtrl>(*page,name);
  Check(field&&field->IsShownOnScreen(),"actual detail owns visible native name field");
  Check(!ui::XNavNameEditor::PreserveDrafts(*page,ui::LightMode::Day,true),"idle detail has no editing draft or field focus");
  saved_route.name=renamed.ToStdString();saved_route.revision="renamed";
  saved_point.name=renamed.ToStdString();saved_point.revision="renamed";
  wxMilliSleep(1100);page->Update({},ui::LightMode::Day);page->Layout();
  field=Find<wxTextCtrl>(*page,name);
  Check(field&&field->GetValue()==renamed,"actual detail refresh reflects external name change");
#ifdef __WXGTK__
  Check(field->GetLabel()!=renamed,"native field value differs from obsolete GTK label observer");
  Check(!OriginalLabelObservation(&owner,renamed),"original recursive label guard misses actual refreshed name");
#endif
  Check(test::HasVisibleNavigationName(owner,kind,renamed),"owned detail exact native name is observed");
  auto*stale=new wxStaticText(&owner,wxID_ANY,renamed);
  field->ChangeValue("Wrong value");
  Check(!test::HasVisibleNavigationName(owner,kind,renamed),"unrelated matching label cannot hide wrong native value");
  field->ChangeValue(renamed);field->Hide();
  Check(!test::HasVisibleNavigationName(owner,kind,renamed),"hidden native name is refused");field->Show();
  field->SetName("Other field");
  Check(!test::HasVisibleNavigationName(owner,kind,renamed),"wrong field identity is refused");field->SetName(name);
  auto*duplicate=new wxTextCtrl(page,wxID_ANY,renamed);duplicate->SetName(name);
  Check(!test::HasVisibleNavigationName(owner,kind,renamed),"duplicate matching field is refused");delete duplicate;
  auto*other=new ui::ProductPanel(&owner,actions);other->SetSize(0,0,760,360);
  other->ShowObject(route?"route":"point",route,ui::LightMode::Day);other->Show();other->Layout();
  Check(!test::HasVisibleNavigationName(owner,kind,renamed),"duplicate selected detail page is refused");delete other;
  page->Hide();
  Check(!test::HasVisibleNavigationName(owner,kind,renamed),"hidden selected detail page is refused");page->Show();
  Check(!test::HasVisibleNavigationName(owner,route?ui::ProductPage::WaypointDetail:ui::ProductPage::RouteDetail,renamed),
        "value on another detail page cannot satisfy selection");
  Check(!test::HasVisibleNavigationName(owner,ui::ProductPage::Routes,renamed),"non-detail page cannot satisfy selection");
  delete field;
  Check(!test::HasVisibleNavigationName(owner,kind,renamed),"missing native name field is refused");
  delete stale;delete page;
 }
#ifdef __WXGTK__
 std::cout<<"Route and waypoint external refresh: original GTK label guard misses both; owned native value reader passes\n";
#endif
}
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
  delete editor;delete locked;CheckNameObservation(*frame_);
  std::cout<<checks<<" native name editor checks passed\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';result_=1;}frame_->Destroy();ExitMainLoop();}
};
}
wxIMPLEMENT_APP(TestApp);

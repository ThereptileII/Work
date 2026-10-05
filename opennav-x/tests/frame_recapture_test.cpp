// Offline native GTK stacking regression using the extracted production boundary.
#include "ui/FloatingSurface.h"
#include <wx/app.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/timer.h>
#include <wx/uiaction.h>
#include <cstdlib>
#include <gtk/gtk.h>
#include <gdk/gdkx.h>
#include <X11/Xlib.h>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <vector>
namespace opennav::ui {
class Shell {
 public:
  std::vector<wxWindow*> chart_overlays_;
  void RestackChartControls();
};
#include "shell-restack.inc"
}
namespace opennav {
ui::Shell* shell=nullptr;
bool xnav=true,transient=false;
bool IsXNav(){return xnav;}
bool HasXNavTransientSurface(){return xnav && transient;}
#include "integration-recapture.inc"
}
class MyFrame : public wxFrame {
 public:
  MyFrame():wxFrame(nullptr,wxID_ANY,"TEST recapture owner",{0,0},{1280,800},wxBORDER_NONE){}
  void OnRecaptureTimer(wxTimerEvent&);
  void Raise() override {++raises;wxFrame::Raise();}
  int raises=0;
};
#include "frame-recapture.inc"
namespace {
class App final:public wxApp {
 public:
 bool OnInit() override {
  owner=new MyFrame;canvas=new wxPanel(owner,wxID_ANY);
  auto* layout=new wxBoxSizer(wxVERTICAL);layout->Add(canvas,1,wxEXPAND);owner->SetSizer(layout);
  canvas->Bind(wxEVT_LEFT_DOWN,[this](wxMouseEvent&){++canvas_clicks;});
  surface=new opennav::ui::XNavFloatingSurface(*owner,"TEST recapture overlay");
  auto* button=new wxButton(surface,wxID_ANY,"+",wxDefaultPosition,{44,44});
  button->Bind(wxEVT_BUTTON,[this](wxCommandEvent&){++clicks;});
  auto* tools=new wxBoxSizer(wxHORIZONTAL);tools->Add(button);surface->SetSizerAndFit(tools);
  model.chart_overlays_.push_back(surface);opennav::shell=&model;
  owner->Show();surface->Present({980,549});
  timer.SetOwner(this);Bind(wxEVT_TIMER,&App::Step,this);timer.StartOnce(150);return true;
 }
 int OnRun() override{wxApp::OnRun();return failed?1:0;}
 private:
 void Check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);++checks;}
 static Window Id(wxWindow* w){return GDK_WINDOW_XID(gtk_widget_get_window(w->GetHandle()));}
 bool NativeAbove(wxWindow* a,wxWindow* b){
  auto* d=GDK_WINDOW_XDISPLAY(gtk_widget_get_window(owner->GetHandle()));
  XSync(d,False);Window root,parent,*children=nullptr;unsigned count=0;
  Check(XQueryTree(d,DefaultRootWindow(d),&root,&parent,&children,&count)!=0,"native X tree available");
  int ai=-1,bi=-1;for(unsigned i=0;i<count;++i){if(children[i]==Id(a))ai=i;if(children[i]==Id(b))bi=i;}
  if(children)XFree(children);Check(ai>=0&&bi>=0,"owner and surface retain native identities");return ai>bi;
 }
 void Fire(){wxTimerEvent event(timer);owner->OnRecaptureTimer(event);}
 void Capture(const char* name) {
  // Root-window pixels include independently owned GTK surfaces. wxScreenDC
  // can omit these surfaces under Xvfb, so use the same native collector as
  // the integrated app evidence. Names are fixed literals owned by this test.
  Check(std::system((std::string("import -window root ")+name).c_str())==0,
        "capture actual native root pixels");
 }

 void Step(wxTimerEvent&){try{switch(step++){
  case 0:
   canvas->SetFocus();surface->Present({980,549});break;
  case 1:{
   Check(NativeAbove(surface,owner),"initial surface above owner");
   owner->Raise();Check(NativeAbove(owner,surface),"actual owner Raise reproduces inversion");
   break;}
  case 2:{
   Check(NativeAbove(owner,surface),"owner still obscures control after native repaint opportunity");
   Capture("before-repair.png");
   surface->RestackAboveOwner();Check(NativeAbove(surface,owner),"native restack repairs inversion");
   const auto position=surface->GetPosition();auto* focus=wxWindow::FindFocus();
   Fire();Check(NativeAbove(surface,owner),"production recapture restores surface before returning");
   Check(surface->GetPosition()==position,"repair does not move control");
   Check(wxWindow::FindFocus()==focus,"repair does not change focus");
   break;}
  case 3:{
   Check(NativeAbove(surface,owner),"repaired order survives the next native event cycle");
   Capture("after-repair.png");
   wxUIActionSimulator pointer;Check(pointer.MouseMove({1002,571})&&pointer.MouseClick(),"click repaired native button");break;}
  case 4:{
   Check(clicks==1&&canvas_clicks==0,"repaired control receives click instead of chart");
   opennav::xnav=false;Fire();Check(NativeAbove(owner,surface),"Legacy recapture retains upstream raise");
   opennav::xnav=true;surface->RestackAboveOwner();
   opennav::transient=true;int before=owner->raises;Fire();Check(owner->raises==before,"existing transient surface guard suppresses raise");
   opennav::transient=false;surface->Hide();Fire();Check(!gtk_widget_get_visible(surface->GetHandle()),"recapture does not show explicitly hidden surface");
   Check(!surface->IsShown(),"logical hidden state unchanged");
   surface->Present({980,549});break;}
  case 5:{
   other=new wxFrame(nullptr,wxID_ANY,"TEST unrelated window",{0,0},{80,80},wxBORDER_NONE);other->Show();other->Raise();break;}
  case 6:{
   auto* focus=wxWindow::FindFocus();surface->RestackAboveOwner();
   Check(NativeAbove(other,surface),"restack stays below unrelated higher window");
   Check(wxWindow::FindFocus()==focus,"restack does not steal unrelated focus");
   std::cout<<checks<<" native recapture checks passed\n";Finish();return;}
 }timer.StartOnce(150);}catch(const std::exception& e){std::cerr<<"FAILED: "<<e.what()<<'\n';failed=true;Finish();}}
 void Finish(){timer.Stop();opennav::shell=nullptr;if(other)other->Destroy();surface->Destroy();owner->Destroy();ExitMainLoop();}
 MyFrame* owner=nullptr;wxPanel* canvas=nullptr;opennav::ui::XNavFloatingSurface* surface=nullptr;wxFrame* other=nullptr;
 opennav::ui::Shell model;wxTimer timer;int step=0,checks=0,clicks=0,canvas_clicks=0;bool failed=false;
};
}
wxIMPLEMENT_APP_NO_MAIN(App);
int main(int argc,char** argv){return wxEntry(argc,argv);}

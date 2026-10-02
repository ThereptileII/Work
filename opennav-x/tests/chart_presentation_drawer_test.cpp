// Offline, non-installed interaction driver. All chart values are synthetic.
#include "ui/ChartPresentationDrawer.h"
#include <wx/app.h>
#include <wx/dcscreen.h>
#include <wx/dcmemory.h>
#include <wx/filename.h>
#include <wx/timer.h>
#include <wx/log.h>
#include <fstream>
#include <cstdlib>
#include <iostream>
#ifdef __WXGTK__
#include <gtk/gtk.h>
#endif
using namespace opennav;
class TestApp:public wxApp {
 wxFrame *host=nullptr;ui::XNavChartPresentationDrawer *drawer=nullptr;
 application::ChartPresentationState state;wxTimer timer;wxString out;
 bool finished=false;
 int step=0,checks=0,ais=0,enc=0,sounding=0,orient=0,style=0,paints=0;ui::LightMode light=ui::LightMode::Day;
 void Check(bool value,const char *message){++checks;if(!value){std::cerr<<message<<std::endl;std::exit(1);}}
 wxWindow *Find(wxWindow *root,const wxString &label){for(auto *child:root->GetChildren()){if(child->GetLabel()==label)return child;if(auto *nested=Find(child,label))return nested;}return nullptr;}
 ui::XNavButton *Button(const char *name){return dynamic_cast<ui::XNavButton *>(Find(drawer,name));}
 ui::XNavScroll *Body(){for(auto *child:drawer->GetChildren())if(auto *body=dynamic_cast<ui::XNavScroll *>(child))return body;return nullptr;}
 void Click(const char *name){auto *b=Button(name);Check(b&&b->IsShownOnScreen()&&b->IsEnabled(),name);wxCommandEvent event(wxEVT_BUTTON,b->GetId());event.SetEventObject(b);b->ProcessWindowEvent(event);}
 void Refresh(){drawer->Update(state,light);}
 void WatchPaint(wxWindow *root){root->Bind(wxEVT_PAINT,[this](wxPaintEvent &e){++paints;e.Skip();});for(auto *child:root->GetChildren())WatchPaint(child);}
 void Capture(const char *name){
 Check(drawer->GetScreenRect()==wxRect(host->ClientToScreen({682,80}),wxSize(398,674)),"398x674 canonical drawer");
 const auto origin=host->ClientToScreen({0,0});
#ifdef __WXGTK__
 auto *pixels=gdk_pixbuf_get_from_window(gdk_get_default_root_window(),origin.x,origin.y,1280,800);Check(pixels,"actual screen capture");Check(gdk_pixbuf_save(pixels,(out+"/"+name+".png").utf8_str(),"png",nullptr,nullptr),"save capture");g_object_unref(pixels);
#else
 wxScreenDC screen;wxBitmap bitmap(1280,800);wxMemoryDC memory(bitmap);
 Check(memory.Blit(0,0,1280,800,&screen,origin.x,origin.y),"actual screen capture");memory.SelectObject(wxNullBitmap);
 Check(bitmap.SaveFile(out+"/"+name+".png",wxBITMAP_TYPE_PNG),"save capture");
#endif
 }

 public:bool OnInit()override {
 wxLog::SetActiveTarget(new wxLogStderr());
 wxSetAssertHandler([](const wxString &,int,const wxString &,const wxString &condition,const wxString &){std::cerr<<condition<<std::endl;std::abort();});
 if(argc!=2)return false;out=argv[1];wxFileName::Mkdir(out,wxS_DIR_DEFAULT,wxPATH_MKDIR_FULL);wxInitAllImageHandlers();
 host=new wxFrame(nullptr,wxID_ANY,"OFFLINE TEST ONLY - chart presentation",{0,0},{1280,800},wxBORDER_NONE);host->SetClientSize(1280,800);host->SetBackgroundColour(ui::Colour(ui::Theme(light).background));
 state.available=true;state.format=application::ChartFormat::Vector;state.format_reason="Synthetic quilt reference: Vector; other members may differ";state.orientation=application::ChartOrientation::NorthUp;
 state.ais_vessels={true,true,""};state.enc_text={false,true,""};state.depth_soundings={true,true,""};
 application::NavigationActions actions;actions.chart_presentation=[this]{return state;};
 actions.set_chart_ais=[this](bool value){++ais;state.ais_vessels.visible=value;return application::ChartPresentationResult{{true,""},state};};
 actions.set_chart_enc_text=[this](bool value){++enc;Check(value,"request inverse ENC preference");return application::ChartPresentationResult{{false,"Synthetic rejection; native preference unchanged"},state};};
 actions.set_chart_soundings=[this](bool value){++sounding;Check(!value,"request hide soundings");return application::ChartPresentationResult{{true,"Synthetic unchanged native readback"},state};};
 actions.set_chart_orientation=[this](application::ChartOrientation value){++orient;state.orientation=value;return application::ChartPresentationResult{{true,""},state};};
 drawer=new ui::XNavChartPresentationDrawer(*host,actions);drawer->on_style_preferences=[this]{++style;};Refresh();host->Show();drawer->Open(wxRect(host->ClientToScreen({80,68}),wxSize(1014,698)),state,light);WatchPaint(drawer);
 timer.SetOwner(this);Bind(wxEVT_TIMER,&TestApp::Step,this);timer.StartOnce(200);return true;
 }
 void Step(wxTimerEvent &){if(finished)return;switch(step++) {
 case 0:Check(Button("AIS vessels")->IsSelected(),"observed AIS on");Check(!Button("ENC text labels")->IsSelected(),"observed ENC text off");Check(Button("Depth soundings")->IsSelected(),"observed soundings on");Check(!Find(drawer,"Vector")&&!Find(drawer,"Raster"),"format has no actionable button");Check(!Find(drawer,"Chart symbols")&&!Find(drawer,"Depth contours"),"managed layers have no action");Capture("chart-day-top");Click("AIS vessels");break;
 case 1:Check(ais==1&&!Button("AIS vessels")->IsSelected(),"AIS uses actual readback");Click("ENC text labels");break;
 case 2:Check(enc==1&&!Button("ENC text labels")->IsSelected(),"rejected ENC request never selects optimistic state");Click("Depth soundings");break;
 case 3:Check(sounding==1&&Button("Depth soundings")->IsSelected(),"successful command still renders native readback");Body()->Scroll(0,40);break;
 case 4:Click("Course up");break;
 case 5:Check(orient==1&&Button("Course up")->IsSelected(),"explicit Course up readback");Click("Head up");break;
 case 6:Check(orient==2&&Button("Head up")->IsSelected(),"explicit Head up readback");Click("North up");break;
 case 7:Check(orient==3&&Button("North up")->IsSelected(),"explicit North up readback");Body()->Scroll(0,1000);break;
 case 8:Capture("chart-day-bottom");Click("Chart palette preferences");break;
 case 9:Check(style==1,"palette settings action separate from chart format");state.format=application::ChartFormat::Raster;state.format_reason="Synthetic current chart: Raster";state.enc_text.editable=false;state.enc_text.reason="Raster chart text is image content";state.depth_soundings.editable=false;state.depth_soundings.reason="Raster chart soundings are image content";light=ui::LightMode::Dusk;Refresh();Body()->Scroll(0,0);break;
 case 10:Check(!Button("ENC text labels")->IsEnabled()&&!Button("Depth soundings")->IsEnabled(),"raster ENC controls disabled");Check(Button("Depth soundings")->IsSelected(),"raster preserves observed ENC preference");Capture("chart-dusk-raster");state={};state.reason="Synthetic chart canvas unavailable";light=ui::LightMode::Night;Refresh();break;
 case 11:Check(!Button("AIS vessels")->IsShown()&&!Button("ENC text labels")->IsShown()&&!Button("Depth soundings")->IsShown(),"unobserved layers are not false off toggles");Check(!Button("North up")->IsEnabled()&&!Button("North up")->IsSelected(),"unavailable orientation has no selected guess");Capture("chart-night-unavailable");state.available=true;state.orientation=application::ChartOrientation::NorthUp;state.ais_vessels={true,true,""};Refresh();Body()->Scroll(0,0);break;
 case 12:Click("AIS vessels");drawer->Dismiss();drawer->Open(wxRect(host->ClientToScreen({80,68}),wxSize(1014,698)),state,light);break;
 case 13:Check(ais==1,"queued toggle invalidated across explicit reopen");Body()->Scroll(0,40);Click("Course up");drawer->Dismiss();drawer->Present(wxRect(host->ClientToScreen({80,68}),wxSize(1014,698)));break;
 case 14:{Check(orient==3,"queued orientation invalidated by dismissal even without Open");const auto offset=Body()->GetViewStart();Check(offset.y>0,"scrolled orientation viewport retained by Present");const int previous=paints;Refresh();wxTheApp->Yield(true);Check(paints==previous,"identical Update causes no paint");Check(Body()->GetViewStart()==offset,"identical Update preserves scroll");drawer->Open(wxRect(host->ClientToScreen({80,68}),wxSize(1014,698)),state,light);break;}
 case 15:Check(Body()->GetViewStart()==wxPoint(0,0),"explicit Open resets scroll");drawer->SetInterfaceScale(150);drawer->Present(wxRect(host->ClientToScreen({80,68}),wxSize(1014,698)));Check(host->GetScreenRect().Contains(drawer->GetScreenRect()),"scaled component remains contained");finished=true;timer.Stop();std::ofstream((out+"/result.json").ToStdString())<<"{\"passed\":true,\"checks\":"<<checks<<",\"fixture_only\":true,\"captures\":[\"chart-day-top\",\"chart-day-bottom\",\"chart-dusk-raster\",\"chart-night-unavailable\"]}";drawer->Destroy();host->Destroy();ExitMainLoop();break;
 }if(!finished)timer.StartOnce(200);}
};wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc,char **argv) {return wxEntry(argc,argv);}

// Offline UI/owned-model checks only. No OpenCPN profile, chart or equipment.
#include "ui/SearchDrawer.h"
#include "ui/Shell.h"
#include <wx/app.h>
#include <wx/dcscreen.h>
#include <wx/dcmemory.h>
#include <wx/filename.h>
#include <wx/log.h>
#include <wx/timer.h>
#include <wx/uiaction.h>
#include <iostream>
#include <clocale>
#include <stdexcept>
#ifdef __WXGTK__
#include <gtk/gtk.h>
#endif
using namespace opennav;
namespace {
class TestApp final : public wxApp {
public:
  bool OnInit() override {
    if (argc!=2) return false;
    // The real OpenCPN app initializes the user locale before constructing UI.
    std::setlocale(LC_CTYPE, "");
    wxLog::SetActiveTarget(new wxLogStderr());
    wxSetAssertHandler([](const wxString &,int,const wxString &,const wxString &s,const wxString &){std::cerr<<s<<'\n';std::abort();});
    output_=argv[1]; wxFileName::Mkdir(output_,wxS_DIR_DEFAULT,wxPATH_MKDIR_FULL); wxInitAllImageHandlers();
    frame_=new wxFrame(nullptr,wxID_ANY,"TEST ONLY - saved-object search",{0,0},{1280,800},wxBORDER_NONE);
    frame_->SetClientSize(1280,800);frame_->SetBackgroundColour(ui::Colour(ui::Theme(ui::LightMode::Day).surface));
    application::NavigationActions actions;
    actions.catalog=[this]{return catalog_;};
    actions.view_waypoint=[this](const std::string &id){++centers_;last_=id;return application::CommandResult{true,"Viewed"};};
    actions.view_route=[this](const std::string &){++route_views_;};
    actions.activate=[this](const application::Route &){++mutations_;return application::CommandResult{};};
    drawer_=new ui::XNavSearchDrawer(*frame_,actions);
    drawer_->on_select=[this](const std::string &id,bool route){++selected_;last_=id;last_route_=route;};
    frame_->Show();frame_->Raise();
    timer_.SetOwner(this);Bind(wxEVT_TIMER,&TestApp::Step,this);timer_.StartOnce(180);
    return true;
  }
  int OnRun() override {wxApp::OnRun();return failed_?1:0;}
private:
  void Check(bool ok,const char *message) {if(!ok)throw std::runtime_error(message);++checks_;}
  wxRect Workspace(){return {80,68,1014,698};}
  wxTextCtrl *Input(){return dynamic_cast<wxTextCtrl *>(wxWindow::FindWindowByName("Search saved routes and waypoints",drawer_));}
  ui::XNavButton *Row(const wxString &name){return dynamic_cast<ui::XNavButton *>(wxWindow::FindWindowByName(name,drawer_));}
  void Click(const wxString &name) {
    auto *row=Row(name);Check(row!=nullptr,"requested result exists");
    wxCommandEvent e(wxEVT_BUTTON,row->GetId());e.SetEventObject(row);row->ProcessWindowEvent(e);
  }
  ui::XNavButton *Button(wxWindow *parent,const char *name) {
    auto *button=dynamic_cast<ui::XNavButton *>(wxWindow::FindWindowByName(name,parent));
    Check(button && button->IsShownOnScreen() && button->IsEnabled(),name);
    return button;
  }
  void Press(wxWindow *parent,const char *name) {
    auto *button=Button(parent,name);
    wxCommandEvent event(wxEVT_BUTTON,button->GetId());event.SetEventObject(button);
    button->ProcessWindowEvent(event);
  }
  void Capture(const wxString &name) {
    const auto size=frame_->GetClientSize();const auto origin=frame_->ClientToScreen({0,0});
#ifdef __WXGTK__
    // wxScreenDC can reuse a stale root bitmap on GTK; capture fresh pixels.
    auto *pixels=gdk_pixbuf_get_from_window(gdk_get_default_root_window(),origin.x,origin.y,size.x,size.y);
    Check(pixels!=nullptr,"fresh root-window pixels");
    const bool saved=gdk_pixbuf_save(pixels,(output_+"/"+name+".png").utf8_str(),"png",nullptr,nullptr);
    g_object_unref(pixels);Check(saved,"save component capture");
#else
    wxScreenDC screen;wxBitmap bitmap(size.x,size.y);wxMemoryDC dc(bitmap);
    Check(dc.Blit(0,0,size.x,size.y,&screen,origin.x,origin.y),"capture pixels");dc.SelectObject(wxNullBitmap);
    Check(bitmap.SaveFile(output_+"/"+name+".png",wxBITMAP_TYPE_PNG),"save component capture");
#endif
  }
  void CheckSearchButton() {
    auto *search=dynamic_cast<ui::XNavButton *>(wxWindow::FindWindowByName("Search saved routes and waypoints",frame_));
    Check(search && search->IsShownOnScreen(),"integrated Search entry visible");
    const auto bounds=search->GetScreenRect();
    Check(bounds.width==44 && bounds.height==44,"Search entry retains prototype touch size");
    Check(frame_->GetScreenRect().Contains(bounds),"Search entry fits current viewport");
    for (auto *sibling:search->GetParent()->GetChildren())
      if (sibling!=search && sibling->IsShownOnScreen())
        Check(!bounds.Intersects(sibling->GetScreenRect()),"Search does not overlap another top-bar control");
  }
  void Models() {
    application::Waypoint w;w.id="w1";w.name="TEST waypoint Å";catalog_.waypoints.push_back(w);
    application::Route r;r.id="r1";r.name="TEST route";catalog_.routes.push_back(r);
    Check(ui::FindNavigationObjects(catalog_,"wayPOINT").rows.size()==1,"case-insensitive substring");
    Check(ui::FindNavigationObjects(catalog_,wxString::FromUTF8("å")).rows.size()==1,"Unicode names remain searchable");
    auto copy=ui::FindNavigationObjects(catalog_,"waypoint");
    auto changed=catalog_;changed.waypoints.clear();
    Check(copy.rows.front().name==wxString::FromUTF8("TEST waypoint Å"),"search values owned independently of source catalog");
    Check(!ui::HasUniqueSearchObject(changed,copy.rows.front()),"deleted identity rejected");
    changed=catalog_;changed.waypoints.push_back(w);
    Check(ui::FindNavigationObjects(changed,"").rows.size()==1,"all ambiguous waypoint identities excluded");
    Check(!ui::HasUniqueSearchObject(changed,copy.rows.front()),"ambiguous identity rejected");
    changed=catalog_;changed.routes.front().id=w.id;
    Check(ui::FindNavigationObjects(changed,"").rows.size()==2,"route and waypoint namespaces distinct");
    changed={};
    for(int i=0;i<105;++i){w.id=std::to_string(i);changed.waypoints.push_back(w);}
    auto bounded=ui::FindNavigationObjects(changed,"");
    Check(bounded.rows.size()==100 && bounded.limited,"large catalog bounded explicitly");
    changed={};changed.truncated=true;
    Check(ui::FindNavigationObjects(changed,"").limited,"upstream catalog truncation retained");
    w.id="";changed.waypoints.push_back(w);
    Check(ui::FindNavigationObjects(changed,"").rows.empty(),"empty identities cannot select");
  }
  void Step(wxTimerEvent &) {
    if (finished_) return;
    int next_delay=180;
    try {
      switch(step_++) {
      case 0:
        Models();drawer_->Open(Workspace(),ui::LightMode::Day);break;
      case 1:
        {const auto r=drawer_->GetScreenRect();std::cout<<"drawer "<<r.x<<","<<r.y<<","<<r.width<<","<<r.height<<"\n";}
        Check(drawer_->GetScreenRect()==wxRect(682,80,398,674),"reference drawer bounds");
        Check(Input()->HasFocus(),"opening focuses search input");
        Check(Row("TEST route: Saved route")!=nullptr,"saved route exposed");
        Capture("saved-objects-day");
        Click(wxString::FromUTF8("TEST waypoint Å: Saved waypoint"));catalog_.waypoints.clear();break;
      case 2:
        Check(selected_==0 && centers_==0,"deletion before deferred selection neither centers nor selects");
        Check(Row(wxString::FromUTF8("TEST waypoint Å: Saved waypoint"))==nullptr,"deleted row removed");
        Click("TEST route: Saved route");break;
      case 3:
        Check(selected_==1 && last_=="r1" && last_route_,"route selects only owned identity");
        Check(route_views_==0 && mutations_==0 && centers_==0,"route selection does not persist visibility or activate");
        Click("TEST route: Saved route");drawer_->Dismiss();drawer_->Open(Workspace(),ui::LightMode::Day);break;
      case 4:
        Check(selected_==1,"selection queued before dismissal cannot enter reopened drawer");
        Input()->SetValue("missing");Check(Row("TEST route: Saved route")==nullptr,"input filters immediately");
        drawer_->Update(ui::LightMode::Night);break;
      case 5:
        Capture("empty-night");Input()->SetValue("ROUTE");Click("TEST route: Saved route");Input()->SetValue("missing");break;
      case 6:
        Check(selected_==1,"queued selection invalidated by changed query");
        {
          wxKeyEvent e(wxEVT_CHAR_HOOK);e.m_keyCode=WXK_ESCAPE;e.SetEventObject(Input());
          Check(drawer_->FilterEvent(e)==wxEventFilter::Event_Processed && !drawer_->IsShown(),"Escape dismisses root search drawer");
        }
        {
          application::Waypoint w;w.id="w2";w.name="TEST restored waypoint";catalog_.waypoints.push_back(w);
          drawer_->Open(Workspace(),ui::LightMode::Day);
          auto *row=Row("TEST restored waypoint: Saved waypoint");Check(row!=nullptr,"restored owned result exists");
          row->SetFocus();
        }
        break;
      case 7:
        {
          auto *row=Row("TEST restored waypoint: Saved waypoint");Check(row->HasFocus(),"result can receive keyboard focus");
          wxUIActionSimulator input;Check(input.Char(WXK_RETURN),"send native Return key to result");
        }
        break;
      case 8:
        Check(selected_==2 && centers_==1 && last_=="w2" && !last_route_,"Return centers and selects exactly one saved waypoint");
        Check(route_views_==0 && mutations_==0,"search has no activation or route visibility mutation");
        Click("TEST route: Saved route");drawer_->Destroy();drawer_=nullptr;break;
      case 9:
        Check(selected_==2,"destroyed drawer discards pending selection");
        {
          manager_=std::make_unique<wxAuiManager>(frame_);
          auto *chart=new wxPanel(frame_,wxID_ANY);chart_host_=chart;chart->SetBackgroundColour(ui::Colour(ui::Theme(ui::LightMode::Day).surface));
          manager_->AddPane(chart,wxAuiPaneInfo().CenterPane().Name("offline-chart"));
          ui::ShellActions actions;actions.navigation_panes={"offline-chart"};
          actions.navigation.catalog=[this]{return catalog_;};
          chart_state_.available=true;chart_state_.format=application::ChartFormat::Vector;
          chart_state_.format_reason="OFFLINE TEST chart state";
          chart_state_.ais_vessels={true,true,{}};
          actions.navigation.chart_presentation=[this]{++chart_reads_;return chart_state_;};
          actions.navigation.set_chart_ais=[this](bool requested) {
            ++layer_requests_;last_layer_request_=requested;
            return application::ChartPresentationResult{{false,"OFFLINE TEST rejected layer request"},chart_state_};
          };
          shell_=std::make_unique<ui::Shell>(*frame_,*manager_,std::move(actions),ui::LightMode::Day,false);
        }
        break;
      case 10:
        CheckSearchButton();Capture("shell-1280");frame_->SetClientSize(853,600);frame_->Layout();break;
      case 11:
        CheckSearchButton();Capture("shell-853");
        {
          auto *search=wxWindow::FindWindowByName("Search saved routes and waypoints",frame_);
          wxCommandEvent e(wxEVT_BUTTON,search->GetId());e.SetEventObject(search);search->ProcessWindowEvent(e);
        }
        break;
      case 12:
        Check(shell_->DrawerRegion().has_value(),"integrated Search button opens drawer");
        Check(frame_->GetScreenRect().Contains(*shell_->DrawerRegion()),"compact Search drawer remains within viewport");
        Capture("shell-853-search");
        // Exercise the actual Shell entry points; component behavior has its
        // own focused harness. Keep all existing Search captures unchanged.
        {
          auto *search=dynamic_cast<ui::XNavSearchDrawer *>(wxWindow::FindWindowByName("Chart search",frame_));
          Check(search && search->IsShownOnScreen(),"Search is open before closing it to reach Layers");
          Press(search,"Close sheet");
        }
        next_delay=350;break; // Shell's 250ms placement tick restores overlays.
      case 13:
        Press(frame_,"Chart layers");break;
      case 14:
        layers_drawer_=dynamic_cast<ui::XNavChartPresentationDrawer *>(wxWindow::FindWindowByName("Chart presentation",frame_));
        Check(layers_drawer_ && layers_drawer_->IsShownOnScreen() &&
              shell_->DrawerRegion()==layers_drawer_->GetScreenRect(),"Layers opens the integrated chart drawer");
        layer_toggle_=Button(layers_drawer_,"AIS vessels");
        Check(chart_reads_>0 && layer_toggle_->IsSelected(),"Layers displays copied navigation state");
        Press(layers_drawer_,"AIS vessels");break;
      case 15:
        Check(layer_requests_==1 && !last_layer_request_ && layer_toggle_->IsSelected(),
              "Shell forwards one failed layer action and retains unchanged readback");
        {
          wxKeyEvent event(wxEVT_CHAR_HOOK);event.m_keyCode=WXK_ESCAPE;event.SetEventObject(layer_toggle_);
          Check(layers_drawer_->FilterEvent(event)==wxEventFilter::Event_Processed,"chart drawer consumes Escape");
        }
        Check(!shell_->DrawerRegion(),"Escape closes the integrated chart drawer");
        chart_state_.ais_vessels.visible=false;
        Press(frame_,"Open navigation menu");break;
      case 16:
        settings_drawer_=dynamic_cast<ui::XNavSettingsDrawer *>(wxWindow::FindWindowByName("OpenNav preferences",frame_));
        Check(settings_drawer_ && settings_drawer_->IsShownOnScreen(),"Settings opens through the Shell navigation entry");
        Press(settings_drawer_,"Settings section: Navigation");break;
      case 17:
        Press(settings_drawer_,"Chart presentation");break;
      case 18:
        Check(layers_drawer_->IsShownOnScreen() && !settings_drawer_->IsShown(),
              "Settings Navigation entry replaces Settings with chart presentation");
        Check(!layer_toggle_->IsSelected(),"Settings entry opens fresh copied chart readback");
        Press(layers_drawer_,"Close sheet");break;
      case 19:
        Check(!layers_drawer_->IsShown() && !shell_->DrawerRegion(),"Close returns to the chart without a lingering drawer");
        // Retain the same compact breakpoint; change only available chart
        // height to exercise the Layers/tool-strip collision boundary.
        frame_->SetClientSize(853,frame_->GetClientSize().y-chart_host_->GetSize().y+250);
        frame_->Layout();next_delay=350;break;
      case 20:
        {
          auto *layers=wxWindow::FindWindowByName("OpenNav chart layers",frame_);
          auto *tools=wxWindow::FindWindowByName("OpenNav chart tools",frame_);
          Check(chart_host_->IsShownOnScreen() && chart_host_->GetSize().y==250,
                "short-canvas integration check reaches the 250px overlap boundary");
          Check(layers && tools && tools->IsShownOnScreen() && !layers->IsShown(),
                "Layers hides while the short chart keeps its bottom tool strip visible");
        }
        std::cout<<checks_<<" focused search checks passed; native Windows and boat remain pending\n";
        finished_=true;timer_.Stop();shell_.reset();manager_->UnInit();manager_.reset();frame_->Destroy();ExitMainLoop();break;
      }
    } catch(const std::exception &e) {std::cerr<<"FAILED: "<<e.what()<<'\n';failed_=true;finished_=true;timer_.Stop();shell_.reset();if(manager_){manager_->UnInit();manager_.reset();}frame_->Destroy();ExitMainLoop();}
    if (!finished_) timer_.StartOnce(next_delay);
  }
  std::unique_ptr<wxAuiManager> manager_;std::unique_ptr<ui::Shell> shell_;
  wxString output_;wxFrame *frame_=nullptr;ui::XNavSearchDrawer *drawer_=nullptr;
  wxPanel *chart_host_=nullptr;
  ui::XNavChartPresentationDrawer *layers_drawer_=nullptr;
  ui::XNavSettingsDrawer *settings_drawer_=nullptr;
  ui::XNavButton *layer_toggle_=nullptr;
  application::ChartPresentationState chart_state_;
  int chart_reads_=0,layer_requests_=0;bool last_layer_request_=true;
  application::Catalog catalog_;wxTimer timer_;int step_=0,checks_=0,selected_=0,centers_=0,route_views_=0,mutations_=0;
  bool failed_=false,last_route_=false,finished_=false;std::string last_;
};
}
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc,char **argv) {return wxEntry(argc,argv);}

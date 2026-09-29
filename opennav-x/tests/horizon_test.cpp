// Dedicated offline component process. Literal HTML data cannot enter product.
#include "ui/Horizon.h"
#include <wx/app.h>
#include <wx/dcscreen.h>
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
#include <cmath>
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
  bool OnInit() override {
    wxLog::SetActiveTarget(new wxLogStderr());
    wxSetAssertHandler([](const wxString &,int,const wxString &,const wxString &s,const wxString &){std::cerr<<s<<std::endl;std::abort();});
    if(argc!=2)return false;output_=argv[1];
    if(!wxFileName::Mkdir(output_,wxS_DIR_DEFAULT,wxPATH_MKDIR_FULL))return false;
    wxInitAllImageHandlers();
    frame_=new wxFrame(nullptr,wxID_ANY,"TEST ONLY - Horizon",{0,0},{1280,800},wxBORDER_NONE);
    Resize(1280,800);frame_->SetBackgroundColour(ui::Colour(ui::Theme(mode_).surface));
    host_=new wxPanel(frame_,wxID_ANY);
    auto *layout=new wxBoxSizer(wxVERTICAL);
    layout->Add(host_,1,wxEXPAND);frame_->SetSizer(layout);frame_->Layout();
    horizon_=new ui::XNavHorizon(host_,[this]{++passage_;},[this](const auto &a){
      if(application::HorizonActionAllowed(a,state_,ais_,now_)){++actions_;last_=a;}
    });
    horizon_->SetSize(80,634,1014,132);frame_->Show();frame_->Raise();
    timer_.SetOwner(this);Bind(wxEVT_TIMER,&TestApp::Step,this);timer_.Start(250);return true;
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
    const auto frame=NativeRect(frame_),host=NativeRect(host_),component=NativeRect(horizon_);
    std::ofstream out((output_+"/native-layout.jsonl").ToStdString(),std::ios::app);
    const auto rect=[&](const wxRect &r){out<<'['<<r.x<<','<<r.y<<','<<r.width<<','<<r.height<<']';};
    const auto window=[&](const char *name,wxWindow *w,const wxRect &r){
      out<<",\""<<name<<"\":{\"native_handle\":"<<reinterpret_cast<std::uintptr_t>(w->GetHandle())
         <<",\"shown\":"<<(w->IsShownOnScreen()?"true":"false")<<",\"screen\":";rect(r);out<<'}';};
    out<<"{\"phase\":\""<<phase<<"\",\"step\":"<<step_<<",\"frame_client\":";rect(client);
    window("frame",frame_,frame);window("host",host_,host);window("horizon",horizon_,component);
    if(target) {
      const auto bounds=NativeRect(target);window("target",target,bounds);
      auto *hit=wxFindWindowAtPoint(bounds.GetTopLeft()+wxPoint(bounds.width/2,bounds.height/2));
      out<<",\"pointer_hit_handle\":"<<(hit?reinterpret_cast<std::uintptr_t>(hit->GetHandle()):0);
    }
    out<<"}\n";out.close();
    Check(frame_->IsShownOnScreen()&&host_->IsShownOnScreen()&&horizon_->IsShownOnScreen(),"actual fixture parents and component are shown");
    Check(host==client,"fixture host fills actual frame client");
    Check(host.Contains(component),"actual component is contained by visible host");
    Check(component==wxRect(host_->ClientToScreen(horizon_->GetPosition()),horizon_->GetSize()),"native component rectangle matches requested placement");

  }
  void Check(bool ok,const char *why){++checks_;if(!ok)throw std::runtime_error(why);}
  vessel::Sample Reading(double n){return {n,"TEST ONLY GPS",stamp_,vessel::Validity::Measured};}
  void Current() {
    now_=stamp_;state_={};ais_={};
    state_.navigation.latitude_deg=Reading(58);state_.navigation.longitude_deg=Reading(16);
    state_.navigation.sog_kn=Reading(6.3);state_.navigation.cog_deg=Reading(43);
    auto r=std::make_shared<vessel::RouteProgressSnapshot>();
    r->route_id="test-route";r->revision_scope="test";r->route_revision=1;r->active_waypoint_id="first";r->active_waypoint_index=0;
    r->waypoint_count=2;r->remaining_distance_nm=5;r->observed_at=stamp_;r->position_observed_at=stamp_;r->position_source="TEST ONLY GPS";
    r->source="TEST ONLY route";r->state=vessel::RouteState::Valid;
    r->remaining_steps={{"first","Next waypoint",58.1,16.1,2,45},{"last","Harbour",58.2,16.2,3,80}};state_.navigation.route=r;
    vessel::AisTarget t;t.mmsi=123456789;t.name="Owned traffic";t.source="TEST ONLY AIS";t.active=true;t.upstream_alarm=true;
    t.latitude_deg=Reading(58.2);t.longitude_deg=Reading(16.3);t.cpa_nm=Reading(.4);t.tcpa_minutes=Reading(12);t.observed_at=stamp_;
    ais_.available=true;ais_.observed_at=stamp_;ais_.targets={t};Feed();
  }
  void Feed() {
    advice_=smartnav::Advise(state_,{},ais_,now_);
    view_=application::PresentHorizon(state_,advice_,ais_,now_);horizon_->Update(view_,mode_);
  }
  ui::XNavButton *Button(int i){return dynamic_cast<ui::XNavButton *>(wxWindow::FindWindowByName(i<0?wxString("Full passage"):i?wxString::Format("Horizon event %d",i):wxString("Horizon now"),horizon_));}
  void Click(int i) {
    auto *button=Button(i);Check(button&&button->IsEnabled()&&button->IsShownOnScreen(),"real horizon control available");
    RecordLayout("horizon-pointer",button);
    const auto r=button->GetScreenRect();const auto p=r.GetTopLeft()+wxPoint(r.width/2,r.height/2);
    Check(wxFindWindowAtPoint(p)==button,"actual pointer hit resolves horizon control");
    wxUIActionSimulator input;Check(input.MouseMove(p.x,p.y)&&input.MouseClick(),"native pointer click injected");
  }
  void Geometry(int width,int y,int height,int inset,int top,int event_top,int event_height) {
    const auto g=horizon_->Geometry();Check(g.horizon.width==width&&g.horizon.height==height,"prototype horizon dimensions");
    Check(g.heading.x==g.horizon.x+inset&&g.heading.y==y+top+1&&g.heading.height==22,"prototype heading placement and height");
    Check(g.full_passage.y==g.heading.y&&g.full_passage.GetRight()==g.heading.GetRight(),"Full passage aligned to header end");
    const double edges[5]={0,.8,1.92,3.04,4.04};
    for(int i=0;i<4;++i)if(!view_.items[i].title.empty()) {
      Check(std::abs(g.events[i].x-(g.heading.x+std::lround(g.heading.width*edges[i]/4.04)))<=1,"fractional column left matches CSS");
      Check(g.events[i].y==event_top&&g.events[i].height==event_height,"event top and line box height match CSS");
      Check(g.horizon.Contains(g.events[i]),"event content stays within horizon");
    }
  }
  void Capture(const char *name) {
    RecordLayout(name);
    const auto size=frame_->GetClientSize();const auto origin=frame_->ClientToScreen({0,0});
#ifdef __WXGTK__
    auto *pixels=gdk_pixbuf_get_from_window(gdk_get_default_root_window(),origin.x,origin.y,size.x,size.y);
    Check(pixels!=nullptr,"actual native screen capture");const bool saved=gdk_pixbuf_save(pixels,(output_+"/"+name+".png").utf8_str(),"png",nullptr,nullptr);
    g_object_unref(pixels);Check(saved,"capture saved");
#else
    wxScreenDC screen;wxBitmap image(size.x,size.y);wxMemoryDC memory(image);
    Check(memory.Blit(0,0,size.x,size.y,&screen,origin.x,origin.y),"actual native screen capture");memory.SelectObject(wxNullBitmap);
    Check(image.SaveFile(output_+"/"+name+".png",wxBITMAP_TYPE_PNG),"capture saved");
#endif
    captures_.push_back({name,horizon_->Geometry()});
  }
  void Prototype(ui::LightMode mode) {
    // Exact illustrative HTML text for comparison, never a product state.
    mode_=mode;view_={};using M=application::HorizonMarker;using A=application::HorizonActionKind;
    auto &a=view_.items;a[0].time="NOW";a[0].title="On course";a[0].detail="043° COG · 6.3 kn";a[0].marker=M::Now;a[0].action.kind=A::Follow;
    a[1].time="11:56 · NEXT";a[1].title="Långholmen";a[1].detail="32° starboard turn";a[1].marker=M::Event;a[1].action.kind=A::Passage;
    a[2].time="12:01 ";a[2].secondary_time="+12 min";a[2].title="Freja · crossing";a[2].detail="CPA 0.4 nm · monitor";a[2].marker=M::Traffic;a[2].action.kind=A::Ais;
    a[3].time="14:42 · ARRIVAL";a[3].title="Arkösund";a[3].detail="18.2 nm · ";a[3].detail_accent="43% battery";a[3].marker=M::Arrival;a[3].action.kind=A::Passage;
    horizon_->Update(view_,mode);frame_->SetFocus();wxUIActionSimulator input;input.MouseMove(1279,0);
  }
  void Step(wxTimerEvent &) {
    try {switch(step_++) {
    case 0:Feed();break;
    case 1:Geometry(1014,634,132,25,13,693,45);Check(!Button(0)->IsEnabled(),"unavailable NOW cannot activate follow");Capture("owned-unavailable-day");Current();break;
    case 2:Geometry(1014,634,132,25,13,693,45);Capture("owned-current-day");Click(-1);break;
    case 3:Check(passage_==1,"Full passage invokes route context once");Click(0);break;
    case 4:Check(actions_==1&&last_.kind==application::HorizonActionKind::Follow,"NOW invokes existing follow context");Click(1);break;
    case 5:Check(actions_==2&&last_.mmsi==123456789,"AIS row selects copied current MMSI");Click(2);break;
    case 6:Check(actions_==3&&last_.identity=="first","waypoint opens matching passage");Click(3);break;
    case 7:Check(actions_==4&&last_.kind==application::HorizonActionKind::Passage,"turn opens passage");{
      const auto before=horizon_->PresentationChanges();horizon_->Update(view_,mode_);Check(horizon_->PresentationChanges()==before,"unchanged content does not repaint");
      view_.items[2].severity=smartnav::Severity::Warning;horizon_->Update(view_,mode_);Check(horizon_->PresentationChanges()==before+1,"severity-only change invalidates presentation");break;}
    case 8:Capture("owned-severity-day");{
      const auto before=horizon_->PresentationChanges();view_.items[1].action.mmsi=987654321;horizon_->Update(view_,mode_);
      Check(horizon_->PresentationChanges()==before+1,"identity-only change invalidates action state");break;}
    case 9:Capture("owned-identity-day");now_=stamp_+2s;Feed();break;
    case 10:Capture("owned-aging-day");now_=stamp_+5s;Feed();break;
    case 11:Capture("owned-stale-day");Check(!Button(0)->IsEnabled(),"retained data loses live action");Current();state_.replayed=true;Feed();break;
    case 12:Capture("owned-replay-day");Check(!Button(0)->IsEnabled()&&horizon_->View().historical,"replay visibly inhibits live actions");Current();{
      const auto r=Button(1)->GetScreenRect();wxUIActionSimulator input;input.MouseMove(r.x+r.width/2,r.y+r.height/2);Check(input.MouseDown(),"native press before identity change");break;}
    case 13:view_.items[1].action.mmsi=987654321;ais_.targets[0].mmsi=987654321;
      Check(application::HorizonActionAllowed(view_.items[1].action,state_,ais_,now_),"replacement target would otherwise be action-safe");
      horizon_->Update(view_,mode_);{wxUIActionSimulator input;Check(input.MouseUp(),"native release after identity change");break;}
    case 14:Check(actions_==4,"row identity change during press cancels activation");Prototype(ui::LightMode::Day);break;
    case 15:Geometry(1014,634,132,25,13,693,45);Capture("prototype-fixture-day");Prototype(ui::LightMode::Dusk);break;
    case 16:Capture("prototype-fixture-dusk");Prototype(ui::LightMode::Night);break;
    case 17:Capture("prototype-fixture-night");Resize(1024,640);horizon_->SetSize(70,494,798,112);Prototype(ui::LightMode::Day);break;
    case 18:Geometry(798,494,112,20,9,545,41);Capture("prototype-responsive-125");Resize(853,533);horizon_->SetSize(70,401,627,98);Prototype(ui::LightMode::Day);break;
    case 19:Geometry(627,401,98,20,9,450,38);Capture("prototype-responsive-150");Resize(1280,800);horizon_->SetSize(80,634,1014,132);Prototype(ui::LightMode::Day);{
      const auto r=Button(-1)->GetScreenRect();wxUIActionSimulator input;input.MouseMove(r.x+r.width/2,r.y+r.height/2);break;}
    case 20:Capture("prototype-hover-day");Button(-1)->SetFocus();{wxUIActionSimulator input;Check(input.Char(WXK_RETURN),"real Enter input");break;}
    case 21:Check(passage_==2,"Enter invokes passage once");{wxUIActionSimulator input;input.MouseMove(1279,0);Check(input.Char(WXK_TAB),"real Tab focus navigation");break;}
    case 22:Capture("prototype-focus-day");Current();Button(0)->SetFocus();{wxUIActionSimulator input;Check(input.Char(WXK_SPACE),"real Space input");break;}
    case 23:Check(actions_==5&&last_.kind==application::HorizonActionKind::Follow,"Space invokes follow once");Finish();break;
    }}catch(const std::exception &e){failed_=true;std::cerr<<e.what()<<'\n';Finish();}
  }
  void Finish() {
    timer_.Stop();std::ofstream out((output_+"/result.json").ToStdString());
    const auto rect=[&](const wxRect &r){out<<'['<<r.x<<','<<r.y<<','<<r.width<<','<<r.height<<']';};
    out<<"{\"passed\":"<<(failed_?"false":"true")<<",\"checks\":"<<checks_<<",\"native_dpi_qualification\":false,\"captures\":[";
    for(size_t i=0;i<captures_.size();++i){const auto &c=captures_[i];out<<(i?",":"")<<"{\"name\":\""<<c.name<<"\",\"horizon\":";rect(c.geometry.horizon);
      out<<",\"heading\":";rect(c.geometry.heading);out<<",\"full_passage\":";rect(c.geometry.full_passage);out<<",\"events\":[";
      for(size_t j=0;j<4;++j){if(j)out<<',';rect(c.geometry.events[j]);}out<<"]}";}
    out<<"]}\n";std::cout<<(failed_?"FAIL ":"PASS ")<<checks_<<" horizon component checks\n";frame_->Destroy();ExitMainLoop();
  }
  struct CaptureRecord {std::string name;ui::HorizonGeometry geometry;};
  static inline const vessel::Time stamp_{100s};vessel::Time now_=stamp_;
  wxString output_;wxFrame *frame_=nullptr;wxPanel *host_=nullptr;ui::XNavHorizon *horizon_=nullptr;
  vessel::VesselState state_;vessel::AisState ais_;smartnav::NavigationAdvice advice_;application::HorizonView view_;
  application::HorizonAction last_;ui::LightMode mode_=ui::LightMode::Day;std::vector<CaptureRecord> captures_;
  wxTimer timer_;int step_=0,passage_=0,actions_=0,checks_=0;bool failed_=false;
};
}
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc,char **argv){return wxEntry(argc,argv);}

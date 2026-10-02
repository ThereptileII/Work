// Non-installed offline widget driver. No OpenCPN profile, network or devices.
#include "ui/PreviewPanel.h"
#include "integration/RouteProgressInput.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <wx/app.h>
#include <wx/dcbuffer.h>
#include <wx/dcscreen.h>
#include <wx/filename.h>
#include <wx/frame.h>
#include <wx/log.h>
#include <wx/sizer.h>
#include <wx/timer.h>
#include <wx/uiaction.h>
#ifdef __WXMSW__
#include <windows.h>
#endif
#ifdef __WXGTK__
#include <gtk/gtk.h>
#endif
using namespace opennav;
using namespace std::chrono_literals;
namespace {
const vessel::Time stamp{100s};
vessel::Sample Sample(double value) {
  return {value, "OFFLINE TEST ONLY", stamp, vessel::Validity::Measured};
}
vessel::VesselState Fixture() {
  integration::RouteRead read;
  read.route.active = true;
  read.route.id = "test route";
  read.route.name = "Test passage";
  read.route.active_index = 0;
  read.route.active_point_id = "destination";
  read.route.points = {{"destination", 59.1, 18.1, 0, "Test destination", {}}};
  read.position.latitude_deg = Sample(59);
  read.position.longitude_deg = Sample(18);
  read.upstream_position_valid = true;
  read.upstream_latitude_deg = 59;
  read.upstream_longitude_deg = 18;
  read.range_to_active_nm = 18.2;
  read.bearing_to_active_true_deg = 43;
  integration::RouteProgressInput bridge("offline component test");
  bridge.Complete(read, read, stamp);
  vessel::VesselState state;
  state.navigation = read.position;
  state.navigation.route = bridge.Current();
  state.navigation.sog_kn = Sample(6.3);
  state.navigation.cog_deg = Sample(43);
  state.battery.soc_percent = Sample(68);
  state.battery.voltage_v = Sample(343);
  state.battery.current_a = Sample(6.3);
  state.battery.net_discharge_kw = Sample(2.15);
  state.propulsion.electrical_power_kw = Sample(2.15);
  state.propulsion.motor_rpm = Sample(820);
  state.propulsion.motor_temperature_c = Sample(62);
  return state;
}
class TestApp final : public wxApp {
public:
  bool OnInit() override {
    wxLog::SetActiveTarget(new wxLogStderr());
    wxSetAssertHandler([](const wxString &, int, const wxString &,
                          const wxString &s, const wxString &) {
      std::cerr << s << std::endl;
      std::abort();
    });
    if (argc != 2) return false;
    output_ = argv[1];
    if (!wxFileName::Mkdir(output_, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL)) return false;
    wxInitAllImageHandlers();
    frame_ = new wxFrame(nullptr, wxID_ANY, "TEST ONLY - Energy", {0,0},
                         {1280,800}, wxBORDER_NONE);
    frame_->SetClientSize(1280,800);
    auto *host = new wxPanel(frame_,wxID_ANY);
    auto *layout = new wxBoxSizer(wxVERTICAL);
    layout->Add(host,1,wxEXPAND);
    frame_->SetSizer(layout);
    frame_->Layout();
    host->SetBackgroundStyle(wxBG_STYLE_PAINT);
    host->Bind(wxEVT_PAINT,[this,host](wxPaintEvent &) {
      wxAutoBufferedPaintDC dc(host);
      ui::XNavPainter p(*host,dc,light_);
      dc.SetBackground(wxBrush(ui::Colour(p.c.background)));
      dc.Clear();
      p.Text("OFFLINE COMPONENT TEST / No chart, network, profile or equipment",
              80,20,16,p.c.attention);
      p.Text("TEST DATA / All values in this separate executable are synthetic",
              100,772,12,p.c.attention);
    });
    panel_ = new ui::PreviewPanel(host);
    panel_->SetSize(80,68,1014,698);
    panel_->SetCloseAction([this]{++closed_;panel_->Hide();});
    state_ = Fixture();
    frame_->Show();
    timer_.SetOwner(this);
    Bind(wxEVT_TIMER,&TestApp::Step,this);
    timer_.StartOnce(350);
    return true;
  }
  int OnRun() override { wxApp::OnRun(); return failed_?1:0; }
private:
  void Check(bool ok, const char *message) {
    ++checks_;
    if (!ok) throw std::runtime_error(message);
  }
  void Feed(vessel::Time now = stamp, ui::PreviewPage page = ui::PreviewPage::Energy) {
    auto prediction = smartnav::PredictVesselEnergy(model_,state_,now);
    auto advice = smartnav::Advise(state_,prediction,{},now);
    panel_->Update(page,light_,state_,now,model_,prediction,{},advice);
    frame_->Refresh(false);
  }
  void Capture(const char *name, bool canonical = true) {
    Check(frame_->GetClientSize()==wxSize(1280,800),"canonical native screen");
    Check(panel_->GetScreenRect()==wxRect(80,68,1014,698),"energy fills prototype workspace without timeline");
    Check(panel_->GetParent()->GetScreenRect().Contains(panel_->GetScreenRect()),"host contains painted view");
    const auto origin=frame_->ClientToScreen({0,0});
#ifdef __WXGTK__
    auto *pixels=gdk_pixbuf_get_from_window(gdk_get_default_root_window(),origin.x,origin.y,1280,800);
    Check(pixels!=nullptr,"actual screen pixels");
    const bool saved=gdk_pixbuf_save(pixels,(output_+"/"+name+".png").utf8_str(),"png",nullptr,nullptr);
    g_object_unref(pixels);
    Check(saved,"write capture");
#else
    wxScreenDC screen;
    wxBitmap bitmap(1280,800);
    wxMemoryDC memory(bitmap);
    Check(memory.Blit(0,0,1280,800,&screen,origin.x,origin.y),"actual screen pixels");
    memory.SelectObject(wxNullBitmap);
    Check(bitmap.SaveFile(output_+"/"+name+".png",wxBITMAP_TYPE_PNG),"write capture");
#endif
    if (canonical) names_.push_back(name);
  }
  wxWindow *CloseButton() {
    for (auto *child : panel_->GetChildren())
      if (child->GetLabel() == "Close") return child;
    throw std::runtime_error("Close button missing");
  }
  void Header(const char *phase, bool at_top) {
    auto *close = CloseButton();
    const auto rect = close->GetScreenRect();
    const auto origin = panel_->ClientToScreen({0,0});
    int ux=0,uy=0;panel_->GetScrollPixelsPerUnit(&ux,&uy);
    const auto scroll = panel_->GetViewStart();
    std::ofstream trace((output_+"/header-geometry.jsonl").ToStdString(),std::ios::app);
    trace<<"{\"phase\":\""<<phase<<"\",\"scroll_y\":"<<scroll.y*uy
         <<",\"close\":["<<rect.x<<','<<rect.y<<','<<rect.width<<','<<rect.height<<"]}\n";
    trace.close();
    Check(rect == wxRect(origin.x+panel_->GetClientSize().x-panel_->FromDIP(112),
                        origin.y+panel_->FromDIP(28)-scroll.y*uy,
                        panel_->FromDIP(80),panel_->FromDIP(44)),
          "Close follows header content coordinates through scroll and resize");
    if (at_top) {
      Check(scroll.y==0,"page returns to top");
      Check(panel_->GetScreenRect().Contains(rect),"Close fully visible in header");
    } else {
      Check(scroll.y*uy>panel_->FromDIP(72),"scroll moves header fully offscreen");
      Check(!panel_->GetScreenRect().Intersects(rect),"header Close scrolls offscreen like prototype");
    }
  }
  void ClickClose() {
    auto *close = CloseButton();
    Check(close->IsShownOnScreen() && close->IsEnabled(),"Close shown and enabled");
    const auto rect=close->GetScreenRect();
    Check(panel_->GetScreenRect().Contains(rect),"pointer target fully inside page");
    const wxPoint center{rect.x+rect.width/2,rect.y+rect.height/2};
    const auto hit = [&] {
#ifdef __WXMSW__
      return ::GetForegroundWindow()==static_cast<HWND>(frame_->GetHandle()) &&
             ::WindowFromPoint(POINT{center.x,center.y})==static_cast<HWND>(close->GetHandle());
#else
      return wxFindWindowAtPoint(center)==close;
#endif
    };
    Check(hit(),"exact visible Close pointer target before move");
    wxUIActionSimulator input;
    Check(input.MouseMove(center),"physical Close pointer move");
    Check(hit() && close->GetScreenRect()==rect,"same Close pointer target before mouse-down");
    Check(input.MouseClick(),"physical Close click");
  }
  void Step(wxTimerEvent &) {
    if (finished_) return;
    try {
      switch(step_++) {
      case 0: Feed(); break;
      case 1: {
        const auto &v=panel_->EnergyPresentation();
        Check(v.passage.distance_nm==18.2 && v.prediction.arrival.estimate.has_value(),"real energy contract drives widget");
        Check(v.soc.value==68 && v.remaining_kwh.has_value(),"configured capacity is an explicit estimate");
        Check(panel_->CanScroll(1),"operating details reachable below initial viewport");
        Capture("energy-day"); light_=ui::LightMode::Dusk; Feed(); break;
      }
      case 2: Capture("energy-dusk"); light_=ui::LightMode::Night; Feed(); break;
      case 3: Capture("energy-night"); light_=ui::LightMode::Day;
        state_.battery.soc_percent.observed_at-=20s; Feed(); break;
      case 4:
        Check(!panel_->EnergyPresentation().soc.value && !panel_->EnergyPresentation().prediction.arrival.estimate,
              "stale SOC and dependent destination withheld");
        Capture("energy-stale-day"); state_=Fixture(); state_.navigation.latitude_deg={}; Feed(); break;
      case 5:
        Check(!panel_->EnergyPresentation().passage.distance_nm && !panel_->EnergyPresentation().prediction.arrival.estimate,
              "GPS loss withholds retained route forecast");
        Capture("energy-gps-unavailable-day"); state_=Fixture(); state_.battery.soc_percent=Sample(5); Feed(); break;
      case 6:
        Check(panel_->EnergyPresentation().prediction.arrival.estimate &&
                  !panel_->EnergyPresentation().prediction.arrival.estimate->soc_percent,
              "insufficient energy is not valid zero SOC arrival");
        Capture("energy-shortfall-day"); state_=Fixture(); state_.navigation.route.reset(); Feed(); break;
      case 7:
        Check(!panel_->EnergyPresentation().prediction.arrival.estimate && panel_->EnergyPresentation().prediction.range.estimate,
              "inactive route retains only independent range");
        Capture("energy-inactive-day"); panel_->Step(1); break;
      case 8:
        Check(panel_->GetViewStart().y>0,"details scroll through actual view");
        Header("energy-scrolled",false);
        // Real layout events while scrolled used to replace logical header y
        // with client y; returning to the top then moved Close into a card.
        panel_->SetSize(80,68,1000,698); break;
      case 9: panel_->SetSize(80,68,1014,698); break;
      case 10: panel_->Scroll(0,0); break;
      case 11:
        Capture("header-return-energy",false);
        Header("energy-resized-return",true); ClickClose(); break;
      case 12:
        Check(closed_==1 && !panel_->IsShown(),"one physical Close hides page");
        state_=Fixture();panel_->Show();Feed();break;
      case 13:
        Header("energy-reopened",true);
        Feed(stamp,ui::PreviewPage::Diagnostics);break;
      case 14:
        Header("diagnostics-entry",true);panel_->Step(1);break;
      case 15:
        Header("diagnostics-scrolled",false);
        panel_->SetSize(80,68,1000,698);break;
      case 16:
        panel_->SetSize(80,68,1014,698);break;
      case 17:
        Feed(stamp,ui::PreviewPage::Route);break;
      case 18:
        Header("route-after-scrolled-diagnostics",true);Feed();break;
      case 19:
        Header("energy-reentry-after-route",true);
        Capture("header-reentry-energy",false);ClickClose();break;
      case 20:
        Check(closed_==2 && !panel_->IsShown(),"reentered Energy closes once by pointer");
        Finish();break;
      }
    } catch(const std::exception &e) {
      failed_=true;std::cerr<<e.what()<<std::endl;Finish();
    }
    // Native pointer delivery and capture may pump events. Never reenter Step.
    if (!finished_) timer_.StartOnce(350);
  }
  void Finish() {
    if (finished_) return;
    finished_=true;
    timer_.Stop();
    std::ofstream f((output_+"/result.json").ToStdString());
    f<<"{\"passed\":"<<(failed_?"false":"true")<<",\"checks\":"<<checks_<<",\"captures\":[";
    for(std::size_t i=0;i<names_.size();++i) {if(i)f<<',';f<<'"'<<names_[i]<<'"';}
    f<<"]}\n";f.close();frame_->Destroy();ExitMainLoop();
  }
  wxString output_;
  wxFrame *frame_=nullptr;
  ui::PreviewPanel *panel_=nullptr;
  wxTimer timer_;
  vessel::VesselState state_;
  smartnav::EnergyModel model_{24.8,25,.5,"Explicit offline test assumptions"};
  ui::LightMode light_=ui::LightMode::Day;
  std::vector<std::string> names_;
  int step_=0,checks_=0,closed_=0;
  bool failed_=false,finished_=false;
};
} // namespace
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc,char **argv) {return wxEntry(argc,argv);}

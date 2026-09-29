#include "ui/SettingsDrawer.h"
#include <wx/dcbuffer.h>
#include <wx/sizer.h>
#include <array>
#include <cmath>

namespace opennav::ui {
namespace {
const std::array<const char *,8> titles{{"Vessel","Navigation","Sensors","Autopilot","Radar","Display","System","Help"}};
wxString Number(double v, const wxString &unit) {
  return std::isfinite(v) ? wxString::Format("%.3g ",v)+unit : "Not configured";
}
}
XNavSettingsDrawer::XNavSettingsDrawer(wxWindow &owner, SettingsDrawerActions actions)
    : XNavDrawer(owner,"OpenNav preferences"), actions_(std::move(actions)) {
  SetWide(true);
  SetHeading("PREFERENCES","A helm of your own",false);
  Build();
}
void XNavSettingsDrawer::Select(SettingsSection section) {
  if (static_cast<unsigned>(section)>=titles.size()) return;
  section_=section;
  Build();
}
void XNavSettingsDrawer::Update(const ProductState &state,LightMode mode) {
  state_=state;
  const bool changed=mode!=light_;
  SetLight(mode);
  if(changed) {
    for(auto *b:buttons_)b->SetLightMode(mode);
    for(auto *b:tabs_buttons_)b->SetLightMode(mode);
    for(const auto &choice:light_buttons_)choice.first->SetSelected(choice.second==mode);
    tabs_->SetBackgroundColour(Colour(Theme(mode).background));
  }
  for(auto *p:copies_)p->Refresh(false);
}
void XNavSettingsDrawer::CopyBlock(int height,std::function<void(XNavPainter &,int)> draw) {
  auto *p=new wxPanel(body_,wxID_ANY);
  p->SetLabel(wxEmptyString);
  p->SetBackgroundStyle(wxBG_STYLE_PAINT);
  p->SetMinSize(FromDIP(wxSize(300,height)));
  p->Bind(wxEVT_PAINT,[this,p,draw](wxPaintEvent &){
    wxAutoBufferedPaintDC dc(p);XNavPainter painter(*p,dc,light_);
    dc.SetBackground(wxBrush(Colour(painter.c.background)));dc.Clear();
    draw(painter,p->ToDIP(p->GetClientSize().x));
  });
  EnableScrollGesture(*p);
  content_->Add(p,0,wxEXPAND);copies_.push_back(p);
}
void XNavSettingsDrawer::Link(const wxString &title,const wxString &detail,XNavIcon icon,
                              std::function<void()> action) {
  auto *b=new XNavButton(body_,wxID_ANY,title,title);
  b->SetSuiteLink(detail,icon);b->SetLightMode(light_);
  b->SetMinSize(FromDIP(wxSize(300,72)));b->Enable(bool(action));
  b->Bind(wxEVT_BUTTON,[this,action](wxCommandEvent &){if(action)CallAfter(action);});
  content_->Add(b,0,wxEXPAND);buttons_.push_back(b);
}
void XNavSettingsDrawer::Page(const wxString &title,const wxString &detail,XNavIcon icon,ProductPage page) {
  Link(title,detail,icon,actions_.page?std::function<void()>([this,page]{actions_.page(page);}):std::function<void()>{});
}
void XNavSettingsDrawer::Button(const wxString &label,std::function<void()> action,ButtonRole role) {
  auto *b=new XNavButton(body_,wxID_ANY,label,label);
  b->SetRole(role);b->SetLightMode(light_);b->SetMinSize(FromDIP(wxSize(300,48)));
  b->Enable(bool(action));b->Bind(wxEVT_BUTTON,[this,action](wxCommandEvent &){if(action)CallAfter(action);});
  content_->Add(b,0,wxEXPAND|wxBOTTOM,FromDIP(10));buttons_.push_back(b);
}
void XNavSettingsDrawer::Build() {
  ClearBody();tabs_buttons_.clear();buttons_.clear();light_buttons_.clear();copies_.clear();
  tabs_=new wxPanel(body_,wxID_ANY);tabs_->SetLabel(wxEmptyString);
  tabs_->SetBackgroundColour(Colour(Theme(light_).background));
  tabs_->SetMinSize(FromDIP(wxSize(300,79)));
  for(unsigned i=0;i<titles.size();++i) {
    auto *b=new XNavButton(tabs_,wxID_ANY,titles[i],wxString("Settings section: ")+titles[i]);
    b->SetSettingsTab();b->SetRole(ButtonRole::Segment);b->SetLightMode(light_);
    b->SetSelected(i==static_cast<unsigned>(section_));
    b->SetMinSize(FromDIP(wxSize(1,37)));
    b->Bind(wxEVT_BUTTON,[this,i](wxCommandEvent &){CallAfter([this,i]{Select(static_cast<SettingsSection>(i));});});
    tabs_buttons_.push_back(b);
  }
  tabs_->Bind(wxEVT_SIZE,[this](wxSizeEvent &event){
    double x=0;int y=0;const int width=tabs_->GetClientSize().x,gap=FromDIP(5),height=FromDIP(37);
    for(auto *b:tabs_buttons_) {
      const double w=UiTextWidth(*tabs_,b->GetLabel(),11)+FromDIP(22);
      if(x && x+w>width){x=0;y+=height+gap;}
      b->SetSize(std::lround(x),y,std::lround(x+w)-std::lround(x),height);x+=w+gap;
    }
    const int wanted=y+height;
    if(tabs_->GetMinSize().y!=wanted){tabs_->SetMinSize(wxSize(FromDIP(300),wanted));body_->Layout();body_->FitInside();}
    event.Skip();
  });
  content_->Add(tabs_,0,wxEXPAND|wxBOTTOM,FromDIP(23));
  switch(section_) {
    case SettingsSection::Vessel:
      CopyBlock(218,[this](XNavPainter &p,int width){
        p.TextTracked("VESSEL ASSUMPTIONS",0,4,9,p.c.accent,650,1.17);
        const wxString labels[]={"Draft","Safety margin","Usable battery capacity","Minimum reserve"};
        const wxString values[]={Number(state_.settings.hazard.draft_m,"m"),Number(state_.settings.hazard.safety_margin_m,"m"),
          Number(state_.settings.energy.battery.capacity_kwh,"kWh"),Number(state_.settings.energy.battery.reserve_soc_percent,"%")};
        for(int i=0;i<4;++i){int y=32+i*44;p.Text(labels[i],0,y,12,p.c.secondary,false,width/2);
          p.TextWeight(values[i],width/2,y,12,p.c.primary,500,width/2,true);p.Rule(0,y+28,width);}
      });
      Page("Vessel dimensions","Draft and advisory corridor margin",XNavIcon::Ownship,ProductPage::VesselSettings);
      Page("Battery & reserve","Capacity, reserve and measured consumption",XNavIcon::Energy,ProductPage::EnergySettings);
      Link("Chart safety depth","OpenCPN chart contours and depth alarms",XNavIcon::Layers,actions_.advanced);
      CopyBlock(80,[](XNavPainter &p,int width){
        p.Text("Draft and margin do not change chart safety contours.",0,16,11,p.c.secondary,false,width);
        p.Text("Configure chart safety depth in OpenCPN settings.",0,38,11,p.c.secondary,false,width);
      });
      break;
    case SettingsSection::Navigation:
      Page("Navigation preferences","Units, chart orientation and navigation alarms",XNavIcon::Compass,ProductPage::NavigationSettings);
      Page("Chart presentation","XNav or Standard, light and display",XNavIcon::Layers,ProductPage::Display);
      Link("Charts & coverage","Configured OpenCPN charts and connections",XNavIcon::Chart,actions_.advanced);
      Page("Alarms & thresholds","Inspect current navigation conditions",XNavIcon::Bell,ProductPage::Alerts);
      Page("Passage library","Saved OpenCPN routes",XNavIcon::Route,ProductPage::Routes);
      Page("Waypoints","Saved places and chart marks",XNavIcon::Pin,ProductPage::Waypoints);
      break;
    case SettingsSection::Sensors:
      CopyBlock(144,[](XNavPainter &p,int width){
        p.TextTracked("CONNECTED TO YOUR BOAT",0,9,10,p.c.secondary,650,1.3);
        p.TextTracked("Every signal has a source.",0,33,23,p.c.primary,700,-.6,width);
        p.Wrapped("Connect, assign and verify each measurement before putting it on your helm.",
                  0,73,13,21,width,p.c.secondary,2);
      });
      Page("Manage sensors","Observed sources and per-signal quality",XNavIcon::Instruments,ProductPage::Sources);
      Link("Add a sensor","NMEA 2000, NMEA 0183 or Signal K",XNavIcon::Plus,actions_.advanced);
      Page("Source health","Freshness, cadence and dropout states",XNavIcon::Shield,ProductPage::SourceHealth);
      break;
    case SettingsSection::Autopilot:
      CopyBlock(86,[this](XNavPainter &p,int width){
        p.Text("Control",0,12,12,p.c.secondary);p.TextWeight(state_.pilot.enabled?"Enabled":"Off",width/2,12,12,p.c.primary,500,width/2,true);
        p.Rule(0,40,width);p.Text(state_.pilot.fresh?"Current pilot feedback":"Pilot feedback unavailable",0,58,12,p.c.secondary,false,width);
      });
      Page("Adapter & capabilities","Connection, acknowledgement and modes",XNavIcon::Settings,ProductPage::PilotSettings);
      Page("Helm controls","Standby, Auto and heading adjustments",XNavIcon::Instruments,ProductPage::Pilot);
      CopyBlock(90,[](XNavPainter &p,int width){p.Text("Steering requires explicit control enablement",0,18,11,p.c.secondary,false,width);
        p.Text("and confirmation from the adapter.",0,39,11,p.c.secondary,false,width);});
      break;
    case SettingsSection::Radar:
      CopyBlock(104,[this](XNavPainter &p,int width){p.TextTracked("RADAR",0,4,9,p.c.accent,650,1.17);
        p.Text(state_.radar.available?"Adapter reported available":"Radar unavailable",0,30,23,p.c.primary,false,width);
        p.Text("A validated display adapter is required.",0,74,12,p.c.secondary,false,width);});
      Page("Radar status","Availability and supported capabilities",XNavIcon::Radar,ProductPage::Radar);
      Link("Plugins & adapters","OpenCPN integration components",XNavIcon::Layers,actions_.plugins);
      break;
    case SettingsSection::Display: {
      CopyBlock(34,[](XNavPainter &p,int width){p.TextTracked("LIGHT FOR THE MOMENT",0,4,10,p.c.secondary,400,1.5,width);});
      auto *row=new wxBoxSizer(wxHORIZONTAL);
      for(const auto mode:{LightMode::Day,LightMode::Dusk,LightMode::Night}) {
        const wxString name=mode==LightMode::Day?"Day":mode==LightMode::Dusk?"Dusk":"Night";
        auto *b=new XNavButton(body_,wxID_ANY,name,"Display "+name);b->SetRole(ButtonRole::Segment);
        b->SetSelected(mode==light_);b->SetLightMode(light_);b->SetMinSize(FromDIP(wxSize(48,40)));b->Enable(bool(actions_.theme));
        b->Bind(wxEVT_BUTTON,[this,mode](wxCommandEvent &){CallAfter([this,mode]{
          if(actions_.theme) actions_.theme(mode);
          Select(SettingsSection::Display);
        });});
        row->Add(b,1);buttons_.push_back(b);light_buttons_.push_back({b,mode});
      }
      content_->Add(row,0,wxEXPAND|wxBOTTOM,FromDIP(20));
      Page("Personalise instruments","Choose the four primary values",XNavIcon::Sliders,ProductPage::RailLayout);
      Page("Chart presentation","XNav or Standard and display options",XNavIcon::Layers,ProductPage::Display);
      Button("Toggle fullscreen",actions_.fullscreen);
      CopyBlock(90,[](XNavPainter &p,int width){p.Text("Interface scale follows Windows display scaling.",0,18,11,p.c.secondary,false,width);
        p.Text("Display brightness remains a hardware setting.",0,39,11,p.c.secondary,false,width);});
      break;
    }
    case SettingsSection::System:
      Link("Diagnostics","Versions, data quality and current source state",XNavIcon::Settings,actions_.diagnostics);
      Page("Recordings & commissioning","Read-only observation and field capture",XNavIcon::Instruments,ProductPage::Commissioning);
      Page("Export diagnostics","Choose the information to include",XNavIcon::Settings,ProductPage::FieldReport);
      Link("Plugins & adapters","Advanced OpenCPN plugin settings",XNavIcon::Layers,actions_.plugins);
      Button("Legacy mode",actions_.legacy);
      Button("Safe mode",actions_.safe);
      Page("Advanced / Legacy Settings","Connections, charts and additional preferences",XNavIcon::Settings,ProductPage::NavigationSettings);
      break;
    case SettingsSection::Help:
      CopyBlock(145,[](XNavPainter &p,int width){p.TextTracked("OPENNAV X",0,4,9,p.c.accent,650,1.17);
        p.Text("Charts and navigation are owned by OpenCPN.",0,36,12,p.c.secondary,false,width);
        p.Text("Predictions are advisory. Missing data stays unavailable.",0,62,12,p.c.secondary,false,width);
        p.Text("Legacy and Safe keep the same navigation profile.",0,88,12,p.c.secondary,false,width);});
      Link("Diagnostics","Inspect this build and its source health",XNavIcon::Settings,actions_.diagnostics);
      Link("Advanced OpenCPN settings","Existing charts, connections and preferences",XNavIcon::Settings,actions_.advanced);
      break;
  }
  body_->Layout();body_->FitInside();
}
} // namespace opennav::ui

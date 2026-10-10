#include "application/Brand.h"
#include "ui/SettingsDrawer.h"
#include "ui/DisplaySizing.h"
#include "application/Version.h"
#include "integration/BuildFeatures.h"
#include <wx/dcbuffer.h>
#include <wx/graphics.h>
#include <wx/sizer.h>
#include <wx/tokenzr.h>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

namespace opennav::ui {
namespace {
const std::array<const char *,8> titles{{"Vessel","Navigation","Sensors","Autopilot","Radar","Display","System","Help"}};
wxString InputNumber(double v) {
  return std::isfinite(v) ? wxString::FromUTF8(application::SettingNumber(v)) : wxString{};
}
}
XNavSettingsDrawer::XNavSettingsDrawer(wxWindow &owner, SettingsDrawerActions actions)
    : XNavDrawer(owner,"OpenNav preferences"), actions_(std::move(actions)) {
  if (actions_.display) display_draft_=display_applied_=actions_.display();
  SetWide(true);
  SetHeading("PREFERENCES","A helm of your own",false);
  Build();
}
void XNavSettingsDrawer::Open(const wxRect &workspace) {
  Present(workspace);
  if (!IsShown()) return;
  // Root reopening must replace a remembered offscreen child before resetting
  // scroll, or native activation restores that child and scrolls back to it.
  const auto index = static_cast<std::size_t>(section_);
  if (index < tabs_buttons_.size()) {
    auto *tab = tabs_buttons_[index];
    if (tab && tab->IsShown() && tab->IsEnabled()) tab->SetFocus();
  }
  // Prototype openPanel('settings') starts the drawer body at the top.
  // Keep this separate from Present(), which also runs during live refresh.
  body_->Scroll(0, 0);
}
void XNavSettingsDrawer::Select(SettingsSection section) {
  if (static_cast<unsigned>(section)>=titles.size()) return;
  // The prototype renders select values from the last applied preferences on
  // every tab/theme rebuild. A pending choice is discarded on navigation.
  if (display_dirty_) {
    display_draft_ = actions_.display ? actions_.display() : display_applied_;
    display_dirty_ = false;
  }
  section_=section;
  Build();
}
void XNavSettingsDrawer::Update(const ProductState &state,LightMode mode) {
  state_=state;
  const bool changed=mode!=light_;
  SetLight(mode);
  if(changed) {
    for(auto *b:buttons_)b->SetLightMode(mode);
    if(scale_field_)scale_field_->SetLightMode(mode);
    if(layout_field_)layout_field_->SetLightMode(mode);
    for(auto *b:tabs_buttons_)b->SetLightMode(mode);
    for(const auto &choice:light_buttons_)choice.first->SetSelected(choice.second==mode);
    tabs_->SetBackgroundColour(Colour(Theme(mode).background));
    if(light_track_) {
      light_track_->SetBackgroundColour(Colour(Theme(mode).surface));
      light_track_->Refresh(false);
    }
    for(auto *frame:input_frames_)frame->Refresh(false);
    for(auto *panel:field_containers_)panel->SetBackgroundColour(Colour(Theme(mode).background));
    for(std::size_t i=0;i<field_captions_.size();++i)
      field_captions_[i]->SetForegroundColour(Colour(i==1||i==2?Theme(mode).muted:Theme(mode).secondary));
    for(auto *field:fields_)if(field){
      field->SetBackgroundColour(Colour(Theme(mode).background));
      field->SetForegroundColour(Colour(Theme(mode).primary));
    }
    if(message_)message_->SetForegroundColour(Colour(Theme(mode).secondary));
  }
  if(!draft_dirty_ && section_==SettingsSection::Vessel){
    const std::array<wxString,5> current{{wxString::FromUTF8(state.vessel_name),
      InputNumber(state.settings.hazard.draft_m),InputNumber(state.chart_safety_depth_m),
      InputNumber(state.settings.energy.battery.capacity_kwh),
      InputNumber(state.settings.energy.battery.reserve_soc_percent)}};
    if(!draft_initialized_ || current!=draft_){
      loading_=true;draft_=current;draft_initialized_=true;
      for(std::size_t i=0;i<fields_.size();++i)if(fields_[i])fields_[i]->ChangeValue(draft_[i]);
      loading_=false;
    }
  }
  if (!display_dirty_ && actions_.display) {
    const auto current = actions_.display();
    if (current.scale_percent != display_draft_.scale_percent ||
        current.layout != display_draft_.layout) {
      SetDisplayPreferences(current);
    }
  }
  for(auto *p:copies_)p->Refresh(false);
}
void XNavSettingsDrawer::ResetDraft(){
  draft_initialized_=false;draft_dirty_=false;touched_.fill(false);feedback_.clear();
  display_dirty_=false;
  if (actions_.display) SetDisplayPreferences(actions_.display());
  if (display_message_) display_message_->SetLabel(wxEmptyString);
}
void XNavSettingsDrawer::SetDisplayPreferences(const application::DisplayPreferences &value) {
  display_draft_=display_applied_=value;
  if(scale_field_)scale_field_->SetSelection((value.scale_percent-100)/25);
  if(layout_field_)layout_field_->SetSelection(static_cast<int>(value.layout));
  ApplyScale();
}
void XNavSettingsDrawer::ApplyScale() {
  const int scale=display_applied_.scale_percent;
  const int field_height=DisplayFieldHeight(scale);
  const int field_font=DisplayFieldFont(scale);
  SetInterfaceScale(scale);
  for (auto *frame:input_frames_)
    frame->SetMinSize(FromDIP(wxSize(80,field_height)));
  for (auto *field:fields_) if (field) field->SetFont(UiFont(*this,field_font));
  if(scale_field_)scale_field_->SetInterfaceScale(scale);
  if(layout_field_)layout_field_->SetInterfaceScale(scale);
  if (body_) { body_->Layout();body_->FitInside(); }
}
void XNavSettingsDrawer::DisplayForm() {
  auto choice_field=[this](const wxString &title,const std::vector<wxString> &labels) {
    CopyBlock(25,[title](XNavPainter &p,int width){p.Text(title,0,3,12,p.c.secondary,false,width);});
    auto *field=new XNavChoiceField(body_,wxID_ANY,labels,title);
    field->SetLightMode(light_);
    field->SetInterfaceScale(display_applied_.scale_percent);
    field->Enable(bool(actions_.save_display));
    content_->Add(field,0,wxEXPAND|wxBOTTOM,FromDIP(18));
    return field;
  };
  scale_field_=choice_field("Interface scale",{"100%","125%","150%"});
  scale_field_->SetSelection((display_draft_.scale_percent-100)/25);
  scale_field_->Bind(wxEVT_CHOICE,[this](wxCommandEvent &event){
    display_draft_.scale_percent=100+event.GetInt()*25;display_dirty_=true;
  });
  layout_field_=choice_field("Chart layout",{"Balanced","Chart focus","Instrument focus"});
  layout_field_->SetSelection(static_cast<int>(display_draft_.layout));
  layout_field_->Bind(wxEVT_CHOICE,[this](wxCommandEvent &event){
    display_draft_.layout=static_cast<application::ChartLayout>(event.GetInt());display_dirty_=true;
  });
  CopyBlock(33,[](XNavPainter &p,int width){p.TextTracked("LIGHT FOR THE MOMENT",0,4,10,p.c.secondary,400,1.5,width);});
  light_track_=new wxPanel(body_,wxID_ANY);
  light_track_->SetName("Display light track");
  light_track_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  light_track_->SetBackgroundColour(Colour(Theme(light_).surface));
  light_track_->Bind(wxEVT_PAINT,[this](wxPaintEvent &){
    wxAutoBufferedPaintDC dc(light_track_);
    dc.SetBackground(wxBrush(Colour(Theme(light_).background)));dc.Clear();
    dc.SetPen(*wxTRANSPARENT_PEN);dc.SetBrush(wxBrush(Colour(Theme(light_).surface)));
    dc.DrawRoundedRectangle(wxPoint(0,0),light_track_->GetClientSize(),FromDIP(9));
  });
  auto *row=new wxBoxSizer(wxHORIZONTAL);
  for(const auto mode:{LightMode::Day,LightMode::Dusk,LightMode::Night}) {
    const wxString name=mode==LightMode::Day?"Day":mode==LightMode::Dusk?"Dusk":"Night";
    auto *b=new XNavButton(light_track_,wxID_ANY,name,"Display "+name);b->SetSegmentInTrack();
    b->SetSelected(mode==light_);b->SetLightMode(light_);b->SetMinSize(FromDIP(wxSize(48,40)));b->Enable(bool(actions_.theme));
    b->Bind(wxEVT_BUTTON,[this,mode](wxCommandEvent &){CallAfter([this,mode]{
      if(actions_.theme) actions_.theme(mode);
      Select(SettingsSection::Display);
    });});
    if(mode!=LightMode::Day)row->AddSpacer(FromDIP(4));
    row->Add(b,1);buttons_.push_back(b);light_buttons_.push_back({b,mode});
  }
  auto *track_layout=new wxBoxSizer(wxVERTICAL);
  track_layout->Add(row,1,wxEXPAND|wxALL,FromDIP(4));
  light_track_->SetSizer(track_layout);
  content_->Add(light_track_,0,wxEXPAND|wxBOTTOM,FromDIP(30));
  Button("Apply display preferences",[this]{SaveDisplay();},ButtonRole::Primary);
  display_message_=new wxStaticText(body_,wxID_ANY,wxEmptyString);
  display_message_->SetFont(UiFont(*this,11));
  display_message_->SetForegroundColour(Colour(Theme(light_).secondary));
  display_message_->Hide();
  content_->Add(display_message_,0,wxEXPAND|wxBOTTOM,FromDIP(12));
  Page("Personalise instruments","Choose the four primary values",XNavIcon::Sliders,ProductPage::RailLayout);
  Button("Toggle fullscreen",actions_.fullscreen);
}
void XNavSettingsDrawer::SaveDisplay() {
  if(!actions_.save_display)return;
  auto result=actions_.save_display(display_draft_);
  if(result.ok) {
    display_dirty_=false;
    display_applied_=display_draft_;
    ApplyScale();
  }
  if(display_message_) {
    display_message_->SetLabel(result.ok?wxString{}:wxString::FromUTF8(result.message));
    display_message_->Show(!result.ok);
    if(!result.ok)display_message_->Wrap(FromDIP(340));
    body_->Layout();body_->FitInside();
  }
}
void XNavSettingsDrawer::VesselForm(){
  auto add_field=[this](wxWindow *parent,wxBoxSizer *layout,const wxString &label,
                        std::size_t index,int bottom,bool paired=false){
    auto *outer=new wxPanel(parent,wxID_ANY);outer->SetBackgroundColour(Colour(Theme(light_).background));
    field_containers_.push_back(outer);
    auto *column=new wxBoxSizer(wxVERTICAL);
    auto *caption=new wxStaticText(outer,wxID_ANY,label);
    caption->SetFont(UiFont(*this,paired?9:12));
    caption->SetForegroundColour(Colour(paired?Theme(light_).muted:Theme(light_).secondary));
    caption->SetMinSize(wxSize(-1,FromDIP(paired?14:18)));
    field_captions_.push_back(caption);
    column->Add(caption,0,wxBOTTOM,FromDIP(paired?0:8));
    auto *frame=new wxPanel(outer,wxID_ANY);frame->SetBackgroundStyle(wxBG_STYLE_PAINT);
    frame->SetMinSize(FromDIP(wxSize(80,46)));
    frame->Bind(wxEVT_PAINT,[this,frame](wxPaintEvent &){
      wxAutoBufferedPaintDC dc(frame);dc.SetBackground(wxBrush(Colour(Theme(light_).background)));dc.Clear();
      std::unique_ptr<wxGraphicsContext> graphics(wxGraphicsContext::Create(dc));
      if(graphics){graphics->SetBrush(wxBrush(Colour(Theme(light_).background)));
        graphics->SetPen(wxPen(Colour(Theme(light_).border),1));
        const auto size=frame->GetClientSize();
        graphics->DrawRoundedRectangle(.5,.5,size.x-1.,size.y-1.,FromDIP(8));}
    });
    auto *inner=new wxBoxSizer(wxHORIZONTAL);
    auto *input=new wxTextCtrl(frame,wxID_ANY,draft_[index],wxDefaultPosition,
                               wxDefaultSize,wxBORDER_NONE);
    input->SetName(label);input->SetFont(UiFont(*this,14));
    input->SetBackgroundColour(Colour(Theme(light_).background));
    input->SetForegroundColour(Colour(Theme(light_).primary));
    input->SetMaxLength(index==0?120:64);
    inner->Add(input,1,wxALIGN_CENTER_VERTICAL|wxLEFT|wxRIGHT,FromDIP(13));
    frame->SetSizer(inner);column->Add(frame,0,wxEXPAND);
    outer->SetSizer(column);
    outer->SetMinSize(FromDIP(wxSize(80,paired?60:72)));
    layout->Add(outer,paired?1:0,wxEXPAND|wxBOTTOM,FromDIP(bottom));
    fields_[index]=input;input_frames_.push_back(frame);
    input->Bind(wxEVT_TEXT,[this,index,input](wxCommandEvent &){
      if(!loading_){draft_[index]=input->GetValue();draft_dirty_=true;touched_[index]=true;}
    });
  };
  auto *single=new wxBoxSizer(wxVERTICAL);
  // The immutable HTML's block margins collapse at the grid boundary. Its
  // first field, paired grid, final two fields and Save are 36/27/16/26px apart.
  add_field(body_,single,"Vessel name",0,36);content_->Add(single,0,wxEXPAND);
  auto *pair=new wxBoxSizer(wxHORIZONTAL);
  add_field(body_,pair,wxString::FromUTF8("Draft · metres"),1,27,true);pair->AddSpacer(FromDIP(16));
  add_field(body_,pair,wxString::FromUTF8("Safety depth · metres"),2,27,true);content_->Add(pair,0,wxEXPAND);
  single=new wxBoxSizer(wxVERTICAL);
  add_field(body_,single,wxString::FromUTF8("Usable battery capacity · kWh"),3,16);
  add_field(body_,single,wxString::FromUTF8("Minimum reserve · %"),4,26);content_->Add(single,0,wxEXPAND);
  Button("Save vessel profile",[this]{SaveVessel();},ButtonRole::Primary);
  message_=new wxStaticText(body_,wxID_ANY,feedback_);
  message_->SetFont(UiFont(*this,11));
  message_->SetForegroundColour(Colour(Theme(light_).secondary));
  content_->Add(message_,0,wxEXPAND|wxBOTTOM,FromDIP(12));
  Button("Run boat setup",actions_.boat_setup);
  Page("Advanced vessel model","Advisory corridor margin and vessel assumptions",XNavIcon::Ownship,ProductPage::VesselSettings);
  Page("Advanced battery model","Measured consumption and battery source",XNavIcon::Energy,ProductPage::EnergySettings);
}
void XNavSettingsDrawer::SaveVessel(){
  if(!actions_.save_vessel)return;
  auto value=[this](std::size_t index){return application::ParseSettingNumber(draft_[index].ToStdString(wxConvUTF8));};
  try {
    auto next=actions_.settings?actions_.settings():state_.settings;
    const auto draft=touched_[1]?value(1):next.hazard.draft_m;
    const auto safety=touched_[2]?value(2):state_.chart_safety_depth_m;
    const auto capacity=touched_[3]?value(3):next.energy.battery.capacity_kwh;
    const auto reserve=touched_[4]?value(4):next.energy.battery.reserve_soc_percent;
    auto edited_range=[this](std::size_t index,double number,double lower,double upper){
      return !touched_[index] || std::isnan(number) ||
             (std::isfinite(number)&&number>=lower&&number<=upper);
    };
    if(!edited_range(1,draft,.1,20)||!edited_range(2,safety,.1,30)||
       !edited_range(3,capacity,1,500)||!edited_range(4,reserve,5,90)||
       ((touched_[1]||touched_[2])&&std::isfinite(draft)&&std::isfinite(safety)&&safety<draft))
      throw std::invalid_argument("Use draft 0.1–20 m, safety depth 0.1–30 m and ≥ draft, capacity 1–500 kWh, reserve 5–90%. Blank model fields stay unconfigured; blank safety depth keeps the current chart value.");
    if(touched_[1])next.hazard.draft_m=draft;
    if(touched_[3])next.energy.battery.capacity_kwh=capacity;
    if(touched_[4])next.energy.battery.reserve_soc_percent=reserve;
    if(touched_[3]||touched_[4])next.energy.battery.source=
      "User-configured usable battery energy and reserve / OpenCPN profile";
    auto name=draft_[0];name.Trim(true).Trim(false);
    auto result=actions_.save_vessel(next,name.ToStdString(wxConvUTF8),
                                      touched_[2]?safety:std::numeric_limits<double>::quiet_NaN());
    feedback_=wxString::FromUTF8(result.message);
    if(result.ok){state_.settings=next;state_.vessel_name=name.ToStdString(wxConvUTF8);
      if(touched_[2]&&std::isfinite(safety))state_.chart_safety_depth_m=safety;
      draft_dirty_=false;touched_.fill(false);}
  } catch(const std::exception &error){feedback_=wxString::FromUTF8(error.what());}
  if(message_){message_->SetLabel(feedback_);message_->Wrap(FromDIP(340));body_->Layout();body_->FitInside();}
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
  b->SetDisplayAction(48);
  b->SetInterfaceScale(display_applied_.scale_percent);
  b->Enable(bool(action));b->Bind(wxEVT_BUTTON,[this,action](wxCommandEvent &){if(action)CallAfter(action);});
  content_->Add(b,0,wxEXPAND|wxBOTTOM,FromDIP(10));buttons_.push_back(b);
}
void XNavSettingsDrawer::Build() {
  ClearBody();tabs_buttons_.clear();buttons_.clear();light_buttons_.clear();copies_.clear();
  scale_field_=nullptr;layout_field_=nullptr;display_message_=nullptr;light_track_=nullptr;
  fields_.fill(nullptr);input_frames_.clear();field_containers_.clear();field_captions_.clear();message_=nullptr;
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
      VesselForm();
      break;
    case SettingsSection::Navigation:
      Page("Navigation preferences","Units, chart orientation and navigation alarms",XNavIcon::Compass,ProductPage::NavigationSettings);
      Link("Chart presentation","Layers, orientation and chart palette",XNavIcon::Layers,actions_.chart_presentation);
      Page("Weather (GRIBstream)","Forecast wind, chart wind layer and provider token",XNavIcon::Compass,ProductPage::Weather);
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
      Page("Helm controls","Standby, Auto and heading adjustments",XNavIcon::Instruments,ProductPage::Pilot);
      CopyBlock(90,[](XNavPainter &p,int width){
        const bool loopback=integration::PilotLoopbackTestsEnabled();
        const bool manual=integration::PilotManualSerialEnabled();
        p.Text(loopback ? "Developer loopback testing only; control starts OFF."
                       : manual ? "Manual serial control is available; each session starts OFF."
                                : "This installation observes pilot status only.",
               0,18,11,p.c.secondary,false,width);
        p.Text(loopback ? "Physical equipment commands are unavailable."
                       : manual ? "Enable it explicitly. SmartNav never steers."
                                : "Use the pilot's own controls to steer.",
               0,39,11,p.c.secondary,false,width);});
      break;
    case SettingsSection::Radar:
      CopyBlock(104,[this](XNavPainter &p,int width){p.TextTracked("RADAR",0,4,9,p.c.accent,650,1.17);
        p.Text(state_.radar.available?"Adapter reported available":"Radar unavailable",0,30,23,p.c.primary,false,width);
        p.Text("A validated display adapter is required.",0,74,12,p.c.secondary,false,width);});
      Page("Radar status","Availability and supported capabilities",XNavIcon::Radar,ProductPage::Radar);
      Link("Plugins & adapters","OpenCPN integration components",XNavIcon::Layers,actions_.plugins);
      break;
    case SettingsSection::Display: {
      DisplayForm();
      break;
    }
    case SettingsSection::System: {
      auto title_lines=std::make_shared<std::vector<wxString>>(
          1,"A complete helm. A cared-for system.");
      CopyBlock(126,[title_lines](XNavPainter &p,int width){
        p.TextTracked(wxString(application::brand::Name) + wxString::FromUTF8(" · ")+wxString::FromUTF8(application::Version),
                      0,9,10,p.c.secondary,650,1.3,width);
        for(std::size_t i=0;i<title_lines->size();++i)
          p.TextTracked((*title_lines)[i],0,33+30*i,23,p.c.primary,700,-.6,width);
        p.Wrapped("Set up, maintain and recover your navigation workspace.",
                  0,73+30*(title_lines->size()-1),13,21,width,p.c.secondary,2);
      });
      auto *intro=copies_.back();
      intro->Bind(wxEVT_SIZE,[this,intro,title_lines](wxSizeEvent &event){
        wxClientDC dc(intro);dc.SetFont(UiFontWeight(*intro,23,700));
        const int width=intro->GetClientSize().x;
        const double tracking=-.6*intro->FromDIP(100)/100.;
        wxStringTokenizer words("A complete helm. A cared-for system."," ");
        title_lines->clear();wxString line;
        while(words.HasMoreTokens()) {
          const auto word=words.GetNextToken();
          const auto candidate=line.empty()?word:line+" "+word;
          if(!line.empty() && dc.GetTextExtent(candidate).x+tracking*(candidate.length()-1)>width) {
            title_lines->push_back(line);line=word;
          } else line=candidate;
        }
        title_lines->push_back(line);
        const int height=FromDIP(126+30*(title_lines->size()-1));
        if(intro->GetMinSize().y!=height) {
          intro->SetMinSize(wxSize(FromDIP(300),height));body_->Layout();body_->FitInside();
        }
        intro->Refresh(false);event.Skip();
      });
      Link("Installation & recovery","Installer unavailable; recovery controls below",XNavIcon::Download,{});
      Link("Updates","Update controls unavailable",XNavIcon::Refresh,{});
      Link("Export settings backup","Vessel, display, sources and calibration",XNavIcon::Shield,actions_.backup_export);
      Link("Import settings backup","Validate and review before restoring",XNavIcon::Shield,actions_.backup_import);
      Link("Diagnostics","Versions, data quality and source health",XNavIcon::Instruments,actions_.diagnostics);
      Link("Plugins","OpenCPN adapters and plugin settings",XNavIcon::Layers,actions_.plugins);
      Link("Help & guides","Basic help; guides unavailable",XNavIcon::Info,
           [this]{Select(SettingsSection::Help);});
      Link("About & licenses","Version above; license viewer unavailable",XNavIcon::Info,{});
      Link("Run vessel setup","Setup wizard unavailable; use Vessel tab",XNavIcon::Boat,{});
      Page("Interface & recovery","Legacy, Safe Mode, restart and diagnostics",XNavIcon::Shield,ProductPage::System);
      Link("Advanced / Legacy Settings",actions_.advanced
          ? "Connections, charts and additional preferences" : "OpenCPN settings unavailable",
          XNavIcon::Settings,actions_.advanced);
      break;
    }
    case SettingsSection::Help:
      CopyBlock(145,[](XNavPainter &p,int width){p.TextTracked(application::brand::Name,0,4,9,p.c.accent,650,1.17);
        p.Text("Charts and navigation are owned by OpenCPN.",0,36,12,p.c.secondary,false,width);
        p.Text("Predictions are advisory. Missing data stays unavailable.",0,62,12,p.c.secondary,false,width);
        p.Text("Legacy and Safe keep the same navigation profile.",0,88,12,p.c.secondary,false,width);});
      Link("Diagnostics","Inspect this build and its source health",XNavIcon::Settings,actions_.diagnostics);
      Link("Advanced OpenCPN settings","Existing charts, connections and preferences",XNavIcon::Settings,actions_.advanced);
      break;
  }
  ApplyScale();
  body_->Layout();body_->FitInside();
}
} // namespace opennav::ui

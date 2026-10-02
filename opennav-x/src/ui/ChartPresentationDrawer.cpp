#include "ui/ChartPresentationDrawer.h"
#include <cmath>
#include <wx/dcbuffer.h>
#include <wx/tokenzr.h>

namespace opennav::ui {
namespace {
using application::ChartOrientation;
using application::ChartFormat;
// Actual immutable Windows Layers rows: four 63.5px and four 52px rows.
constexpr std::array<double, 8> row_top{{0,63.5,115.5,179,231,283,346.5,398.5}};
constexpr std::array<double, 8> row_height{{63.5,52,63.5,52,52,63.5,52,63.5}};
constexpr std::array<unsigned, 3> editable_rows{{1,2,3}};
constexpr std::array<const char *, 8> labels{{"Chart symbols", "ENC text labels",
    "AIS vessels", "Depth soundings", "Depth contours", "Route corridor",
    "Wind vectors", "Radar overlay"}};
constexpr std::array<const char *, 3> orientation_labels{{"North up", "Course up", "Head up"}};
wxString W(const std::string &value) { return wxString::FromUTF8(value); }
bool Same(const application::ChartLayerState &a,const application::ChartLayerState &b) {
  return a.visible==b.visible && a.editable==b.editable && a.reason==b.reason;
}
bool Same(const application::ChartPresentationState &a,const application::ChartPresentationState &b) {
  return a.available==b.available && a.reason==b.reason && a.format==b.format &&
      a.format_reason==b.format_reason && a.orientation==b.orientation &&
      Same(a.ais_vessels,b.ais_vessels) && Same(a.enc_text,b.enc_text) &&
      Same(a.depth_soundings,b.depth_soundings) && Same(a.chart_symbols,b.chart_symbols) &&
      Same(a.depth_contours,b.depth_contours) && Same(a.route_corridor,b.route_corridor) &&
      Same(a.wind_vectors,b.wind_vectors) && Same(a.radar_overlay,b.radar_overlay);
}
}

XNavChartPresentationDrawer::XNavChartPresentationDrawer(
    wxWindow &owner, application::NavigationActions actions)
    : XNavDrawer(owner, "Chart presentation"), actions_(std::move(actions)) {
  SetHeading("CHART PRESENTATION", "Make it your chart", false);
  Bind(wxEVT_SHOW,[this](wxShowEvent &event) {
    if(!event.IsShown())++command_generation_;
    event.Skip();
  });
  CopyBlock(26, [](XNavPainter &p, int width) {
    p.TextTracked("CHART FORMAT",0,0,10,p.c.secondary,400,1,width);
  });
  format_=new wxPanel(body_,wxID_ANY);
  format_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  format_->SetMinSize(FromDIP(wxSize(1,48)));
  format_->Bind(wxEVT_PAINT,&XNavChartPresentationDrawer::PaintFormat,this);
  EnableScrollGesture(*format_);
  content_->Add(format_,0,wxEXPAND|wxBOTTOM,FromDIP(20));

  rows_=new wxPanel(body_,wxID_ANY);
  rows_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  rows_->SetMinSize(FromDIP(wxSize(1,462)));
  rows_->Bind(wxEVT_PAINT,&XNavChartPresentationDrawer::PaintRows,this);
  EnableScrollGesture(*rows_);
  for(unsigned i=0;i<layers_.size();++i) {
    auto *button=new XNavButton(rows_,wxID_ANY,labels[editable_rows[i]],labels[editable_rows[i]]);
    button->SetToggle();
    button->Bind(wxEVT_BUTTON,[this,i](wxCommandEvent &) {
      const auto generation=command_generation_;
      CallAfter([this,i,generation] {
        if(generation==command_generation_)ChangeLayer(i);
      });
    });
    layers_[i]=button;
  }
  rows_->Bind(wxEVT_SIZE,[this](wxSizeEvent &event) {
    for(unsigned i=0;i<layers_.size();++i) {
      const auto row=editable_rows[i];
      layers_[i]->SetSize(rows_->GetClientSize().x-FromDIP(48),
          FromDIP(std::lround(row_top[row]+(row_height[row]-49)/2)),
          FromDIP(48),FromDIP(49));
    }
    event.Skip();
  });
  content_->Add(rows_,0,wxEXPAND);
  CopyBlock(51, [](XNavPainter &p, int width) {
    p.TextTracked("ORIENTATION",0,23,10,p.c.secondary,400,1,width);
  });
  orientation_track_=new wxPanel(body_,wxID_ANY);
  orientation_track_->SetName("Chart orientation");
  orientation_track_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  orientation_track_->Bind(wxEVT_PAINT,[this](wxPaintEvent &) {
    wxAutoBufferedPaintDC dc(orientation_track_);
    dc.SetBackground(wxBrush(Colour(Theme(light_).background)));dc.Clear();
    dc.SetPen(*wxTRANSPARENT_PEN);dc.SetBrush(wxBrush(Colour(Theme(light_).surface)));
    dc.DrawRoundedRectangle(wxPoint(0,0),orientation_track_->GetClientSize(),FromDIP(9));
  });
  auto *choices=new wxBoxSizer(wxHORIZONTAL);
  for(unsigned i=0;i<orientations_.size();++i) {
    auto *button=new XNavButton(orientation_track_,wxID_ANY,orientation_labels[i],orientation_labels[i]);
    button->SetSegmentInTrack();button->SetMinSize(FromDIP(wxSize(1,40)));
    button->Bind(wxEVT_BUTTON,[this,i](wxCommandEvent &) {
      const auto generation=command_generation_;
      CallAfter([this,i,generation] {
        if(generation==command_generation_)ChangeOrientation(static_cast<ChartOrientation>(i));
      });
    });
    if(i)choices->AddSpacer(FromDIP(4));
    choices->Add(button,1);orientations_[i]=button;
  }
  auto *track_padding=new wxBoxSizer(wxVERTICAL);
  track_padding->Add(choices,1,wxEXPAND|wxALL,FromDIP(4));
  orientation_track_->SetSizer(track_padding);
  content_->Add(orientation_track_,0,wxEXPAND|wxBOTTOM,FromDIP(20));
  notes_=CopyBlock(1,[this](XNavPainter &p,int width) {
    for(const auto &line:note_lines_)
      p.Text(line.first,0,line.second,11,p.c.secondary,false,width);
  });
  notes_->Bind(wxEVT_SIZE,[this](wxSizeEvent &event) {
    ReflowNotes();event.Skip();
  });
  style_=new XNavButton(body_,wxID_ANY,"Chart palette preferences","Chart palette preferences");
  style_->SetMinSize(FromDIP(wxSize(1,48)));style_->SetDisplayAction(48);
  style_->SetHint("Open the separate XNav / Standard chart palette preference");
  style_->Bind(wxEVT_BUTTON,[this](wxCommandEvent &) {
    const auto generation=command_generation_;
    CallAfter([this,generation] {
      if(IsShownOnScreen() && generation==command_generation_ && on_style_preferences)on_style_preferences();
    });
  });
  content_->Add(style_,0,wxEXPAND);
  Update(actions_.chart_presentation?actions_.chart_presentation():application::ChartPresentationState{},light_);
}

void XNavChartPresentationDrawer::Open(const wxRect &workspace,
    const application::ChartPresentationState &state,LightMode mode) {
  ++command_generation_;
  if(!feedback_.empty()) {feedback_.clear();rendered_=false;}
  Update(state,mode);Present(workspace);body_->Scroll(0,0);
}

wxPanel *XNavChartPresentationDrawer::CopyBlock(
    int height,std::function<void(XNavPainter &,int)> draw) {
  auto *panel=new wxPanel(body_,wxID_ANY);
  panel->SetBackgroundStyle(wxBG_STYLE_PAINT);
  panel->SetMinSize(FromDIP(wxSize(1,height)));
  panel->Bind(wxEVT_PAINT,[this,panel,draw](wxPaintEvent &) {
    wxAutoBufferedPaintDC dc(panel);XNavPainter p(*panel,dc,light_);
    dc.SetBackground(wxBrush(Colour(p.c.background)));dc.Clear();
    draw(p,panel->ToDIP(panel->GetClientSize().x));
  });
  EnableScrollGesture(*panel);content_->Add(panel,0,wxEXPAND);copies_.push_back(panel);
  return panel;
}

void XNavChartPresentationDrawer::Update(const application::ChartPresentationState &state,LightMode mode) {
  const bool style_available=bool(on_style_preferences);
  if(rendered_ && Same(state_,state) && light_==mode && style_available_==style_available)return;
  const bool layout_changed=!rendered_ || style_available_!=style_available;
  rendered_=true;style_available_=style_available;
  state_=state;SetLight(mode);
  ReflowNotes();
  rows_->SetBackgroundColour(Colour(Theme(mode).background));
  orientation_track_->SetBackgroundColour(Colour(Theme(mode).surface));
  const std::array<const application::ChartLayerState *,3> states{{&state_.enc_text,&state_.ais_vessels,&state_.depth_soundings}};
  const std::array<bool,3> callbacks{{bool(actions_.set_chart_enc_text),bool(actions_.set_chart_ais),bool(actions_.set_chart_soundings)}};
  for(unsigned i=0;i<layers_.size();++i) {
    const auto &layer=*states[i];auto *button=layers_[i];
    button->SetLightMode(mode);button->Show(layer.visible.has_value());
    button->SetSelected(layer.visible.value_or(false));
    button->Enable(state_.available && layer.visible.has_value() && layer.editable && callbacks[i]);
    button->SetHint(W(layer.reason));
  }
  for(unsigned i=0;i<orientations_.size();++i) {
    orientations_[i]->SetLightMode(mode);
    orientations_[i]->SetSelected(state_.orientation && *state_.orientation==static_cast<ChartOrientation>(i));
    orientations_[i]->Enable(state_.available && state_.orientation.has_value() && bool(actions_.set_chart_orientation));
  }
  format_->SetName(wxString("Observed chart format: ")+
      (state_.format==ChartFormat::Vector?"Vector":state_.format==ChartFormat::Raster?"Raster":"Unavailable"));
  style_->SetLightMode(mode);style_->Show(bool(on_style_preferences));
  format_->Refresh(false);rows_->Refresh(false);orientation_track_->Refresh(false);
  for(auto *panel:copies_)panel->Refresh(false);
  if(layout_changed) {body_->Layout();body_->FitInside();}
}

void XNavChartPresentationDrawer::ReflowNotes() {
  const int width=notes_->GetClientSize().x;
  if(width<=0)return;
  const auto format=state_.format==ChartFormat::Vector?"Vector":
      state_.format==ChartFormat::Raster?"Raster":"Unavailable";
  const wxString status=!feedback_.empty()?feedback_:
      !state_.available?W(state_.reason):!state_.orientation?"Chart orientation unavailable":wxString{};
  const std::array<wxString,5> paragraphs{{
      wxString("Observed chart format: ")+format+". This is not a style switch.",
      W(state_.format_reason),
      "Chart symbols and depth contours remain under OpenCPN presentation and safety settings. "
      "ENC controls do not alter text or soundings embedded in raster charts.",
      "Course up needs a current course; Head up needs current heading. Check source health.",
      status}};
  wxClientDC dc(notes_);dc.SetFont(UiFont(*notes_,11));
  note_lines_.clear();double y=0;
  // Active .drawer-body .note: 11px / 1.65. Round each accumulated line
  // position, rather than losing the fractional leading on every line.
  const auto emit=[&](const wxString &line) {
    note_lines_.emplace_back(line,std::lround(y));y+=11*1.65;
  };
  for(const auto &paragraph:paragraphs) {
    if(paragraph.empty())continue;
    wxStringTokenizer words(paragraph," ");wxString line;
    while(words.HasMoreTokens()) {
      auto word=words.GetNextToken();
      const auto candidate=line.empty()?word:line+" "+word;
      if(!line.empty() && dc.GetTextExtent(candidate).x>width) {
        emit(line);line.clear();
      }
      // Keep even an unbroken provider message within the note surface.
      while(dc.GetTextExtent(word).x>width && word.length()>1) {
        std::size_t count=word.length()-1;
        while(count>1 && dc.GetTextExtent(word.Left(count)).x>width)--count;
        emit(word.Left(count));word=word.Mid(count);
      }
      line=line.empty()?word:line+" "+word;
    }
    if(!line.empty())emit(line);
    y+=18;
  }
  const int height=notes_->FromDIP(static_cast<int>(std::ceil(y)));
  if(notes_->GetMinSize().y!=height) {
    notes_->SetMinSize(wxSize(1,height));
    body_->Layout();body_->FitInside();
  }
  notes_->Refresh(false);
}

void XNavChartPresentationDrawer::ChangeLayer(unsigned index) {
  const std::array<const application::ChartLayerState *,3> states{{&state_.enc_text,&state_.ais_vessels,&state_.depth_soundings}};
  if(!IsShownOnScreen() || index>=states.size() || !state_.available || !states[index]->editable || !states[index]->visible)return;
  const auto callback=index==0?actions_.set_chart_enc_text:index==1?actions_.set_chart_ais:actions_.set_chart_soundings;
  if(callback)Accept(callback(!*states[index]->visible));
}
void XNavChartPresentationDrawer::ChangeOrientation(ChartOrientation orientation) {
  if(IsShownOnScreen() && state_.available && state_.orientation && actions_.set_chart_orientation)
    Accept(actions_.set_chart_orientation(orientation));
}
void XNavChartPresentationDrawer::Accept(application::ChartPresentationResult result) {
  feedback_=result.command.ok?wxString{}:W(result.command.message);
  rendered_=false;
  Update(result.state,light_);
}

void XNavChartPresentationDrawer::PaintFormat(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(format_);const auto c=Theme(light_);
  dc.SetBackground(wxBrush(Colour(c.background)));dc.Clear();dc.SetPen(*wxTRANSPARENT_PEN);
  const auto size=format_->GetClientSize();
  dc.SetBrush(wxBrush(Colour(c.surface)));dc.DrawRoundedRectangle(0,0,size.x,size.y,FromDIP(9));
  const int gap=FromDIP(4),available=size.x-3*gap;
  dc.SetFont(UiFontWeight(*format_,10,400));
  for(int i=0;i<2;++i) {
    const int x=gap+i*(available/2+gap),width=i?available-available/2:available/2;
    const bool selected=state_.format==(i?ChartFormat::Raster:ChartFormat::Vector);
    if(selected) {dc.SetBrush(wxBrush(Colour(c.selected)));dc.DrawRoundedRectangle(x,gap,width,FromDIP(40),FromDIP(6));}
    dc.SetTextForeground(Colour(selected?c.primary:c.muted));
    const wxString text=i?"Raster":"Vector";const auto extent=dc.GetTextExtent(text);
    dc.DrawText(text,x+(width-extent.x)/2,(size.y-extent.y)/2);
  }
}

void XNavChartPresentationDrawer::PaintRows(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(rows_);XNavPainter p(*rows_,dc,light_);
  dc.SetBackground(wxBrush(Colour(p.c.background)));dc.Clear();
  const int width=rows_->ToDIP(rows_->GetClientSize().x);
  const std::array<const application::ChartLayerState *,8> states{{&state_.chart_symbols,&state_.enc_text,
      &state_.ais_vessels,&state_.depth_soundings,&state_.depth_contours,&state_.route_corridor,
      &state_.wind_vectors,&state_.radar_overlay}};
  for(unsigned i=0;i<labels.size();++i) {
    const int top=std::lround(row_top[i]),height=std::lround(row_height[i]);
    wxString detail;
    if(i==0)detail="Managed in OpenCPN; no single on/off state";
    else if(i==4)detail="OpenCPN safety presentation";
    else if(i>=5 || !states[i]->editable)detail=W(states[i]->reason);
    else if(i==1)detail="Master ENC text; buoy/light choices retained";
    else if(i==2)detail="Names, vectors and closest approach";
    const int label_y=detail.empty()?(height-18)/2:height<60?7:13;
    p.Text(labels[i],0,top+label_y,12,p.c.secondary,false,width-85);
    if(!detail.empty())p.Text(detail,0,top+label_y+22,11,p.c.muted,false,width-65);
    if(i==0 || i==4)p.TextWeight("Managed",width-74,top+(height-16)/2,10,p.c.muted,400,74,true);
    else if(i>=5 || !states[i]->visible)p.TextWeight("Unavailable",width-74,top+(height-16)/2,10,p.c.muted,400,74,true);
    p.Rule(0,std::lround(row_top[i]+row_height[i])-1,width);
  }
}
} // namespace opennav::ui

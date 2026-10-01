#include "ui/HealthDrawer.h"
#include "ui/PrototypeGeometry.h"
#include <cmath>
#include <wx/dcbuffer.h>

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
std::uint32_t Ink(application::SignalState state, const Palette &c) {
  using S = application::SignalState;
  return state == S::Current ? c.accent
       : state == S::Aging || state == S::Stale || state == S::Estimated ||
                 state == S::Uncertain || state == S::Invalid ? c.attention : c.muted;
}
}
XNavHealthDrawer::XNavHealthDrawer(wxWindow &owner, HealthDrawerActions actions)
    : XNavDrawer(owner, "OpenNav source health"), actions_(std::move(actions)) {
  SetHeading("SOURCE HEALTH", "Know what to trust", false);
}
void XNavHealthDrawer::Update(application::SourceHealthView view, LightMode mode) {
  bool rebuild = view_.signals.size() != view.signals.size() || !intro_;
  if (!rebuild)
    for (std::size_t i=0; i<view.signals.size(); ++i)
      rebuild |= view.signals[i].id != view_.signals[i].id;
  view_ = std::move(view);
  SetLight(mode);
  if (rebuild) Build();
  intro_->Refresh(false);
  bool relayout=false;
  for (std::size_t i=0; i<view_.signals.size(); ++i) {
    const auto &s=view_.signals[i];
    auto text=W(s.status);
    if(s.age) text += wxString::Format(W(" · %.1f s"),s.age->count()/1000.);
    summaries_[i]->SetLightMode(mode);
    summaries_[i]->SetDisclosure(text,Ink(s.state,Theme(mode)),expanded_[i]);
    const double top=i*(prototype::health_summary+prototype::health_gap);
    const int height=std::lround(top+prototype::health_summary)-std::lround(top);
    const auto wanted=FromDIP(wxSize(300,height-(expanded_[i]?12:0)));
    if(summaries_[i]->GetMinSize()!=wanted) {
      summaries_[i]->SetMinSize(wanted);relayout=true;
    }
    details_[i]->SetBackgroundColour(Colour(Theme(mode).surface));
    if(expanded_[i]) details_[i]->Refresh(false);
    configure_[i]->SetLightMode(mode);
    configure_[i]->Enable(bool(actions_.configure) && !view_.historical);
  }
  manage_->SetLightMode(mode); diagnostics_->SetLightMode(mode);
  manage_->Enable(bool(actions_.manage) && !view_.historical);
  diagnostics_->Enable(bool(actions_.diagnostics));
  if(relayout){body_->Layout();body_->FitInside();}
}
void XNavHealthDrawer::Toggle(std::size_t index) {
  if(index>=expanded_.size())return;
  const int scroll=body_->GetViewStart().y;
  expanded_[index]=!expanded_[index];
  details_[index]->Show(expanded_[index]);
  Update(view_,light_);
  body_->Layout();body_->FitInside();body_->Scroll(0,scroll);
}
void XNavHealthDrawer::Build() {
  ClearBody();summaries_.clear();configure_.clear();details_.clear();
  expanded_.assign(view_.signals.size(),false);
  intro_=new wxPanel(body_,wxID_ANY);
  intro_->SetName("Source health provenance");
  intro_->SetMinSize(FromDIP(wxSize(300,46)));
  intro_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  intro_->Bind(wxEVT_PAINT,[this](wxPaintEvent &){
    wxAutoBufferedPaintDC dc(intro_);XNavPainter p(*intro_,dc,light_);
    dc.SetBackground(wxBrush(Colour(p.c.background)));dc.Clear();
    p.Tag(view_.historical?"HISTORICAL DATA":"SIGNAL OBSERVATIONS",0,0,240,view_.historical);
  });
  content_->Add(intro_,0,wxEXPAND);
  for(std::size_t i=0;i<view_.signals.size();++i) {
    const auto &signal=view_.signals[i];
    auto *summary=new XNavButton(body_,wxID_ANY,W(signal.title),"Inspect source "+W(signal.id));
    summary->SetMinSize(FromDIP(wxSize(300,69)));
    summary->Bind(wxEVT_BUTTON,[this,i](wxCommandEvent &){CallAfter([this,i]{Toggle(i);});});
    content_->Add(summary,0,wxEXPAND);summaries_.push_back(summary);
    auto *detail=new wxPanel(body_,wxID_ANY);
    detail->SetName("Source detail "+W(signal.id));
    detail->SetMinSize(FromDIP(wxSize(300,313)));
    detail->SetBackgroundStyle(wxBG_STYLE_PAINT);
    detail->Bind(wxEVT_PAINT,[this,i,detail](wxPaintEvent &){
      wxAutoBufferedPaintDC dc(detail);XNavPainter p(*detail,dc,light_);
      dc.SetBackground(wxBrush(Colour(p.c.background)));dc.Clear();
      dc.SetPen(*wxTRANSPARENT_PEN);dc.SetBrush(wxBrush(Colour(p.c.surface)));
      dc.DrawRoundedRectangle(0,detail->FromDIP(-8),detail->GetClientSize().x,
          detail->GetClientSize().y+detail->FromDIP(8),detail->FromDIP(8));
      if(i>=view_.signals.size())return;
      const auto &s=view_.signals[i];const int w=detail->ToDIP(detail->GetClientSize().x)-24;
      const wxString labels[]={"Selected source","Measurement","Observed cadence","Source precedence","Validity"};
      const wxString values[]={W(s.source.empty()?"Not observed":s.source),W(s.measurement),
          s.frequency_hz?wxString::Format("%.1f Hz",*s.frequency_hz):"Not measured",
          s.priority?wxString::Format("%u",*s.priority):"Not reported",W(s.status)};
      for(int row=0;row<5;++row) {
        const int y=std::lround(row*prototype::health_detail_row);
        p.Text(labels[row],12,y+13,12,p.c.secondary,false,w/2-8);
        p.TextWeight(values[row],12+w/2,y+13,12,p.c.primary,500,w/2,true);
        p.Rule(12,std::lround((row+1)*prototype::health_detail_row)-1,w);
      }
    });
    EnableScrollGesture(*detail);
    auto *configure=new XNavButton(detail,wxID_ANY,
        signal.id=="online"?"Online AIS settings":signal.id=="ais"?"View traffic":
        signal.id=="pilot"?"Adapter configuration":"Configure this sensor",
        "Configure source "+W(signal.id));
    configure->Bind(wxEVT_BUTTON,[this,i](wxCommandEvent &){
      if(view_.historical||!actions_.configure||i>=view_.signals.size())return;
      auto signal=view_.signals[i];
      CallAfter([this,signal]{if(!view_.historical&&actions_.configure)actions_.configure(signal);});
    });
    detail->Bind(wxEVT_SIZE,[detail,configure](wxSizeEvent &e){
      configure->SetSize(detail->FromDIP(12),detail->FromDIP(253),
          detail->GetClientSize().x-detail->FromDIP(24),detail->FromDIP(48));e.Skip();
    });
    content_->Add(detail,0,wxEXPAND);detail->Hide();details_.push_back(detail);configure_.push_back(configure);
    content_->AddSpacer(FromDIP(8));
  }
  manage_=new XNavButton(body_,wxID_ANY,"Manage all sensors","Manage all sensors");
  manage_->SetSuiteLink("Assign, prioritise or reconnect",XNavIcon::Instruments);
  manage_->SetMinSize(FromDIP(wxSize(300,72)));
  manage_->Bind(wxEVT_BUTTON,[this](wxCommandEvent &){if(!view_.historical&&actions_.manage)CallAfter(actions_.manage);});
  content_->Add(manage_,0,wxEXPAND|wxTOP,FromDIP(12));
  diagnostics_=new XNavButton(body_,wxID_ANY,"Export diagnostics","Export diagnostics");
  diagnostics_->SetDisplayAction(44);
  diagnostics_->SetInterfaceScale(InterfaceScale());
  diagnostics_->Bind(wxEVT_BUTTON,[this](wxCommandEvent &){if(actions_.diagnostics)CallAfter(actions_.diagnostics);});
  content_->Add(diagnostics_,0,wxEXPAND|wxTOP,FromDIP(10));
  body_->Layout();body_->FitInside();
}
} // namespace opennav::ui

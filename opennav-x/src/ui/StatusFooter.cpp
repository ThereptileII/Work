#include "ui/StatusFooter.h"
#include <wx/dcbuffer.h>
#include <wx/graphics.h>
#include <wx/timer.h>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <tuple>
#include <vector>

namespace opennav::ui {
namespace {
struct Part {
  wxString text;
  int size = 9, weight = 400;
  std::uint32_t ink;
  double tracking = 0;
  bool dot = false, separator = false;
};
using Parts = std::vector<Part>;
double Scale(wxWindow &w) { return w.FromDIP(1000) / 1000.0; }
std::uint32_t StateInk(application::SignalState state, const Palette &c) {
  using S = application::SignalState;
  return state == S::Current ? c.healthy : state == S::Unavailable ? c.muted : c.attention;
}
Parts Left(const application::FooterView &v, LightMode mode) {
  const auto c = Theme(mode); const auto ink = StateInk(v.position_state,c);
  auto position=wxString::FromUTF8(v.position);position.Replace("   "," "); // HTML collapses whitespace.
  return {{"",9,400,ink,0,true}, {wxString::FromUTF8(v.navigation_state),7,500,ink,.7},
          {"/",9,400,c.border,0,false,true}, {position,9,400,c.muted}};
}
Parts Middle(const application::FooterView &v, LightMode mode) {
  const auto c = Theme(mode);
  return {{"COG",9,400,c.muted}, {wxString::FromUTF8(v.cog),7,500,StateInk(v.cog_state,c),.7},
          {"/",9,400,c.border,0,false,true}, {"XTE",9,400,c.muted},
          {wxString::FromUTF8(v.xte),9,500,c.secondary,.18}};
}
Parts Health(const application::FooterView &v, LightMode mode) {
  const auto c = Theme(mode);
  return {{wxString::FromUTF8(v.health_source),9,400,c.muted},
          {"",9,400,StateInk(v.health_state,c),0,true}, {"/",9,400,c.border,0,false,true},
          {wxString::FromUTF8(v.health_summary),9,400,c.muted}, {wxString::FromUTF8("↗"),9,400,c.muted}};
}
double Advance(wxWindow &w, const Part &p) {
  if (p.dot) return w.FromDIP(4);
  double width = 0;
  if (!p.tracking) width = UiTextWidth(w,p.text,p.size,p.weight);
  else for (auto ch:p.text) width += UiTextWidth(w,wxString(ch),p.size,p.weight) + p.tracking*Scale(w);
  return width + (p.separator ? w.FromDIP(10) : 0);
}
double Width(wxWindow &w,const Parts &parts) {
  double result = parts.empty() ? 0 : w.FromDIP(8)*(parts.size()-1);
  for (const auto &p:parts) result += Advance(w,p);
  return result;
}
void Draw(wxWindow &w,wxDC &dc,const Parts &parts,double x,int height,double brightness=1.0) {
  std::unique_ptr<wxGraphicsContext> g(wxGraphicsContext::CreateFromUnknownDC(dc));
  if (!g) return;
  for(const auto &p:parts) {
    auto color=Colour(p.ink);
    if(brightness!=1) color=wxColour(std::min(255,int(color.Red()*brightness)),std::min(255,int(color.Green()*brightness)),std::min(255,int(color.Blue()*brightness)));
    const double start=x+(p.separator?w.FromDIP(5):0);
    if(p.dot) {
      g->SetPen(*wxTRANSPARENT_PEN); g->SetBrush(wxBrush(color));
      g->DrawEllipse(start,(height-w.FromDIP(4))/2.0,w.FromDIP(4),w.FromDIP(4));
    } else {
      g->SetFont(UiFontWeight(w,p.size,p.weight),color);
      double tw,th,descent,leading;
      g->GetTextExtent(p.text,&tw,&th,&descent,&leading);
      const double y=(height-th)/2.0;
      if(!p.tracking) g->DrawText(p.text,start,y);
      else {
        double cursor=start;
        for(auto ch:p.text) {const wxString s(ch);g->DrawText(s,cursor,y);cursor+=UiTextWidth(w,s,p.size,p.weight)+p.tracking*Scale(w);}
      }
    }
    x+=Advance(w,p)+w.FromDIP(8);
  }
}
auto Visual(const application::FooterView &v) {
  return std::make_tuple(v.navigation_state,v.position,v.cog,v.xte,v.health_source,
                        v.health_summary,v.position_state,v.cog_state,v.health_state,v.historical);
}
}
class FooterHealthButton final : public XNavButton {
 public:
  FooterHealthButton(wxWindow *parent,std::function<void()> callback)
      :XNavButton(parent,wxID_ANY,"Source health","Footer source health") {
    SetMinSize({0,0}); SetRole(ButtonRole::Quiet);
    SetHint("Inspect current vessel data and separate online AIS status");
    Bind(wxEVT_BUTTON,[callback=std::move(callback)](wxCommandEvent &){if(callback)callback();});
    hover_timer_.SetOwner(this);
    Bind(wxEVT_ENTER_WINDOW,[this](wxMouseEvent &e){Animate(1.08);e.Skip();});
    Bind(wxEVT_LEAVE_WINDOW,[this](wxMouseEvent &e){Animate(1);e.Skip();});
    Bind(wxEVT_TIMER,[this](wxTimerEvent &){
      const double progress=std::min(1.0,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-hover_start_).count()/160.0);
      // CSS default ease: cubic-bezier(.25,.1,.25,1), evaluated by x.
      double lo=0,hi=1;
      for(int i=0;i<16;++i) {const double t=(lo+hi)/2,u=1-t;const double x=3*u*u*t*.25+3*u*t*t*.25+t*t*t;if(x<progress)lo=t;else hi=t;}
      const double t=(lo+hi)/2,u=1-t,ease=3*u*u*t*.1+3*u*t*t+t*t*t;
      brightness_=progress==1?hover_to_:hover_from_+(hover_to_-hover_from_)*ease;
      if(progress==1)hover_timer_.Stop();
      Refresh(false);
    },hover_timer_.GetId());
    Bind(wxEVT_PAINT,[this](wxPaintEvent &){
      wxAutoBufferedPaintDC dc(this);dc.SetBackground(wxBrush(Colour(Theme(mode_).background)));dc.Clear();
      Draw(*this,dc,Health(view_,mode_),FromDIP(6),GetClientSize().y,brightness_);
      if(HasKeyboardFocus()) {dc.SetPen(wxPen(Colour(Theme(mode_).border)));dc.SetBrush(*wxTRANSPARENT_BRUSH);dc.DrawRectangle(GetClientRect());}
    });
  }
  void Present(const application::FooterView &view,LightMode mode) {
    const bool changed=Visual(view_)!=Visual(view)||mode_!=mode;
    view_=view;mode_=mode;SetLightMode(mode);
    if(changed)Refresh(false);
  }
  wxSize NaturalSize() {
    wxClientDC dc(this);dc.SetFont(UiFontWeight(*this,9,400));
    return {int(std::ceil(Width(*this,Health(view_,mode_))))+FromDIP(12),dc.GetTextExtent("Mg").y+FromDIP(2)};
  }
 private:
  application::FooterView view_;
  LightMode mode_=LightMode::Day;
  void Animate(double to) {
    hover_from_=brightness_;hover_to_=to;hover_start_=std::chrono::steady_clock::now();
    hover_timer_.Start(16);
  }
  wxTimer hover_timer_;
  std::chrono::steady_clock::time_point hover_start_;
  double brightness_=1,hover_from_=1,hover_to_=1;
};
XNavStatusFooter::XNavStatusFooter(wxWindow *parent,std::function<void()> health)
    :wxPanel(parent,wxID_ANY) {
  SetName("OpenNav status footer");SetLabel("SKAGER status footer");
  SetBackgroundStyle(wxBG_STYLE_PAINT);SetMinSize(FromDIP(wxSize(0,34)));
  health_=new FooterHealthButton(this,std::move(health));
  Bind(wxEVT_PAINT,&XNavStatusFooter::Paint,this);
  Bind(wxEVT_SIZE,[this](wxSizeEvent &e){Layout();Refresh(false);e.Skip();});
}
void XNavStatusFooter::Update(application::FooterView view,LightMode mode) {
  const bool changed=Visual(view_)!=Visual(view)||mode_!=mode;
  view_=std::move(view);mode_=mode;
  if(changed) {health_->Present(view_,mode_);Layout();Refresh(false);}
}
bool XNavStatusFooter::Layout() {
  const auto size=GetClientSize();const int padding=FromDIP(20);
  const auto health=health_->NaturalSize();
  health_->SetSize(size.x-padding-health.x,(size.y-health.y)/2,health.x,health.y);
  left_={padding,0,int(std::ceil(Width(*this,Left(view_,mode_)))),size.y};
  middle_visible_=size.x>FromDIP(1100);
  const int middle_width=int(std::ceil(Width(*this,Middle(view_,mode_))));
  // CSS space-between: equal *free gaps*, not a fixed centered middle group.
  const double gap=(size.x-2*padding-left_.width-middle_width-health.x)/2.0;
  middle_={int(std::lround(padding+left_.width+gap)),0,middle_visible_?middle_width:0,size.y};
  return true;
}
void XNavStatusFooter::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);const auto c=Theme(mode_);
  dc.SetBackground(wxBrush(Colour(c.background)));dc.Clear();
  dc.SetPen(wxPen(Colour(c.border),FromDIP(1)));dc.DrawLine(0,0,GetClientSize().x,0);
  Draw(*this,dc,Left(view_,mode_),left_.x,GetClientSize().y);
  if(middle_visible_)Draw(*this,dc,Middle(view_,mode_),middle_.x,GetClientSize().y);
}
} // namespace opennav::ui

#include "ui/Horizon.h"
#include "ui/PrototypeIcons.h"
#include "ui/PrototypeGeometry.h"
#include <wx/bmpbndl.h>
#include <wx/dcbuffer.h>
#include <wx/graphics.h>
#include <wx/event.h>
#include <wx/toplevel.h>
#include <algorithm>
#include <cmath>
#include <memory>

namespace opennav::ui {
namespace {
double Scale(wxWindow &w) {return w.FromDIP(1000)/1000.0;}
wxColour Ink(std::uint32_t value,double brightness=1,double opacity=1,std::uint32_t background=0) {
  const auto c=Colour(value),bg=Colour(background);
  const auto channel=[&](int v,int b){return std::min(255,int(std::lround((v*opacity+b*(1-opacity))*brightness)));};
  return {static_cast<unsigned char>(channel(c.Red(),bg.Red())),static_cast<unsigned char>(channel(c.Green(),bg.Green())),static_cast<unsigned char>(channel(c.Blue(),bg.Blue()))};
}
// Normal line boxes measured from the immutable prototype's canonical render:
// 8/9/10/11/12/13px -> 9/10/11/12/14/15px. The native Windows comparison gate
// must independently verify its installed-font result; Linux is not acceptance.
int LineHeight(int size) {return size<=11?size+1:size==12?14:15;}
double DrawText(wxWindow &w,wxGraphicsContext &g,const wxString &text,double x,double y,int size,int weight,
                const wxColour &color,double maximum=0,double tracking=0) {
  g.SetFont(UiFontWeight(w,size,weight),color);
  wxString shown=text;
  if(maximum>0 && UiTextWidth(w,shown,size,weight)>maximum) {
    while(!shown.empty() && UiTextWidth(w,shown+wxString::FromUTF8("…"),size,weight)>maximum)shown.RemoveLast();
    shown+=wxString::FromUTF8("…");
  }
  if(!tracking){g.DrawText(shown,x,y);return UiTextWidth(w,shown,size,weight);}
  // Measure prefixes as runs: summing rounded native glyph advances drifts
  // several pixels across the tracked eyebrow and moves its separator.
  wxString prefix;
  for(auto ch:shown){g.DrawText(wxString(ch),x+UiTextWidth(w,prefix,size,weight)+prefix.length()*tracking*Scale(w),y);prefix+=ch;}
  return UiTextWidth(w,shown,size,weight)+shown.length()*tracking*Scale(w);
}
std::uint32_t MarkerInk(const application::HorizonItem &item,const Palette &c) {
  using M=application::HorizonMarker;
  if(item.severity!=smartnav::Severity::Information || item.marker==M::Traffic)return c.attention;
  return item.marker==M::Now || item.marker==M::Arrival ? c.accent : item.marker==M::Unavailable ? c.muted : c.secondary;
}
}
class HorizonButton final:public XNavButton {
 public:
  HorizonButton(wxWindow *parent,const wxString &label,bool full,std::function<void(const application::HorizonAction &)> action)
      :XNavButton(parent,wxID_ANY,label,label),full_(full) {
    SetMinSize({0,0});
    Bind(wxEVT_LEFT_DOWN,[this](wxMouseEvent &e){armed_=item_.action;e.Skip();});
    // XNavButton handles Return in CHAR_HOOK on wxMSW, before dialog
    // navigation can consume the key-down. Preserve the row identity at that
    // same boundary so the later key-up cannot activate changed content.
    Bind(wxEVT_CHAR_HOOK,[this](wxKeyEvent &e){
      if(e.GetKeyCode()==WXK_RETURN&&!e.IsAutoRepeat())armed_=item_.action;
      e.Skip();
    });
    Bind(wxEVT_KEY_DOWN,[this](wxKeyEvent &e){if((e.GetKeyCode()==WXK_RETURN || e.GetKeyCode()==WXK_SPACE)&&!e.IsAutoRepeat())armed_=item_.action;e.Skip();});
    Bind(wxEVT_BUTTON,[this,action=std::move(action)](wxCommandEvent &){
      // A changed identity during press/release must not activate the new row.
      const bool allowed=action && IsEnabled() && IsShownOnScreen() && (full_ || (armed_ && *armed_==item_.action));
      const auto request=item_.action; // callback may refresh/rebuild this view
      armed_.reset();
      if(allowed)action(request);
    });
    Bind(wxEVT_PAINT,&HorizonButton::Paint,this);
    Bind(wxEVT_SET_FOCUS,[this](wxFocusEvent &e){GetParent()->Refresh(false);e.Skip();});
    Bind(wxEVT_KILL_FOCUS,[this](wxFocusEvent &e){GetParent()->Refresh(false);e.Skip();});
  }
  bool KeyboardFocus() const{return HasKeyboardFocus();}
  void Present(application::HorizonItem item,LightMode mode) {
    const bool changed=!(item_==item)||mode_!=mode;
    item_=std::move(item);mode_=mode;
    if(changed){SetLightMode(mode);Enable(full_||item_.action.kind!=application::HorizonActionKind::None);SetHint(wxString::FromUTF8(item_.title+" · "+item_.detail));Refresh(false);}
  }
  void Geometry(bool first,bool narrow,int title,int detail,int gap) {
    first_=first;narrow_=narrow;title_=title;detail_=detail;gap_=gap;
  }
  double NaturalWidth(){return UiTextWidth(*this,"Full passage ",11)+FromDIP(7)+UiTextWidth(*this,wxString::FromUTF8("↗"),11);}
 private:
  void Paint(wxPaintEvent &) {
    wxAutoBufferedPaintDC dc(this);const auto c=Theme(mode_);
    dc.SetBackground(wxBrush(Colour(c.background)));dc.Clear();
    std::unique_ptr<wxGraphicsContext> g(wxGraphicsContext::Create(dc));if(!g)return;
    g->EnableOffset(false); // coordinates already align odd-width rules to pixels
    const double brightness=IsHovered()&&IsEnabled()?1.08:1;
    const double opacity=IsEnabled()?1:.42;
    const auto ink=[&](std::uint32_t value){return Ink(value,brightness,opacity,c.background);};
    if(full_) {
      const double x=DrawText(*this,*g,"Full passage ",0,FromDIP(4),11,400,ink(c.secondary));
      DrawText(*this,*g,wxString::FromUTF8("↗"),x+FromDIP(7),FromDIP(4),11,400,ink(c.secondary));return;
    }
    if(!first_){g->SetPen(wxPen(ink(c.border),FromDIP(1)));g->StrokeLine(.5,0,.5,GetClientSize().y);}
    const double x=first_?0:FromDIP(narrow_?13:17),right=FromDIP(narrow_?8:15);
    const double width=std::max(0.,GetClientSize().x-x-right);
    const bool now=item_.marker==application::HorizonMarker::Now;
    const int time_size=now?8:10;
    double y=0;
    const auto time=wxString::FromUTF8(item_.time);
    const double advance=DrawText(*this,*g,time,x,y,time_size,400,ink(now?c.accent:c.secondary),width);
    if(!narrow_&&!item_.secondary_time.empty())DrawText(*this,*g,wxString::FromUTF8(item_.secondary_time),x+advance+FromDIP(6),y+FromDIP(LineHeight(time_size)-LineHeight(8)),8,400,ink(c.muted),std::max(1.,width-advance-FromDIP(6)));
    y+=FromDIP(LineHeight(time_size)+gap_);
    DrawText(*this,*g,wxString::FromUTF8(item_.title),x,y,title_,550,ink(c.primary),width);
    y+=FromDIP(LineHeight(title_)+gap_);
    const double text=DrawText(*this,*g,wxString::FromUTF8(item_.detail),x,y,detail_,400,ink(c.muted),width);
    if(!item_.detail_accent.empty() && text<width)DrawText(*this,*g,wxString::FromUTF8(item_.detail_accent),x+text,y,detail_,400,ink(c.accent),width-text);
  }
  application::HorizonItem item_;
  std::optional<application::HorizonAction> armed_;
  LightMode mode_=LightMode::Day;
  bool full_=false,first_=false,narrow_=false;
  int title_=13,detail_=10,gap_=4;
};
XNavHorizon::XNavHorizon(wxWindow *parent,std::function<void()> passage,
                       std::function<void(const application::HorizonAction &)> activate)
    :wxPanel(parent,wxID_ANY) {
  SetBackgroundStyle(wxBG_STYLE_PAINT);SetName("Navigation horizon / advisory only");
  passage_=new HorizonButton(this,"Full passage",true,[passage=std::move(passage)](const auto &){if(passage)passage();});
  for(size_t i=0;i<events_.size();++i)
    events_[i]=new HorizonButton(this,i?wxString::Format("Horizon event %d",int(i)):wxString("Horizon now"),false,activate);
  Bind(wxEVT_PAINT,&XNavHorizon::Paint,this);
  Bind(wxEVT_SIZE,[this](wxSizeEvent &e){Layout();Refresh(false);e.Skip();});
}
void XNavHorizon::Update(application::HorizonView view,LightMode mode) {
  if(view_==view && mode_==mode)return;
  ++presentation_changes_;view_=std::move(view);mode_=mode;
  passage_->Present({},mode);
  for(size_t i=0;i<events_.size();++i){events_[i]->Present(view_.items[i],mode);events_[i]->Show(!view_.items[i].title.empty()&&!(mobile_&&i==0));}
  Refresh(false);
}
bool XNavHorizon::Layout() {
  const auto size=GetClientSize();const auto *top=wxGetTopLevelParent(this);
  const int viewport=ToDIP(top?top->GetClientSize().x:size.x);
  const int viewport_height=ToDIP(top?top->GetClientSize().y:size.y);
  const auto layout=prototype::Desktop(viewport,viewport_height);
  mobile_=viewport<=760;narrow_=viewport<=1100;
  short_=viewport>760 && viewport_height<=600;
  const bool compact=viewport>760 && viewport_height<=740;
  inset_=layout.timeline_padding_x;
  const int top_padding=layout.timeline_padding_top;
  const int margin=layout.timeline_events_margin_top;
  gap_=mobile_?5:compact?3:4;
  title_size_=layout.timeline_event_title_size;
  detail_size_=layout.timeline_event_small_size;
  const int heading_height=FromDIP(22);
  heading_={FromDIP(inset_),FromDIP(top_padding+1),size.x-FromDIP(2*inset_),heading_height};
  const int link_width=int(std::ceil(passage_->NaturalWidth()));
  passage_->SetSize(heading_.GetRight()+1-link_width,heading_.y,link_width,heading_height);
  event_y_=heading_.y+heading_height+FromDIP(margin);
  // CSS grid items stretch to the measured row bounds; their hit rectangles
  // are not limited to the three painted text line boxes. These trailing
  // insets come from the immutable 132/112/98 px desktop layouts.
  const int event_bottom_inset=FromDIP(short_?11:20);
  const int event_height=std::max(0,size.y-event_y_-event_bottom_inset);
  const double ratios[5]={0,.8,1.92,3.04,4.04};
  for(size_t i=0;i<events_.size();++i) {
    const double left=mobile_ ? double(i?i-1:0)/3 : ratios[i]/4.04;
    const double right=mobile_ ? double(i?i:0)/3 : ratios[i+1]/4.04;
    const int x=heading_.x+int(std::lround(left*heading_.width));
    events_[i]->SetSize(x,event_y_,int(std::lround(right*heading_.width))-int(std::lround(left*heading_.width)),event_height);
    events_[i]->Geometry(i==(mobile_?1:0),narrow_,title_size_,detail_size_,gap_);
    events_[i]->Show(!view_.items[i].title.empty()&&!(mobile_&&i==0));
  }
  return true;
}
HorizonGeometry XNavHorizon::Geometry() const {
  HorizonGeometry g;g.horizon=GetScreenRect();g.heading={ClientToScreen(heading_.GetTopLeft()),heading_.GetSize()};g.full_passage=passage_->GetScreenRect();
  for(size_t i=0;i<events_.size();++i)g.events[i]=events_[i]->IsShown()?events_[i]->GetScreenRect():wxRect{};
  return g;
}
void XNavHorizon::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);const auto c=Theme(mode_);dc.SetBackground(wxBrush(Colour(c.background)));dc.Clear();
  std::unique_ptr<wxGraphicsContext> g(wxGraphicsContext::Create(dc));if(!g)return;
  g->EnableOffset(false);
  g->SetPen(wxPen(Colour(c.border),FromDIP(1)));g->StrokeLine(0,.5,GetClientSize().x,.5);
  const int eyebrow=short_||mobile_?8:9;const double label_y=heading_.y+(heading_.height-FromDIP(LineHeight(eyebrow)))/2.;
  const auto svg=wxString::Format("<svg xmlns='http://www.w3.org/2000/svg' width='24' height='24' viewBox='0 0 24 24'><path d='%s' fill='none' stroke='#%06x' stroke-width='1.65' stroke-linecap='round' stroke-linejoin='round'/></svg>",PrototypeIconPath(XNavIcon::Spark),c.accent);
  const auto bytes=svg.ToUTF8();const int icon=FromDIP(13);
  dc.DrawBitmap(wxBitmapBundle::FromSVG(bytes.data(),{icon,icon}).GetBitmap({icon,icon}),heading_.x,heading_.y+(heading_.height-icon)/2,true);
  double x=heading_.x+FromDIP(19);
  x+=DrawText(*this,*g,"YOUR HORIZON",x,label_y,eyebrow,650,Colour(c.accent),0,eyebrow*.13);
  x+=FromDIP(mobile_?6:12);
  const int advisory_size=mobile_?8:9;
  const double advisory_y=heading_.y+(heading_.height-FromDIP(LineHeight(advisory_size)))/2.;
  g->SetPen(wxPen(Colour(c.border),FromDIP(1)));g->StrokeLine(x+.5,advisory_y,x+.5,advisory_y+FromDIP(LineHeight(advisory_size)));
  x+=FromDIP(mobile_?9:13);
  DrawText(*this,*g,wxString::FromUTF8(view_.advisory_label),x,advisory_y,advisory_size,400,Colour(c.muted),std::max(1.,passage_->GetPosition().x-x-FromDIP(12)));
  for(size_t i=0;i<events_.size();++i) {
    const auto *button=events_[i];if(!button->IsShown())continue;
    const auto rect=button->GetRect();const bool first=i==(mobile_?1:0);
    g->SetPen(wxPen(Colour(c.border),FromDIP(1)));
    g->StrokeLine(rect.x,event_y_-FromDIP(12)+.5,rect.GetRight()+1-(i==3?FromDIP(12):0),event_y_-FromDIP(12)+.5);
    if(!first)g->StrokeLine(rect.x+.5,rect.y,rect.x+.5,rect.GetBottom()+1);
    const int dot_x=rect.x+(first?0:FromDIP(17)),dot_y=event_y_-FromDIP(16);
    if(view_.items[i].marker==application::HorizonMarker::Now) {
      g->SetPen(*wxTRANSPARENT_PEN);g->SetBrush(wxBrush(wxColour(182,239,206,18)));
      g->DrawEllipse(dot_x-FromDIP(4),dot_y-FromDIP(4),FromDIP(17),FromDIP(17));
    }
    const auto color=Colour(MarkerInk(view_.items[i],c));
    g->SetPen(wxPen(color,FromDIP(2)));g->SetBrush(wxBrush(view_.items[i].marker==application::HorizonMarker::Now?color:Colour(c.background)));
    g->DrawEllipse(dot_x+FromDIP(1),dot_y+FromDIP(1),FromDIP(7),FromDIP(7));
  }
  const auto focus=[&](HorizonButton *b){if(b->KeyboardFocus()){const auto r=b->GetRect();g->SetPen(wxPen(Colour(c.accent),FromDIP(2)));g->SetBrush(*wxTRANSPARENT_BRUSH);g->DrawRectangle(r.x-FromDIP(4),r.y-FromDIP(4),r.width+FromDIP(8),r.height+FromDIP(8));}};
  focus(passage_);for(auto *button:events_)focus(button);
}
} // namespace opennav::ui

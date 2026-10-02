#include "ui/SearchDrawer.h"
#include "ui/PrototypeIcons.h"
#include <map>
#include <memory>
#include <wx/bmpbndl.h>
#include <wx/dcbuffer.h>
#include <wx/graphics.h>
#ifdef __WXGTK__
#include <gtk/gtk.h>
#endif

namespace opennav::ui {
namespace {
constexpr std::size_t maximum_results = 100;
wxString Name(const std::string &name, bool route) {
  return name.empty() ? wxString(route ? "Unnamed route" : "Unnamed waypoint")
                      : wxString::FromUTF8(name);
}
// Reuse the native XNav button's keyboard, capture, deferred dispatch and pan
// behavior; only its paint follows prototype .list-card rather than .btn.
class SearchRow final : public XNavButton {
public:
  SearchRow(wxWindow *parent, const SearchMatch &match, LightMode light)
      : XNavButton(parent, wxID_ANY, match.name, match.name + ": " + match.detail),
        detail_(match.detail), icon_(match.route ? XNavIcon::Route : XNavIcon::Pin), light_(light) {
    SetMinSize(FromDIP(wxSize(80, 71)));
    SetLightMode(light);
    Bind(wxEVT_PAINT, [this](wxPaintEvent &) {
      wxAutoBufferedPaintDC dc(this);
      XNavPainter p(*this, dc, light_);
      dc.SetBackground(wxBrush(Colour(p.c.background))); dc.Clear();
      const int width = ToDIP(GetClientSize().x);
      if (IsHovered()) {
        dc.SetPen(*wxTRANSPARENT_PEN); dc.SetBrush(wxBrush(Colour(p.c.surface)));
        dc.DrawRectangle(GetClientRect());
      }
      for (const auto item : {std::pair{icon_, 0}, std::pair{XNavIcon::Chevron, width-22}}) {
        const auto svg = wxString::Format(
          "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 24 24\"><path d=\"%s\" fill=\"none\" stroke=\"#%06x\" stroke-width=\"1.65\" stroke-linecap=\"round\" stroke-linejoin=\"round\"/></svg>",
          wxString::FromUTF8(PrototypeIconPath(item.first)), p.c.accent);
        const auto bitmap = wxBitmapBundle::FromSVG(svg.utf8_str(), FromDIP(wxSize(22,22))).GetBitmap(FromDIP(wxSize(22,22)));
        if (bitmap.IsOk()) dc.DrawBitmap(bitmap, FromDIP(item.second), FromDIP(24), true);
      }
      p.TextWeight(GetLabel(), 34, 20, 12, p.c.primary, 550, width-68);
      p.Text(detail_, 34, 42, 9, p.c.muted, false, width-68);
      p.Rule(0, 70, width);
      if (HasKeyboardFocus()) {
        dc.SetBrush(*wxTRANSPARENT_BRUSH); dc.SetPen(wxPen(Colour(p.c.accent)));
        dc.DrawRoundedRectangle(1,1,GetClientSize().x-2,GetClientSize().y-2,FromDIP(6));
      }
    });
  }
private:
  wxString detail_;
  XNavIcon icon_;
  LightMode light_;
};
}
SearchMatches FindNavigationObjects(const application::Catalog &catalog, const wxString &query) {
  SearchMatches result;
  result.limited = catalog.truncated;
  const auto needle = query.Lower();
  // Ambiguous GUIDs cannot select a unique native object. Keep route/waypoint
  // identities separate and exclude every duplicate, including its first copy.
  std::map<std::pair<bool,std::string>, std::size_t> identities;
  for (const auto &w : catalog.waypoints) ++identities[{false,w.id}];
  for (const auto &r : catalog.routes) ++identities[{true,r.id}];
  const auto add = [&](const std::string &id, const std::string &name, bool route, const wxString &detail) {
    if (id.empty() || identities[{route,id}] != 1) return;
    const auto title = Name(name,route);
    if (!title.Lower().Contains(needle)) return;
    if (result.rows.size() == maximum_results) { result.limited = true; return; }
    result.rows.push_back({id,route,title,detail});
  };
  for (const auto &w : catalog.waypoints)
    add(w.id,w.name,false,wxString::FromUTF8(w.in_route ? "Saved waypoint · In route" : "Saved waypoint"));
  for (const auto &r : catalog.routes)
    add(r.id,r.name,true,wxString::FromUTF8(r.active ? "Saved route · Active" : "Saved route"));
  return result;
}
bool HasUniqueSearchObject(const application::Catalog &catalog, const SearchMatch &match) {
  if (match.id.empty()) return false;
  std::size_t count = 0;
  if (match.route) { for (const auto &r : catalog.routes) if (r.id == match.id) ++count; }
  else { for (const auto &w : catalog.waypoints) if (w.id == match.id) ++count; }
  return count == 1;
}
XNavSearchDrawer::XNavSearchDrawer(wxWindow &owner, application::NavigationActions actions)
    : XNavDrawer(owner, "Chart search"), actions_(std::move(actions)) {
  SetHeading("FIND YOUR NEXT DESTINATION", "Search the coast", false);
  // Reserve the CSS outline's 2px stroke + 3px offset outside the 44px
  // editor. Keep the field, result and note origins at their prototype positions.
  body_->GetSizer()->GetItem(content_)->SetBorder(FromDIP(17));
  body_->GetSizer()->GetItem(std::size_t{0})->AssignSpacer(wxSize(0,FromDIP(19)));
  input_frame_ = new wxPanel(body_,wxID_ANY);
  input_frame_->SetName("Search field"); input_frame_->SetLabel(wxEmptyString);
  input_frame_->SetMinSize(FromDIP(wxSize(90,54)));
  input_frame_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  input_frame_->Bind(wxEVT_PAINT,[this](wxPaintEvent &){
    wxAutoBufferedPaintDC dc(input_frame_);
    dc.SetBackground(wxBrush(Colour(Theme(light_).background))); dc.Clear();
    std::unique_ptr<wxGraphicsContext> graphics(wxGraphicsContext::Create(dc));
    if (!graphics) return;
    const auto c=Theme(light_); const auto size=input_frame_->GetClientSize();
    const int inset=FromDIP(5);
    graphics->SetBrush(wxBrush(Colour(c.surface)));
    graphics->SetPen(wxPen(Colour(c.border)));
    graphics->DrawRoundedRectangle(inset+.5,inset+.5,size.x-2*inset-1.,size.y-2*inset-1.,FromDIP(8));
    if(query_->HasFocus()) {
      const int stroke=FromDIP(2);
      graphics->SetBrush(*wxTRANSPARENT_BRUSH);
      graphics->SetPen(wxPen(Colour(c.accent),stroke));
      graphics->DrawRoundedRectangle(stroke/2.,stroke/2.,size.x-stroke,size.y-stroke,FromDIP(12));
    }
  });
  query_ = new wxTextCtrl(input_frame_,wxID_ANY,wxEmptyString,wxDefaultPosition,wxDefaultSize,wxBORDER_NONE);
  query_->SetName("Search saved routes and waypoints");
  query_->SetHint(wxString::FromUTF8("Saved route or waypoint…")); query_->SetMaxLength(128);
  query_->SetFont(UiFont(*this,16));
#ifdef __WXGTK__
  // The rounded XNav frame owns the border and focus ring. Suppress only this
  // editor's native GTK border; retain native caret, text selection and input.
  auto *css=gtk_css_provider_new();
  gtk_css_provider_load_from_data(css,"entry { border: none; box-shadow: none; outline: none; padding: 0; min-height: 0; }",-1,nullptr);
  gtk_style_context_add_provider(gtk_widget_get_style_context(query_->GetHandle()),GTK_STYLE_PROVIDER(css),GTK_STYLE_PROVIDER_PRIORITY_APPLICATION+1);
  g_object_unref(css);
#endif
  auto *input_layout = new wxBoxSizer(wxHORIZONTAL);
  input_layout->Add(query_,1,wxALIGN_CENTER_VERTICAL|wxLEFT|wxRIGHT,FromDIP(18));
  input_frame_->SetSizer(input_layout);
  content_->Add(input_frame_,0,wxEXPAND|wxBOTTOM,FromDIP(10));
  results_ = new wxPanel(body_,wxID_ANY); results_->SetLabel(wxEmptyString);
  rows_ = new wxBoxSizer(wxVERTICAL); results_->SetSizer(rows_);
  content_->Add(results_,0,wxEXPAND|wxLEFT|wxRIGHT,FromDIP(5));
  note_ = new wxPanel(body_,wxID_ANY); note_->SetLabel(wxEmptyString);
  note_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  note_->SetMinSize(FromDIP(wxSize(80,126)));
  note_->Bind(wxEVT_PAINT,&XNavSearchDrawer::PaintNote,this);
  content_->AddSpacer(FromDIP(11));
  content_->Add(note_,0,wxEXPAND|wxLEFT|wxRIGHT,FromDIP(5));
  query_->Bind(wxEVT_TEXT,[this](wxCommandEvent &){RefreshResults();});
  query_->Bind(wxEVT_SET_FOCUS,[this](wxFocusEvent &e){input_frame_->Refresh();e.Skip();});
  query_->Bind(wxEVT_KILL_FOCUS,[this](wxFocusEvent &e){input_frame_->Refresh();e.Skip();});
  on_dismiss = [this]{++generation_;};
}
void XNavSearchDrawer::Open(const wxRect &workspace, LightMode mode) {
  catalog_=actions_.catalog ? actions_.catalog() : application::Catalog{};
  Update(mode); query_->ChangeValue(wxEmptyString);
  RefreshResults(); Present(workspace);
  if (IsShown()) { Raise(); query_->SetFocus(); }
}
void XNavSearchDrawer::Update(LightMode mode) {
  if (themed_ && light_==mode) return;
  const bool changed=light_!=mode;
  themed_=true; SetLight(mode);
  const auto c=Theme(mode);
  results_->SetBackgroundColour(Colour(c.background));
  query_->SetBackgroundColour(Colour(c.surface));
  query_->SetForegroundColour(Colour(c.primary));
  if(changed) RefreshResults();
  input_frame_->Refresh(); note_->Refresh();
}
void XNavSearchDrawer::RefreshResults() {
  ++generation_;
  rows_->Clear(true);
  notice_.clear();
  const auto found=actions_.catalog ? FindNavigationObjects(catalog_,query_->GetValue()) : SearchMatches{};
  for (const auto &match:found.rows) {
    auto *row=new SearchRow(results_,match,light_);
    row->Bind(wxEVT_BUTTON,[this,match,generation=generation_](wxCommandEvent &){
      // Defer until native button dispatch returns; all captured values are
      // owned. Destroying this drawer discards its pending CallAfter events.
      CallAfter([this,match,generation]{Select(match,generation);});
    });
    rows_->Add(row,0,wxEXPAND);
  }
  if(!actions_.catalog) notice_="Saved navigation objects are unavailable.";
  else if(found.rows.empty()) notice_=query_->IsEmpty()?"No saved routes or waypoints.":"No matching saved objects. Try another name.";
  if(found.limited) notice_ += (notice_.empty()?"":" ") + wxString("Results limited to the available catalog. Refine the name.");
  body_->Scroll(0,0); results_->Layout(); body_->Layout(); body_->FitInside(); note_->Refresh();
}
void XNavSearchDrawer::Select(SearchMatch match, std::uint64_t generation) {
  if(generation!=generation_ || !IsShownOnScreen()) return;
  catalog_=actions_.catalog ? actions_.catalog() : application::Catalog{};
  if(!actions_.catalog || !HasUniqueSearchObject(catalog_,match)) {
    RefreshResults(); notice_="This object was removed or its identity is ambiguous. Search again."; note_->Refresh(); return;
  }
  if(!match.route && actions_.view_waypoint) {
    const auto result=actions_.view_waypoint(match.id);
    if(!result.ok) {notice_=wxString::FromUTF8(result.message);note_->Refresh();return;}
  }
  // Route selection opens existing details only: view_route changes visibility
  // and persistence in OpenCPN, so it is deliberately not used for searching.
  const auto selected=on_select;
  if(selected) selected(match.id,match.route);
}
void XNavSearchDrawer::PaintNote(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(note_); XNavPainter p(*note_,dc,light_);
  dc.SetBackground(wxBrush(Colour(p.c.background)));dc.Clear();
  const int width=ToDIP(note_->GetClientSize().x);
  int y=0;
  if(!notice_.empty()) {p.Wrapped(notice_,0,y,11,18,width,p.c.secondary,3);y=54;}
  p.Wrapped("Search covers saved OpenCPN routes and waypoints. Chart place names are not indexed.",0,y,11,18,width,p.c.secondary,3);
}
} // namespace opennav::ui

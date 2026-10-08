#include "ui/RouteContextCard.h"
#include <array>
#include <wx/dcbuffer.h>
#include <wx/sizer.h>

namespace opennav::ui {
namespace { wxString W(const std::string &value) { return wxString::FromUTF8(value); } }
XNavRouteContextCard::XNavRouteContextCard(wxWindow &owner, std::string selected_id,
                                          Action action, std::function<void()> dismissed)
    : wxDialog(&owner, wxID_ANY, "SKAGER route context", wxDefaultPosition,
                wxDefaultSize, wxBORDER_NONE | wxTAB_TRAVERSAL),
      selected_id_(std::move(selected_id)), action_(std::move(action)),
      dismissed_(std::move(dismissed)) {
  SetName("SKAGER route context");
  auto *outer = new wxBoxSizer(wxVERTICAL);
  content_ = new wxPanel(this);
  content_->SetMinSize(FromDIP(wxSize(328, 188)));
  content_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  content_->Bind(wxEVT_PAINT, &XNavRouteContextCard::Paint, this);
  close_ = new XNavIconButton(content_, wxID_ANY, XNavIcon::Close,
                              "Close", "Close selected route");
  close_->SetMinSize(FromDIP(wxSize(48, 48)));
  close_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { Dismiss(); });
  content_->Bind(wxEVT_SIZE, [this](wxSizeEvent &event) {
    close_->SetSize(content_->GetClientSize().x - FromDIP(48), 0,
                    FromDIP(48), FromDIP(48));
    event.Skip();
  });
  outer->Add(content_, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, FromDIP(12));
  auto *actions = new wxBoxSizer(wxHORIZONTAL);
  view_button_ = new XNavButton(this, wxID_ANY, "View on chart", "View selected route on chart");
  details_button_ = new XNavButton(this, wxID_ANY, "Details", "Selected route details and actions");
  for (auto *button : {view_button_, details_button_}) button->SetMinSize(FromDIP(wxSize(156, 52)));
  actions->Add(view_button_, 1, wxEXPAND | wxRIGHT, FromDIP(8));
  actions->Add(details_button_, 1, wxEXPAND);
  outer->Add(actions, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, FromDIP(12));
  navigate_button_ = new XNavButton(this, wxID_ANY, "Activate route",
                                    "Activate the selected route");
  navigate_button_->SetMinSize(FromDIP(wxSize(320, 52)));
  navigate_button_->SetRole(ButtonRole::Primary);
  navigate_button_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
    // Dispatch only the state rendered on the button the human pressed.
    const auto action = view_.active ? RouteContextAction::Stop : RouteContextAction::Activate;
    if (closing_ || !view_.available ||
        (action == RouteContextAction::Activate ? !view_.can_activate : !view_.can_stop))
      return;
    const auto callback = action_;
    const auto id = selected_id_;
    auto *parent = GetParent();
    Dismiss();
    parent->CallAfter([callback, action, id] { if (callback) callback(action, id); });
  });
  outer->Add(navigate_button_, 0, wxEXPAND | wxALL, FromDIP(12));
  const std::array<std::pair<XNavButton *, RouteContextAction>, 2> choices{{
      {view_button_, RouteContextAction::ViewOnChart},
      {details_button_, RouteContextAction::Details}}};
  for (const auto &entry : choices) {
    entry.first->Bind(wxEVT_BUTTON, [this, action = entry.second](wxCommandEvent &) {
      if (closing_ || !view_.available ||
          (action == RouteContextAction::ViewOnChart && !view_.can_view)) return;
      const auto callback = action_;
      const auto id = selected_id_;
      auto *parent = GetParent();
      Dismiss();
      parent->CallAfter([callback, action, id] { if (callback) callback(action, id); });
    });
  }
  SetSizerAndFit(outer);
  Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent &) { Dismiss(); });
  wxEvtHandler::AddFilter(this);
  filter_added_ = true;
  UpdateRoute({}, LightMode::Day);
}
XNavRouteContextCard::~XNavRouteContextCard() {
  if (filter_added_) wxEvtHandler::RemoveFilter(this);
}
void XNavRouteContextCard::Dismiss() {
  if (closing_) return;
  closing_ = true;
  if (dismissed_) dismissed_();
  if (filter_added_) { wxEvtHandler::RemoveFilter(this); filter_added_ = false; }
  Hide();
  Destroy();
}
int XNavRouteContextCard::FilterEvent(wxEvent &event) {
  if (closing_ || !IsShownOnScreen()) return Event_Skip;
  if (event.GetEventType() == wxEVT_CHAR_HOOK) {
    const auto *key = dynamic_cast<wxKeyEvent *>(&event);
    if (key && key->GetKeyCode() == WXK_ESCAPE) { Dismiss(); return Event_Processed; }
  }
  if (event.GetEventType() == wxEVT_LEFT_DOWN || event.GetEventType() == wxEVT_RIGHT_DOWN) {
    auto *window = dynamic_cast<wxWindow *>(event.GetEventObject());
    if (window && window != this && !IsDescendant(window)) Dismiss();
  }
  return Event_Skip;
}
void XNavRouteContextCard::UpdateRoute(const std::optional<application::Route> &route,
                                      LightMode mode) {
  view_ = application::PresentRouteContext(selected_id_, route);
  light_ = mode;
  const auto background = Colour(Theme(mode).elevated);
  SetBackgroundColour(background);
  content_->SetBackgroundColour(background);
  close_->SetLightMode(mode);
  view_button_->SetLightMode(mode);
  details_button_->SetLightMode(mode);
  view_button_->Enable(view_.available && view_.can_view && bool(action_));
  details_button_->Enable(view_.available && bool(action_));
  const bool stop = view_.available && view_.active;
  navigate_button_->SetLabel(stop ? "Stop navigation" : "Activate route");
  navigate_button_->SetToolTip(stop ? "Stop navigating the selected route"
                                    : "Activate the selected route");
  navigate_button_->SetRole(stop ? ButtonRole::Critical : ButtonRole::Primary);
  navigate_button_->SetLightMode(mode);
  navigate_button_->Enable(bool(action_) && (stop ? view_.can_stop : view_.can_activate));
  content_->Refresh(false);
}
bool XNavRouteContextCard::Place(const wxRect &chart) {
  auto area = chart;
  area.Deflate(FromDIP(12));
  const auto size = GetBestSize();
  if (size.x > area.width || size.y > area.height) return false;
  const wxRect position(area.GetRight() - size.x + 1, area.GetTop(), size.x, size.y);
  if (GetScreenRect() != position) SetSize(position);
  return true;
}
void XNavRouteContextCard::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(content_);
  dc.SetBackground(wxBrush(Colour(Theme(light_).elevated)));
  dc.Clear();
  XNavPainter paint(*content_, dc, light_);
  const int width = content_->ToDIP(content_->GetClientSize().x);
  paint.Text(W(view_.name), 0, 8, 20, paint.c.primary, false, width - 56);
  paint.Text(W(view_.status), 0, 40, 11, view_.active ? paint.c.accent : paint.c.secondary,
             false, width);
  paint.Rule(0, 66, width);
  if (!view_.available) {
    paint.Wrapped("This route is no longer available. Select another route on the chart.",
                   0, 86, 13, 21, width, paint.c.secondary, 3);
    return;
  }
  paint.Text("FROM", 0, 82, 10, paint.c.muted);
  paint.Text(W(view_.departure.empty() ? "No departure" : view_.departure), 60, 78,
             14, paint.c.primary, false, width - 60);
  paint.Text("TO", 0, 114, 10, paint.c.muted);
  paint.Text(W(view_.destination.empty() ? "No destination" : view_.destination), 60, 110,
             14, paint.c.primary, false, width - 60);
  paint.Text(wxString::Format("%u waypoints", static_cast<unsigned>(view_.points)),
             0, 154, 12, paint.c.secondary);
}
} // namespace opennav::ui

#include "ui/ContextCard.h"
#include "ui/Sheet.h"
#include "vessel/AisSelection.h"
#include <algorithm>
#include <cmath>
#include <wx/dcbuffer.h>
#include <wx/sizer.h>
#include <wx/weakref.h>

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
wxString Name(const application::Waypoint &p) { return W(p.name.empty() ? "Unnamed waypoint" : p.name); }
bool Current(const vessel::Sample &sample, vessel::Time now) {
  const auto a = vessel::Assess(sample, now);
  return a.value && (a.quality == vessel::Quality::Live ||
      a.quality == vessel::Quality::Aging || a.quality == vessel::Quality::Estimated);
}
wxString Number(const vessel::Sample &sample, vessel::Time now, int decimals,
                const wxString &unit) {
  const auto a = vessel::Assess(sample, now);
  const bool good = a.quality == vessel::Quality::Live ||
                    a.quality == vessel::Quality::Aging ||
                    a.quality == vessel::Quality::Estimated;
  return good && a.value ? wxString::Format("%.*f", decimals, *a.value) + unit
                         : wxString::FromUTF8("—");
}
} // namespace
std::optional<application::CommandResult> WaypointSheet(
    wxWindow &parent, LightMode mode, ContextAction action,
    const application::Waypoint &point,
    const application::NavigationActions &navigation) {
  if (action == ContextAction::GoTo && navigation.go_to_waypoint) {
    if (ConfirmSheet(parent, mode, "Go to " + Name(point),
        "Start a passage to this waypoint? Check the chart and passage before starting.", "START"))
      return navigation.go_to_waypoint(point);
  } else if (action == ContextAction::Edit && navigation.edit_waypoint) {
    auto fields = EditSheet(parent, mode, "Edit waypoint",
        "Active-route, anchor-watch and protected points are read-only here.",
        {{"Name", W(point.name), 128}, {"Description", W(point.description), 2048}});
    if (fields) return navigation.edit_waypoint(point, (*fields)[0], (*fields)[1]);
  } else if (action == ContextAction::Remove && navigation.delete_waypoint) {
    if (ConfirmSheet(parent, mode, "Delete waypoint",
        Name(point) + " will be removed from the shared OpenCPN database. Review before removing it.",
        "Delete waypoint"))
      return navigation.delete_waypoint(point);
  }
  return {};
}
XNavContextCard::XNavContextCard(wxWindow &owner, ContextKind kind, Action action)
    : wxDialog(&owner, wxID_ANY, kind == ContextKind::Ais ? "OpenNav AIS context" : kind == ContextKind::Waypoint ? "OpenNav waypoint context" : "OpenNav chart context",
               wxDefaultPosition, wxDefaultSize, wxBORDER_NONE | wxTAB_TRAVERSAL),
      kind_(kind), action_(std::move(action)) {
  SetName(kind == ContextKind::Ais ? "OpenNav AIS context" : kind == ContextKind::Waypoint ? "OpenNav waypoint context" : "OpenNav chart context");
  auto *outer = new wxBoxSizer(wxVERTICAL);
  content_ = new wxPanel(this);
  content_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  content_->SetMinSize(FromDIP(wxSize(312, kind == ContextKind::Ais ? 236 : kind == ContextKind::Waypoint ? 180 : 116)));
  content_->Bind(wxEVT_PAINT, &XNavContextCard::Paint, this);
  close_ = new XNavIconButton(content_, wxID_ANY, XNavIcon::Close, "Close", "Close selected object");
  close_->SetMinSize(FromDIP(wxSize(48, 48)));
  close_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { Dismiss(); });
  content_->Bind(wxEVT_SIZE, [this](wxSizeEvent &event) {
    close_->SetSize(content_->GetClientSize().x - FromDIP(48), 0, FromDIP(48), FromDIP(48));
    event.Skip();
  });
  outer->Add(content_, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, FromDIP(12));
  auto *grid = new wxGridSizer(2, FromDIP(8), FromDIP(8));
  const auto choices = kind == ContextKind::Ais
      ? std::vector<std::pair<wxString, ContextAction>>{{"Show on chart", ContextAction::ShowAis}, {"Details", ContextAction::Details}}
      : kind == ContextKind::Waypoint ? std::vector<std::pair<wxString, ContextAction>>{{"GO TO", ContextAction::GoTo}, {"Details", ContextAction::Details},
          {"Edit waypoint", ContextAction::Edit}, {"Remove", ContextAction::Remove}}
      : std::vector<std::pair<wxString, ContextAction>>{{"Go to", ContextAction::GoTo}, {"Waypoint", ContextAction::CreateWaypoint},
          {"Measure", ContextAction::Measure}, {"Info", ContextAction::Info}};
  for (const auto &choice : choices) {
    auto *button = new XNavButton(this, wxID_ANY, choice.first, choice.first);
    button->SetMinSize(FromDIP(wxSize(144, 52)));
    if (choice.second == ContextAction::GoTo || choice.second == ContextAction::ShowAis)
      button->SetRole(ButtonRole::Primary);
    button->Bind(wxEVT_BUTTON, [this, kind = choice.second](wxCommandEvent &) {
      if (closing_) return;
      const auto action = action_;
      const auto point = point_.waypoint;
      auto *parent = GetParent();
      Dismiss();
      // Fully release the context window before opening an edit/confirmation
      // sheet; modal dialogs must not inherit a transient mouse/focus event.
      parent->CallAfter([action, kind, point] { if (action) action(kind, point); });
    });
    actions_.push_back({button, choice.second});
    grid->Add(button, 1, wxEXPAND);
  }
  outer->Add(grid, 0, wxEXPAND | wxALL, FromDIP(12));
  SetSizerAndFit(outer);
  Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent &) { Dismiss(); });
  wxEvtHandler::AddFilter(this);
  filter_added_ = true;
  ThemeControls(mode_);
}
XNavContextCard::~XNavContextCard() {
  if (filter_added_) wxEvtHandler::RemoveFilter(this);
}
void XNavContextCard::Dismiss() {
  if (closing_) return;
  closing_ = true;
  if (filter_added_) { wxEvtHandler::RemoveFilter(this); filter_added_ = false; }
  Hide();
  Destroy();
}
int XNavContextCard::FilterEvent(wxEvent &event) {
  if (closing_ || !IsShownOnScreen()) return Event_Skip;
  if (event.GetEventType() == wxEVT_CHAR_HOOK) {
    const auto *key = dynamic_cast<wxKeyEvent *>(&event);
    if (key && key->GetKeyCode() == WXK_ESCAPE) { Dismiss(); return Event_Processed; }
  }
  if (event.GetEventType() == wxEVT_LEFT_DOWN || event.GetEventType() == wxEVT_RIGHT_DOWN) {
    auto *window = dynamic_cast<wxWindow *>(event.GetEventObject());
    if (window && window != this && !IsDescendant(window)) Dismiss();
  }
  return Event_Skip; // Original outside click is delivered once, never replayed.
}
bool XNavContextCard::Place(const wxRect &chart) {
  auto area = chart;
  area.Deflate(FromDIP(12));
  const auto size = GetBestSize();
  if (size.x > area.width || size.y > area.height) return false;
  const wxRect desired(area.GetRight() - size.x + 1, area.GetTop(), size.x, size.y);
  if (GetScreenRect() != desired) SetSize(desired);
  return true;
}
void XNavContextCard::ThemeControls(LightMode mode) {
  mode_ = mode;
  SetBackgroundColour(Colour(Theme(mode).elevated));
  content_->SetBackgroundColour(Colour(Theme(mode).elevated));
  close_->SetLightMode(mode);
  for (auto &entry : actions_) entry.first->SetLightMode(mode);
  content_->Refresh(false);
}
void XNavContextCard::UpdateAis(std::optional<vessel::AisTarget> target,
                               vessel::Time now, bool live, LightMode mode) {
  target_ = std::move(target); now_ = now;
  const bool usable = live && target_ && vessel::AisSelection::CurrentPosition(*target_, now);
  for (auto &entry : actions_)
    entry.first->Enable(entry.second == ContextAction::Details || usable);
  ThemeControls(mode);
}
void XNavContextCard::UpdateWaypoint(application::WaypointContext point,
                                    vessel::Time now, bool live, LightMode mode) {
  point_ = std::move(point); now_ = now;
  for (auto &entry : actions_) {
    bool enabled = point_.waypoint.has_value();
    if (entry.second == ContextAction::GoTo)
      enabled = enabled && live && point_.waypoint->editable && Current(point_.range_nm, now) &&
                Current(point_.bearing_true_deg, now);
    else if (entry.second == ContextAction::Edit)
      enabled = enabled && live && point_.waypoint->editable;
    else if (entry.second == ContextAction::Remove)
      enabled = enabled && live && point_.waypoint->removable;
    entry.first->Enable(enabled);
  }
  ThemeControls(mode);
}
void XNavContextCard::UpdateChartPosition(application::Coordinate position, LightMode mode,
                                          bool live, bool position_current) {
  position_ = position;
  const bool valid = std::isfinite(position.latitude_deg) && std::isfinite(position.longitude_deg) &&
      std::abs(position.latitude_deg) <= 90 && std::abs(position.longitude_deg) <= 180;
  for (auto &entry : actions_) {
    bool enabled = valid;
    if (entry.second == ContextAction::GoTo) enabled = enabled && live && position_current;
    else if (entry.second == ContextAction::CreateWaypoint) enabled = enabled && live;
    entry.first->Enable(enabled);
  }
  ThemeControls(mode);
}
void XNavContextCard::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(content_);
  dc.SetBackground(wxBrush(Colour(Theme(mode_).elevated))); dc.Clear();
  XNavPainter p(*content_, dc, mode_);
  const int width = content_->ToDIP(content_->GetClientSize().x);
  auto metric = [&](const wxString &title, const vessel::Sample &sample,
                     int x, int y, int decimals, const wxString &unit) {
    p.Text(title, x, y, 11, p.c.secondary, false, width / 2 - 12);
    p.Text(Number(sample, now_, decimals, unit), x, y + 18, 25, p.c.primary, false, width / 2 - 12);
  };
  if (kind_ == ContextKind::ChartPosition) {
    p.Text("Chart position", 0, 8, 20, p.c.primary, false, width - 56);
    const bool valid = std::isfinite(position_.latitude_deg) && std::isfinite(position_.longitude_deg) &&
        std::abs(position_.latitude_deg) <= 90 && std::abs(position_.longitude_deg) <= 180;
    p.Text(valid ? wxString::Format("%.5f°", position_.latitude_deg) : wxString("Position unavailable"),
           0, 54, 22, p.c.primary, false, width);
    if (valid) p.Text(wxString::Format("%.5f°", position_.longitude_deg), 0, 84, 22, p.c.primary);
  } else if (kind_ == ContextKind::Ais) {
    p.Text(target_ ? W(target_->name.empty() ? std::to_string(target_->mmsi) : target_->name) : "AIS target unavailable",
           0, 8, 20, p.c.primary, false, width - 56);
    p.Text(target_ ? wxString::Format("MMSI %d", target_->mmsi) : "Target lost or removed", 0, 40, 11, p.c.secondary);
    const bool current = target_ && vessel::AisSelection::CurrentPosition(*target_, now_);
    const auto status = target_ ? (current ? W(target_->status) : wxString("Position stale or unavailable"))
                                : wxString("No current target");
    p.Text(status, 0, 61, 11, target_ && target_->upstream_alarm ? p.c.alarm : (current ? p.c.secondary : p.c.attention), false, width);
    p.Rule(0, 82, width);
    const auto empty = vessel::Sample{};
    metric("SPEED", target_ ? target_->sog_kn : empty, 0, 92, 1, " kn");
    metric("COURSE", target_ ? target_->cog_deg : empty, width / 2, 92, 0, "°T");
    metric("CPA", target_ ? target_->cpa_nm : empty, 0, 150, 2, " NM");
    metric("TCPA", target_ ? target_->tcpa_minutes : empty, width / 2, 150, 0, " min");
    p.Text("Range " + Number(target_ ? target_->range_nm : empty, now_, 1, " NM"), 0, 211, 12, p.c.secondary);
    p.Text("Bearing " + Number(target_ ? target_->bearing_true_deg : empty, now_, 0, "°T"), width / 2, 211, 12, p.c.secondary);
  } else {
    p.Text(point_.waypoint ? Name(*point_.waypoint) : "Waypoint unavailable", 0, 8, 20, p.c.primary, false, width - 56);
    const bool position_valid = point_.waypoint && std::isfinite(point_.waypoint->latitude_deg) &&
        std::isfinite(point_.waypoint->longitude_deg) && std::abs(point_.waypoint->latitude_deg) <= 90 &&
        std::abs(point_.waypoint->longitude_deg) <= 180;
    p.Text(position_valid ? wxString::Format("%.5f°  %.5f°", point_.waypoint->latitude_deg, point_.waypoint->longitude_deg)
                          : wxString("Position unavailable"), 0, 48, 12, p.c.secondary);
    metric("RANGE", point_.range_nm, 0, 87, 1, " NM");
    metric("BEARING", point_.bearing_true_deg, width / 2, 87, 0, "°T");
    p.Text(W(point_.reason), 0, 151, 12, p.c.secondary, false, width);
  }
}
} // namespace opennav::ui

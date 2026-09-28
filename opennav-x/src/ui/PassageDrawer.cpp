#include "ui/PassageDrawer.h"
#include "ui/ContextCard.h"
#include "ui/PrototypeGeometry.h"
#include "ui/Sheet.h"
#include <cmath>
#include <wx/datetime.h>
#include <wx/dcbuffer.h>

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
wxString Dash() { return wxString::FromUTF8("—"); }
wxString Number(std::optional<double> n, int places) {
  return n ? wxString::Format("%.*f", places, *n) : Dash();
}
wxString Arrival(std::optional<double> seconds, const wxDateTime &wall_now) {
  if (!seconds || !std::isfinite(*seconds) || *seconds < 0 ||
      *seconds > 7 * 24 * 3600)
    return Dash();
  if (!wall_now.IsValid())
    return Dash();
  return (wall_now + wxTimeSpan::Seconds(static_cast<long>(*seconds)))
      .Format("%H:%M");
}
wxString Duration(std::optional<double> seconds) {
  if (!seconds || !std::isfinite(*seconds) || *seconds < 0 ||
      *seconds > 7 * 24 * 3600)
    return Dash();
  const auto minutes = static_cast<unsigned>(*seconds / 60);
  return wxString::Format("%uh %um", minutes / 60, minutes % 60);
}
} // namespace
XNavPassageDrawer::XNavPassageDrawer(wxWindow &owner,
                                     application::NavigationActions actions)
    : XNavDrawer(owner, "OpenNav passage"), actions_(std::move(actions)) {
  Build();
}
void XNavPassageDrawer::Build() {
  ClearBody();
  buttons_.clear();
  edits_.clear();
  rows_ = view_.points.size();
  summary_ = new wxPanel(body_, wxID_ANY);
  summary_->SetName("Passage progress");
  summary_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  // The prototype's Windows Segoe rows occupy 77px. All values come from the
  // current owned presentation; empty states do not manufacture route points.
  summary_->SetMinSize(FromDIP(
      wxSize(300, prototype::passage_points_y +
                      static_cast<int>(rows_) * prototype::passage_point_row +
                      126 + (rows_ ? 0 : 38))));
  summary_->Bind(wxEVT_PAINT, &XNavPassageDrawer::Paint, this);
  EnableScrollGesture(*summary_);
  for (const auto &point : view_.points) {
    auto *edit = new XNavButton(summary_, wxID_ANY, "Edit " + W(point.name),
                                "Edit " + W(point.name));
    edit->SetIcon(XNavIcon::Edit);
    edit->SetIconOnly();
    edit->SetIconSize(14);
    edit->SetRole(ButtonRole::Quiet);
    edit->SetLightMode(light_);
    edit->Disable();
    edit->SetHint(
        "Active-route and protected waypoints cannot be edited here.");
    edit->Bind(wxEVT_BUTTON, [this, id = point.id](wxCommandEvent &) {
      CallAfter([this, id] {
        if (!changes_allowed_ || !actions_.waypoint_context)
          return;
        const auto context =
            actions_.waypoint_context(id, vessel::Clock::now());
        if (!context.waypoint || !context.waypoint->editable)
          return;
        const auto result = WaypointSheet(*this, light_, ContextAction::Edit,
                                          *context.waypoint, actions_);
        if (result && !result->ok)
          ConfirmSheet(*this, light_, "Unable to edit waypoint",
                       W(result->message), "Back");
      });
    });
    edits_.push_back({point.id, edit});
  }
  summary_->Bind(wxEVT_SIZE, [this](wxSizeEvent &event) {
    for (std::size_t i = 0; i < edits_.size(); ++i)
      edits_[i].second->SetSize(
          summary_->GetClientSize().x - FromDIP(28),
          FromDIP(prototype::passage_points_y +
                  static_cast<int>(i) * prototype::passage_point_row),
          FromDIP(28), FromDIP(28));
    event.Skip();
  });
  content_->Add(summary_, 0, wxEXPAND);
  auto *row = new wxBoxSizer(wxHORIZONTAL);
  const auto button = [this](const wxString &label,
                             std::function<void()> action) {
    auto *b = new XNavButton(body_, wxID_ANY, label, label);
    b->SetLightMode(light_);
    b->SetMinSize(FromDIP(wxSize(120, 48)));
    b->Bind(wxEVT_BUTTON,
            [this, action](wxCommandEvent &) { CallAfter(action); });
    buttons_.push_back(b);
    return b;
  };
  end_ = button("End navigation", [this] { EndNavigation(); });
  // Reversing an active route is prohibited by the existing OpenCPN boundary.
  reverse_ = button("Reverse route", [] {});
  reverse_->Disable();
  reverse_->SetHint(
      "End navigation, then select the saved route to reverse it.");
  row->Add(end_, 1, wxEXPAND);
  row->AddSpacer(FromDIP(8));
  row->Add(reverse_, 1, wxEXPAND);
  content_->Add(row, 0, wxEXPAND | wxBOTTOM, FromDIP(12));
  plot_ = button("Plot a new passage", [this] {
    if (changes_allowed_ && on_plot)
      on_plot();
  });
  plot_->SetIcon(XNavIcon::Plus);
  plot_->SetInlineIcon();
  content_->Add(plot_, 0, wxEXPAND | wxBOTTOM, FromDIP(12));
  auto *library = button("Passage library", [this] {
    if (on_library)
      on_library();
  });
  library->SetRole(ButtonRole::Quiet);
  content_->Add(library, 0, wxEXPAND | wxBOTTOM, FromDIP(12));
  body_->FitInside();
  body_->Layout();
}
void XNavPassageDrawer::Update(const vessel::VesselState &state,
                               const smartnav::NavigationAdvice &advice,
                               const smartnav::EnergyPrediction &energy,
                               vessel::Time now, LightMode mode,
                               wxDateTime wall_now) {
  wall_now_ = wall_now;
  const auto previous = view_;
  view_ = application::PresentPassage(state, advice, energy, now);
  changes_allowed_ = !state.simulated && !state.replayed;
  selected_.reset();
  if (view_.active && changes_allowed_ && actions_.route) {
    auto selected = actions_.route(view_.route_id);
    if (selected && selected->active && selected->id == view_.route_id)
      selected_ = std::move(selected);
  }
  const bool theme_changed = mode != light_;
  if (theme_changed)
    SetLight(mode);
  bool points_changed = previous.points.size() != view_.points.size();
  if (!points_changed)
    for (std::size_t i = 0; i < view_.points.size(); ++i)
      points_changed |= previous.points[i].id != view_.points[i].id ||
                        previous.points[i].name != view_.points[i].name;
  if (points_changed || previous.route_id != view_.route_id)
    Build();
  summary_->SetBackgroundColour(Colour(Theme(mode).background));
  for (auto *button : buttons_)
    button->SetLightMode(mode);
  for (auto &entry : edits_) {
    bool editable = false;
    if (selected_)
      for (const auto &p : selected_->points)
        if (p.id == entry.first)
          editable = p.editable;
    entry.second->Enable(changes_allowed_ && editable &&
                         static_cast<bool>(actions_.waypoint_context) &&
                         static_cast<bool>(actions_.edit_waypoint));
    entry.second->SetLightMode(mode);
  }
  end_->Enable(selected_.has_value() && static_cast<bool>(actions_.deactivate));
  plot_->Enable(changes_allowed_ && static_cast<bool>(on_plot));
  SetHeading(
      "PASSAGE PLAN",
      view_.destination.empty()
          ? wxString(view_.active ? "Unnamed destination" : "No active passage")
          : W(view_.destination),
      false);
  summary_->Refresh(false);
}
void XNavPassageDrawer::EndNavigation() {
  if (!selected_ || !changes_allowed_ || !actions_.deactivate)
    return;
  // Freeze the reviewed identity/revision before entering a modal event loop.
  // The integration boundary re-resolves it and rejects changed/deleted routes.
  const auto selected = *selected_;
  if (!ConfirmSheet(
          *this, light_, "End navigation",
          "Stop this OpenCPN passage? Configured navigation output connections "
          "retain their normal behavior.",
          "End navigation"))
    return;
  if (!changes_allowed_)
    return;
  const auto result = actions_.deactivate(selected);
  if (!result.ok)
    ConfirmSheet(*this, light_, "Unable to end navigation", W(result.message),
                 "Back");
}
void XNavPassageDrawer::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(summary_);
  XNavPainter p(*summary_, dc, light_);
  dc.SetBackground(wxBrush(Colour(p.c.background)));
  dc.Clear();
  const int width = summary_->ToDIP(summary_->GetClientSize().x),
            split = (width + 16) / 2;
  const int tag = p.Tag(view_.active ? "ACTIVE PASSAGE" : "NO ACTIVE PASSAGE",
                        0, 0, width / 2, false, view_.active);
  if (view_.active)
    p.Tag(wxString::Format("%u waypoints",
                           static_cast<unsigned>(view_.waypoint_count)),
          tag + 8, 0, width - tag - 8);
  p.Stat("DISTANCE REMAINING", Number(view_.distance_nm, 1), "nm", 0, 46,
         split - 16);
  p.Stat("ESTIMATED ARRIVAL", Arrival(view_.seconds, wall_now_), "", split, 46,
         split - 16);
  p.Stat("TIME TO DESTINATION", Duration(view_.seconds), "", 0, 112,
         split - 16);
  p.Stat("BATTERY ON ARRIVAL", Number(view_.arrival_soc, 0), "%", split, 112,
         split - 16);
  p.Text("THE WAY AHEAD", 0, 187, 10, p.c.secondary);
  int y = prototype::passage_points_y;
  for (std::size_t i = 0; i < view_.points.size();
       ++i, y += prototype::passage_point_row) {
    const auto &point = view_.points[i];
    dc.SetPen(wxPen(Colour(p.c.border)));
    dc.DrawLine(p.D(11), p.D(y + 23), p.D(11), p.D(y + 77));
    dc.SetPen(wxPen(Colour(i == 0 ? p.c.accent : p.c.border)));
    dc.SetBrush(wxBrush(Colour(p.c.background)));
    dc.DrawCircle(p.D(11), p.D(y + 11), p.D(11));
    p.Text(wxString::Format("%u", static_cast<unsigned>(point.ordinal)), 8,
           y + 5, 9, i == 0 ? p.c.accent : p.c.secondary);
    p.TextWeight(W(point.name.empty() ? "Unnamed waypoint" : point.name), 37,
                 y + 3, 12, p.c.primary, 500, width - 84);
    p.Text(Number(point.distance_nm, 1) + " nm" + W(" · ") +
               Arrival(point.seconds, wall_now_),
           37, y + 29, 9, p.c.muted, false, width - 84);
    wxString turn = "Turn unavailable";
    if (point.turn_deg && point.course_true_deg)
      turn = wxString::Format(
          W("%.0f° %s · course %03.0f°"), std::abs(*point.turn_deg),
          *point.turn_deg < 0 ? "port" : "starboard", *point.course_true_deg);
    else if (i + 1 == view_.points.size())
      turn = "Destination";
    p.Text(turn, 37, y + 49, 9, p.c.accent, false, width - 40);
  }
  if (view_.points.empty()) {
    p.Text(view_.active ? "Route guidance unavailable" : "No passage selected",
           0, y, 12, p.c.secondary, false, width);
    y += 38;
  }
  // Reuse the exact shared callout surface, translating only its origin.
  const auto origin = dc.GetDeviceOrigin();
  dc.SetDeviceOrigin(origin.x, origin.y + p.D(y + 18));
  const auto title = !view_.arrival_soc    ? "Arrival prediction unavailable"
                     : view_.below_reserve ? "Energy reserve at risk"
                                           : "Estimated arrival charge";
  p.Callout(title,
            view_.arrival_soc
                ? Number(view_.arrival_soc, 0) +
                      "% estimated at destination. Conditions may change."
                : W(view_.reason),
            width, 90, view_.below_reserve);
  dc.SetDeviceOrigin(origin.x, origin.y);
}
} // namespace opennav::ui

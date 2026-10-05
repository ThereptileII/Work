#include "ui/AnchorDrawer.h"
#include "ui/Sheet.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <wx/dcbuffer.h>
#include <wx/graphics.h>

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
wxString Number(std::optional<double> n, int places = 0) {
  return n ? wxString::Format("%.*f", places, *n) : wxString::FromUTF8("—");
}
std::optional<double> Reading(const vessel::Assessment &a) {
  return a.quality == vessel::Quality::Live ||
                 a.quality == vessel::Quality::Aging
             ? a.value
             : std::nullopt;
}
} // namespace
XNavAnchorDrawer::XNavAnchorDrawer(wxWindow &owner,
                                   application::NavigationActions actions)
    : XNavDrawer(owner, "OpenNav anchor watch"), actions_(std::move(actions)) {
  SetHeading("AT REST", "Anchor watch", false);
  summary_ = new wxPanel(body_, wxID_ANY);
  summary_->SetName("Anchor movement");
  summary_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  summary_->SetMinSize(FromDIP(wxSize(300, 560)));
  summary_->Bind(wxEVT_PAINT, &XNavAnchorDrawer::Paint, this);
  EnableScrollGesture(*summary_);
  radius_ = new XNavRange(summary_, "Alarm radius", 20, 100, 5, 50);
  radius_->on_change = [this](int) { summary_->Refresh(false); };
  summary_->Bind(wxEVT_SIZE, [this](wxSizeEvent &e) {
    radius_->SetSize(FromDIP(2), FromDIP(346),
                     summary_->GetClientSize().x - FromDIP(2), FromDIP(46));
    e.Skip();
  });
  content_->Add(summary_, 0, wxEXPAND);
  watch_ = new XNavButton(body_, wxID_ANY, "Set anchor & start watch",
                          "Set anchor & start watch");
  watch_->SetMinSize(FromDIP(wxSize(300, 48)));
  watch_->SetDisplayAction(48);
  watch_->Bind(wxEVT_BUTTON,
               [this](wxCommandEvent &) { CallAfter([this] { Command(); }); });
  content_->Add(watch_, 0, wxEXPAND | wxBOTTOM, FromDIP(11));
  note_ = new wxPanel(body_, wxID_ANY);
  note_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  note_->SetMinSize(FromDIP(wxSize(300, 84)));
  note_->Bind(wxEVT_PAINT, [this](wxPaintEvent &) {
    wxAutoBufferedPaintDC dc(note_);
    XNavPainter p(*note_, dc, light_);
    dc.SetBackground(wxBrush(Colour(p.c.background)));
    dc.Clear();
    const int width = note_->ToDIP(note_->GetClientSize().x);
    p.Wrapped(view_.active ? W(view_.reason)
                           : "Set the anchor at your current GPS position. "
                             "OpenCPN monitors this watch.",
              0, 0, 11, 18, width, p.c.muted, 2);
    p.Wrapped(
        view_.active
            ? "Stop the watch before choosing a new radius. Depth is below the "
              "transducer."
            : "Depth is below the transducer. No advanced drag detection.",
        0, 42, 11, 18, width, p.c.muted, 2);
  });
  EnableScrollGesture(*note_);
  content_->Add(note_, 0, wxEXPAND);
}
void XNavAnchorDrawer::Update(const application::AnchorState &watch,
                              const vessel::VesselState &state,
                              vessel::Time now, LightMode light) {
  view_ = application::PresentAnchor(watch, state, now);
  changes_allowed_ = !state.simulated && !state.replayed;
  SetLight(light);
  radius_->SetLight(light);
  watch_->SetLightMode(light);
  // Existing upstream watches may have negative or >100m radii. Keep their
  // exact label/meaning, never coerce the real watch to the prototype slider.
  radius_->Enable(!view_.active && changes_allowed_);
  radius_->Show(!view_.active || (view_.radius_m && *view_.radius_m >= 20 &&
                                  *view_.radius_m <= 100));
  if (view_.active && view_.radius_m)
    radius_->SetValue(static_cast<int>(std::clamp(*view_.radius_m, 20., 100.)));
  watch_->SetLabel(view_.active ? "Stop anchor watch"
                                : "Set anchor & start watch");
  watch_->SetRole(view_.active ? ButtonRole::Normal : ButtonRole::Primary);
  watch_->Enable(changes_allowed_ &&
                 (view_.active
                      ? bool(actions_.clear_anchor)
                      : view_.can_start && bool(actions_.start_anchor)));
  summary_->Refresh(false);
  note_->Refresh(false);
}
void XNavAnchorDrawer::Command() {
  if (!changes_allowed_)
    return;
  application::CommandResult result;
  if (view_.active && actions_.clear_anchor) {
    const auto identity = view_.identity;
    if (!ConfirmSheet(*this, light_, "Stop anchor watch",
                      "Stop this OpenCPN anchor watch? A temporary anchor mark "
                      "will be removed; existing user waypoints are preserved.",
                      "Stop watch"))
      return;
    result = actions_.clear_anchor(identity);
  } else if (view_.can_start && actions_.start_anchor) {
    const int radius = radius_->GetValue();
    if (!ConfirmSheet(*this, light_, "Set anchor watch",
                      wxString::Format(
                          "Create an anchor mark at the current GPS position "
                          "and start an OpenCPN watch with a %d m radius? "
                          "Active route navigation will stop; the route and its waypoints are kept.",
                          radius),
                      "Set anchor"))
      return;
    result = actions_.start_anchor(radius);
  } else
    return;
  if (!result.ok)
    ConfirmSheet(*this, light_, "Anchor watch unchanged", W(result.message),
                 "Back");
}
void XNavAnchorDrawer::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(summary_);
  XNavPainter p(*summary_, dc, light_);
  dc.SetBackground(wxBrush(Colour(p.c.background)));
  dc.Clear();
  const int width = summary_->ToDIP(summary_->GetClientSize().x);
  int tx = p.Tag(view_.alarm    ? "ANCHOR ALARM"
                 : view_.active ? "WATCH ACTIVE"
                                : "WATCH NOT ARMED",
                 0, 0, width, view_.alarm, view_.active && !view_.alarm);
  p.Tag(view_.gps_age_s ? wxString::Format(wxString::FromUTF8("GPS · %.1f s"),
                                           *view_.gps_age_s)
                        : "GPS unavailable",
        tx + 8, 0, width - tx - 8);
  // SVG viewBox 0 0 300 230: preserve its size and aspect ratio. History and
  // vessel marks use integration-owned metres; the decorative sample trail is
  // never drawn. A plot beyond the radius expands its scale instead of
  // clipping.
  {
    std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(dc));
    if (gc) {
      const double dip = double(summary_->FromDIP(1024)) / 1024.;
      gc->Scale(dip, dip);
      gc->Translate((width - 300.) / 2., 46.);
      gc->SetPen(*wxTRANSPARENT_PEN);
      gc->SetBrush(gc->CreateRadialGradientBrush(150, 115, 150, 115, 156,
                                                 wxColour(182, 239, 206, 11),
                                                 wxColour(182, 239, 206, 0)));
      gc->DrawRectangle(-(width - 300.) / 2., 0, width, 230);
      gc->SetBrush(*wxTRANSPARENT_BRUSH);
      gc->SetPen(wxPen(Colour(p.c.border)));
      for (int r : {98, 65})
        gc->DrawEllipse(150 - r, 115 - r, 2 * r, 2 * r);
      double scale_radius = view_.radius_m.value_or(radius_->GetValue());
      for (const auto &point : view_.history)
        scale_radius =
            (std::max)(scale_radius, std::hypot(*point.east_m, *point.north_m));
      if (view_.vessel_position)
        scale_radius = (std::max)(scale_radius,
                                  std::hypot(*view_.vessel_position->east_m,
                                             *view_.vessel_position->north_m));
      const double scale = scale_radius > 0 ? 87. / scale_radius : 0;
      const double r = view_.radius_m.value_or(radius_->GetValue()) * scale;
      wxPen watch_pen(Colour(p.c.accent));
      wxDash dashes[2] = {4, 5};
      watch_pen.SetDashes(2, dashes);
      watch_pen.SetStyle(wxPENSTYLE_USER_DASH);
      gc->SetPen(watch_pen);
      gc->SetBrush(wxBrush(wxColour(182, 239, 206, 4)));
      gc->DrawEllipse(150 - r, 115 - r, 2 * r, 2 * r);
      gc->SetBrush(*wxTRANSPARENT_BRUSH);
      gc->SetPen(wxPen(
          Colour(view_.distance_m ? NavigationContextInk(light_) : p.c.muted)));
      auto path = gc->CreatePath();
      bool started = false;
      vessel::Time last{};
      for (const auto &point : view_.history) {
        const auto x = 150 + *point.east_m * scale,
                   y = 115 - *point.north_m * scale;
        if (!started || point.observed_at - last > std::chrono::seconds(5))
          path.MoveToPoint(x, y);
        else
          path.AddLineToPoint(x, y);
        last = point.observed_at;
        started = true;
      }
      gc->StrokePath(path);
      if (view_.active) {
        gc->SetPen(*wxTRANSPARENT_PEN);
        gc->SetBrush(wxBrush(Colour(p.c.accent)));
        gc->DrawEllipse(147, 112, 6, 6);
      }
      if (view_.vessel_position) {
        const auto &point = *view_.vessel_position;
        // A dot carries position without inventing heading from movement.
        gc->DrawEllipse(146 + *point.east_m * scale,
                        111 - *point.north_m * scale, 8, 8);
      }
    }
  }
  const auto centered = [&](const wxString &text, int y, int size,
                            std::uint32_t ink, int weight = 400) {
    const int x = (width - summary_->ToDIP(static_cast<int>(
                               UiTextWidth(*summary_, text, size, weight)))) /
                  2;
    p.TextWeight(text, x, y, size, ink, weight);
  };
  centered("N", 48, 8, p.c.muted);
  centered("S", 259, 8, p.c.muted);
  p.Text("W", width / 2 - 109, 155, 8, p.c.muted);
  p.Text("E", width / 2 + 102, 155, 8, p.c.muted);
  centered(Number(view_.display_distance, view_.distance_decimals), 141, 36,
           view_.alarm ? p.c.alarm : p.c.primary);
  centered(view_.distance_unit.empty() ? "distance from anchor"
                                       : W(view_.distance_unit) + " from anchor",
           189, 10, p.c.muted);
  p.Text(view_.inner_alarm ? "Alarm inside radius" : "Alarm radius", 0, 292, 12,
         p.c.secondary);
  p.TextWeight(Number(view_.active
                          ? view_.radius_m
                          : std::optional<double>(radius_->GetValue())) +
                   " m",
               0, 318, 12, p.c.primary, 650);
  const int cell = (width - 16) / 2;
  p.Stat("DEPTH", Number(Reading(view_.depth), 1), "m", 0, 414, cell);
  p.Stat("TRUE WIND", Number(Reading(view_.wind), 1), "kn", cell + 16, 414,
         cell);
  p.Stat("BATTERY", Number(Reading(view_.battery)), "%", 0, 480, cell);
  p.Stat("TRACK HISTORY",
         view_.history_minutes && *view_.history_minutes < 1
             ? "<1"
             : Number(view_.history_minutes),
         "min", cell + 16, 480, cell);
}
} // namespace opennav::ui

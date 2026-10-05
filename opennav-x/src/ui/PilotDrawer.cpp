#include "ui/PilotDrawer.h"
#include "ui/Sheet.h"
#include <cmath>
#include <memory>
#include <wx/dcbuffer.h>
#include <wx/graphics.h>

namespace opennav::ui {
namespace {
const std::array<adapters::PilotAction, 4> actions = {
    adapters::PilotAction::Standby, adapters::PilotAction::Auto,
    adapters::PilotAction::Track, adapters::PilotAction::Wind};
const std::array<const char *, 4> labels = {"Standby", "Auto", "Track", "Wind"};
const std::array<int, 4> deltas = {-10, -1, 1, 10};
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
} // namespace
XNavPilotDrawer::XNavPilotDrawer(wxWindow &owner, PilotDrawerActions callbacks)
    : XNavDrawer(owner, "OpenNav autopilot"), actions_(std::move(callbacks)) {
  SetHeading("HELM CONTROL", "Autopilot", false);
  panel_ = new wxPanel(body_, wxID_ANY);
  panel_->SetName("Pilot heading and controls");
  panel_->SetMinSize(FromDIP(wxSize(300, 728)));
  panel_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  panel_->Bind(wxEVT_PAINT, &XNavPilotDrawer::Paint, this);
  panel_->Bind(wxEVT_SIZE, [this](wxSizeEvent &e) {
    Arrange();
    e.Skip();
  });
  EnableScrollGesture(*panel_);
  for (std::size_t i = 0; i < course_.size(); ++i) {
    const auto label = wxString::FromUTF8(i < 2 ? "−" : "+") +
                       wxString::Format("%d", std::abs(deltas[i])) +
                       wxString::FromUTF8("°");
    course_[i] =
        new XNavButton(panel_, wxID_ANY, label, label + " magnetic course");
    course_[i]->SetTextSize(13);
    course_[i]->Bind(wxEVT_BUTTON, [this, i](wxCommandEvent &) {
      if (queued_ || !Allowed(adapters::PilotAction::AlterCourse))
        return;
      queued_ = true;
      CallAfter([this, i] {
        Request(adapters::PilotAction::AlterCourse, deltas[i]);
      });
    });
    modes_[i] = new XNavButton(panel_, wxID_ANY, labels[i], labels[i]);
    modes_[i]->Bind(wxEVT_BUTTON, [this, i](wxCommandEvent &) {
      if (queued_ || !Allowed(actions[i]))
        return;
      queued_ = true;
      CallAfter([this, i] { Request(actions[i], 0); });
    });
  }
  enable_ =
      new XNavButton(panel_, wxID_ANY, "Enable control", "Enable control");
  enable_->SetToggle();
  enable_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
    if (queued_ || !view_.can_toggle)
      return;
    queued_ = true;
    CallAfter([this] { Toggle(); });
  });
  settings_ = new XNavButton(panel_, wxID_ANY, "Autopilot status & diagnostics",
                             "Autopilot status & diagnostics");
  settings_->SetRole(ButtonRole::Quiet);
  settings_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
    if (actions_.settings)
      CallAfter([this] { actions_.settings(); });
  });
  content_->Add(panel_, 0, wxEXPAND);
  RefreshControls();
}
void XNavPilotDrawer::Arrange() {
  const double width = panel_->ToDIP(panel_->GetClientSize().x);
  const auto rect = [&](double x, int y, double w, int h) {
    return wxRect(
        panel_->FromDIP(wxPoint(std::lround(x), y)),
        panel_->FromDIP(wxSize(std::lround(x + w) - std::lround(x), h)));
  };
  const double course_width = (width - 21.) / 4.;
  for (std::size_t i = 0; i < course_.size(); ++i) {
    course_[i]->SetSize(rect(i * (course_width + 7.), 223, course_width, 48));
    modes_[i]->SetSize(rect((i % 2) * (width + 9.) / 2., 291 + 68 * (i / 2),
                            (width - 9.) / 2., 48));
  }
  enable_->SetSize(rect(width - 48., 422, 48., 49));
  settings_->SetSize(rect(0, 676, width, 48));
}
void XNavPilotDrawer::Update(const adapters::PilotView &pilot,
                             vessel::Time pilot_now,
                             const vessel::VesselState &state,
                             vessel::Time vessel_now, bool permit_control,
                             LightMode light) {
  view_ = application::PresentPilot(pilot, pilot_now, permit_control,
                                    state.replayed);
  simulated_ = false;
#if XNAV_ENABLE_TEST_FIXTURES
  simulated_ = state.simulated;
#endif
  const auto rudder = vessel::Assess(state.rudder.angle_deg, vessel_now);
  rudder_ = (rudder.quality == vessel::Quality::Live ||
             rudder.quality == vessel::Quality::Aging)
                ? rudder.value
                : std::nullopt;
  if (rudder_ && (*rudder_ < -180 || *rudder_ > 180))
    rudder_.reset();
  SetLight(light);
  panel_->SetBackgroundColour(Colour(Theme(light).background));
  RefreshControls();
  panel_->Refresh(false);
}
bool XNavPilotDrawer::Allowed(adapters::PilotAction action) const {
  if (!actions_.command)
    return false;
  switch (action) {
  case adapters::PilotAction::Standby:
    return view_.standby;
  case adapters::PilotAction::Auto:
    return view_.auto_mode;
  case adapters::PilotAction::Track:
    return view_.track;
  case adapters::PilotAction::Wind:
    return view_.wind;
  case adapters::PilotAction::AlterCourse:
    return view_.alter_course;
  }
  return false;
}
void XNavPilotDrawer::RefreshControls() {
  for (std::size_t i = 0; i < course_.size(); ++i) {
    course_[i]->Enable(Allowed(adapters::PilotAction::AlterCourse));
    course_[i]->SetLightMode(light_);
    modes_[i]->Enable(Allowed(actions[i]));
    modes_[i]->SetLightMode(light_);
    const auto mode = std::array<adapters::PilotMode, 4>{
        adapters::PilotMode::Standby, adapters::PilotMode::Auto,
        adapters::PilotMode::Track, adapters::PilotMode::Wind}[i];
    modes_[i]->SetRole(view_.mode == mode ? ButtonRole::Primary
                                          : ButtonRole::Normal);
  }
  enable_->Enable(view_.can_toggle && bool(actions_.enable));
  const wxString enable_label = view_.output_unavailable
                                    ? "Control unavailable" : "Enable control";
  enable_->SetLabel(enable_label);
  enable_->SetName(enable_label);
  enable_->SetHint(enable_label);
  enable_->SetSelected(view_.enabled);
  enable_->SetLightMode(light_);
  settings_->Enable(bool(actions_.settings));
  settings_->SetLightMode(light_);
}
void XNavPilotDrawer::Request(adapters::PilotAction action, double delta) {
  if (!Allowed(action)) {
    queued_ = false;
    return;
  }
  if (action != adapters::PilotAction::Standby &&
      action != adapters::PilotAction::AlterCourse) {
    const auto title =
        "Request " +
        W(adapters::PilotModeName(action == adapters::PilotAction::Auto
                                      ? adapters::PilotMode::Auto
                                  : action == adapters::PilotAction::Track
                                      ? adapters::PilotMode::Track
                                      : adapters::PilotMode::Wind));
    if (!ConfirmSheet(
            *this, light_, title,
            "The mode changes only after fresh pilot feedback confirms it.",
            title)) {
      queued_ = false;
      return;
    }
  }
  // A modal confirmation can outlive the input. Recheck the most recent view;
  // ManualAutopilot independently validates again against current equipment.
  if (Allowed(action))
    actions_.command(action, delta);
  queued_ = false;
}
void XNavPilotDrawer::Toggle() {
  if (!view_.can_toggle || !actions_.enable) {
    queued_ = false;
    return;
  }
  const bool enable = !view_.enabled;
  wxString title = "Enable physical pilot control?",
           accept = "Enable manual control";
  wxString note = "Manual buttons can move the vessel's rudder. Confirm the "
                  "correct pilot, a clear drive area and immediate physical "
                  "STANDBY access. Enable lasts only for this session.";
#if XNAV_ENABLE_TEST_FIXTURES
  if (simulated_) {
    title = "Enable manual simulator";
    note = "Commands affect only the labelled test simulator.";
    accept = "Enable DEMO";
  }
#endif
  if ((!enable || ConfirmSheet(*this, light_, title, note, accept)) &&
      view_.can_toggle)
    actions_.enable(enable);
  queued_ = false;
}
void XNavPilotDrawer::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(panel_);
  XNavPainter p(*panel_, dc, light_);
  dc.SetBackground(wxBrush(Colour(p.c.background)));
  dc.Clear();
  const int width = panel_->ToDIP(panel_->GetClientSize().x);
  const int tag = p.Tag(W(view_.state), 0, 0, width, view_.pending);
  p.Tag(view_.output_unavailable ? "STATUS ONLY"
                                : view_.enabled ? "CONTROL ENABLED" : "CONTROL OFF",
        tag + 8, 0, width - tag - 8,
        !view_.enabled && !view_.output_unavailable, view_.enabled);
  {
    std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(dc));
    if (gc) {
      const double dip = panel_->FromDIP(1024) / 1024.;
      gc->Scale(dip, dip);
      gc->Translate((width - 165.) / 2., 46.);
      gc->Scale(.825, .825);
      gc->SetPen(wxPen(Colour(p.c.border)));
      gc->SetBrush(*wxTRANSPARENT_BRUSH);
      gc->DrawEllipse(12, 12, 176, 176);
      for (int i = 0; i < 36; ++i) {
        gc->PushState();
        gc->Translate(100, 100);
        gc->Rotate(i * 3.141592653589793 / 18.);
        gc->StrokeLine(0, (i % 3 == 0 ? 10 : 15) - 100, 0, -78);
        gc->PopState();
      }
      if (view_.heading_magnetic_deg) {
        gc->Translate(100, 100);
        gc->Rotate(*view_.heading_magnetic_deg * 3.141592653589793 / 180.);
        gc->SetPen(*wxTRANSPARENT_PEN);
        gc->SetBrush(wxBrush(Colour(p.c.accent)));
        auto arrow = gc->CreatePath();
        arrow.MoveToPoint(0, -96);
        arrow.AddLineToPoint(-5, -84);
        arrow.AddLineToPoint(5, -84);
        arrow.CloseSubpath();
        gc->FillPath(arrow);
      }
    }
  }
  const auto heading =
      view_.heading_magnetic_deg
          ? wxString::Format(wxString::FromUTF8("%03d°"),
                             int(std::lround(*view_.heading_magnetic_deg)) %
                                 360)
          : wxString::FromUTF8("—");
  const auto center = [&](const wxString &s, int y, int size, int weight,
                          std::uint32_t ink) {
    const int x =
        (width - panel_->ToDIP(int(UiTextWidth(*panel_, s, size, weight)))) / 2;
    p.TextWeight(s, x, y, size, ink, weight);
  };
  const int heading_width =
      panel_->ToDIP(int(UiTextWidth(*panel_, heading, 45, 350))) -
      2 * (int(heading.length()) - 1);
  p.TextTracked(heading, (width - heading_width) / 2, 96, 45, p.c.primary, 350,
                -2.);
  center(view_.commanded ? "COMMANDED HEADING / M" : "CURRENT HEADING / M", 158,
         9, 400, p.c.muted);
  p.Text(view_.output_unavailable ? "Equipment control unavailable" : "Enable control",
         0, 420, 12, p.c.secondary);
  p.Wrapped(
      view_.output_unavailable
          ? "Status display only. Use the physical helm."
          : "Explicit consent is required before heading controls become available.",
      0, 442, 11, 16, width - 62, p.c.muted, 2);
  p.Rule(0, 486, width);
  p.Text("Pilot feedback", 0, 500, 12, p.c.secondary);
  p.TextWeight(W(view_.connection), 110, 500, 12, p.c.primary, 500, width - 110,
               true);
  p.Rule(0, 531, width);
  p.Text("Rudder", 0, 545, 12, p.c.secondary);
  p.TextWeight(rudder_
                   ? wxString::Format(wxString::FromUTF8("%+.1f°"), *rudder_)
                   : wxString::FromUTF8("—"),
               110, 545, 12, p.c.primary, 500, width - 110, true);
  p.Rule(0, 576, width);
  p.Wrapped(W(view_.note), 0, 592, 11, 18, width, p.c.muted, 4);
}
} // namespace opennav::ui

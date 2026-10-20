#include "ui/TrackDrawer.h"
#include <algorithm>
#include <cmath>
#include <utility>
#include <wx/datetime.h>
#include <wx/dcbuffer.h>

namespace opennav::ui {
namespace {
wxString W(const std::string &value) { return wxString::FromUTF8(value); }
wxString Started(const std::optional<application::TrackSummary> &track) {
  return track && track->started_at
      ? wxDateTime(std::chrono::system_clock::to_time_t(*track->started_at)).Format("%Y-%m-%d %H:%M")
      : wxString("Unavailable");
}
wxString Elapsed(const std::optional<application::TrackSummary> &track) {
  if (!track || !track->started_at) return "Unavailable";
  const auto end = track->recording ? std::chrono::system_clock::now()
      : track->ended_at.value_or(*track->started_at);
  if ((!track->recording && !track->ended_at) || end < *track->started_at) return "Unavailable";
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(end - *track->started_at).count();
  return wxString::Format("%lld:%02lld:%02lld", static_cast<long long>(seconds / 3600),
      static_cast<long long>(seconds / 60 % 60), static_cast<long long>(seconds % 60));
}
}

XNavTrackRecordButton::XNavTrackRecordButton(wxWindow *parent)
    : XNavButton(parent, wxID_ANY, "Tracks", "Open track recording") {
  SetMinSize(FromDIP(wxSize(48, 48)));
  Bind(wxEVT_PAINT, &XNavTrackRecordButton::Paint, this);
}
void XNavTrackRecordButton::Update(const application::TrackRecordingState &state, LightMode mode) {
  const bool gps_warning = state.available && !state.position_fresh;
  if (rendered_ && recording_ == state.recording && pending_ == state.pending_start &&
      gps_warning_ == gps_warning && reason_ == state.reason && light_ == mode) return;
  rendered_ = true;
  light_ = mode;
  recording_ = state.recording;
  pending_ = state.pending_start;
  gps_warning_ = gps_warning;
  reason_ = state.reason;
  const wxString title = recording_ ? "Recording" : pending_ ? "Starting" : "Tracks";
  SetLabel(title + (gps_warning_ ? " / GPS unavailable" : ""));
  SetName("Open track recording: " + GetLabel());
  SetHint("Track recording: " + GetLabel() +
      (state.reason.empty() ? wxString(" / Open recording and saved tracks")
                            : wxString(" / ") + W(state.reason)));
  SetLightMode(mode);
  Refresh(false);
}
void XNavTrackRecordButton::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);
  const auto c = Theme(light_);
  const auto floating = FloatingTheme(light_);
  dc.SetBackground(wxBrush(Colour(floating.surface)));
  dc.Clear();
  const auto size = GetClientSize();
  const bool highlighted = IsHovered() || HasKeyboardFocus() ||
      recording_ || pending_ || gps_warning_;
  dc.SetPen(HasKeyboardFocus() ? wxPen(Colour(c.accent), FromDIP(2)) : *wxTRANSPARENT_PEN);
  dc.SetBrush(wxBrush(Colour(highlighted ? c.selected : floating.surface)));
  dc.DrawRoundedRectangle(FromDIP(2), FromDIP(2), size.x - FromDIP(4),
      size.y - FromDIP(4), FromDIP(9));
  // A steady recording glyph, without the old wide header label. Amber means
  // the recorder is pending or cannot currently collect fresh positions.
  const auto ink = !IsEnabled() ? floating.secondary
      : (gps_warning_ || pending_) ? c.attention
      : recording_ ? c.healthy : highlighted ? c.primary : floating.primary;
  const wxPoint center(size.x / 2, size.y / 2);
  dc.SetPen(wxPen(Colour(ink), FromDIP(2)));
  dc.SetBrush(*wxTRANSPARENT_BRUSH);
  dc.DrawCircle(center, FromDIP(10));
  dc.SetPen(*wxTRANSPARENT_PEN);
  dc.SetBrush(wxBrush(Colour(ink)));
  if (recording_ || pending_)
    dc.DrawCircle(center, FromDIP(5));
}

XNavTrackDrawer::XNavTrackDrawer(wxWindow &owner, application::NavigationActions actions)
    : XNavDrawer(owner, "Track recording"), actions_(std::move(actions)) {
  SetHeading("TRACK RECORDING", "Record your journey", false);
  const auto text = [this](int size, bool bold) {
    auto *label = new wxStaticText(body_, wxID_ANY, "");
    label->SetFont(UiFont(*label, size, bold));
    EnableScrollGesture(*label);
    content_->Add(label, 0, wxEXPAND | wxBOTTOM, FromDIP(20));
    return label;
  };
  status_ = text(20, true);
  details_ = text(14, false);
  notes_ = text(12, false);
  record_ = new XNavButton(body_, wxID_ANY, "Start recording", "Start track recording");
  record_->SetMinSize(FromDIP(wxSize(1, 48)));
  record_->SetDisplayAction(48);
  record_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
    const auto generation = command_generation_;
    const bool stop = state_.recording || state_.pending_start;
    const std::string id = state_.active ? state_.active->id : std::string{};
    CallAfter([this, generation, stop, id] {
      if (generation == command_generation_) ChangeRecording(stop, id);
    });
  });
  content_->Add(record_, 0, wxEXPAND | wxBOTTOM, FromDIP(12));
  saved_ = new XNavButton(body_, wxID_ANY, "Saved tracks", "Open saved tracks");
  saved_->SetMinSize(FromDIP(wxSize(1, 48)));
  saved_->SetDisplayAction(48);
  saved_->SetRole(ButtonRole::Quiet);
  saved_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
    const auto generation = command_generation_;
    CallAfter([this, generation] {
      if (generation == command_generation_ && IsShownOnScreen() && on_saved_tracks)
        on_saved_tracks();
    });
  });
  content_->Add(saved_, 0, wxEXPAND);
  body_->Bind(wxEVT_SIZE, [this](wxSizeEvent &event) { Reflow(); event.Skip(); });
  Bind(wxEVT_SHOW, [this](wxShowEvent &event) {
    if (!event.IsShown()) ++command_generation_;
    event.Skip();
  });
}

void XNavTrackDrawer::Open(const wxRect &workspace,
    const application::TrackRecordingState &state, bool live, LightMode mode) {
  ++command_generation_;
  feedback_.clear();
  Update(state, live, mode);
  Present(workspace);
  body_->Scroll(0, 0);
}

void XNavTrackDrawer::Update(const application::TrackRecordingState &state,
    bool live, LightMode mode) {
  state_ = state;
  live_ = live;
  SetLight(mode);
  status_copy_ = state.recording ? "Recording" : state.pending_start ? "Recording requested"
      : state.available ? "Not recording" : "Recording unavailable";
  const auto distance = state.active && state.active->distance_nm &&
      std::isfinite(*state.active->distance_nm) && *state.active->distance_nm >= 0
      ? wxString::Format("%.2f NM", *state.active->distance_nm) : wxString("Unavailable");
  detail_copy_ = "Started: " + Started(state.active) + "\nElapsed: " + Elapsed(state.active) +
      "\nDistance: " + distance;
  note_copy_.clear();
  if (!live) note_copy_ += "Recording controls are unavailable while viewing recorded or simulated input.\n\n";
  if (state.available && !state.position_fresh) note_copy_ += "GPS unavailable\n\n";
  if ((state.recording || state.pending_start) &&
      (!state.layer_visible || (state.active && !state.active->visible)))
    note_copy_ += "Recording hidden on chart. Tracks visibility does not change recording.\n\n";
  if (!state.reason.empty()) note_copy_ += W(state.reason) + "\n\n";
  if (!feedback_.empty()) note_copy_ += feedback_ + "\n\n";
  note_copy_ += "Closing this drawer does not stop recording. Saved tracks remain in OpenCPN.";
  const auto colors = Theme(mode);
  status_->SetForegroundColour(Colour(state.recording ? colors.healthy : colors.primary));
  details_->SetForegroundColour(Colour(colors.secondary));
  notes_->SetForegroundColour(Colour(state.available && !state.position_fresh
      ? colors.attention : colors.secondary));
  const bool stop = state.recording || state.pending_start;
  record_->SetLabel(stop ? "Stop recording" : "Start recording");
  record_->SetName(stop ? "Stop track recording" : "Start track recording");
  record_->SetRole(stop ? ButtonRole::Normal : ButtonRole::Primary);
  record_->SetLightMode(mode);
  record_->Enable(live && state.available && (stop
      ? state.can_stop && bool(actions_.stop_track)
      : state.can_start && bool(actions_.start_track)));
  saved_->SetLightMode(mode);
  Reflow();
}

void XNavTrackDrawer::Reflow() {
  const int width = body_->GetClientSize().x - FromDIP(44);
  if (width <= 0) return;
  for (const auto &entry : {std::make_pair(status_, status_copy_),
                           std::make_pair(details_, detail_copy_),
                           std::make_pair(notes_, note_copy_)}) {
    entry.first->SetLabel(entry.second);
    entry.first->Wrap(width);
  }
  body_->Layout();
  body_->FitInside();
}

void XNavTrackDrawer::ChangeRecording(bool stop, const std::string &expected_id) {
  if (!IsShownOnScreen() || !live_ || !state_.available) return;
  if (stop ? !state_.can_stop : !state_.can_start) return;
  ++command_generation_; // A double tap cannot reuse the same rendered command.
  const auto result = stop
      ? actions_.stop_track ? actions_.stop_track(expected_id) : application::CommandResult{}
      : actions_.start_track ? actions_.start_track() : application::CommandResult{};
  feedback_ = W(result.message);
  // Read back the native recorder even after failure. Never infer recording
  // state from a successful command, the button label or chart visibility.
  Update(actions_.track_recording ? actions_.track_recording()
      : application::TrackRecordingState{}, live_, light_);
  if (on_changed) on_changed();
}
} // namespace opennav::ui

#include "ui/ProductPanel.h"
#include "ui/NameEditor.h"
#include <algorithm>
#include <cmath>
#include <wx/datetime.h>

namespace opennav::ui {
namespace {
wxString W(const std::string &value) { return wxString::FromUTF8(value); }
wxString TrackTime(const std::optional<std::chrono::system_clock::time_point> &value) {
  return value ? wxDateTime(std::chrono::system_clock::to_time_t(*value)).Format("%Y-%m-%d %H:%M")
               : wxString("Unavailable");
}
wxString TrackDistance(const application::TrackSummary &track) {
  return track.distance_nm && std::isfinite(*track.distance_nm) && *track.distance_nm >= 0
      ? wxString::Format("%.2f NM", *track.distance_nm) : wxString("Unavailable");
}
}

void ProductPanel::ShowTrack(const std::string &id, LightMode mode) {
  track_ = {};
  track_.id = id;
  track_available_ = false;
  RefreshTrack();
  ShowPage(ProductPage::TrackDetail, mode);
}

bool ProductPanel::RefreshTrack() {
  track_refreshed_at_ = std::chrono::steady_clock::now();
  const auto current = actions_.navigation.track
      ? actions_.navigation.track(track_.id) : std::nullopt;
  const bool changed = track_available_ != current.has_value() ||
      (current && (current->revision != track_.revision || current->name != track_.name ||
          current->visible != track_.visible || current->recording != track_.recording ||
          current->editable != track_.editable || current->in_layer != track_.in_layer));
  track_available_ = current.has_value();
  if (current) track_ = *current;
  return changed;
}

void ProductPanel::TrackLibrary() {
  Heading("Saved tracks", "Recorded journeys in your OpenCPN navigation library");
  BeginActions(2);
  Action("Track recording", actions_.track_recording);
  Action("Refresh catalog", [this] { Build(); });
  Action("Routes", [this] { ShowPage(ProductPage::Routes, mode_); });
  Action("Waypoints", [this] { ShowPage(ProductPage::Waypoints, mode_); });
  EndActions();
  // Open/explicit refresh only. Never walk the full native catalog on a timer.
  auto catalog = actions_.navigation.tracks ? actions_.navigation.tracks()
      : application::TrackCatalog{};
  if (!catalog.available) {
    Text(catalog.reason.empty() ? wxString("Track library unavailable") : W(catalog.reason));
  } else {
    std::stable_sort(catalog.tracks.begin(), catalog.tracks.end(),
        [](const application::TrackSummary &a, const application::TrackSummary &b) {
      if (a.recording != b.recording) return a.recording;
      const auto a_time = a.started_at ? a.started_at : a.ended_at;
      const auto b_time = b.started_at ? b.started_at : b.ended_at;
      if (a_time != b_time) return a_time > b_time;
      return a.id < b.id;
    });
    if (catalog.tracks.empty()) Text("No tracks saved. Open Track recording to begin a journey.");
    for (const auto &track : catalog.tracks) {
      const auto title = track.name.empty() ? wxString("Unnamed track") : W(track.name);
      Action(title + (track.recording ? " / RECORDING" : "") + " / " +
          TrackTime(track.started_at) + " / " + TrackDistance(track) +
          (track.visible ? " / Shown" : " / Hidden"),
          [this, id = track.id] { ShowTrack(id, mode_); });
    }
    if (catalog.truncated)
      Text("Catalog capped; open Advanced track manager for the remaining tracks.");
    if (!catalog.reason.empty()) Text(W(catalog.reason));
  }
  Action("Advanced track manager", actions_.navigation.legacy_route_manager);
  Text("Export, delete and convert tracks using the Advanced OpenCPN manager.");
}

void ProductPanel::TrackActions() {
  if (!track_available_) {
    Heading("Track unavailable", "The selected track was removed or cannot be identified.");
    Action("Back to saved tracks", [this] { ShowPage(ProductPage::Tracks, mode_); });
    return;
  }
  // Commands carry the exact identity and revision displayed to the human.
  // A background refresh may update statistics, but never replaces an open draft.
  const auto selected = track_;
  Heading("Track details", selected.recording ? "Current recording" : "Saved journey");
  auto *name_editor = new XNavNameEditor(*this, "Track name", selected.name,
      mode_, interface_scale_, selected.editable && bool(actions_.navigation.rename_track),
      [this, selected](const std::string &name) {
        return actions_.navigation.rename_track(selected, name);
      }, [this, id = selected.id](application::CommandResult result) {
        if (page_ != ProductPage::TrackDetail || track_.id != id) return;
        const auto scroll = GetViewStart();
        RefreshTrack();
        rebuild_pending_ = XNavNameEditor::PreserveDrafts(*this, mode_,
            track_available_ && !state_.vessel.simulated && !state_.vessel.replayed);
        if (!rebuild_pending_) Build();
        result.identity.clear();
        Result(result);
        Scroll(scroll);
      });
  name_editor->Present(mode_, !state_.vessel.simulated && !state_.vessel.replayed);
  body_->Add(name_editor, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(16));
  LiveText([this](const ProductState &) {
    if (!track_available_)
      return wxString("This track is no longer available. Cancel the name edit to refresh.");
    return wxString("Started: ") + TrackTime(track_.started_at) +
        "\nDistance: " + TrackDistance(track_) +
        wxString::Format("\nTrack points: %llu", static_cast<unsigned long long>(track_.point_count)) +
        (track_.recording ? wxString("\nRecording") : "\nEnded: " + TrackTime(track_.ended_at));
  });
  LiveText([this](const ProductState &) {
    if (!track_available_) return wxString{};
    return track_.visible ? wxString("Track visibility: shown when the Tracks chart layer is on")
                          : wxString("Track visibility: hidden on chart");
  });
  if (selected.in_layer) Text("This track belongs to an OpenCPN layer. Its name is managed there.");
  BeginActions(2);
  Action("Fit track on chart", [this, id = selected.id] {
    const auto result = actions_.navigation.view_track(id);
    if (result.ok && actions_.chart) actions_.chart();
    else Result(result);
  }, bool(actions_.navigation.view_track));
  Action(selected.visible ? "Hide track on chart" : "Show track on chart", [this, selected] {
    if (page_ != ProductPage::TrackDetail || track_.id != selected.id ||
        state_.vessel.simulated || state_.vessel.replayed) return;
    auto result = actions_.navigation.set_track_visible(selected, !selected.visible);
    const auto scroll = GetViewStart();
    RefreshTrack();
    rebuild_pending_ = XNavNameEditor::PreserveDrafts(*this, mode_, track_available_);
    if (!rebuild_pending_) Build();
    result.identity.clear();
    Result(result);
    Scroll(scroll);
  }, selected.editable && !selected.in_layer && bool(actions_.navigation.set_track_visible) &&
      !state_.vessel.simulated && !state_.vessel.replayed);
  Action("Track recording", actions_.track_recording);
  Action("Back to saved tracks", [this] { ShowPage(ProductPage::Tracks, mode_); });
  EndActions();
  Action("Advanced track manager", actions_.navigation.legacy_route_manager);
  Text("Export, delete and convert tracks using the Advanced OpenCPN manager. "
       "Track visibility does not change recording.");
}
} // namespace opennav::ui

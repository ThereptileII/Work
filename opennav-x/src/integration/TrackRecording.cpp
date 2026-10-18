#include "integration/TrackRecording.h"
#include "application/NavigationNaming.h"
#include "integration/OpenCPNIntegration.h"
#include "integration/TrackPresentation.h"
#include "integration/TrackRecordingHooks.h"
#include "TrackPropDlg.h"
#include "bbox.h"
#include "chcanv.h"
#include "model/georef.h"
#include "model/navobj_db.h"
#include "model/own_ship.h"
#include "model/routeman.h"
#include "model/select.h"
#include "model/track.h"
#include "ocpn_frame.h"
#include "viewport.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <map>
#include <sstream>
#include <utility>
#include <wx/thread.h>

extern std::vector<Track *> g_TrackList;
extern ActiveTrack *g_pActiveTrack;
extern TrackPropDlg *pTrackPropDialog;
extern bool g_bTrackActive, g_bDeferredStartTrack, g_bDeferredInitDone, bGPSValid;
extern bool g_bTrackCarryOver;

namespace opennav::integration {
namespace {
using application::CommandResult;
using application::TrackSummary;
constexpr std::size_t kCatalogLimit = 256, kRegistryLimit = 10000;
constexpr std::size_t kDistancePointBudget = 20000, kFitPointLimit = 200000;
std::function<vessel::Navigation()> live_position;
bool command_storage_failed = false;
std::string storage_warning;
struct DistanceCopy {
  std::string revision;
  vessel::Time copied_at{};
  double nm = 0;
};
std::map<std::string, DistanceCopy> distance_cache;

std::string Text(const wxString &s) { return s.ToStdString(wxConvUTF8); }
bool Coordinates(double lat, double lon) {
  return std::isfinite(lat) && std::isfinite(lon) &&
         std::abs(lat) <= 90 && std::abs(lon) <= 180;
}
bool FreshPosition() {
  if (!wxIsMainThread() || !live_position || !bGPSValid) return false;
  const auto n = live_position();
  const auto &lat = n.latitude_deg, &lon = n.longitude_deg;
  const auto now = vessel::Clock::now();
  const auto fresh = [](vessel::Quality q) {
    return q == vessel::Quality::Live || q == vessel::Quality::Aging;
  };
  return lat.validity == vessel::Validity::Measured &&
         lon.validity == vessel::Validity::Measured && lat.value && lon.value &&
         !lat.source.empty() && lat.source == lon.source &&
         lat.observed_at == lon.observed_at &&
         fresh(vessel::Assess(lat, now).quality) &&
         fresh(vessel::Assess(lon, now).quality) &&
         Coordinates(*lat.value, *lon.value) &&
         *lat.value == gLat && *lon.value == gLon;
}
bool SampleAllowed() {
  return !opennav::IsXNav() || FreshPosition();
}
void StorageResult(Track *, bool ok) {
  if (wxIsMainThread() && opennav::IsXNav() && !ok) {
    command_storage_failed = true;
    storage_warning = "OpenCPN reported a track storage error; verify saved tracks";
  }
}
Track *Resolve(const std::string &id) {
  if (!wxIsMainThread() || id.empty() || g_TrackList.size() > kRegistryLimit)
    return nullptr;
  Track *match = nullptr;
  for (auto *track : g_TrackList) {
    if (track && Text(track->m_GUID) == id) {
      if (match) return nullptr;
      match = track;
    }
  }
  return match;
}
Track *Active() {
  // Prove membership before dereferencing the process-global pointer.
  if (!g_pActiveTrack || g_TrackList.size() > kRegistryLimit) return nullptr;
  const auto found = std::find(g_TrackList.begin(), g_TrackList.end(), g_pActiveTrack);
  if (found == g_TrackList.end()) return nullptr;
  return Resolve(Text((*found)->m_GUID)) == *found ? *found : nullptr;
}
bool Editable(Track &track) {
  return !track.m_bIsInLayer && track.m_TrackHyperlinkList &&
         track.m_TrackHyperlinkList->GetCount() <= 128 &&
         !(TrackPropDlg::getInstanceFlag() && pTrackPropDialog &&
           pTrackPropDialog->IsShown() && pTrackPropDialog->GetTrack() == &track);
}
std::optional<std::chrono::system_clock::time_point> Time(TrackPoint *point) {
  if (!point) return {};
  auto value = point->GetCreateTime();
  if (!value.IsValid()) return {};
  // Pinned Track::GetDateTime uses FromUTC before user/wall-clock conversion.
  return std::chrono::system_clock::from_time_t(value.FromUTC().GetTicks());
}
TrackSummary Summary(Track &track, std::size_t &distance_budget) {
  TrackSummary copy;
  copy.id = Text(track.m_GUID);
  copy.name = Text(track.GetName(true));
  copy.point_count = static_cast<std::size_t>((std::max)(0, track.GetnPoints()));
  copy.recording = &track == g_pActiveTrack && g_bTrackActive && track.IsRunning();
  copy.visible = track.IsVisible();
  copy.in_layer = track.m_bIsInLayer;
  copy.editable = Editable(track);
  auto *first = copy.point_count ? track.GetPoint(0) : nullptr;
  auto *last = copy.point_count ? track.GetLastPoint() : nullptr;
  copy.started_at = Time(first);
  copy.ended_at = copy.recording ? std::nullopt : Time(last);
  // Mutation revision covers native editable attributes and lifecycle, not
  // growing point geometry: a fresh sample must not invalidate a rename form.
  std::ostringstream revision;
  revision.imbue(std::locale::classic());
  revision << std::quoted(copy.id) << std::quoted(Text(track.GetName()))
           << std::quoted(Text(track.m_TrackDescription))
           << std::quoted(Text(track.m_TrackStartString))
           << std::quoted(Text(track.m_TrackEndString))
           << std::quoted(Text(track.m_Colour)) << ';' << track.m_width << ';'
           << static_cast<int>(track.m_style) << ';'
           << (track.m_bIsInLayer ? track.m_LayerID : 0) << ';'
           << copy.recording << copy.visible
           << copy.editable << copy.in_layer;
  if (track.m_TrackHyperlinkList) {
    revision << ';' << track.m_TrackHyperlinkList->GetCount();
    std::size_t links = 0;
    for (auto *node = track.m_TrackHyperlinkList->GetFirst(); node && links < 128;
         node = node->GetNext(), ++links) {
      if (const auto *link = node->GetData())
        revision << std::quoted(link->GUID) << std::quoted(Text(link->Link))
                 << std::quoted(Text(link->DescrText)) << std::quoted(Text(link->LType));
      else revision << ";missing link";
    }
  }
  copy.revision = revision.str();
  revision << ';' << copy.point_count;
  for (auto *point : {first, last}) {
    if (point) revision << ';' << std::setprecision(17) << point->m_lat << ','
                        << point->m_lon << ',' << point->GetTimeString();
  }
  const auto geometry_revision = revision.str();
  const auto now = vessel::Clock::now();
  auto cached = distance_cache.find(copy.id);
  if (cached != distance_cache.end() && cached->second.revision == geometry_revision &&
      now - cached->second.copied_at < std::chrono::seconds(5)) {
    copy.distance_nm = cached->second.nm;
  } else if (copy.point_count && copy.point_count <= distance_budget) {
    distance_budget -= copy.point_count;
    for (std::size_t i = 0; i < copy.point_count; ++i) {
      const auto *point = track.GetPoint(static_cast<int>(i));
      if (!point || !Coordinates(point->m_lat, point->m_lon)) return copy;
    }
    // Use the pinned recorder's own length calculation. No UI geodesy and no
    // unbounded Length() call from high-frequency recording-state polling.
    const auto distance = track.Length();
    if (std::isfinite(distance) && distance >= 0) {
      copy.distance_nm = distance;
      if (distance_cache.size() >= kCatalogLimit) distance_cache.clear();
      distance_cache[copy.id] = {geometry_revision, now, distance};
    }
  }
  return copy;
}
Track *Resolve(const TrackSummary &selected) {
  auto *track = Resolve(selected.id);
  std::size_t budget = 0;
  return track && !selected.revision.empty() &&
                 Summary(*track, budget).revision == selected.revision
             ? track : nullptr;
}
CommandResult ThreadFailure() {
  return {false, "Track commands require the application thread"};
}
} // namespace

void ConfigureTrackRecording(std::function<vessel::Navigation()> position) {
  if (!wxIsMainThread()) return;
  live_position = std::move(position);
  NativeTrackHooks() = {SampleAllowed, StorageResult};
}
void ClearTrackRecordingBridge() {
  if (!wxIsMainThread()) return;
  live_position = {};
  distance_cache.clear();
  // Do not unregister the model gate: XNav shutdown must not sample stale
  // globals after its live source has gone away. Legacy still passes through.
}
bool NativeTrackStartAllowed() {
  return wxIsMainThread() && g_bDeferredInitDone && !g_pActiveTrack &&
         !g_bTrackActive && g_TrackList.size() < kRegistryLimit &&
         pSelect && g_pRouteMan && pWayPointMan && FreshPosition();
}
application::TrackRecordingState CopyTrackRecording() {
  application::TrackRecordingState state;
  if (!wxIsMainThread()) {
    state.reason = "Track state requires the application thread";
    return state;
  }
  state.layer_visible = TrackLayerVisible();
  state.pending_start = g_bDeferredStartTrack;
  state.position_fresh = FreshPosition();
  state.available = g_bDeferredInitDone && pSelect && g_pRouteMan && pWayPointMan &&
                    g_TrackList.size() <= kRegistryLimit;
  auto *active = Active();
  state.recording = active && g_bTrackActive && active->IsRunning();
  if (active) {
    std::size_t budget = kDistancePointBudget;
    state.active = Summary(*active, budget);
  }
  const bool consistent = (g_pActiveTrack == nullptr && !g_bTrackActive) || state.recording;
  state.can_start = state.available && consistent && !state.recording &&
                    g_TrackList.size() < kRegistryLimit && state.position_fresh;
  state.can_stop = state.available && consistent && (state.recording || state.pending_start);
  if (!state.available) state.reason = "OpenCPN track storage is not ready";
  else if (!consistent) state.reason = "OpenCPN recording state changed; refresh before commanding";
  else if (!storage_warning.empty()) state.reason = storage_warning;
  else if (state.recording && !state.position_fresh)
    state.reason = "Recording remains enabled; waiting for a fresh position before adding points";
  else if (state.pending_start)
    state.reason = "OpenCPN carryover is waiting to start with a fresh position";
  else if (!state.recording && g_TrackList.size() == kRegistryLimit)
    state.reason = "Track registry exceeds the safe recording-start limit";
  else if (!state.position_fresh) state.reason = "A fresh measured position is required to start";
  else state.reason = state.recording ? "Recording sailing track" : "Ready to record";
  return state;
}
application::TrackCatalog CopyTrackCatalog() {
  application::TrackCatalog catalog;
  if (!wxIsMainThread()) {
    catalog.reason = "Track catalog requires the application thread";
    return catalog;
  }
  if (!g_bDeferredInitDone || g_TrackList.size() > kRegistryLimit) {
    catalog.reason = "Track catalog unavailable or exceeds the safe registry limit";
    return catalog;
  }
  catalog.available = true;
  std::map<std::string, std::size_t> identities;
  for (auto *track : g_TrackList) if (track) ++identities[Text(track->m_GUID)];
  std::size_t budget = kDistancePointBudget;
  // Reverse native insertion order is deterministic; imported GPX files can
  // contain older dates, so this is not presented as a chronological log.
  for (auto it = g_TrackList.rbegin(); it != g_TrackList.rend(); ++it) {
    auto *track = *it;
    if (!track) continue;
    const auto id = Text(track->m_GUID);
    if (id.empty() || identities[id] != 1) {
      catalog.reason = "Some tracks have missing or ambiguous identities";
      continue;
    }
    if (catalog.tracks.size() == kCatalogLimit) {
      catalog.truncated = true;
      break;
    }
    catalog.tracks.push_back(Summary(*track, budget));
  }
  if (catalog.truncated)
    catalog.reason = "Showing up to 256 tracks; use Advanced for the full list";
  return catalog;
}
std::optional<TrackSummary> CopyTrack(const std::string &id) {
  auto *track = Resolve(id);
  if (!track || !g_bDeferredInitDone) return {};
  std::size_t budget = kDistancePointBudget;
  return Summary(*track, budget);
}
CommandResult StartTrack(MyFrame &frame) {
  if (!wxIsMainThread()) return ThreadFailure();
  const auto state = CopyTrackRecording();
  if (state.recording) {
    g_bDeferredStartTrack = false;
    g_bTrackCarryOver = true;
    return {true, "Track recording is already enabled", state.active->id};
  }
  if (!state.can_start) return {false, state.reason};
  command_storage_failed = false;
  frame.TrackOn();
  const auto after = CopyTrackRecording();
  if (!after.recording || !after.active)
    return {false, command_storage_failed ? "OpenCPN could not create the track" :
                                          "Track did not start; refresh current state"};
  g_bDeferredStartTrack = false;
  g_bTrackCarryOver = true;
  return {!command_storage_failed,
          command_storage_failed ? "Recording enabled, but OpenCPN reported a storage error" :
                                   "Track recording started",
          after.active->id};
}
CommandResult StopTrack(MyFrame &frame, const std::string &expected) {
  if (!wxIsMainThread()) return ThreadFailure();
  const auto before = CopyTrackRecording();
  if (!before.available) return {false, before.reason};
  if (!g_pActiveTrack && !g_bTrackActive) {
    // Empty identity deliberately cancels only a pending carryover. It cannot
    // stop a subsequently started recorder. Repeated Stop is otherwise inert.
    if (before.pending_start && !expected.empty())
      return {false, "Recording changed; refresh the pending start"};
    g_bDeferredStartTrack = false;
    g_bTrackCarryOver = false;
    return {true, before.pending_start ? "Pending track start cancelled" :
                                       "Track recording is already stopped", expected};
  }
  if (!before.can_stop || !before.active || expected.empty() || before.active->id != expected)
    return {false, "Active track changed; refresh before stopping"};
  g_bDeferredStartTrack = false;
  command_storage_failed = false;
  Track *returned = frame.TrackOff(true);
  const auto after = CopyTrackRecording();
  if (g_pActiveTrack || g_bTrackActive || after.recording)
    return {false, "OpenCPN still reports an active recording"};
  g_bTrackCarryOver = false;
  // TrackOff can destroy the old pointer or return a different daily track.
  // Establish membership again before reading or saving either identity.
  Track *retained = nullptr;
  for (auto *candidate : g_TrackList) if (candidate && candidate == returned) retained = candidate;
  if (!returned) {
    return {!command_storage_failed,
            command_storage_failed ? "Recording stopped; OpenCPN reported a storage error" :
                                     "Recording stopped; OpenCPN discarded the short track"};
  }
  if (!retained || Resolve(Text(retained->m_GUID)) != retained)
    return {false, "Recording stopped; retained track identity could not be verified"};
  const auto id = Text(retained->m_GUID);
  // Native UpdateTrack flushes the retained points, including simplification.
  // Its result is an upstream acceptance result, not an independent disk audit.
  const bool saved = NavObj_dB::GetInstance().UpdateTrack(retained);
  StorageResult(retained, saved);
  distance_cache.erase(id);
  return {saved && !command_storage_failed,
          saved && !command_storage_failed ? "Recording stopped; OpenCPN accepted the track save" :
                                             "Recording stopped; track save reported an error",
          id};
}
CommandResult RenameTrack(const TrackSummary &selected, const std::string &name) {
  if (!wxIsMainThread()) return ThreadFailure();
  auto *track = Resolve(selected);
  if (!track || !Editable(*track))
    return {false, "Track changed or is protected; refresh before renaming"};
  if (!application::ValidNavigationName(name))
    return {false, "Enter a valid track name of 1–128 UTF-8 bytes"};
  const auto title = wxString::FromUTF8(name);
  if (title.empty()) return {false, "Track name is not valid UTF-8"};
  const auto previous = track->GetName();
  if (previous == title) return {true, "Track name unchanged", selected.id};
  track->SetName(title);
  const bool saved = NavObj_dB::GetInstance().UpdateDBTrackAttributes(track);
  StorageResult(track, saved);
  if (!saved) {
    track->SetName(previous);
    return {false, "Track rename could not be confirmed; verify saved state", selected.id};
  }
  distance_cache.erase(selected.id);
  return {true, "Track renamed", selected.id};
}
CommandResult SetTrackVisible(const TrackSummary &selected, bool visible) {
  if (!wxIsMainThread()) return ThreadFailure();
  auto *track = Resolve(selected);
  if (!track || !Editable(*track))
    return {false, "Track changed or is protected; refresh before changing visibility"};
  const bool previous = track->IsVisible();
  if (previous == visible) return {true, "Track visibility unchanged", selected.id};
  track->SetVisible(visible);
  const bool saved = NavObj_dB::GetInstance().UpdateDBTrackAttributes(track);
  StorageResult(track, saved);
  if (!saved) {
    track->SetVisible(previous);
    return {false, "Track visibility save reported an error; verify saved state", selected.id};
  }
  return {true, visible ? "Track shown" : "Track hidden", selected.id};
}
CommandResult ViewTrack(MyFrame &frame, const std::string &id) {
  if (!wxIsMainThread()) return ThreadFailure();
  auto *track = Resolve(id);
  auto *canvas = frame.GetPrimaryCanvas();
  if (!track || !canvas) return {false, "Track or chart canvas unavailable"};
  const int count = track->GetnPoints();
  if (count < 1 || static_cast<std::size_t>(count) > kFitPointLimit)
    return {false, "Track has no points or exceeds the safe chart-fit limit", id};
  LLBBox bounds;
  for (int i = 0; i < count; ++i) {
    auto *point = track->GetPoint(i);
    if (!point || !Coordinates(point->m_lat, point->m_lon))
      return {false, "Track contains unavailable coordinates", id};
    LLBBox point_bounds;
    point_bounds.Set(point->m_lat, point->m_lon, point->m_lat, point->m_lon);
    bounds.Expand(point_bounds);
  }
  if (!bounds.GetValid() || bounds.GetMinLat() <= -85 || bounds.GetMaxLat() >= 85 ||
      bounds.GetLonRange() >= 180)
    return {false, "Track extent cannot be fitted safely in this chart projection", id};
  const double lat = (bounds.GetMinLat() + bounds.GetMaxLat()) / 2;
  double lon = (bounds.GetMinLon() + bounds.GetMaxLon()) / 2;
  double east, north, west, south;
  // Native Mercator conversion and IDL-aware native bounds, not a parallel
  // geodesy implementation. Rotation enlarges the required screen rectangle.
  toSM(bounds.GetMaxLat(), bounds.GetMaxLon(), lat, lon, &east, &north);
  toSM(bounds.GetMinLat(), bounds.GetMinLon(), lat, lon, &west, &south);
  const double width = std::abs(east - west), height = std::abs(north - south);
  const double angle = canvas->GetVP().rotation;
  const double rotated_width = std::abs(std::cos(angle)) * width + std::abs(std::sin(angle)) * height;
  const double rotated_height = std::abs(std::sin(angle)) * width + std::abs(std::cos(angle)) * height;
  const auto size = canvas->GetClientSize();
  if (size.x <= 0 || size.y <= 0) return {false, "Chart canvas has no usable size", id};
  const double scale = (std::min)(1.0, .75 * (std::min)(
      size.x / (std::max)(1.0, rotated_width),
      size.y / (std::max)(1.0, rotated_height)));
  if (!std::isfinite(scale) || scale <= 0) return {false, "Track chart scale unavailable", id};
  if (!track->IsVisible()) {
    std::size_t budget = 0;
    const auto shown = SetTrackVisible(Summary(*track, budget), true);
    if (!shown.ok) return shown;
  }
  const auto layer = SetTrackLayerVisible(true);
  if (!layer.ok) return layer;
  if (lon > 180) lon -= 360;
  frame.JumpToPosition(canvas, lat, lon, scale);
  return {true, "Chart view requested for track", id};
}
} // namespace opennav::integration

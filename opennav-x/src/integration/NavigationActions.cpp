#include "integration/NavigationActions.h"
#include "application/NavigationNaming.h"
#include "chcanv.h"
#include "integration/NavigationObjects.h"
#include "integration/RoutePassageSettings.h"
#include "integration/TrackPresentation.h"
#include "integration/TrackRecording.h"
#include "integration/WeatherOverlay.h"
#include "model/ais_state_vars.h"
#include "model/navobj_db.h"
#include "model/route.h"
#include "model/route_point.h"
#include "model/routeman.h"
#include "model/select.h"
#include "ocpn_frame.h"
#include "routemanagerdialog.h"
#include "viewport.h"
#include "vessel/AisSelection.h"
#include "undo.h"
#include <cmath>
#include <set>
#include <utility>
#include <vector>
#include <wx/log.h>
#include <wx/thread.h>
extern RouteManagerDialog *pRouteManagerDialog;
extern int options_lastPage, options_subpage;
namespace opennav::integration {
namespace {
bool OwnsCanvas(MyFrame &frame, ChartCanvas &canvas) {
  for (auto *child : frame.GetChildren())
    if (child == &canvas) return true;
  return false;
}
bool RegisteredDraft(::Route *route) {
  if (!route || !pRouteList) return false;
  bool registered = false;
  for (auto *node = pRouteList->GetFirst(); node; node = node->GetNext())
    registered |= node->GetData() == route;
  if (!registered || route->GetGUID().empty()) return false;
  int identities = 0;
  for (auto *node = pRouteList->GetFirst(); node; node = node->GetNext()) {
    const auto *candidate = node->GetData();
    if (candidate && candidate->GetGUID() == route->GetGUID()) ++identities;
  }
  return identities == 1;
}
bool InOtherRoute(::Route &route, RoutePoint *point) {
  for (auto *node = pRouteList->GetFirst(); node; node = node->GetNext()) {
    auto *other = node->GetData();
    if (other && other != &route && other->pRoutePointList->Find(point))
      return true;
  }
  return false;
}
application::NavigationNameSuggestion SuggestDraftRouteName(ChartCanvas &canvas) {
  auto *route = canvas.m_pMouseRoute;
  if (!canvas.m_routeState || !RegisteredDraft(route) || route->GetnPoints() < 1)
    return {"Route", false};
  if (!route->m_RouteNameString.empty())
    return {route->m_RouteNameString.ToStdString(wxConvUTF8), false};
  const auto endpoint = [&canvas, route](RoutePoint *point)
      -> application::NavigationNameSuggestion {
    if (!point) return {};
    const auto own = point->GetName().Trim(true).Trim(false).ToStdString(wxConvUTF8);
    // A reused mark's numeric name can be intentional. Only the new route's
    // generated defaults need a fresh chart lookup.
    if (application::ValidNavigationName(own) &&
        (point->IsShared() || InOtherRoute(*route, point) ||
         !application::GeneratedNavigationName(own)))
      return {own, false};
    return CopyNavigationNameSuggestion(canvas, {point->m_lat, point->m_lon}, false);
  };
  const auto first = endpoint(route->GetPoint(1));
  const auto last = endpoint(route->GetPoint(route->GetnPoints()));
  // Coordinates are useful defaults too, but never claim chart provenance.
  return application::SuggestRouteName(first.name, last.name,
                                       first.from_chart, last.from_chart);
}
application::CommandResult SaveNewRouteDraft(MyFrame &frame, ChartCanvas &canvas,
                                             const std::string &name,
                                             const std::string &description) {
  if (!wxIsMainThread() || !OwnsCanvas(frame, canvas))
    return {false, "Route saving requires its application canvas"};
  if (!canvas.m_routeState || canvas.m_bAppendingRoute || !canvas.m_pMouseRoute)
    return {false, "No new route to save"};
  auto *route = canvas.m_pMouseRoute;
  if (!RegisteredDraft(route))
    return {false, "Route changed; review before saving"};
  if (route->GetnPoints() < 2 || route->IsActive() || route->m_bIsInLayer)
    return {false, "Add at least two route points before saving"};
  const auto identity = route->GetGUID();
  const auto title = wxString::FromUTF8(name), detail = wxString::FromUTF8(description);
  if (!application::ValidNavigationName(name) || description.size() > 2048 ||
      description.find('\0') != std::string::npos ||
      title.empty() || (!description.empty() && detail.empty()))
    return {false, "Enter a route name of 1–128 bytes"};
  const auto old_name = route->m_RouteNameString, old_description = route->m_RouteDescription;
  struct PointName {
    RoutePoint *point;
    wxString name;
    bool shown;
  };
  std::vector<PointName> renamed_points;
  std::set<RoutePoint *> seen;
  for (int i = 1; i <= route->GetnPoints(); ++i) {
    auto *point = route->GetPoint(i);
    if (!point || !seen.insert(point).second || point->m_bIsInLayer ||
        point->m_bIsolatedMark || point->m_bRPIsBeingEdited ||
        !application::ShouldNameNewRoutePoint(
            point->GetName().ToStdString(wxConvUTF8), point->IsShared(),
            InOtherRoute(*route, point))) continue;
    const auto suggested = CopyNavigationNameSuggestion(
        canvas, {point->m_lat, point->m_lon}, false);
    if (!application::ValidNavigationName(suggested.name)) continue;
    renamed_points.push_back({point, point->GetName(), point->IsNameShown()});
    point->SetName(wxString::FromUTF8(suggested.name));
    // Both pinned mouse/touch creation paths hide new numbered names.
    // SetName changes only text/extents, so show the assigned place name too.
    point->SetNameShown(true);
  }
  route->m_RouteNameString = title;
  route->m_RouteDescription = detail;
  // The native operation persists the route and each point's name/visibility.
  // Check its result before FinishRoute destroys the editable creation state.
  if (!NavObj_dB::GetInstance().UpdateRoute(route)) {
    for (const auto &previous : renamed_points) {
      previous.point->SetName(previous.name);
      previous.point->SetNameShown(previous.shown);
    }
    route->m_RouteNameString = old_name;
    route->m_RouteDescription = old_description;
    return {false, "Route could not be saved; draft retained"};
  }
  return {true, "Route saved", identity.ToStdString(wxConvUTF8)};
}
// A named save has already prepared this exact draft. This synchronous scope
// lets the central native gate avoid a second lookup/rename. No object pointer
// escapes integration or survives the immediate FinishRoute call.
struct PreparedRouteFinish;
const PreparedRouteFinish *prepared_finish = nullptr;
struct PreparedRouteFinish {
  ChartCanvas &canvas;
  wxString identity;
  const PreparedRouteFinish *previous;
  PreparedRouteFinish(ChartCanvas &source, const wxString &id)
      : canvas(source), identity(id), previous(prepared_finish) {
    prepared_finish = this;
  }
  ~PreparedRouteFinish() { prepared_finish = previous; }
};
} // namespace
application::CommandResult PrepareNativeRouteFinish(MyFrame &frame,
                                                    ChartCanvas &canvas) {
  if (!wxIsMainThread() || !OwnsCanvas(frame, canvas))
    return {false, "Route saving requires its application canvas"};
  auto *route = canvas.m_pMouseRoute;
  // Null drafts are the explicit cancellation path. Native append and short
  // draft cleanup retain their existing behavior.
  if (!canvas.m_routeState || canvas.m_bAppendingRoute || !route)
    return {true, {}};
  if (!RegisteredDraft(route))
    return {false, "Route changed; review before saving"};
  if (route->GetnPoints() < 2) return {true, {}};
  if (prepared_finish && &prepared_finish->canvas == &canvas &&
      prepared_finish->identity == route->GetGUID()) return {true, {}};
  return SaveNewRouteDraft(frame, canvas, SuggestDraftRouteName(canvas).name,
                           route->m_RouteDescription.ToStdString(wxConvUTF8));
}
application::NavigationActions
MakeNavigationActions(MyFrame &frame,
                      std::function<vessel::Navigation()> position,
                      std::function<application::AnchorState(vessel::Time)> anchor) {
  // Only XNav constructs these actions. Leave Legacy's route, mark and track
  // selection radius unchanged while making the thin route ink easier to hit.
  if (pSelect) pSelect->SetRouteSegmentSelectionScale(1.5f);
  application::NavigationActions a;
  ConfigureTrackRecording(position);
  auto result = [&frame](application::CommandResult r) {
    if (!wxIsMainThread()) return r;
    frame.InvalidateAllGL();
    frame.RefreshAllCanvas(false);
    if (pRouteManagerDialog) {
      pRouteManagerDialog->UpdateRouteListCtrl();
      pRouteManagerDialog->UpdateWptListCtrl();
      pRouteManagerDialog->UpdateTrkListCtrl();
    }
    wxLogMessage("SKAGER navigation action: %s",
                 wxString::FromUTF8(r.message));
    return r;
  };
  a.track_recording = CopyTrackRecording;
  a.tracks = CopyTrackCatalog;
  a.track = CopyTrack;
  a.start_track = [&frame, result] { return result(StartTrack(frame)); };
  a.stop_track = [&frame, result](const std::string &id) {
    return result(StopTrack(frame, id));
  };
  a.rename_track = [result](const application::TrackSummary &track,
                            const std::string &name) {
    return result(RenameTrack(track, name));
  };
  a.set_track_visible = [result](const application::TrackSummary &track, bool show) {
    return result(SetTrackVisible(track, show));
  };
  a.view_track = [&frame, result](const std::string &id) {
    return result(ViewTrack(frame, id));
  };
  // XNav presentation layers report their actual display preferences. Passage
  // visibility is independent of arrival policy and chart hazard assessment.
  // Forecast status remains credential-free, separate from measured wind.
  const auto with_wind = [&frame] {
    auto state = CopyChartPresentation(frame);
    if (state.available) {
      state.wind_vectors = {WeatherOverlayVisible(), true, WeatherLayerReason()};
      state.route_corridor = {PassageVisible(), true,
                             "Active passage; width set in route settings"};
      state.tracks = {TrackLayerVisible(), true, "Display recorded and saved tracks"};
    }
    return state;
  };
  a.chart_presentation = with_wind;
  a.set_chart_tracks = [with_wind, result](bool show) {
    application::ChartPresentationResult r;
    r.command = result(SetTrackLayerVisible(show));
    r.state = with_wind();
    return r;
  };
  a.set_chart_ais = [&frame, with_wind](bool show) {
    auto r = SetChartAis(frame, show); r.state = with_wind(); return r;
  };
  a.set_chart_enc_text = [&frame, with_wind](bool show) {
    auto r = SetChartEncText(frame, show); r.state = with_wind(); return r;
  };
  a.set_chart_soundings = [&frame, with_wind](bool show) {
    auto r = SetChartSoundings(frame, show); r.state = with_wind(); return r;
  };
  a.set_chart_wind = [&frame, with_wind](bool show) {
    application::ChartPresentationResult r;
    if (!wxIsMainThread()) {
      r.command = {false, "Chart presentation requires the application thread"};
      return r;
    }
    SetWeatherOverlayVisible(show);
    frame.InvalidateAllGL();
    frame.RefreshAllCanvas(false);
    r.state = with_wind();
    const bool ok = r.state.wind_vectors.visible == show;
    r.command = {ok, ok ? "Chart presentation applied"
                        : "Chart presentation changed; inspect current state"};
    return r;
  };
  a.set_chart_corridor = [&frame, with_wind](bool show) {
    application::ChartPresentationResult r;
    r.command = SetPassageVisible(show);
    if (r.command.ok) {
      frame.InvalidateAllGL();
      frame.RefreshAllCanvas(false);
    }
    r.state = with_wind();
    return r;
  };
  a.weather_time = [] { return WeatherDisplayTime(); };
  a.set_weather_time = [&frame](std::optional<std::chrono::system_clock::time_point> t) {
    SetWeatherDisplayTime(t);
    if (WeatherOverlayVisible()) {
      frame.InvalidateAllGL();
      frame.RefreshAllCanvas(false);
    }
  };
  a.set_chart_orientation = [&frame, with_wind](application::ChartOrientation mode) {
    auto r = SetChartOrientation(frame, mode); r.state = with_wind(); return r;
  };
  a.catalog = CopyNavigationCatalog;
  a.route = CopyNavigationRoute;
  a.waypoint_context = [position](const std::string &id, vessel::Time now) {
    return CopyWaypointContext(id, position(), now);
  };
  a.ais = [position](vessel::Time now) { return CopyAisState(position(), now); };
  a.anchor = std::move(anchor);
  a.anchor_watches = CopyAnchorWatchSelection;
  a.suggest_waypoint_name = [&frame](application::Coordinate position) {
    return CopyNavigationNameSuggestion(frame, position, false);
  };
  a.suggest_route_name = [&frame] {
    if (!wxIsMainThread()) return application::NavigationNameSuggestion{"Route", false};
    auto *canvas = frame.GetPrimaryCanvas();
    return canvas ? SuggestDraftRouteName(*canvas)
                  : application::NavigationNameSuggestion{"Route", false};
  };
  a.view_ais = [&frame, position](int mmsi) {
    const auto now = vessel::Clock::now();
    const auto copied = CopyAisState(position(), now);
    vessel::AisSelection check;
    if (!check.Select(mmsi, copied, now))
      return application::CommandResult{false, "Target position unavailable, ambiguous or stale"};
    for (const auto &t : copied.targets)
      if (t.mmsi == mmsi) {
        auto *canvas = frame.GetPrimaryCanvas();
        if (!canvas) break;
        if (!canvas->GetShowAIS()) frame.ToggleAISDisplay(canvas);
        frame.JumpToPosition(canvas, *t.latitude_deg.value, *t.longitude_deg.value, canvas->GetVPScale());
        frame.InvalidateAllGL(); frame.RefreshAllCanvas(false);
        return application::CommandResult{true, "Selected existing OpenCPN AIS target"};
      }
    return application::CommandResult{false, "Chart canvas unavailable"};
  };
  a.activate = [position, result](const auto &r) {
    return result(ActivateRoute(r, position()));
  };
  a.activate_after_anchor = [position, result](const auto &r, const auto &watch) {
    return result(ActivateRouteAfterAnchor(r, position(), watch));
  };
  a.deactivate = [result](const auto &r) { return result(StopRoute(r)); };
  a.reverse = [result](const auto &r) { return result(ReverseRoute(r)); };
  a.edit_route = [result](const auto &r, const auto &name,
                          const auto &description) {
    return result(EditRoute(r, name, description));
  };
  a.set_route_passage = [result](const auto &r, const auto &passage) {
    return result(EditRoutePassage(r, passage));
  };
  a.edit_waypoint = [result](const auto &p, const auto &name,
                             const auto &description) {
    return result(EditWaypoint(p, name, description));
  };
  a.delete_waypoint = [result](const auto &p) {
    return result(DeleteWaypoint(p));
  };
  a.delete_route = [result](const auto &r) { return result(DeleteRoute(r)); };
  a.create_waypoint = [result](const auto &p, const auto &name,
                               const auto &description) {
    return result(CreateWaypoint(p, name, description));
  };
  a.go_to = [position, result](const auto &p, const auto &name) {
    return result(GoTo(p, name, position()));
  };
  a.go_to_waypoint = [position, result](const auto &p) {
    return result(GoToWaypoint(p, position()));
  };
  a.chart_position = [&frame]() -> std::optional<application::Coordinate> {
    auto *cc = frame.GetPrimaryCanvas();
    if (!cc)
      return {};
    const auto &vp = cc->GetVP();
    if (!std::isfinite(vp.clat) || !std::isfinite(vp.clon))
      return {};
    double lon = vp.clon;
    while (lon > 180)
      lon -= 360;
    while (lon < -180)
      lon += 360;
    return application::Coordinate{vp.clat, lon};
  };
  a.view_waypoint = [&frame](const std::string &id) {
    // Resolve exactly one registered identity using the same copied contract
    // as the card. Chart centering needs valid mark geometry, not a GPS fix.
    const auto context = CopyWaypointContext(id, {}, vessel::Clock::now());
    const auto &p = context.waypoint;
    if (!p || !std::isfinite(p->latitude_deg) || !std::isfinite(p->longitude_deg) ||
        std::abs(p->latitude_deg) > 90 || std::abs(p->longitude_deg) > 180)
      return application::CommandResult{false, "Waypoint position unavailable"};
    auto *canvas = frame.GetPrimaryCanvas();
    if (!canvas) return application::CommandResult{false, "Chart canvas unavailable"};
    frame.JumpToPosition(canvas, p->latitude_deg, p->longitude_deg, canvas->GetVPScale());
    return application::CommandResult{true, "Waypoint shown on chart"};
  };
  a.view_route = [&frame](const std::string &id) {
    if (!pRouteList)
      return;
    for (auto *n = pRouteList->GetFirst(); n; n = n->GetNext()) {
      auto *route = n->GetData();
      if (route && route->GetGUID() == wxString::FromUTF8(id)) {
        auto *p = route->GetPoint(1);
        if (p) {
          route->SetVisible(true);
          NavObj_dB::GetInstance().UpdateRoute(route);
          frame.JumpToPosition(frame.GetPrimaryCanvas(), p->m_lat, p->m_lon,
                               0.01);
        }
        break;
      }
    }
  };
  a.start_route = [&frame] {
    auto *cc = frame.GetPrimaryCanvas();
    cc->StartRoute();
    // Explicit Finish/Cancel governs this touch workflow. The stock focus-loss
    // behavior would finish before a user could press Undo or name the route.
    cc->m_FinishRouteOnKillFocus = false;
    cc->SetFocus();
  };
  a.finish_route = [&frame] {
    auto *cc = frame.GetPrimaryCanvas();
    cc->FinishRoute();
    if (!cc->m_routeState) cc->m_FinishRouteOnKillFocus = true;
  };
  a.can_finish_route = [&frame] {
    if (!wxIsMainThread()) return false;
    auto *cc = frame.GetPrimaryCanvas();
    if (!cc || !OwnsCanvas(frame, *cc) || !cc->m_routeState ||
        cc->m_bAppendingRoute || !RegisteredDraft(cc->m_pMouseRoute))
      return false;
    auto *route = cc->m_pMouseRoute;
    // Match SaveNewRouteDraft's route checks without changing navigation,
    // names, persistence or focus. The save operation checks again on dispatch.
    return route->GetnPoints() >= 2 && !route->IsActive() && !route->m_bIsInLayer;
  };
  a.finish_route_named = [&frame, result](const std::string &name,
                                          const std::string &description) {
    if (!wxIsMainThread())
      return application::CommandResult{false, "Route saving requires the application thread"};
    auto *cc = frame.GetPrimaryCanvas();
    if (!cc) return application::CommandResult{false, "Chart canvas unavailable"};
    const auto saved = SaveNewRouteDraft(frame, *cc, name, description);
    if (!saved.ok) return saved;
    const PreparedRouteFinish prepared(*cc, wxString::FromUTF8(saved.identity));
    cc->FinishRoute();
    if (cc->m_routeState || cc->m_pMouseRoute)
      return application::CommandResult{false,
          "Route saved, but completion was withheld; review the current draft", saved.identity};
    cc->m_FinishRouteOnKillFocus = true;
    return result(saved);
  };
  a.can_undo_route_point = [&frame] {
    auto *cc = frame.GetPrimaryCanvas();
    if (!cc || cc->m_routeState <= 1 || !cc->m_pMouseRoute ||
        cc->m_pMouseRoute->GetnPoints() < 2 || !cc->undo ||
        cc->undo->InUndoableAction() || !cc->undo->AnythingToUndo())
      return false;
    const auto *next = cc->undo->GetNextUndoableAction();
    if (!next || next->type != Undo_AppendWaypoint || next->after.empty() ||
        next->after.front() != cc->m_pMouseRoute)
      return false;
    return true;
  };
  a.undo_route_point = [&frame, result, available = a.can_undo_route_point] {
    if (!available())
      return application::CommandResult{false, "No current route point to undo"};
    auto *cc = frame.GetPrimaryCanvas();
    const bool undone = cc->undo->UndoLastAction();
    cc->SetFocus();
    return result({undone, undone ? "Last route point removed" : "Could not undo point"});
  };
  a.cancel_route = [&frame, result] {
    auto *cc = frame.GetPrimaryCanvas();
    if (!cc || !g_pRouteMan || cc->m_routeState == 0 || cc->m_bAppendingRoute)
      return application::CommandResult{false, "No new route to cancel"};
    auto *route = cc->m_pMouseRoute;
    if (route && (route->IsActive() || route->m_bIsInLayer))
      return application::CommandResult{false, "Protected route cannot be cancelled"};
    // Finish resets upstream cursor/creation/toolbar/undo state. Prevent its
    // save path, then ask the normal route manager to remove only this draft.
    cc->m_pMouseRoute = nullptr;
    cc->FinishRoute();
    cc->m_FinishRouteOnKillFocus = true;
    if (route) g_pRouteMan->DeleteRoute(route);
    return result({true, "Route creation cancelled"});
  };
  a.chart_tools = [&frame] {
    application::ChartToolState state;
    if (!wxIsMainThread()) return state;
    auto *cc = frame.GetPrimaryCanvas();
    if (!cc) return state;
    state.available = true;
    state.measure_active = cc->IsMeasureActive();
    state.awaiting_first_measure_point = state.measure_active && cc->m_nMeasureState == 1;
    const bool editing = cc->m_routeState || cc->isRouteEditing() || cc->isMarkEditing();
    state.can_measure = !editing;
    state.can_place_waypoint = !editing && !state.measure_active;
    return state;
  };
  a.cancel_measure = [&frame] {
    if (!wxIsMainThread()) return;
    auto *cc = frame.GetPrimaryCanvas();
    if (!cc || !cc->IsMeasureActive()) return;
    cc->CancelMeasureRoute();
    cc->InvalidateGL();
    cc->Refresh(false);
  };
  a.measure = [&frame] {
    if (!wxIsMainThread()) return;
    auto *cc = frame.GetPrimaryCanvas();
    if (!cc || cc->m_routeState || cc->isRouteEditing() || cc->isMarkEditing()) return;
    if (cc->IsMeasureActive())
      cc->CancelMeasureRoute();
    else
      cc->StartMeasureRoute();
    cc->InvalidateGL();
    cc->Refresh(false);
    cc->SetFocus();
  };
  a.object_info = [&frame] {
    auto *cc = frame.GetPrimaryCanvas();
    const auto &vp = cc->GetVP();
    const auto size = cc->GetClientSize();
    cc->ShowObjectQueryWindow(size.x / 2, size.y / 2, vp.clat, vp.clon);
  };
  a.object_info_at = [&frame](application::Coordinate p) {
    auto *cc = frame.GetPrimaryCanvas();
    if (!cc || !std::isfinite(p.latitude_deg) || !std::isfinite(p.longitude_deg) ||
        std::abs(p.latitude_deg) > 90 || std::abs(p.longitude_deg) > 180) return;
    wxPoint pixel;
    if (!cc->GetCanvasPointPix(p.latitude_deg, p.longitude_deg, &pixel)) return;
    cc->ShowObjectQueryWindow(pixel.x, pixel.y, p.latitude_deg, p.longitude_deg);
  };
  a.orientation = [&frame] {
    auto *cc = frame.GetPrimaryCanvas();
    frame.SetUpMode(cc, cc->GetUpMode() == NORTH_UP_MODE ? COURSE_UP_MODE
                                                         : NORTH_UP_MODE);
  };
  a.toggle_ais = [&frame] { frame.ToggleAISDisplay(frame.GetPrimaryCanvas()); };
  a.fullscreen = [&frame] { frame.ToggleFullScreen(); };
  a.alarm_preferences = [] {
    application::AlarmPreferences state;
    if (!wxIsMainThread()) return state;
    const auto number = [](double value) -> std::optional<double> {
      return std::isfinite(value) && value >= 0 ? std::make_optional(value)
                                               : std::nullopt;
    };
    state.available = true;
    state.cpa_enabled = g_bCPAWarn;
    state.cpa_nm = number(g_CPAWarn_NM);
    state.tcpa_enabled = g_bTCPA_Max;
    state.tcpa_min = number(g_TCPA_Max);
    state.range_enabled = g_bCPAMax;
    state.range_nm = number(g_CPAMax_NM);
    state.target_dialog_enabled = g_bAIS_CPA_Alert;
    // Both upstream options suppress low-speed/moored CPA alarms, even when
    // only the hide-moored display option was selected.
    state.moored_suppressed = g_bAIS_CPA_Alert_Suppress_Moored || g_bHideMoored;
    return state;
  };
  a.legacy_settings = [&frame] { frame.DoSettings(); };
  a.connections_settings = [&frame] {
    // Pinned OpenCPN options::CreateControls creates Display, Charts, then
    // Connections. Open the actual connection editor, not the last-used tab.
    // DoOptionsDialog consumes this page before showing its modeless dialog.
    // Restore the previous preference now; OpenCPN may later remember the
    // page the user actually leaves selected when closing its own dialog.
    const int previous_page = options_lastPage;
    const int previous_subpage = options_subpage;
    options_lastPage = 2;
    options_subpage = -1;
    frame.DoSettings();
    options_lastPage = previous_page;
    options_subpage = previous_subpage;
  };
  const auto alarm_settings = [&frame](int page) {
    if (!wxIsMainThread()) return;
    // Pinned options::CreateControls: Ships/AIS Targets = (3,1),
    // User Interface/Sounds = (4,1). Restore the prior initial-page preference
    // after opening; the user's eventual selection remains owned by OpenCPN.
    struct RestoreOptions {
      int page, subpage;
      ~RestoreOptions() {
        options_lastPage = page;
        options_subpage = subpage;
      }
    } restore{options_lastPage, options_subpage};
    options_lastPage = page;
    options_subpage = 1;
    frame.DoSettings();
  };
  a.ais_alarm_settings = [alarm_settings] { alarm_settings(3); };
  a.alarm_sound_settings = [alarm_settings] { alarm_settings(4); };
  a.plugin_settings = [&frame] {
    // Pinned options::CreateControls creates Display, Charts, Connections,
    // Ships, User Interface, then Plugins. Plugin-added pages follow these.
    // Reuse upstream's own initial-page state, without owning its dialog.
    options_lastPage = 5;
    options_subpage = -1;
    frame.DoSettings();
  };
  a.legacy_route_manager = [&frame] {
    pRouteManagerDialog = RouteManagerDialog::getInstance(&frame);
    pRouteManagerDialog->UpdateLists();
    pRouteManagerDialog->Show();
    pRouteManagerDialog->Raise();
  };
  a.start_anchor = [position, result](double radius) {
    return result(StartAnchor(position(), radius));
  };
  a.clear_anchor = [result](const auto &id) { return result(ClearAnchor(id)); };
  return a;
}
} // namespace opennav::integration

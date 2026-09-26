#include "integration/NavigationActions.h"
#include "chcanv.h"
#include "integration/NavigationObjects.h"
#include "model/navobj_db.h"
#include "model/route.h"
#include "model/route_point.h"
#include "model/routeman.h"
#include "ocpn_frame.h"
#include "routemanagerdialog.h"
#include "viewport.h"
#include "vessel/AisSelection.h"
#include "undo.h"
#include <cmath>
#include <wx/log.h>
extern RouteManagerDialog *pRouteManagerDialog;
extern int options_lastPage, options_subpage;
namespace opennav::integration {
application::NavigationActions
MakeNavigationActions(MyFrame &frame,
                      std::function<vessel::Navigation()> position,
                      std::function<application::AnchorState()> anchor) {
  application::NavigationActions a;
  auto result = [&frame](application::CommandResult r) {
    frame.InvalidateAllGL();
    frame.RefreshAllCanvas(false);
    if (pRouteManagerDialog) {
      pRouteManagerDialog->UpdateRouteListCtrl();
      pRouteManagerDialog->UpdateWptListCtrl();
    }
    wxLogMessage("OpenNav navigation action: %s",
                 wxString::FromUTF8(r.message));
    return r;
  };
  a.catalog = CopyNavigationCatalog;
  a.ais = [position](vessel::Time now) { return CopyAisState(position(), now); };
  a.anchor = std::move(anchor);
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
  a.deactivate = [result] { return result(StopRoute()); };
  a.reverse = [result](const auto &r) { return result(ReverseRoute(r)); };
  a.edit_route = [result](const auto &r, const auto &name,
                          const auto &description) {
    return result(EditRoute(r, name, description));
  };
  a.edit_waypoint = [result](const auto &p, const auto &name,
                             const auto &description) {
    return result(EditWaypoint(p, name, description));
  };
  a.delete_waypoint = [result](const auto &p) {
    return result(DeleteWaypoint(p));
  };
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
    auto *p = pWayPointMan ? pWayPointMan->FindWaypointByGuid(id) : nullptr;
    if (p)
      frame.JumpToPosition(frame.GetPrimaryCanvas(), p->m_lat, p->m_lon,
                           frame.GetPrimaryCanvas()->GetVPScale());
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
    cc->m_FinishRouteOnKillFocus = true;
  };
  a.finish_route_named = [&frame, result](const std::string &name,
                                          const std::string &description) {
    auto *cc = frame.GetPrimaryCanvas();
    if (!cc || cc->m_routeState == 0 || cc->m_bAppendingRoute ||
        !cc->m_pMouseRoute || !pRouteList)
      return application::CommandResult{false, "No new route to save"};
    auto *route = cc->m_pMouseRoute;
    if (route->GetnPoints() < 2 || route->IsActive() || route->m_bIsInLayer)
      return application::CommandResult{false, "Add at least two route points before saving"};
    const auto identity = route->GetGUID();
    int identities = 0;
    bool registered = false;
    for (auto *node = pRouteList->GetFirst(); node; node = node->GetNext()) {
      const auto *candidate = node->GetData();
      registered |= candidate == route;
      if (candidate && candidate->GetGUID() == identity) ++identities;
    }
    if (!registered || identity.empty() || identities != 1)
      return application::CommandResult{false, "Route changed; review before saving"};
    const auto title = wxString::FromUTF8(name), detail = wxString::FromUTF8(description);
    if (name.empty() || name.size() > 128 || description.size() > 2048 ||
        name.find('\0') != std::string::npos || description.find('\0') != std::string::npos ||
        title.empty() || (!description.empty() && detail.empty()))
      return application::CommandResult{false, "Enter a route name of 1–128 bytes"};
    const auto old_name = route->m_RouteNameString, old_description = route->m_RouteDescription;
    route->m_RouteNameString = title;
    route->m_RouteDescription = detail;
    // FinishRoute uses this same native database operation but discards its
    // result. Check persistence first, retaining the editable draft on failure.
    if (!NavObj_dB::GetInstance().UpdateRoute(route)) {
      route->m_RouteNameString = old_name;
      route->m_RouteDescription = old_description;
      return application::CommandResult{false, "Route could not be saved; draft retained"};
    }
    cc->FinishRoute();
    cc->m_FinishRouteOnKillFocus = true;
    return result({true, "Route saved", identity.ToStdString(wxConvUTF8)});
  };
  a.undo_route_point = [&frame, result] {
    auto *cc = frame.GetPrimaryCanvas();
    if (!cc || cc->m_routeState <= 1 || !cc->m_pMouseRoute ||
        cc->m_pMouseRoute->GetnPoints() < 2 || !cc->undo ||
        cc->undo->InUndoableAction() || !cc->undo->AnythingToUndo())
      return application::CommandResult{false, "No route point to undo"};
    const auto *next = cc->undo->GetNextUndoableAction();
    if (!next || next->type != Undo_AppendWaypoint || next->after.empty() ||
        next->after.front() != cc->m_pMouseRoute)
      return application::CommandResult{false, "No current route point to undo"};
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
  a.measure = [&frame] {
    auto *cc = frame.GetPrimaryCanvas();
    if (cc->IsMeasureActive())
      cc->CancelMeasureRoute();
    else
      cc->StartMeasureRoute();
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
  a.legacy_settings = [&frame] { frame.DoSettings(); };
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

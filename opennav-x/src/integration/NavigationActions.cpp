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
#include <cmath>
#include <wx/log.h>
extern RouteManagerDialog *pRouteManagerDialog;
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
  a.ais = [position] { return CopyAisState(position(), vessel::Clock::now()); };
  a.anchor = std::move(anchor);
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
  a.start_route = [&frame] { frame.GetPrimaryCanvas()->StartRoute(); };
  a.finish_route = [&frame] { frame.GetPrimaryCanvas()->FinishRoute(); };
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
  a.orientation = [&frame] {
    auto *cc = frame.GetPrimaryCanvas();
    frame.SetUpMode(cc, cc->GetUpMode() == NORTH_UP_MODE ? COURSE_UP_MODE
                                                         : NORTH_UP_MODE);
  };
  a.toggle_ais = [&frame] { frame.ToggleAISDisplay(frame.GetPrimaryCanvas()); };
  a.fullscreen = [&frame] { frame.ToggleFullScreen(); };
  a.legacy_settings = [&frame] { frame.DoSettings(); };
  a.plugin_settings = a.legacy_settings;
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

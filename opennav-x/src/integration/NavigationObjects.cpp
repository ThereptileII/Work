#include "integration/NavigationObjects.h"
#include "application/AnchorRouteTransition.h"
#include "application/NavigationNaming.h"
#include "integration/AnchorGeometry.h"
#include "integration/AisObservationTime.h"
#include "MarkInfo.h"
#include "RoutePropDlgImpl.h"
#include "chcanv.h"
#include "chartdb.h"
#include "s52plib.h"
#include "s57chart.h"
#include <memory>
#include "model/ais_decoder.h"
#include "model/ais_target_data.h"
#include "model/georef.h"
#include "model/navobj_db.h"
#include "model/navutil_base.h"
#include "model/own_ship.h"
#include "model/plugin_comm.h"
#include "model/route.h"
#include "model/route_point.h"
#include "model/routeman.h"
#include "model/select.h"
#include "ocpn_frame.h"
#include "undo.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <wx/thread.h>

extern bool bGPSValid;
extern ChartDB *ChartData;
extern s52plib *ps52plib;
extern MarkInfoDlg *g_pMarkInfoDialog;
extern MyFrame *gFrame;
extern RoutePropDlgImpl *pRoutePropDialog;
extern wxString g_default_wp_icon, g_AW1GUID, g_AW2GUID;
extern RoutePoint *pAnchorWatchPoint1, *pAnchorWatchPoint2;
extern double AnchorPointMinDist;
extern int g_nAWMax;
extern bool AnchorAlertOn1, AnchorAlertOn2;

namespace opennav::integration {
namespace {
using application::CommandResult;
void Thread() {
  if (!wxIsMainThread())
    throw std::logic_error(
        "OpenCPN object access requires the application thread");
}
std::string String(const wxString &s) { return s.ToStdString(wxConvUTF8); }
std::string Revision(const application::Waypoint &w) {
  std::ostringstream s;
  s.imbue(std::locale::classic());
  s << std::quoted(w.id) << std::quoted(w.name) << std::quoted(w.description)
    << std::setprecision(17) << w.latitude_deg << ',' << w.longitude_deg << ','
    << w.editable << w.removable << w.in_route;
  return s.str();
}
application::Waypoint Copy(RoutePoint *point) {
  application::Waypoint w;
  w.id = String(point->m_GUID);
  w.name = String(point->GetName());
  w.description = String(point->GetDescription());
  w.latitude_deg = point->m_lat;
  w.longitude_deg = point->m_lon;
  w.in_route = point->m_bIsInRoute;
  w.editable = !point->m_bIsInLayer && !point->m_bRPIsBeingEdited &&
               point->GetIconName() != "mob" && point != pAnchorWatchPoint1 &&
               point != pAnchorWatchPoint2;
  if (g_pRouteMan && g_pRouteMan->GetpActiveRoute() &&
      g_pRouteMan->GetpActiveRoute()->GetIndexOf(point) > 0)
    w.editable = false;
  if (g_pMarkInfoDialog && g_pMarkInfoDialog->IsShown() &&
      g_pMarkInfoDialog->GetRoutePoint() == point)
    w.editable = false;
  if (pRouteList)
    for (auto *n = pRouteList->GetFirst(); n; n = n->GetNext()) {
      auto *route = n->GetData();
      if (route && route->GetIndexOf(point) > 0) {
        w.in_route = true;
        if (route->IsActive() || route->m_bIsInLayer || route->m_bIsBeingEdited)
          w.editable = false;
      }
    }
  if (!std::isfinite(w.latitude_deg) || !std::isfinite(w.longitude_deg) ||
      std::abs(w.latitude_deg) > 90 || std::abs(w.longitude_deg) > 180 ||
      w.id.empty())
    w.editable = false;
  w.removable = w.editable && !w.in_route;
  w.revision = Revision(w);
  return w;
}
application::Route Copy(::Route *route) {
  application::Route r;
  r.id = String(route->GetGUID());
  r.name = String(route->m_RouteNameString);
  r.description = String(route->m_RouteDescription);
  r.active = route->IsActive();
  r.visible = route->IsVisible();
  r.editable = !r.id.empty() && !r.active && !route->m_bIsInLayer &&
               !route->m_bIsBeingCreated && !route->m_bIsBeingEdited;
  if (pRoutePropDialog && pRoutePropDialog->IsShown() &&
      pRoutePropDialog->GetRoute() == route)
    r.editable = false;
  std::ostringstream revision;
  revision.imbue(std::locale::classic());
  revision << std::quoted(r.id) << std::quoted(r.name)
           << std::quoted(r.description) << r.active << r.visible << r.editable;
  std::set<std::string> ids;
  for (auto *node = route->pRoutePointList->GetFirst(); node;
       node = node->GetNext()) {
    auto *point = node->GetData();
    if (!point) {
      r.editable = false;
      continue;
    }
    auto w = Copy(point);
    if (!std::isfinite(w.latitude_deg) || !std::isfinite(w.longitude_deg) ||
        std::abs(w.latitude_deg) > 90 || std::abs(w.longitude_deg) > 180)
      r.editable = false;
    if (w.id.empty() || !ids.insert(w.id).second || point->m_bRPIsBeingEdited)
      r.editable = false;
    if (!r.points.empty()) {
      if (std::isfinite(point->m_seg_len) && point->m_seg_len >= 0)
        w.incoming_nm = point->m_seg_len;
      const double c = point->GetCourse();
      if (std::isfinite(c) && c >= 0 && c <= 360)
        w.incoming_course_true_deg = c == 360 ? 0 : c;
    }
    if (!r.points.empty() && (!w.incoming_nm || !w.incoming_course_true_deg))
      r.editable = false;
    revision << std::quoted(w.revision);
    // The first point has no incoming leg; upstream does not initialize its
    // route-properties course. Only copy stored courses for subsequent legs.
    if (!r.points.empty()) {
      if (w.incoming_nm && w.incoming_course_true_deg)
        revision << std::setprecision(17) << *w.incoming_nm << ","
                 << *w.incoming_course_true_deg;
      else
        revision << "invalid leg";
    }
    r.points.push_back(std::move(w));
  }
  revision << r.editable;
  r.revision = revision.str();
  return r;
}
::Route *Resolve(const application::Route &selected) {
  if (!g_pRouteMan || !pRouteList)
    return nullptr;
  ::Route *match = nullptr;
  for (auto *n = pRouteList->GetFirst(); n; n = n->GetNext()) {
    auto *r = n->GetData();
    if (r && String(r->GetGUID()) == selected.id) {
      if (match)
        return nullptr;
      match = r;
    }
  }
  return match && Copy(match).revision == selected.revision ? match : nullptr;
}
RoutePoint *Resolve(const application::Waypoint &selected) {
  if (!pWayPointMan)
    return nullptr;
  RoutePoint *match = nullptr;
  for (auto *n = pWayPointMan->GetWaypointList()->GetFirst(); n;
       n = n->GetNext()) {
    auto *p = n->GetData();
    if (p && String(p->m_GUID) == selected.id) {
      if (match)
        return nullptr;
      match = p;
    }
  }
  return match && Copy(match).revision == selected.revision ? match : nullptr;
}
bool TextValid(const std::string &name, const std::string &description) {
  return application::ValidNavigationName(name) && description.size() <= 2048 &&
         description.find('\0') == std::string::npos;
}
bool Position(const vessel::Navigation &n, vessel::Time now) {
  const auto &lat = n.latitude_deg;
  const auto &lon = n.longitude_deg;
  const auto a = vessel::Assess(lat, now), b = vessel::Assess(lon, now);
  return bGPSValid && lat.validity == vessel::Validity::Measured &&
         lon.validity == vessel::Validity::Measured && lat.value && lon.value &&
         lat.source == lon.source && lat.observed_at == lon.observed_at &&
         (a.quality == vessel::Quality::Live ||
          a.quality == vessel::Quality::Aging) &&
         (b.quality == vessel::Quality::Live ||
          b.quality == vessel::Quality::Aging) &&
         *lat.value == gLat && *lon.value == gLon &&
         std::abs(*lat.value) <= 90 && std::abs(*lon.value) <= 180;
}
CommandResult NavigateTo(RoutePoint *destination, bool existing) {
  // Match pinned canvasMenu.cpp ID_DEF_MENU_GOTO_HERE / ID_WP_MENU_GOTO:
  // OpenCPN owns the temporary two-point route and deletes it on arrival.
  auto *origin = new RoutePoint(gLat, gLon, g_default_wp_icon,
                                wxEmptyString, wxEmptyString);
  pSelect->AddSelectableRoutePoint(gLat, gLon, origin);
  auto *route = new ::Route;
  pRouteList->Append(route);
  route->AddPoint(origin);
  route->AddPoint(destination);
  if (existing) destination->SetShared(true);
  pSelect->AddSelectableRouteSegment(gLat, gLon, destination->m_lat,
                                     destination->m_lon, origin, destination,
                                     route);
  route->m_RouteNameString = "Go to " + destination->GetName();
  route->m_RouteStartString = "Here";
  route->m_RouteEndString = destination->GetName();
  route->m_bDeleteOnArrival = true;
  g_pRouteMan->ActivateRoute(route, destination);
  return {true, "Go To started", String(route->GetGUID())};
}
} // namespace
application::NavigationNameSuggestion CopyNavigationNameSuggestion(
    MyFrame &frame, application::Coordinate position, bool route) {
  Thread();
  const auto fallback = [&] { return application::SuggestNavigationName(position, route, {}); };
  auto *canvas = frame.GetPrimaryCanvas();
  if (!canvas || !ps52plib || !ChartData || !ChartData->IsValid() || ChartData->IsBusy() ||
      !std::isfinite(position.latitude_deg) || std::abs(position.latitude_deg) > 85 ||
      !std::isfinite(position.longitude_deg) || std::abs(position.longitude_deg) > 180)
    return fallback();
  std::set<ChartBase *> charts;
  if (!canvas->GetQuiltMode()) {
    if (canvas->m_singleChart) charts.insert(canvas->m_singleChart);
  } else {
    const auto indexes = canvas->GetQuiltIndexArray();
    if (indexes.size() > 32) return fallback();
    const auto *cache = ChartData->GetChartCache();
    if (!cache || cache->size() > 1000) return fallback();
    for (std::size_t i = 0; i < cache->size(); ++i) {
      const auto *entry = static_cast<CacheEntry *>(cache->Item(i));
      if (entry && entry->pChart && entry->b_in_use &&
          std::find(indexes.begin(), indexes.end(), entry->dbIndex) != indexes.end())
        charts.insert(static_cast<ChartBase *>(entry->pChart));
    }
  }
  std::vector<application::ChartNameCandidate> names;
  // GetFirstQuiltChart/GetNextQuiltChart can open charts. Use only existing
  // cache pointers above; never load charts or call plugin/network lookup.
  // The pinned native query owns its list, not the borrowed chart objects.
  const double radius = .5 / (60. * std::cos(position.latitude_deg * std::acos(-1.) / 180.));
  for (auto *base : charts) {
    auto *chart = dynamic_cast<s57chart *>(base);
    if (!chart) continue;
    std::unique_ptr<ListOfObjRazRules> objects(chart->GetObjRuleListAtLatLon(
        position.latitude_deg, position.longitude_deg, radius, &canvas->GetVP(), MASK_POINT));
    if (!objects) continue;
    if (objects->GetCount() > 1024) return fallback();
    for (auto *node = objects->GetFirst(); node; node = node->GetNext()) {
      const auto *rule = node->GetData();
      auto *object = rule ? rule->obj : nullptr;
      if (!object || object->Primitive_type != GEO_POINT || object->npt != 1 ||
          !application::RelevantChartNameFeature(std::string(object->FeatureName, 6))) continue;
      const auto attribute = [&](const char *key) {
        const int index = object->GetAttributeIndex(key);
        if (index < 0 || !object->attVal || static_cast<std::size_t>(index) >= object->attVal->size())
          return wxString{};
        const auto *value = object->attVal->Item(index);
        if (!value || value->valType != OGR_STR || !value->value) return wxString{};
        return object->GetAttrValueAsString(key).Trim(true).Trim(false);
      };
      auto name = attribute("OBJNAM");
      if (name.empty()) name = attribute("NOBJNM");
      if (name.empty() || !std::isfinite(object->m_lat) || !std::isfinite(object->m_lon) ||
          std::abs(object->m_lat) > 90 || std::abs(object->m_lon) > 180) continue;
      double distance = 0;
      DistanceBearingMercator(position.latitude_deg, position.longitude_deg,
                              object->m_lat, object->m_lon, nullptr, &distance);
      names.push_back({String(name), distance});
      if (names.size() > 1024) return fallback();
    }
  }
  return application::SuggestNavigationName(position, route, names);
}
application::WaypointContext CopyWaypointContext(
    const std::string &id, const vessel::Navigation &position, vessel::Time now) {
  Thread();
  application::WaypointContext result;
  result.observed_at = now;
  result.reason = "Waypoint unavailable";
  if (id.empty() || !pWayPointMan) return result;
  RoutePoint *selected = nullptr;
  for (auto *node = pWayPointMan->GetWaypointList()->GetFirst(); node;
       node = node->GetNext()) {
    auto *point = node->GetData();
    if (point && String(point->m_GUID) == id) {
      if (selected) { result.reason = "Waypoint identity ambiguous"; return result; }
      selected = point;
    }
  }
  if (!selected) return result;
  result.waypoint = Copy(selected);
  const auto &point = *result.waypoint;
  if (!std::isfinite(point.latitude_deg) || !std::isfinite(point.longitude_deg) ||
      std::abs(point.latitude_deg) > 90 || std::abs(point.longitude_deg) > 180) {
    result.reason = "Waypoint position invalid";
    return result;
  }
  if (!Position(position, now)) {
    result.reason = "Vessel position unavailable or stale";
    return result;
  }
  // Same direct rhumb-line range used by the pinned waypoint manager and
  // chart cursor. This is not remaining route distance or a route calculation.
  double bearing = NAN, distance = NAN;
  DistanceBearingMercator(point.latitude_deg, point.longitude_deg,
                          *position.latitude_deg.value,
                          *position.longitude_deg.value, &bearing, &distance);
  if (!std::isfinite(distance) || distance < 0 || !std::isfinite(bearing) ||
      bearing < 0 || bearing > 360) {
    result.reason = "Waypoint range unavailable";
    return result;
  }
  const auto source = "OpenCPN direct waypoint rhumb-line / " + position.latitude_deg.source;
  result.range_nm = {distance, source, position.latitude_deg.observed_at,
                     vessel::Validity::Estimated};
  result.bearing_true_deg = {bearing, source, position.latitude_deg.observed_at,
                             vessel::Validity::Estimated};
  result.range_nm.freshness = result.bearing_true_deg.freshness =
      position.latitude_deg.freshness;
  result.reason = "Direct range from vessel";
  return result;
}
application::Catalog CopyNavigationCatalog() {
  Thread();
  application::Catalog catalog;
  if (pRouteList)
    for (auto *node = pRouteList->GetFirst(); node; node = node->GetNext()) {
      if (catalog.routes.size() >= 1000) {
        catalog.truncated = true;
        break;
      }
      if (node->GetData())
        catalog.routes.push_back(Copy(node->GetData()));
    }
  if (pWayPointMan)
    for (auto *node = pWayPointMan->GetWaypointList()->GetFirst(); node;
         node = node->GetNext()) {
      if (catalog.waypoints.size() >= 10000) {
        catalog.truncated = true;
        break;
      }
      if (node->GetData())
        catalog.waypoints.push_back(Copy(node->GetData()));
    }
  return catalog;
}
std::optional<application::Route> CopyNavigationRoute(const std::string &id) {
  Thread();
  if (id.empty() || !pRouteList) return {};
  ::Route *match = nullptr;
  for (auto *node = pRouteList->GetFirst(); node; node = node->GetNext()) {
    auto *route = node->GetData();
    if (route && String(route->GetGUID()) == id) {
      if (match) return {};
      match = route;
    }
  }
  return match ? std::optional<application::Route>(Copy(match)) : std::nullopt;
}
application::AnchorWatchSelection CopyAnchorWatchSelection() {
  Thread();
  application::AnchorWatchSelection selection;
  selection.reason = "Anchor watch changed or unavailable; refresh selection";
  for (const auto &slot : {std::make_pair(pAnchorWatchPoint1, g_AW1GUID),
                           std::make_pair(pAnchorWatchPoint2, g_AW2GUID)}) {
    auto *point = slot.first;
    const auto id = String(slot.second);
    if (!point && id.empty()) continue;
    if (!point || id.empty() || !pWayPointMan) return selection;
    bool registered = false;
    int identities = 0;
    for (auto *node = pWayPointMan->GetWaypointList()->GetFirst(); node;
         node = node->GetNext()) {
      auto *candidate = node->GetData();
      registered |= candidate == point;
      if (candidate && String(candidate->m_GUID) == id) ++identities;
    }
    // Check membership before dereferencing a retained upstream pointer.
    if (!registered || identities != 1) return selection;
    auto copy = Copy(point);
    if (copy.id != id || copy.revision.empty() ||
        (!selection.watches.empty() && selection.watches.front().id == id))
      return selection;
    selection.watches.push_back(std::move(copy));
  }
  selection.available = true;
  selection.reason.clear();
  return selection;
}
namespace {
application::CommandResult ActivateRouteTransition(
    const application::Route &selected, const vessel::Navigation &position,
    const application::AnchorWatchSelection *confirmed) {
  Thread();
  auto *route = Resolve(selected);
  if (!g_pRouteMan || !route || !Copy(route).editable || route->GetnPoints() < 2)
    return {
        false,
        "Route changed, active, protected or being edited; refresh selection",
        {}};
  if (!Position(position, vessel::Clock::now()))
    return {false,
            "Fresh selected position is required to choose an activation point",
            {}};
  auto *best =
      g_pRouteMan->FindBestActivatePoint(route, gLat, gLon, gCog, gSog);
  const auto watches = CopyAnchorWatchSelection();
  return application::CommitRouteActivation(watches, confirmed, [&] {
    const bool visible = route->IsVisible();
    route->SetVisible(true);
    if (NavObj_dB::GetInstance().UpdateRoute(route)) return true;
    route->SetVisible(visible);
    return false;
  }, [&]() -> application::CommandResult {
    // Match the pinned chart activation path. Deactivation clears progress,
    // not the route or its marks; ordinary ActivateRoute does not do this.
    if (g_pRouteMan->GetpActiveRoute() && !g_pRouteMan->DeactivateRoute())
      return {false, "Could not stop current navigation; anchor watch retained"};
    // This transition preserves every mark, including SKAGER-created anchors.
    // Explicit ClearAnchor has a separate, confirmed temporary-mark lifecycle.
    pAnchorWatchPoint1 = pAnchorWatchPoint2 = nullptr;
    g_AW1GUID.Clear();
    g_AW2GUID.Clear();
    AnchorAlertOn1 = AnchorAlertOn2 = false;
    for (const auto &watch : watches.watches) {
      wxJSONValue message;
      message["GUID"] = wxString::FromUTF8(watch.id);
      SendJSONMessageToAllPlugins("OCPN_ANCHOR_WATCH_CLEARED", message);
    }
    // At the pinned revision this call returns true and delegates plugin-owned
    // route handling normally. Do not introduce an alternate progress engine.
    const bool activated = g_pRouteMan->ActivateRoute(route, best);
    return {activated, activated ? "Route activated using OpenCPN"
                                  : "Route activation failed; anchor watch stopped",
            selected.id};
  });
}
} // namespace
application::CommandResult ActivateRoute(const application::Route &selected,
                                         const vessel::Navigation &position) {
  return ActivateRouteTransition(selected, position, nullptr);
}
application::CommandResult ActivateRouteAfterAnchor(
    const application::Route &selected, const vessel::Navigation &position,
    const application::AnchorWatchSelection &confirmed) {
  return ActivateRouteTransition(selected, position, &confirmed);
}
application::CommandResult StopRoute(const application::Route &selected) {
  Thread();
  // Resolve the rendered identity AND revision immediately before dispatch.
  // A confirmation for route A must never stop a newly active route B.
  auto *route = selected.id.empty() || selected.revision.empty() ? nullptr : Resolve(selected);
  if (!selected.active || !route || !route->IsActive() ||
      g_pRouteMan->GetpActiveRoute() != route)
    return {false, "Selected route changed or is no longer active; review the route", {}};
  g_pRouteMan->DeactivateRoute();
  return {true, "Navigation stopped", selected.id};
}
application::CommandResult ReverseRoute(const application::Route &selected) {
  Thread();
  auto *route = Resolve(selected);
  if (!route || !Copy(route).editable || !pSelect)
    return {false,
            "Stop navigation and refresh the editable route before reversing",
            {}};
  pSelect->DeleteAllSelectableRouteSegments(route);
  route->Reverse(false);
  pSelect->AddAllSelectableRouteSegments(route);
  if (!NavObj_dB::GetInstance().UpdateRoute(route)) {
    pSelect->DeleteAllSelectableRouteSegments(route);
    route->Reverse(false);
    pSelect->AddAllSelectableRouteSegments(route);
    return {false, "Could not save route reversal; restored route order", {}};
  }
  return {true, "Route reversed; waypoint names preserved", selected.id};
}
application::CommandResult EditRoute(const application::Route &selected,
                                     const std::string &name,
                                     const std::string &description) {
  Thread();
  auto *route = Resolve(selected);
  if (!route || !Copy(route).editable || !TextValid(name, description))
    return {false, "Route changed/protected or invalid name (1–128 bytes)", {}};
  const auto previous_name = route->m_RouteNameString,
             previous_description = route->m_RouteDescription;
  route->m_RouteNameString = wxString::FromUTF8(name);
  route->m_RouteDescription = wxString::FromUTF8(description);
  if (!NavObj_dB::GetInstance().UpdateRoute(route)) {
    route->m_RouteNameString = previous_name;
    route->m_RouteDescription = previous_description;
    return {false, "Route save failed", {}};
  }
  return {true, "Route saved", selected.id};
}
application::CommandResult EditWaypoint(const application::Waypoint &selected,
                                        const std::string &name,
                                        const std::string &description) {
  Thread();
  auto *point = Resolve(selected);
  if (!point || !Copy(point).editable || !TextValid(name, description))
    return {
        false, "Waypoint changed/protected or invalid name (1–128 bytes)", {}};
  const auto old_name = point->GetName(),
             old_description = point->GetDescription();
  point->SetName(wxString::FromUTF8(name));
  point->m_MarkDescription = wxString::FromUTF8(description);
  if (!NavObj_dB::GetInstance().UpdateRoutePoint(point)) {
    point->SetName(old_name);
    point->m_MarkDescription = old_description;
    return {false, "Waypoint save failed", {}};
  }
  return {true, "Waypoint saved", selected.id};
}
application::CommandResult
DeleteWaypoint(const application::Waypoint &selected) {
  Thread();
  auto *point = Resolve(selected);
  if (!point || !Copy(point).removable || !pSelect)
    return {false,
            "Only an unchanged isolated, unprotected waypoint can be deleted "
            "in this version",
            {}};
  auto *canvas = gFrame ? gFrame->GetPrimaryCanvas() : nullptr;
  auto *undo = canvas ? canvas->undo : nullptr;
  if (!undo || undo->InUndoableAction() ||
      !undo->BeforeUndoableAction(Undo_DeleteWaypoint, point, Undo_IsOrphanded,
                                  nullptr))
    return {
        false, "Finish the current chart edit before deleting a waypoint", {}};
  if (!NavObj_dB::GetInstance().DeleteRoutePoint(point)) {
    undo->CancelUndoableAction(true);
    return {false, "Waypoint delete failed; mark retained", {}};
  }
  if (g_pMarkInfoDialog)
    g_pMarkInfoDialog->ClearData();
  pSelect->DeleteSelectablePoint(point, SELTYPE_ROUTEPOINT);
  pWayPointMan->RemoveRoutePoint(point);
  // The stock undo action owns the orphaned point, exactly as canvas deletion.
  // OpenNav retains only the value snapshot, never this pointer.
  undo->AfterUndoableAction(nullptr);
  return {true, "Waypoint deleted", selected.id};
}
application::CommandResult CreateWaypoint(application::Coordinate position,
                                          const std::string &name,
                                          const std::string &description) {
  Thread();
  if (!pWayPointMan || !pSelect || !TextValid(name, description) ||
      !std::isfinite(position.latitude_deg) ||
      !std::isfinite(position.longitude_deg) ||
      std::abs(position.latitude_deg) > 90 ||
      std::abs(position.longitude_deg) > 180)
    return {false, "Valid position and waypoint name required", {}};
  auto *point = new RoutePoint(position.latitude_deg, position.longitude_deg,
                               g_default_wp_icon, wxString::FromUTF8(name),
                               wxEmptyString);
  point->m_bIsolatedMark = true;
  point->m_MarkDescription = wxString::FromUTF8(description);
  if (!NavObj_dB::GetInstance().InsertRoutePoint(point)) {
    delete point;
    return {false, "Waypoint save failed", {}};
  }
  pSelect->AddSelectableRoutePoint(position.latitude_deg,
                                   position.longitude_deg, point);
  return {true, "Waypoint created", String(point->m_GUID)};
}
application::CommandResult GoTo(application::Coordinate destination,
                                 const std::string &name,
                                 const vessel::Navigation &position) {
  Thread();
  if (pAnchorWatchPoint1 || pAnchorWatchPoint2 || !g_AW1GUID.empty() || !g_AW2GUID.empty())
    return {false, "Stop the anchor watch before starting Go To", {}};
  if (!g_pRouteMan || !pRouteList || !pSelect || !pWayPointMan ||
      !Position(position, vessel::Clock::now()))
    return {false, "Go To requires a current GPS position", {}};
  if (g_pRouteMan->GetpActiveRoute())
    return {false, "Stop current navigation before starting a new destination", {}};
  if (!TextValid(name, {}) || !std::isfinite(destination.latitude_deg) ||
      !std::isfinite(destination.longitude_deg) ||
      std::abs(destination.latitude_deg) > 90 ||
      std::abs(destination.longitude_deg) > 180)
    return {false, "Choose a valid destination", {}};
  auto *point = new RoutePoint(destination.latitude_deg, destination.longitude_deg,
                               g_default_wp_icon, wxString::FromUTF8(name),
                               wxEmptyString);
  pSelect->AddSelectableRoutePoint(point->m_lat, point->m_lon, point);
  return NavigateTo(point, false);
}
application::CommandResult GoToWaypoint(const application::Waypoint &selected,
                                         const vessel::Navigation &position) {
  Thread();
  if (pAnchorWatchPoint1 || pAnchorWatchPoint2 || !g_AW1GUID.empty() || !g_AW2GUID.empty())
    return {false, "Stop the anchor watch before starting Go To", {}};
  auto *point = Resolve(selected);
  if (!point || !Copy(point).editable)
    return {false, "Waypoint changed or protected; select it again", {}};
  if (!g_pRouteMan || !pRouteList || !pSelect ||
      !Position(position, vessel::Clock::now()))
    return {false, "Go To requires a current GPS position", {}};
  if (g_pRouteMan->GetpActiveRoute())
    return {false, "Stop current navigation before starting a new destination", {}};
  return NavigateTo(point, true);
}
vessel::AisState CopyAisState(const vessel::Navigation &position,
                              vessel::Time now) {
  Thread();
  struct Observation { std::time_t report; std::optional<vessel::Time> at; };
  static std::map<int, Observation> clocks;
  std::map<int, Observation> current_clocks;
  vessel::AisState state;
  state.observed_at = now;
  state.source = "OpenCPN AIS model";
  if (!g_pAIS) {
    clocks.clear();
    return state;
  }
  state.available = true;
  const auto wall = wxDateTime::Now();
  const bool own_position = Position(position, now);
  for (const auto &entry : g_pAIS->GetTargetList()) {
    if (state.targets.size() >= 2000)
      break;
    const auto &p = entry.second;
    if (!p || p->b_removed)
      continue;
    vessel::AisTarget t;
    t.mmsi = p->MMSI;
    t.active = p->b_active;
    t.lost = p->b_lost;
    t.doubtful = p->b_positionDoubtful;
    t.upstream_alarm = p->n_alert_state != AIS_NO_ALERT;
    t.source = state.source;
    if (p->b_nameValid) {
      t.name = std::string(
          p->ShipName,
          std::find(p->ShipName, p->ShipName + sizeof(p->ShipName), '\0'));
      while (!t.name.empty() && (t.name.back() == '@' || t.name.back() == ' '))
        t.name.pop_back();
    }
    t.status = t.lost ? "Lost" : t.doubtful ? "Position doubtful" : t.active ? "Active" : "Inactive";
    if (t.active && !t.lost && !t.doubtful) {
      // Match pinned AisTargetData::BuildQueryResult's class/status boundary.
      // A Class-B/base/meteo report has no navigational status. SART status
      // codes mean active/testing, not the ordinary vessel-status enum.
      if (p->Class == AIS_SART) {
        if (p->NavStatus == RESERVED_14) t.status = "Active distress beacon";
        else if (p->NavStatus == UNDEFINED) t.status = "Distress beacon testing";
        else t.status += " / Beacon status unavailable";
      } else if (p->Class != AIS_BASE && p->Class != AIS_CLASS_B && p->Class != AIS_METEO) {
        if (p->NavStatus >= 0 && p->NavStatus <= 21 && p->NavStatus != UNDEFINED)
          t.status += " / " + String(ais_get_status(p->NavStatus));
        else t.status += " / Navigation status unavailable";
      }
    }
    auto at = AisObservationAt(p->PositionReportTicks, wall, now);
    const auto previous = clocks.find(t.mmsi);
    if (previous != clocks.end() && previous->second.report == p->PositionReportTicks)
      at = previous->second.at;
    // Target report age is the retained upstream observation, not this copy's
    // UI-read time. The enclosing state retains the separate copy timestamp.
    t.observed_at = at.value_or(vessel::Time{});
    // Convert a given upstream observation exactly once. Re-pairing wall and
    // monotonic clocks on every UI read introduces sub-millisecond backwards
    // jitter and incorrectly invalidates a retained target selection.
    current_clocks.emplace(t.mmsi, Observation{p->PositionReportTicks, at});
    auto sample = [&](double value, double minimum, double maximum,
                      bool relative = false) {
      vessel::Sample s;
      s.source = t.source + " / MMSI " + std::to_string(t.mmsi);
      if (!at || !p->b_positionOnceValid || !t.active || t.lost || t.doubtful ||
          !std::isfinite(value) || value < minimum || value > maximum ||
          (relative && !own_position))
        return s;
      s.value = value;
      s.validity =
          relative ? vessel::Validity::Estimated : vessel::Validity::Measured;
      s.observed_at =
          relative ? std::min(*at, position.latitude_deg.observed_at) : *at;
      s.freshness = relative ? vessel::Freshness{}
                             : vessel::Freshness{std::chrono::seconds(15),
                                                 std::chrono::seconds(60)};
      return s;
    };
    t.latitude_deg = sample(p->Lat, -90, 90);
    t.longitude_deg = sample(p->Lon, -180, 180);
    t.sog_kn = sample(p->SOG, 0, 102.2);
    t.cog_deg = sample(p->COG, 0, 359.999);
    t.heading_true_deg = sample(p->HDG, 0, 359);
    t.range_nm = sample(p->Range_NM, 0, 30000, true);
    t.bearing_true_deg = sample(p->Brg, 0, 359.999, true);
    if (p->bCPA_Valid) {
      t.cpa_nm = sample(p->CPA, 0, 30000, true);
      t.tcpa_minutes = sample(p->TCPA, -100000, 100000, true);
    }
    state.targets.push_back(std::move(t));
  }
  std::sort(state.targets.begin(), state.targets.end(),
            [](const auto &a, const auto &b) { return a.mmsi < b.mmsi; });
  clocks = std::move(current_clocks); // Removed targets retain no bridge state.
  return state;
}
application::AnchorState ObserveAnchor(const vessel::Navigation &position,
                                       vessel::Time now) {
  Thread();
  application::AnchorState s;
  s.observed_at = now;
  s.source = "OpenCPN normal anchor watch";
  s.state = "No anchor watch";
  s.distance_units_per_m = toUsrDistance(1. / 1852.);
  s.distance_unit = String(getUsrDistanceUnit());
  auto *point = pAnchorWatchPoint1 ? pAnchorWatchPoint1 : pAnchorWatchPoint2;
  if (!point || !pWayPointMan)
    return s;
  // Validate registration before dereferencing the retained upstream pointer.
  bool registered = false;
  for (auto *n = pWayPointMan->GetWaypointList()->GetFirst(); n;
       n = n->GetNext())
    if (n->GetData() == point) {
      registered = true;
      break;
    }
  if (!registered) {
    s.state = "Invalid anchor waypoint";
    return s;
  }
  s.waypoint_id = String(point->m_GUID);
  s.anchor = application::Coordinate{point->m_lat, point->m_lon};
  double radius = g_nAWMax;
  point->GetName().ToDouble(&radius);
  radius = AnchorDistFix(radius, AnchorPointMinDist, g_nAWMax);
  if (std::isfinite(radius))
    s.radius_m = radius;
  s.alarm = point == pAnchorWatchPoint1 ? AnchorAlertOn1 : AnchorAlertOn2;
  s.state =
      s.alarm ? "OpenCPN anchor alarm" : "Watching; no drag-intelligence claim";
  if (Position(position, now)) {
    ObserveAnchorPosition(s, position, now);
  } else
    s.state = "Position unavailable/stale; anchor watch needs attention";
  return s;
}
application::CommandResult StartAnchor(const vessel::Navigation &position,
                                       double radius) {
  Thread();
  if (pAnchorWatchPoint1 || pAnchorWatchPoint2 || !g_AW1GUID.empty() || !g_AW2GUID.empty())
    return {
        false,
        "An upstream anchor watch already exists; clear it explicitly first",
        {}};
  if (!Position(position, vessel::Clock::now()) || !std::isfinite(radius) ||
      radius < AnchorPointMinDist || radius > g_nAWMax ||
      radius != std::round(radius))
    return {false,
            "Fresh position and whole-metre radius within OpenCPN anchor limits required",
            {}};
  if (!pWayPointMan || !pSelect || !g_pRouteMan)
    return {false, "Navigation storage unavailable", {}};
  auto *point = new RoutePoint(
      *position.latitude_deg.value, *position.longitude_deg.value, "anchor",
      wxString::Format("%.0f", radius), wxEmptyString);
  point->m_bIsolatedMark = true;
  // Human-readable ownership annotation; the clear path also recognizes the
  // exact historical descriptions for upgrade continuity.
  point->m_MarkDescription = "SKAGER temporary anchor watch";
  // Persist first: a failed anchor save must leave navigation running. The
  // pinned DeactivateRoute clears active progress without deleting the route.
  const auto started = application::CommitAnchorWatch(
      [&] { return NavObj_dB::GetInstance().InsertRoutePoint(point); },
      [] { return !g_pRouteMan->GetpActiveRoute() || g_pRouteMan->DeactivateRoute(); },
      [&] { return NavObj_dB::GetInstance().DeleteRoutePoint(point); },
      [&] {
        pSelect->AddSelectableRoutePoint(point->m_lat, point->m_lon, point);
        pAnchorWatchPoint1 = point;
        g_AW1GUID = point->m_GUID;
      });
  if (started == application::AnchorStartResult::SaveFailed ||
      started == application::AnchorStartResult::StopFailed) {
    delete point;
    return {false, started == application::AnchorStartResult::SaveFailed
                       ? "Anchor save failed; navigation and watch unchanged"
                       : "Could not stop navigation; anchor watch unchanged", {}};
  }
  if (started == application::AnchorStartResult::RollbackFailed) {
    // Keep failed rollback data visible/selectable and report its identity;
    // never leave an invisible persisted orphan or arm a conflicting watch.
    pSelect->AddSelectableRoutePoint(point->m_lat, point->m_lon, point);
    return {false, "Could not stop navigation or remove saved anchor mark; watch not armed",
            String(point->m_GUID)};
  }
  wxJSONValue message;
  message["GUID"] = g_AW1GUID;
  SendJSONMessageToAllPlugins("OCPN_ANCHOR_WATCH_SET", message);
  return {true, "Anchor watch set", String(point->m_GUID)};
}
application::CommandResult ClearAnchor(const std::string &id) {
  Thread();
  if (id.empty())
    return {false, "No anchor selected", {}};
  RoutePoint *point = nullptr;
  const bool first = String(g_AW1GUID) == id;
  if (first) {
    point = pAnchorWatchPoint1;
    pAnchorWatchPoint1 = nullptr;
    g_AW1GUID.Clear();
  } else if (String(g_AW2GUID) == id) {
    point = pAnchorWatchPoint2;
    pAnchorWatchPoint2 = nullptr;
    g_AW2GUID.Clear();
  } else
    return {false, "Anchor changed; refresh selection", {}};
  // Upstream may watch any existing user waypoint. Only delete an unchanged
  // OpenNav-created isolated anchor; preserve a shared or repurposed user mark.
  bool removed = false;
  if (point && pWayPointMan &&
      pWayPointMan->FindWaypointByGuid(id) == point &&
      (point->GetDescription() == "SKAGER temporary anchor watch" ||
       point->GetDescription() == "OpenNav temporary anchor watch" ||
       point->GetDescription() ==
           "OpenNav anchor watch; radius stored using OpenCPN semantics") &&
      point->m_bIsolatedMark && !point->IsShared() && Copy(point).removable) {
    const auto deleted = DeleteWaypoint(Copy(point));
    if (!deleted.ok) {
      // Restore the watch if its removal cannot be persisted. Do not claim a
      // completed clear while leaving an unexpected live chart mark behind.
      if (first) { pAnchorWatchPoint1 = point; g_AW1GUID = wxString::FromUTF8(id); }
      else { pAnchorWatchPoint2 = point; g_AW2GUID = wxString::FromUTF8(id); }
      return {false, "Could not remove anchor mark; watch retained", id};
    }
    removed = true;
  }
  if (first) AnchorAlertOn1 = false;
  else AnchorAlertOn2 = false;
  wxJSONValue message;
  message["GUID"] = wxString::FromUTF8(id);
  SendJSONMessageToAllPlugins("OCPN_ANCHOR_WATCH_CLEARED", message);
  return {true, removed ? "Anchor watch and temporary mark removed"
                        : "Anchor watch cleared; existing user waypoint retained", id};
}
// Presentation inspection and commands run only on the application thread.
// Keep this block free of chart loading, preference copies and navigation math.
application::ChartPresentationState CopyChartPresentation(MyFrame &frame) {
  application::ChartPresentationState state;
  if (!wxIsMainThread()) {
    state.reason = "Chart presentation requires the application thread";
    return state;
  }
  auto *canvas = frame.GetPrimaryCanvas();
  if (!canvas) {
    state.reason = "Chart canvas unavailable";
    return state;
  }
  state.available = true;
  switch (canvas->GetUpMode()) {
  case NORTH_UP_MODE: state.orientation = application::ChartOrientation::NorthUp; break;
  case COURSE_UP_MODE: state.orientation = application::ChartOrientation::CourseUp; break;
  case HEAD_UP_MODE: state.orientation = application::ChartOrientation::HeadUp; break;
  }
  int family = CHART_FAMILY_UNKNOWN;
  state.format_reason = "Current chart format unavailable";
  if (canvas->GetQuiltMode()) {
    // Only inspect the already-selected entry. Never open a chart or rebuild a stack.
    if (ChartData && ChartData->IsValid() && !ChartData->IsBusy()) {
      const int index = canvas->GetQuiltReferenceChartIndex();
      if (index >= 0 && index < ChartData->GetChartTableEntries()) {
        family = ChartData->GetDBChartFamily(index);
        state.format_reason = "Format of the current quilt reference chart";
      }
    }
  } else if (canvas->m_singleChart) {
    family = canvas->m_singleChart->GetChartFamily();
    state.format_reason = "Format of the current chart";
  }
  if (family == CHART_FAMILY_VECTOR) state.format = application::ChartFormat::Vector;
  else if (family == CHART_FAMILY_RASTER) state.format = application::ChartFormat::Raster;
  const bool enc = ps52plib && state.format == application::ChartFormat::Vector &&
                   canvas->GetENCDisplayCategory() != DISPLAYBASE;
  const std::string enc_reason = enc ? "" :
      "Requires a vector chart and an ENC display category above Base";
  state.ais_vessels = {canvas->GetShowAIS(), true, "Chart visibility only; reception and alarms unchanged"};
  // The prototype's Symbol labels row means the upstream ENC text master switch.
  // Preserve independent buoy-label and light-description preferences.
  state.enc_text = {canvas->GetShowENCText(), enc, enc_reason};
  state.depth_soundings = {canvas->GetShowENCDepth(), enc, enc_reason};
  return state;
}
namespace {
enum class PresentationLayer { Ais, EncText, Soundings };
application::ChartPresentationResult SetPresentationLayer(
    MyFrame &frame, PresentationLayer layer, bool show) {
  auto before = CopyChartPresentation(frame);
  if (!before.available) return {{false, before.reason}, std::move(before)};
  const auto field = [layer](const application::ChartPresentationState &state)
      -> const application::ChartLayerState & {
    if (layer == PresentationLayer::Ais) return state.ais_vessels;
    if (layer == PresentationLayer::EncText) return state.enc_text;
    return state.depth_soundings;
  };
  if (!field(before).editable)
    return {{false, field(before).reason}, std::move(before)};
  auto *canvas = frame.GetPrimaryCanvas();
  if (!canvas) return {{false, "Chart canvas unavailable"}, CopyChartPresentation(frame)};
  const bool current = layer == PresentationLayer::Ais ? canvas->GetShowAIS() :
      layer == PresentationLayer::EncText ? canvas->GetShowENCText() : canvas->GetShowENCDepth();
  if (current != show) {
    switch (layer) {
    case PresentationLayer::Ais: frame.ToggleAISDisplay(canvas); break;
    case PresentationLayer::EncText: frame.ToggleENCText(canvas); break;
    case PresentationLayer::Soundings: frame.ToggleSoundings(canvas); break;
    }
  }
  auto after = CopyChartPresentation(frame);
  const bool ok = after.available && field(after).visible == show;
  return {{ok, ok ? "Chart presentation applied" : "Chart presentation changed; inspect current state"},
          std::move(after)};
}
} // namespace
application::ChartPresentationResult SetChartAis(MyFrame &frame, bool show) {
  return SetPresentationLayer(frame, PresentationLayer::Ais, show);
}
application::ChartPresentationResult SetChartEncText(MyFrame &frame, bool show) {
  return SetPresentationLayer(frame, PresentationLayer::EncText, show);
}
application::ChartPresentationResult SetChartSoundings(MyFrame &frame, bool show) {
  return SetPresentationLayer(frame, PresentationLayer::Soundings, show);
}
application::ChartPresentationResult SetChartOrientation(
    MyFrame &frame, application::ChartOrientation orientation) {
  auto before = CopyChartPresentation(frame);
  if (!before.available) return {{false, before.reason}, std::move(before)};
  int mode;
  switch (orientation) {
  case application::ChartOrientation::NorthUp: mode = NORTH_UP_MODE; break;
  case application::ChartOrientation::CourseUp: mode = COURSE_UP_MODE; break;
  case application::ChartOrientation::HeadUp: mode = HEAD_UP_MODE; break;
  default: return {{false, "Unknown chart orientation"}, std::move(before)};
  }
  auto *canvas = frame.GetPrimaryCanvas();
  if (!canvas) return {{false, "Chart canvas unavailable"}, CopyChartPresentation(frame)};
  if (canvas->GetUpMode() != mode) frame.SetUpMode(canvas, mode);
  auto after = CopyChartPresentation(frame);
  const bool ok = after.available && after.orientation == orientation;
  return {{ok, ok ? "Chart orientation applied; rotation depends on source data" :
                       "Chart orientation changed; inspect current state"}, std::move(after)};
}
} // namespace opennav::integration

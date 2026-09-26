#include "NavigationObjectScenario.h"
#include "chcanv.h"
#include "integration/NavigationObjects.h"
#include "integration/OpenCPNIntegration.h"
#include "model/ais_decoder.h"
#include "model/ais_target_data.h"
#include "model/comm_drv_registry.h"
#include "model/comm_util.h"
#include "model/conn_params.h"
#include "model/navobj_db.h"
#include "model/own_ship.h"
#include "model/route.h"
#include "model/route_point.h"
#include "model/routeman.h"
#include "ocpn_frame.h"
#include "undo.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <thread>
#include <wx/filefn.h>
#include <wx/jsonwriter.h>
#include <wx/thread.h>
#include <wx/timer.h>
extern bool g_bDeferredInitDone;
extern MyFrame *gFrame;
namespace opennav::test {
namespace {
using namespace integration;
using namespace vessel;
using namespace std::chrono_literals;
std::string directory;
wxJSONValue report;
int step = 0, waited = 0, advice_waited = 0;
bool finished = false;
std::string mark_id, anchor_id;
Route *test_route = nullptr;
application::Waypoint retained_mark;
application::Route retained_route;
std::shared_ptr<AisTargetData> target;
bool advisory_fixture = false;
bool added_late_connection = false;
int late_connection_ticks = 0;
bool settings_capture_started = false;
int settings_capture_waited = 0;
std::time_t AisTicksNow() { return wxDateTime::Now().ToUTC().GetTicks(); }
// Explicit decoder-state injection in the isolated no-output fixture. Keep
// its synthetic reports current while Python exercises the actual target card.
// The ordinary AIS timer may recalculate CPA/alarms; refresh this test state as
// a continuing simulated feed, rather than depending on timer phase ordering.
class AisFixtureFeed final : public wxTimer {
  void Notify() override {
    if (!target || !g_pAIS) { Stop(); return; }
    target->PositionReportTicks = AisTicksNow();
    if (advisory_fixture) {
      target->n_alert_state = AIS_ALERT_NO_DIALOG_SET;
      target->CPA = .42; target->TCPA = 7.5; target->bCPA_Valid = true;
    }
  }
};
std::unique_ptr<AisFixtureFeed> ais_feed;
void Check(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}
void Record(const char *text) {
  wxJSONValue r;
  r["check"] = wxString::FromUTF8(text);
  report["checks"].Append(r);
}
void Write() {
  wxJSONWriter writer;
  wxString s;
  writer.Write(report, s);
  const auto path =
      wxString::FromUTF8(directory) + "/objects-fixture-results.json";
  {
    std::ofstream out((path + ".pending").ToStdString(wxConvUTF8));
    out << s.ToStdString(wxConvUTF8);
  }
  Check(wxRenameFile(path + ".pending", path, true),
        "Publish fixture evidence");
}
application::Waypoint Mark(const std::string &id) {
  for (const auto &w : CopyNavigationCatalog().waypoints)
    if (w.id == id)
      return w;
  throw std::runtime_error("Expected waypoint missing");
}
application::Route RouteCopy() {
  for (const auto &r : CopyNavigationCatalog().routes)
    if (r.id == test_route->GetGUID().ToStdString(wxConvUTF8))
      return r;
  throw std::runtime_error("Expected route missing");
}
void AddRoute() {
  test_route = new Route;
  test_route->m_GUID = "OPENNAV-ALPHA-OBJECT-ROUTE";
  test_route->m_RouteNameString = "ALPHA TEST route";
  for (int i = 0; i < 3; ++i) {
    auto *w =
        new RoutePoint(gLat + .04 * (i + 1), gLon + .05 * (i + 1), "diamond",
                       wxString::Format("ALPHA TEST %d", i + 1),
                       wxString::Format("OPENNAV-ALPHA-POINT-%d", i + 1));
    test_route->AddPoint(w, false);
  }
  pRouteList->Append(test_route);
  Check(NavObj_dB::GetInstance().InsertRoute(test_route),
        "Insert actual test route");
}
} // namespace
void StopObjectScenario() {
  ais_feed.reset();
  target.reset();
  advisory_fixture = false;
  finished = true;
}
void EnableObjectScenario(const std::string &profile) {
  Check(!profile.empty() && wxFileExists(wxString::FromUTF8(profile) +
                                         "/OPENNAV_OBJECT_FIXTURE"),
        "Object driver requires marked disposable profile");
  directory = profile;
  report["result"] = wxString("running");
  report["phase"] = wxString("input");
  report["fixture"] =
      wxString("Explicit loopback-only object contract test; no device output");
  Write();
}
void ObjectScenarioStep(const vessel::Navigation &selected) {
  if (directory.empty() || finished || !g_bDeferredInitDone)
    return;
  try {
    Check(wxIsMainThread(), "Application thread required");
    Check(++waited < 60, "Object scenario timed out");
    if (!added_late_connection) {
      Check(TheConnectionParams().empty(), "Late-add fixture requires no initial connections");
      Check(!selected.latitude_deg.value, "No selected GPS may precede connection addition");
      // Give the actual application two completed navigation timer passes with
      // no input. Add through the same API used by the normal connection editor.
      if (++late_connection_ticks < 3) return;
      std::ifstream in(directory + "/OPENNAV_OBJECT_INPUT_PORT");
      unsigned port = 0;
      Check(bool(in >> port) && port >= 1024 && port <= 65535,
            "Explicit loopback input port required");
      auto *connection = new ConnectionParams(wxString::Format(
          "1;0;127.0.0.1;%u;0;;4800;1;0;0;;0;;0;0;0;0;1;"
          "ISOLATED late-add GPS and AIS test;0;;0;1;", port));
      Check(connection->Valid && connection->Type == NETWORK &&
                connection->NetProtocol == TCP &&
                connection->NetworkAddress == "127.0.0.1" &&
                connection->Protocol == PROTO_NMEA0183 &&
                connection->IOSelect == DS_TYPE_INPUT && connection->bEnabled,
            "Late connection must be enabled input-only loopback NMEA0183");
      connection->b_IsSetup = false;
      TheConnectionParams().push_back(connection);
      UpdateDatastreams();
      added_late_connection = true;
      report["late_connection_added_after_deferred"] = true;
      Record("No startup GPS; real input connection added after deferred initialization");
      report["phase"] = wxString("connection-added");
      Write();
      return;
    }
    if (!selected.latitude_deg.value ||
        Assess(selected.latitude_deg, Clock::now()).quality != Quality::Live)
      return;
    if (step == 0) {
      auto network_target = g_pAIS ? g_pAIS->Get_Target_Data_From_MMSI(990000002)
                                  : std::shared_ptr<AisTargetData>{};
      if (!network_target || !network_target->b_positionOnceValid) return;
      Check(std::abs(*selected.latitude_deg.value - 56.7) < 1e-6 &&
                std::abs(*selected.longitude_deg.value - 12.6) < 1e-6 &&
                selected.sog_kn.value && std::abs(*selected.sog_kn.value - 6.3) < 1e-6,
            "Late-added real connection delivers selected GPS without restart");
      Check(std::abs(network_target->Lat - 56.82) < 1e-6 &&
                std::abs(network_target->Lon - 12.9) < 1e-6 &&
                std::abs(network_target->SOG - 7) < 1e-6 &&
                network_target->COG == 0 && network_target->HDG == 0,
            "Actual AIVDM decoding preserves target position and motion");
      Check(!g_pAIS->Get_Target_Data_From_MMSI(990000003),
            "Malformed AIS checksum cannot create a target");
      const auto received = CopyAisState(selected, Clock::now());
      auto observed = std::find_if(received.targets.begin(), received.targets.end(),
                                  [](const auto &t) { return t.mmsi == 990000002; });
      report["acquired_ais"]["active"] = network_target->b_active;
      report["acquired_ais"]["lost"] = network_target->b_lost;
      report["acquired_ais"]["doubtful"] = network_target->b_positionDoubtful;
      report["acquired_ais"]["source_report_age_seconds"] =
          static_cast<int>(AisTicksNow() - network_target->PositionReportTicks);
      report["acquired_ais"]["copied"] = observed != received.targets.end();
      report["acquired_ais"]["has_position"] =
          observed != received.targets.end() && observed->latitude_deg.value.has_value();
      Write();
      Check(observed != received.targets.end() && observed->latitude_deg.value &&
                Assess(observed->latitude_deg, Clock::now()).quality == Quality::Live,
            "Actual AIS acquisition reaches owned current OpenNav target");
      Record("Late-added GPS and actual TCP AIVDM AIS acquire without process restart");
      Check(pRouteList && pRouteList->IsEmpty() && pWayPointMan,
            "Empty disposable profile required");
      for (const auto &driver : GetActiveDrivers()) {
        auto a = GetAttributes(driver);
        auto i = a.find("ioDirection");
        Check(i == a.end() || i->second == "IN",
              "Object fixture refuses output connection");
      }
      bool refused = false;
      std::thread worker([&] {
        try {
          CopyNavigationCatalog();
        } catch (const std::logic_error &) {
          refused = true;
        }
      });
      worker.join();
      Check(refused, "Reject off-thread object access");
      Record("Thread boundary rejects worker access");
      auto created = CreateWaypoint({gLat + .1, gLon + .1}, "ALPHA TEST mark",
                                    "Retain snapshot across delete");
      Check(created.ok, "Create waypoint");
      mark_id = created.identity;
      auto old = Mark(mark_id);
      Check(EditWaypoint(old, "ALPHA TEST edited", "New description").ok,
            "Edit waypoint");
      Check(!EditWaypoint(old, "STALE EDIT", "Must fail").ok,
            "Reject old waypoint revision");
      retained_mark = Mark(mark_id);
      Check(!DeleteWaypoint(old).ok, "Reject deletion using old revision");
      Record("Waypoint create, edit and stale selection rejection");
      Check(DeleteWaypoint(retained_mark).ok, "Delete isolated mark");
      Check(retained_mark.name == "ALPHA TEST edited",
            "Retained mark copy remains independent");
      Check(!DeleteWaypoint(retained_mark).ok, "Repeat delete fails closed");
      Check(gFrame->GetPrimaryCanvas()->undo->UndoLastAction(),
            "Stock canvas undo restores deleted mark");
      Check(Mark(mark_id).name == retained_mark.name, "Restored mark identity");
      Record("Deletion uses OpenCPN undo; retained snapshot survives");
      Check(!GoTo({gLat + .2, gLon + .2}, "No GPS", Navigation{}).ok,
            "Go To refuses missing selected position");
      Check(!GoTo({91, 0}, "Invalid position", selected).ok,
            "Go To refuses invalid destination");
      Check(GoTo({gLat + .2, gLon + .2}, "TEST destination", selected).ok,
            "Go To creates upstream temporary route");
      auto *goto_route = g_pRouteMan->GetpActiveRoute();
      Check(goto_route && goto_route->GetnPoints() == 2 &&
                goto_route->m_bDeleteOnArrival &&
                g_pRouteMan->GetpActivePoint() == goto_route->GetPoint(2),
            "Go To targets second point using native arrival lifecycle");
      Check(!GoTo({gLat + .3, gLon + .3}, "Replacement", selected).ok,
            "Go To cannot silently replace active navigation");
      Check(g_pRouteMan->DeleteRoute(goto_route), "Remove temporary test route");
      Check(GoToWaypoint(Mark(mark_id), selected).ok,
            "Go To existing waypoint");
      goto_route = g_pRouteMan->GetpActiveRoute();
      Check(goto_route && goto_route->GetPoint(2)->m_GUID ==
                                wxString::FromUTF8(mark_id),
            "Go To reuses original waypoint identity");
      Check(g_pRouteMan->DeleteRoute(goto_route) &&
                pWayPointMan->FindWaypointByGuid(mark_id),
            "Temporary route deletion preserves existing destination mark");
      Record("Go To native temporary route, GPS/identity guards and user-waypoint lifetime");
      AddRoute();
      auto before = RouteCopy();
      Check(before.points.size() == 3 && before.points[1].incoming_nm ==
                                             test_route->GetPoint(2)->m_seg_len,
            "Stored leg distance copied");
      Check(EditRoute(before, "ALPHA TEST renamed route", "Shared database").ok,
            "Edit route");
      Check(!ReverseRoute(before).ok, "Reject old route revision");
      before = RouteCopy();
      Check(ReverseRoute(before).ok, "Reverse route through core");
      auto reversed = RouteCopy();
      Check(reversed.points.front().id == before.points.back().id,
            "Upstream reversed order");
      retained_route = before;
      Record("Route name edit, upstream reversal and immutable revision guard");
      auto first = test_route->GetPoint(1), second = test_route->GetPoint(2);
      auto id = second->m_GUID;
      second->m_GUID = first->m_GUID;
      Check(!RouteCopy().editable, "Ambiguous repeated identity protected");
      Check(!ReverseRoute(RouteCopy()).ok, "Ambiguous reversal refused");
      second->m_GUID = id;
      test_route->m_bIsInLayer = true;
      Check(!EditRoute(RouteCopy(), "LAYER", "refuse").ok,
            "Layer route protected");
      test_route->m_bIsInLayer = false;
      Record("Repeated waypoint identity and layer protections");
      Check(!ActivateRoute(RouteCopy(), Navigation{}).ok,
            "Activation requires selected position");
      Check(ActivateRoute(RouteCopy(), selected).ok,
            "Activate from fresh selected position");
      Check(!ReverseRoute(RouteCopy()).ok, "Active route reverse refused");
      Check(!EditRoute(RouteCopy(), "ACTIVE", "refuse").ok,
            "Active route edit refused");
      Check(!DeleteWaypoint(
                 Mark(test_route->GetPoint(1)->m_GUID.ToStdString(wxConvUTF8)))
                 .ok,
            "Route member delete refused");
      Record("Activation uses selected navigation; active route edits refused");
    } else if (step == 1) {
      Check(g_pRouteMan->GetpActiveRoute() == test_route,
            "Normal progress preserves active route");
      Check(StopRoute().ok, "Stop route");
      auto started = StartAnchor(selected, 50);
      Check(started.ok, "Start upstream anchor watch");
      anchor_id = started.identity;
      auto *created_anchor = pWayPointMan->FindWaypointByGuid(anchor_id);
      Check(created_anchor && created_anchor->GetName() == "50" &&
                created_anchor->GetIconName() == "anchor",
            "Anchor chart label uses whole metres and anchor icon");
      Check(!StartAnchor(selected, 50).ok, "No implicit anchor replacement");
      Check(!DeleteWaypoint(Mark(anchor_id)).ok, "Anchor mark protected");
      Record("Stop navigation and explicit anchor-watch creation");
    } else if (step == 2) {
      auto watch = ObserveAnchor(selected, Clock::now());
      Check(watch.waypoint_id == anchor_id && watch.distance_m.value &&
                watch.radius_m == 50,
            "Normal upstream anchor result copied");
      Check(!watch.alarm, "At-anchor fixture not an alarm");
      Check(ClearAnchor(anchor_id).ok, "Clear upstream watch");
      Check(!ClearAnchor(anchor_id).ok, "Cleared watch cannot be reused");
      Check(!pWayPointMan->FindWaypointByGuid(anchor_id),
            "Clearing owned anchor removes its isolated mark");
      Record(
          "Anchor observation, whole-metre label, anchor icon and owned-mark removal");
      Check(!StartAnchor(selected, 50.25).ok,
            "Fractional radius is not silently rounded");
      auto user_watch = StartAnchor(selected, 60);
      Check(user_watch.ok, "Second anchor watch");
      auto *user_mark = pWayPointMan->FindWaypointByGuid(user_watch.identity);
      user_mark->m_MarkDescription = "User-owned anchorage";
      Check(NavObj_dB::GetInstance().UpdateRoutePoint(user_mark),
            "Persist user annotation");
      Check(ClearAnchor(user_watch.identity).ok &&
                pWayPointMan->FindWaypointByGuid(user_watch.identity) == user_mark,
            "Clearing user-repurposed watch preserves its waypoint");
      auto old_watch = StartAnchor(selected, 70);
      Check(old_watch.ok, "Create old-release ownership fixture");
      auto *old_mark = pWayPointMan->FindWaypointByGuid(old_watch.identity);
      old_mark->SetName("70.000000");
      old_mark->SetIconName("diamond");
      old_mark->m_MarkDescription =
          "OpenNav anchor watch; radius stored using OpenCPN semantics";
      Check(NavObj_dB::GetInstance().UpdateRoutePoint(old_mark),
            "Persist exact Beta 1 watch shape");
      Check(ClearAnchor(old_watch.identity).ok &&
                !pWayPointMan->FindWaypointByGuid(old_watch.identity),
            "Clearing an upgraded Beta 1 watch removes its owned mark");
      Record("Anchor ownership preserves repurposed user marks and recognizes Beta 1 watches");
      Check(g_pAIS != nullptr, "AIS service exists");
      target = std::make_shared<AisTargetData>(AisTargetCallbacks{});
      target->MMSI = 990000001;
      std::strcpy(target->ShipName, "ALPHA TEST AIS");
      target->b_nameValid = true;
      target->b_positionOnceValid = true;
      target->b_positionDoubtful = false;
      target->b_active = true;
      target->b_lost = false;
      target->Lat = gLat + .02;
      target->Lon = gLon + .03;
      target->SOG = 7;
      target->COG = 210;
      target->HDG = 211;
      target->Range_NM = 1.234;
      target->Brg = 37;
      target->CPA = .42;
      target->TCPA = 7.5;
      target->bCPA_Valid = true;
      target->PositionReportTicks = AisTicksNow();
      g_pAIS->GetTargetList()[target->MMSI] = target;
      auto ais = CopyAisState(selected, Clock::now());
      auto it = std::find_if(ais.targets.begin(), ais.targets.end(),
                             [](const auto &t) { return t.mmsi == 990000001; });
      Check(it != ais.targets.end() && it->range_nm.value == 1.234 &&
                it->cpa_nm.value == .42 && it->tcpa_minutes.value == 7.5,
            "AIS copies upstream results without calculation");
      auto retained = *it;
      for (int read = 0; read < 64; ++read)
        Check(CopyAisState(selected, Clock::now()).targets.front().latitude_deg.observed_at ==
                  retained.latitude_deg.observed_at,
              "Repeated reads preserve exact AIS position observation epoch");
      target->b_lost = true;
      auto lost = CopyAisState(selected, Clock::now());
      Check(!lost.targets.front().cpa_nm.value,
            "Lost AIS suppresses relative data");
      target->b_lost = false;
      target->SOG = 102.3;
      Check(!CopyAisState(selected, Clock::now()).targets.front().sog_kn.value,
            "AIS NA speed not shown as valid");
      target->SOG = 7;
      target->PositionReportTicks -= 70;
      auto stale = CopyAisState(selected, Clock::now());
      Check(Assess(stale.targets.front().range_nm, Clock::now()).quality ==
                Quality::Stale,
            "Repeated copy preserves AIS age");
      target->PositionReportTicks = AisTicksNow();
      g_pAIS->GetTargetList().erase(target->MMSI);
      Check(retained.cpa_nm.value == .42, "AIS copy survives target removal");
      g_pAIS->GetTargetList()[target->MMSI] = target;
      ais_feed = std::make_unique<AisFixtureFeed>();
      ais_feed->Start(100);
      Record("AIS upstream CPA/TCPA, unavailable sentinel, loss, freshness and "
             "lifetime");
      gFrame->GetPrimaryCanvas()->ShowRoutePropertiesDialog("Test route",
                                                            test_route);
      report["phase"] = wxString("route-card");
    } else if (step == 4) {
      // Exact options-close path from the pinned source, while an XNav object
      // page has hidden the native canvas. Rebuilding must reconcile its AUI
      // pane immediately, without requiring an application restart.
      if (!settings_capture_started) {
        gFrame->ScheduleReconfigAndSettingsReload(false, false);
        auto *canvas = gFrame->GetPrimaryCanvas();
        Check(canvas && canvas->IsShown() && canvas->GetClientSize().x > 100 &&
                  canvas->GetClientSize().y > 100,
              "Settings reconfiguration restores visible usable chart canvas");
        Record("Options canvas reconfiguration from hidden XNav page restores chart");
        report["phase"] = wxString("settings-return");
        settings_capture_started = true;
      }
      if (!wxFileExists(wxString::FromUTF8(directory) + "/settings-return-observed")) {
        Check(++settings_capture_waited < 15, "Settings chart screenshot was not observed");
        Write();
        return;
      }
    } else if (step == 5) {
      gFrame->GetPrimaryCanvas()->ShowMarkPropertiesDialog(
          pWayPointMan->FindWaypointByGuid(mark_id));
      report["phase"] = wxString("waypoint-card");
    } else if (step == 8) {
      ShowAISTargetQueryDialog(gFrame->GetPrimaryCanvas(), target->MMSI);
      report["phase"] = wxString("ais-card");
    } else if (step == 9) {
      // Explicit decoder-state fixture, not a second collision calculation.
      // The no-dialog upstream state allows testing the advisory presentation
      // without acknowledging or suppressing a real device alarm.
      target->n_alert_state = AIS_ALERT_NO_DIALOG_SET;
      advisory_fixture = true;
      report["phase"] = wxString("ais-advice");
    } else if (step == 10 && !wxFileExists(wxString::FromUTF8(directory)+"/ais-advice-observed")) {
      Check(++advice_waited < 15, "Actual shell AIS advice observation timed out");
      Write();
      return;
    } else if (step == 11) {
      advisory_fixture = false;
      target->n_alert_state = AIS_NO_ALERT;
      Check(retained_route.points.front().id != RouteCopy().points.front().id,
            "Retained pre-reversal route remains independent");
      report["result"] = wxString("passed");
      report["phase"] = wxString("done");
      Record(
          "Route, waypoint and AIS chart selection use owned deferred cards");
      finished = true;
    }
    if (target)
      target->PositionReportTicks = AisTicksNow();
    ++step;
    Write();
  } catch (const std::exception &e) {
    report["result"] = wxString("failed");
    report["error"] = wxString::FromUTF8(e.what());
    finished = true;
    Write();
  }
}
} // namespace opennav::test

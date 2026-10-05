#include "RouteProgressScenario.h"
#include "integration/OpenCPNIntegration.h"
#include "concanv.h"
#include "model/route.h"
#include "model/route_point.h"
#include "model/routeman.h"
#include "model/own_ship.h"
#include "model/comm_drv_registry.h"
#include "ocpn_plugin.h"
#include "ocpn_frame.h"
#include "chcanv.h"
#include <wx/jsonwriter.h>
#include <wx/filename.h>
#include <wx/filefn.h>
#include <wx/thread.h>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

extern bool g_bDeferredInitDone;
extern APConsole* console;
extern MyFrame* gFrame;
namespace opennav::test {
using namespace vessel;
using namespace std::chrono_literals;
namespace {
std::string directory;
int step=0, waited=0;
bool finished=false;
Route* route=nullptr; // Test driver only; never exposed to consumers.
RouteProgress retained;
wxJSONValue report;
wxString third_id;
void Check(bool condition,const char* reason) {if(!condition) throw std::runtime_error(reason);}
void Write() {
  wxJSONWriter writer;wxString text;writer.Write(report,text);
  const wxString path=wxString::FromUTF8(directory)+"/route-fixture-results.json";
  const auto pending=path+".pending";
  {std::ofstream out(pending.ToStdString(wxConvUTF8),std::ios::binary);out<<text.ToStdString(wxConvUTF8);}
  Check(wxRenameFile(pending,path,true),"Cannot publish route fixture evidence");
}
void Record(const char* label,const RouteProgress& s) {
  wxJSONValue entry;
  entry["check"]=wxString::FromUTF8(label);
  entry["state"]=wxString::FromUTF8(RouteStateName(s->state));
  entry["route_id"]=wxString::FromUTF8(s->route_id);
  entry["revision_scope"]=wxString::FromUTF8(s->revision_scope);
  entry["route_revision"]=wxString::Format("%llu",static_cast<unsigned long long>(s->route_revision));
  entry["active_waypoint_id"]=wxString::FromUTF8(s->active_waypoint_id);
  if(s->active_waypoint_index) entry["active_waypoint_index"]=static_cast<int>(*s->active_waypoint_index);
  if(s->remaining_distance_nm) entry["remaining_distance_nm"]=*s->remaining_distance_nm;
  if(s->cross_track_error_nm) entry["cross_track_error_nm"]=*s->cross_track_error_nm;
  if(s->cross_track_direction) entry["cross_track_direction_to_steer"]=
      *s->cross_track_direction==CrossTrackDirection::Left ? "left" : "right";
  entry["observed_steady_ms"]=wxString::Format("%lld",static_cast<long long>(std::chrono::duration_cast<Duration>(s->observed_at.time_since_epoch()).count()));
  if(s->position_observed_at) entry["position_steady_ms"]=wxString::Format("%lld",static_cast<long long>(std::chrono::duration_cast<Duration>(s->position_observed_at->time_since_epoch()).count()));
  entry["source"]=wxString::FromUTF8(s->source);entry["position_source"]=wxString::FromUTF8(s->position_source);
  // Actual upstream pixel projection for isolated rendered-line checks. These
  // test-only copies are never a replacement route or progress calculation.
  if (route && gFrame && gFrame->GetPrimaryCanvas() &&
      g_pRouteMan->GetpActiveRoute() == route) {
    auto *canvas = gFrame->GetPrimaryCanvas();
    for (int i = 1; i <= route->GetnPoints(); ++i) {
      auto *point = route->GetPoint(i);
      wxPoint pixel;
      canvas->GetCanvasPointPix(point->m_lat, point->m_lon, &pixel);
      pixel = canvas->ClientToScreen(pixel);
      wxJSONValue item;
      item["x"] = pixel.x; item["y"] = pixel.y;
      entry["route_pixels"].Append(item);
    }
    const auto stock = g_pRouteMan->GetActiveRoutePen()->GetColour();
    entry["stock_active_ink"].Append(stock.Red());
    entry["stock_active_ink"].Append(stock.Green());
    entry["stock_active_ink"].Append(stock.Blue());
  }
  report["checks"].Append(entry);
}
void Invalid(const RouteProgress& s,const char* label) {
  Check(s && s->state!=RouteState::Valid && !s->remaining_distance_nm &&
            !s->cross_track_error_nm && !s->cross_track_direction,label);Record(label,s);
}
void Valid(const RouteProgress& s,std::size_t index,const char* label) {
  Check(s && s->state==RouteState::Valid && s->active_waypoint_index==index && s->remaining_distance_nm,label);
  Check(AssessRoute(*s,Clock::now()).remaining_distance_nm.has_value(),"Freshness rejected valid fixture");
  double upstream=g_pRouteMan->GetCurrentRngToActivePoint();
  for(int i=static_cast<int>(index)+2;i<=route->GetnPoints();++i) upstream+=route->GetPoint(i)->m_seg_len;
  Check(std::abs(*s->remaining_distance_nm-upstream)<1e-9,"Snapshot differs from actual normal upstream progress");
  Check(s->cross_track_error_nm && s->cross_track_direction &&
            *s->cross_track_error_nm == g_pRouteMan->GetCurrentXTEToActivePoint() &&
            *s->cross_track_direction == (g_pRouteMan->GetXTEDir() < 0
                ? CrossTrackDirection::Left : CrossTrackDirection::Right),
        "Cross-track magnitude and direction match actual completed native progress");
  Check(console && !console->IsShown(),
        "XNav active route exposes the Legacy navigation console over the data rail");
  report["xnav_legacy_console_hidden"]=true;
  Record(label,s);
}
void NewRoute() {
  Check(pRouteList && pRouteList->IsEmpty(),"Route fixture requires an empty disposable profile");
  // Even in this opt-in test build, reject any transport which can send output.
  for(const auto& driver:GetActiveDrivers()) {
    const auto& a=GetAttributes(driver);const auto it=a.find("ioDirection");
    Check(it==a.end() || it->second=="IN","Route fixture refuses output-capable connections");
  }
  route=new Route;route->m_GUID="OPENNAV-TEST-route";route->m_RouteNameString="SIMULATED route contract";
  for(int i=0;i<3;++i) {
    auto* p=new RoutePoint(gLat+0.1*(i+1),gLon+0.1*(i+1),"diamond",
                           wxString::Format("SIM %d",i+1),wxString::Format("OPENNAV-TEST-point-%d",i+1));
    p->SetWaypointArrivalRadius(-1);route->AddPoint(p,false);
  }
  pRouteList->Append(route);g_pRouteMan->ActivateRoute(route,route->GetPoint(1));
  // RouteManagerDialog::OnRteActivateClick requests this repaint after its
  // mutation. A fixture-created route must do the same: background navigation
  // observations do not promise a canvas repaint on every platform. This only
  // schedules painting; normal upstream processing still owns route progress.
  gFrame->RefreshAllCanvas();
}
}  // namespace
void EnableRouteScenario(const std::string& profile) {
  Check(!profile.empty() && wxFileExists(wxString::FromUTF8(profile)+"/OPENNAV_ROUTE_FIXTURE"),
        "Route fixture requires its explicit disposable profile marker");
  directory=profile;report["result"]=wxString::FromUTF8("running");report["phase"]=wxString::FromUTF8("input");
  report["fixture"]=wxString::FromUTF8("Synthetic loopback position and test routes; no device output");Write();
}
void RouteScenarioStep(const RouteProgress& s) {
  if(directory.empty() || finished || !g_bDeferredInitDone) return;
  try {
    Check(wxIsMainThread(),"Route scenario left the application thread");
    Check(++waited<90,"Route fixture timed out waiting for normal processing");
    bool repaint = false;
    switch(step) {
      case 0:
        if(!s->position_observed_at || Clock::now()-*s->position_observed_at>2s) return;
        Invalid(s,"no active route");NewRoute();break;
      case 1:Invalid(s,"activation awaiting coherent pass");break;
      case 2:repaint=true;Valid(s,0,"first point real upstream progress");retained=s;
        g_pRouteMan->ActivateRoutePoint(route,route->GetPoint(2));Invalid(CurrentRouteProgress(),"consumer invalidates point change");break;
      case 3:Invalid(s,"changed active point");break;
      case 4:repaint=true;Valid(s,1,"middle point real upstream progress");
        Check(g_pRouteMan->ActivateNextPoint(route,true),"Cannot skip fixture point");break;
      case 5:Invalid(s,"skipped point transition");break;
      case 6:repaint=true;Valid(s,2,"final point real upstream progress");
        route->GetPoint(1)->m_lat=gLat;route->GetPoint(1)->m_lon=gLon;
        route->GetPoint(1)->SetWaypointArrivalRadius(0.1);route->UpdateSegmentDistances();
        g_pRouteMan->ActivateRoutePoint(route,route->GetPoint(1));break;
      case 7:Invalid(s,"arrival during normal progress rejects old range");
        Check(g_pRouteMan->GetpActivePoint()==route->GetPoint(2),"Upstream did not advance on arrival");break;
      case 8:repaint=true;Valid(s,1,"coherent pass after arrival");route->Reverse(false);
        g_pRouteMan->ActivateRoutePoint(route,route->GetPoint(2));break;
      case 9:Invalid(s,"actual route reversal");break;
      case 10:repaint=true;Valid(s,1,"coherent reversed route");route->GetPoint(3)->m_lat+=0.02;
        route->UpdateSegmentDistances();break;
      case 11:Invalid(s,"actual active route edit");break;
      case 12:Valid(s,1,"coherent edited route");third_id=route->GetPoint(3)->m_GUID;
        route->GetPoint(3)->m_GUID=route->GetPoint(2)->m_GUID;break;
      case 13:Invalid(s,"repeated waypoint identity");route->GetPoint(3)->m_GUID=third_id;break;
      case 14:Invalid(s,"restored geometry requires new pass");break;
      case 15:Valid(s,1,"restored route");report["phase"]=wxString::FromUTF8("stop-input");break;
      case 16:
        if(s->state!=RouteState::StalePosition) return;
        Invalid(s,"stopped loopback position becomes stale");report["phase"]=wxString::FromUTF8("resume-input");break;
      case 17:repaint=true;
        if(s->state!=RouteState::Valid) return;
        Valid(s,1,"fresh position resumes route contract");g_pRouteMan->DeactivateRoute();
        Invalid(CurrentRouteProgress(),"immediate deactivation invalidation");break;
      case 18:repaint=true;
        Invalid(s,"deactivated route remains unavailable");
        g_pRouteMan->ActivateRoute(route,route->GetPoint(2));
        Invalid(CurrentRouteProgress(),"reactivation awaiting normal progress");break;
      case 19:Invalid(s,"reactivated route transition");break;
      case 20:repaint=true;
        Valid(s,1,"active route immediately before deletion");
        Check(g_pRouteMan->DeleteRoute(route),"Cannot delete active fixture route");route=nullptr;
        Check(retained && retained->remaining_distance_nm && retained->active_waypoint_id=="OPENNAV-TEST-point-1",
              "Retained immutable snapshot did not survive deletion");
        Invalid(CurrentRouteProgress(),"active deletion invalidates immediately; retained copy safe");break;
      case 21:
        Invalid(s,"deleted active route remains unavailable on next normal pass");
        report["result"]=wxString::FromUTF8("passed");report["phase"]=wxString::FromUTF8("done");finished=true;break;
    }
    // Direct fixture mutations bypass the route UI commands which normally
    // request canvas repaint (for example OnRteActivateClick/OnRteReverseClick).
    // Software blits only its invalidated region, unlike the continuous GL
    // path: notify after those mutations, without performing navigation work.
    if (repaint) gFrame->RefreshAllCanvas();
    ++step;Write();
  } catch(const std::exception& e) {
    report["result"]=wxString::FromUTF8("failed");report["error"]=wxString::FromUTF8(e.what());finished=true;Write();
  }
}
}  // namespace opennav::test

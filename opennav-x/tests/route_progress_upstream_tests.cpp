#include "integration/OpenCPNRouteReader.h"
#include "integration/RoutePassWatch.h"
#include "model/georef.h"
#include "model/route.h"
#include "model/route_point.h"
#include "model/routeman.h"
#include "model/own_ship.h"
#include <gtest/gtest.h>
#include <wx/app.h>
#include <wx/init.h>
#include <memory>
#include <thread>

using namespace opennav;
using namespace opennav::integration;
using namespace opennav::vessel;
using namespace std::chrono_literals;
namespace {
class OpenNavRouteGeometry : public ::testing::Test {
 protected:
  std::unique_ptr<wxInitializer> initializer;
  WayPointman* previous_waypoints = pWayPointMan;
  RouteList* previous_routes = pRouteList;
  Routeman* previous_manager = g_pRouteMan;
  std::unique_ptr<WayPointman> waypoints;
  RouteList routes;
  std::unique_ptr<Routeman> manager;
  std::unique_ptr<Route> route;
  void SetUp() override {
    wxApp::SetInstance(new wxAppConsole);
    initializer=std::make_unique<wxInitializer>();
    ASSERT_TRUE(initializer->IsOk());
    waypoints = std::make_unique<WayPointman>([](wxString){return wxColour(0,0,0);});
    waypoints->m_pLegacyIconArray = new SortedArrayOfMarkIcon([](MarkIcon*, MarkIcon*){return 0;});
    waypoints->m_pExtendedIconArray = new SortedArrayOfMarkIcon([](MarkIcon*, MarkIcon*){return 0;});
    pWayPointMan = waypoints.get(); pRouteList = &routes;
    manager = std::make_unique<Routeman>(RoutePropDlgCtx{}, RoutemanDlgCtx{});
    g_pRouteMan = manager.get();
    route = std::make_unique<Route>();
    route->m_GUID="synthetic-antimeridian-route";
    // Both sides of 180; actual upstream AddPoint stores Mercator leg lengths.
    route->AddPoint(new RoutePoint(10,179.7,"","A","A"),false,true);
    route->AddPoint(new RoutePoint(10,-179.8,"","B","B"),false,true);
    route->AddPoint(new RoutePoint(11,-179.5,"","C","C"),false,true);
    routes.Append(route.get());
  }
  void TearDown() override {
    manager->DeactivateRoute(); manager.reset(); route.reset();
    routes.Clear(); waypoints.reset();
    pWayPointMan=previous_waypoints; pRouteList=previous_routes; g_pRouteMan=previous_manager;
    initializer.reset();
  }
  RouteRead Read(std::size_t index) {
    manager->ActivateRoute(route.get(),route->GetPoint(static_cast<int>(index)+1));
    RouteRead r;
    r.route=CopyActiveRoute(manager.get());
    const Time t{100s};
    r.position.latitude_deg={10,"synthetic position",t,Validity::Measured};
    r.position.longitude_deg={179.5,"synthetic position",t,Validity::Measured};
    r.upstream_position_valid=true; r.upstream_latitude_deg=10; r.upstream_longitude_deg=179.5;
    const auto* p=manager->GetpActivePoint();
    // Same pinned function as RoutemanGui::UpdateProgress. The production
    // bridge only reads GetCurrentRngToActivePoint; it never calls geometry.
    r.range_to_active_nm=DistGreatCircle(10,179.5,p->m_lat,p->m_lon);
    return r;
  }
};
TEST_F(OpenNavRouteGeometry, AntimeridianFirstMiddleFinalMatchUpstream) {
  ASSERT_LT(route->GetPoint(2)->m_seg_len,100); // Crossing 180 is the short leg.
  for(std::size_t index=0;index<3;++index) {
    const auto r=Read(index);
    double expected=*r.range_to_active_nm;
    float console_distance=static_cast<float>(*r.range_to_active_nm);
    bool following=false;
    for(auto* node=route->pRoutePointList->GetFirst();node;node=node->GetNext()) {
      const auto* p=node->GetData();
      if(following) {expected+=p->m_seg_len;console_distance+=static_cast<float>(p->m_seg_len);}
      if(p==route->m_pRouteActivePoint) following=true;
    }
    RouteProgressInput input("upstream-test");input.Complete(r,r,Time{100s});
    ASSERT_EQ(input.Current()->state,RouteState::Valid);
    ASSERT_TRUE(input.Current()->remaining_distance_nm);
    EXPECT_DOUBLE_EQ(*input.Current()->remaining_distance_nm,expected);
    EXPECT_NEAR(*input.Current()->remaining_distance_nm,console_distance,0.001);
  }
}
TEST_F(OpenNavRouteGeometry, ActualReverseAndEditChangeRevision) {
  auto a=Read(1);RouteProgressInput input("upstream-test");input.Complete(a,a,Time{100s});
  const auto retained=input.Current();
  route->Reverse(false); auto b=Read(1);
  input.Complete(a,b,Time{101s});EXPECT_EQ(input.Current()->state,RouteState::RouteChanged);
  EXPECT_GT(input.Current()->route_revision,retained->route_revision);
  input.Complete(b,b,Time{102s});EXPECT_EQ(input.Current()->state,RouteState::Valid);
  route->GetPoint(3)->m_lat=12;route->UpdateSegmentDistances();auto c=Read(1);
  input.Complete(b,c,Time{103s});EXPECT_EQ(input.Current()->state,RouteState::RouteChanged);
  EXPECT_TRUE(retained->remaining_distance_nm); // Owned copies survive mutation.
}
TEST_F(OpenNavRouteGeometry, RemovedRouteIsNotDereferencedAndSnapshotSurvivesDeletion) {
  auto a=Read(1);RouteProgressInput input("upstream-test");input.Complete(a,a,Time{100s});
  auto retained=input.Current();routes.DeleteObject(route.get());
  EXPECT_FALSE(CopyActiveRoute(manager.get()).registered);
  manager->DeactivateRoute();route.reset();waypoints.reset();pWayPointMan=nullptr;
  input.CheckCurrent(CopyActiveRoute(manager.get()),Time{101s});
  EXPECT_EQ(input.Current()->state,RouteState::NoActiveRoute);
  EXPECT_TRUE(retained->remaining_distance_nm);EXPECT_EQ(retained->active_waypoint_id,"B");
}
TEST_F(OpenNavRouteGeometry, RepeatedObjectAndGuidAreRejected) {
  auto a=Read(1);route->pRoutePointList->Append(route->GetPoint(2));
  a.route=CopyActiveRoute(manager.get());RouteProgressInput input("upstream-test");input.Complete(a,a,Time{100s});
  EXPECT_FALSE(input.Current()->remaining_distance_nm);
  route->pRoutePointList->DeleteNode(route->pRoutePointList->GetLast());
  route->GetPoint(3)->m_GUID="B";a.route=CopyActiveRoute(manager.get());
  RouteProgressInput second("upstream-test");second.Complete(a,a,Time{100s});
  EXPECT_EQ(second.Current()->state,RouteState::AmbiguousPoint);
}
TEST_F(OpenNavRouteGeometry, NestedEventGuardRejectsEditThenRestore) {
  auto a=Read(1);RoutePassWatch watch;
  wxEvtHandler handler;
  handler.Bind(wxEVT_BUTTON,[&](wxCommandEvent&) {
    const double lat=route->GetPoint(3)->m_lat;
    route->GetPoint(3)->m_lat=12;route->UpdateSegmentDistances();
    route->GetPoint(3)->m_lat=lat;route->UpdateSegmentDistances();
  });
  wxCommandEvent event(wxEVT_BUTTON);handler.ProcessEvent(event);
  EXPECT_TRUE(watch.Finish());
  auto b=Read(1);EXPECT_TRUE(SameRoute(a.route,b.route));b.interrupted=true;
  RouteProgressInput input("upstream-test");input.Complete(a,b,Time{100s});
  EXPECT_EQ(input.Current()->state,RouteState::InterruptedPass);
  EXPECT_FALSE(input.Current()->remaining_distance_nm);
}
TEST_F(OpenNavRouteGeometry, CopyRefusesWorkerThread) {
  bool rejected=false;
  std::thread worker([&]{try {CopyActiveRoute(manager.get());}catch(const std::logic_error&){rejected=true;}});
  worker.join();EXPECT_TRUE(rejected);
}
}  // namespace

// Headless seam around the exact production resolver, not a replacement algorithm.
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <vector>
#include <initializer_list>
using wxString = std::string;
struct wxColour {};
struct ChartCanvas {};
bool main_thread = true, style = true;
bool wxIsMainThread() { return main_thread; }
std::size_t steps = 0;
template<class T> struct List {
  struct Node {
    T* data;
    Node* next = nullptr;
    T* GetData() { return data; }
    Node* GetNext() { ++steps; return next; }
  };
  std::vector<Node> nodes;
  void Set(const std::vector<T*>& items) {
    nodes.clear();
    for (auto* p : items) nodes.push_back({p});
    for (std::size_t i=1; i<nodes.size(); ++i) nodes[i-1].next=&nodes[i];
  }
  Node* GetFirst() { return nodes.empty() ? nullptr : &nodes[0]; }
};
struct RoutePoint {
  wxString icon = "diamond";
  bool m_bIsInRoute=true, shared=false, m_bIsInLayer=false, m_bIsActive=false,
       m_bPtIsSelected=false, m_bRPIsBeingEdited=false, drag=false;
  wxString GetIconName() { return icon; }
  bool IsShared() { return shared; }
  bool IsDragHandleEnabled() { return drag; }
};
struct Route {
  List<RoutePoint> list;
  List<RoutePoint>* pRoutePointList=&list;
  bool eligible=true, m_bIsBeingCreated=false,
       m_bRtIsSelected=false, m_bRtIsActive=true;
  bool m_bIsBeingEdited=false;
  int m_hiliteWidth=0, m_width=-1, m_style=-1;
  std::string m_Colour;
  bool IsVisible() { return eligible; }
};
struct Routeman {
  Route* active=nullptr;
  RoutePoint* point=nullptr;
  Route* GetpActiveRoute() { return active; }
  RoutePoint* GetpActivePoint() { return point; }
};
Routeman manager;
Routeman* g_pRouteMan=&manager;
List<Route> routes;
List<Route>* pRouteList=&routes;
RoutePoint* pAnchorWatchPoint1=nullptr;
RoutePoint* pAnchorWatchPoint2=nullptr;
wxString g_default_wp_icon="circle", g_default_routepoint_icon="diamond";
bool ChartActiveRouteInk(ChartCanvas&, wxColour&) { return style; }
constexpr int WIDTH_UNDEFINED=-1, wxPENSTYLE_INVALID=-1;
int g_route_line_width=2;
PRODUCTION_ROUTE_STYLE
double MarkerScale(ChartCanvas&) { return 1; }
PRODUCTION_CODES
PRODUCTION_GENERIC
PRODUCTION_MARKER
REFERENCE_MARKER
int checks=0;
void Check(bool good, const char* why) {
  ++checks;
  if (!good) { std::cerr << why << '\n'; std::exit(1); }
}
ChartCanvas canvas;
int Resolve(RoutePoint& point, bool pinned=true) {
  const int actual=ChartWaypointMarker(canvas,point,pinned);
#if HAS_REFERENCE
  const auto saved=steps;
  Check(actual==ReferenceMarker(canvas,point,pinned),"Previous production result differs");
  steps=saved;
#endif
  return actual;
}
DecodedWaypointMarker Marker(RoutePoint& p) { return DecodeWaypointMarker(Resolve(p)); }
int main() {
  Route a,b;
  RoutePoint p,q,r,s;
  routes.Set({&a}); a.list.Set({&p,&q,&r}); manager.active=&a; manager.point=&q;
  Check(Marker(p).kind==WaypointMarkerKind::Visited,"Point before active is visited");
  Check(Marker(q).kind==WaypointMarkerKind::RoutePoint,"Pointer alone does not imply active flag");
  q.m_bIsActive=true;
  Check(Marker(q).kind==WaypointMarkerKind::Active,"Actual active point keeps active marker");
  Check(Marker(r).kind==WaypointMarkerKind::RoutePoint,"Point after active remains unvisited");
  a.list.Set({&q,&p,&q,&r});
  Check(Marker(p).kind==WaypointMarkerKind::RoutePoint,"First active occurrence controls progress");
  a.list.Set({&p,&q,&p,&r});
  Check(Marker(p).ordinal==0 && Marker(p).kind==WaypointMarkerKind::RoutePoint,
        "Repeated point never gains visited classification");
  a.list.Set({&p,&q,&r}); b.list.Set({&p}); routes.Set({&a,&b});
  Check(Marker(p).ordinal==0 && Marker(p).kind==WaypointMarkerKind::RoutePoint,
        "Hidden/shared route occurrence remains unnumbered and unvisited");
  b.eligible=false; Check(!Resolve(p),"Custom route owns shared point presentation"); b.eligible=true;
  b.m_bIsBeingCreated=true; Check(!Resolve(p),"Creating route owns shared point presentation"); b.m_bIsBeingCreated=false;
  routes.Set({&b,&a}); b.list.Set({&q});
  Check(Marker(p).kind==WaypointMarkerKind::Visited,"Other route active-pointer occurrence cannot replace owner index");
  routes.Set({&a}); manager.point=&s;
  Check(Marker(p).kind==WaypointMarkerKind::RoutePoint,"Missing active point leaves route unvisited");
  manager.point=nullptr;
  Check(Marker(p).kind==WaypointMarkerKind::RoutePoint,"No active point leaves route unvisited");
  manager.point=&q; manager.active=&b;
  Check(Marker(p).kind==WaypointMarkerKind::RoutePoint,"Different active route leaves route unvisited");
  manager.active=&a; p.shared=true;
  Check(Marker(p).ordinal==0 && Marker(p).kind==WaypointMarkerKind::Visited,"Shared flag keeps existing visited semantics"); p.shared=false;
  a.m_bRtIsSelected=true;
  Check(Marker(p).role==WaypointMarkerRole::SelectedRoute,"Selected route role retained"); a.m_bRtIsSelected=false;
  a.m_bRtIsActive=false;
  Check(Marker(p).role==WaypointMarkerRole::Inactive,"Inactive route role retained"); a.m_bRtIsActive=true;
  for (bool* flag : {&p.m_bIsInLayer,&p.drag}) { *flag=true; Check(!Resolve(p),"Protected icon remains stock"); *flag=false; }
  pAnchorWatchPoint1=&p; Check(!Resolve(p),"Anchor watch remains stock"); pAnchorWatchPoint1=nullptr;
  p.icon="mob"; Check(!Resolve(p),"MOB remains stock"); p.icon="fuel";
  Check(!Resolve(p,false),"Custom icon remains stock"); p.icon="diamond";
  style=false; Check(!Resolve(p),"Legacy mode remains stock"); style=true;
  main_thread=false; Check(!Resolve(p),"Wrong thread remains stock"); main_thread=true;
  g_pRouteMan=nullptr; Check(!Resolve(p),"Missing route manager remains stock"); g_pRouteMan=&manager;
  pRouteList=nullptr; Check(!Resolve(p),"Missing route library remains stock"); pRouteList=&routes;
  p.m_bIsInRoute=false;
  Check(Marker(p).kind==WaypointMarkerKind::Standalone,"Standalone marker needs no route walk"); p.m_bIsInRoute=true;
  q.icon="mob"; Check(!Resolve(p),"MOB elsewhere in owner preserves emergency artwork"); q.icon="diamond";
  // Deterministic differential coverage: topology, duplicate pointers, state flags,
  // active pointer/owner changes between calls. There is no cross-call cache.
  std::mt19937 rng(317);
  std::vector<RoutePoint> points(8);
  for (int trial=0; trial<2000; ++trial) {
    for (auto* route : {&a,&b}) {
      std::vector<RoutePoint*> contents;
      for (unsigned i=0,n=rng()%12; i<n; ++i) contents.push_back(&points[rng()%points.size()]);
      route->list.Set(contents); route->eligible=rng()%4!=0;
      route->m_bIsBeingCreated=rng()%9==0; route->m_bRtIsActive=rng()%2;
      route->m_bRtIsSelected=rng()%2;
    }
    routes.Set(rng()%2 ? std::vector<Route*>{&a,&b} : std::vector<Route*>{&b,&a});
    manager.active=rng()%3==0 ? nullptr : (rng()%2 ? &a : &b);
    manager.point=rng()%3==0 ? nullptr : &points[rng()%points.size()];
    for(auto& point:points) {
      point.m_bIsActive=rng()%3==0; point.shared=rng()%4==0;
      point.m_bPtIsSelected=rng()%4==0; point.m_bRPIsBeingEdited=rng()%4==0;
      Resolve(point);
    }
  }
  a.eligible=true; a.m_bIsBeingCreated=false; a.m_bRtIsSelected=false; a.m_bRtIsActive=true;
  routes.Set({&a}); manager.active=&a;
  std::vector<RoutePoint> large(4096);
  std::vector<RoutePoint*> contents;
  for(auto& point:large) contents.push_back(&point);
  contents.pop_back(); a.list.Set(contents); manager.point=&large[4094]; large[4094].m_bIsActive=true;
  Check(Marker(large[100]).ordinal==0 && Marker(large[100]).kind==WaypointMarkerKind::Visited,
        "Raw point index above 99 still classifies visited");
  steps=0; Check(Marker(large[0]).kind==WaypointMarkerKind::Visited,"Max-budget point remains visited");
  std::cout << "4095-point route: " << steps << " node advances per marker\n";
  Check(steps==8191,"Only library walk plus unchanged emergency-style scan; no owner progress rescan");
  contents.push_back(&large[4095]); a.list.Set(contents);
  Check(!Resolve(large[0]),"4097 combined route/point nodes exceed unchanged budget");
  // Same object, changed order/active pointer: no stale progress or ordinal.
  a.list.Set({&r,&p,&q}); manager.point=&p;
  Check(Marker(r).ordinal==1 && Marker(r).kind==WaypointMarkerKind::Visited,"Reordering is observed immediately");
  manager.point=&r;
  Check(Marker(p).ordinal==2 && Marker(p).kind==WaypointMarkerKind::RoutePoint,"Active pointer change is immediate");
  std::cout << checks << " marker semantic/cost checks passed\n";
}

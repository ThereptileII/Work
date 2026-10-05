#!/usr/bin/env python3
"""Generate an offline native test of the actual route activation transition.

Only the OpenCPN storage/notification boundary is replaced by mutable fixtures.
The transition function and shared commit/consent policy are production inputs.
"""
import argparse
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
source = args.source.read_text(encoding='utf-8')
signature = 'application::CommandResult ActivateRouteTransition('
start = source.index(signature)
brace = source.index('{', start)
depth = 1
end = brace + 1
while depth:
    depth += (source[end] == '{') - (source[end] == '}')
    end += 1
production = source[start:end]
preamble = r'''
#include "application/AnchorRouteTransition.h"
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
namespace application=opennav::application;
namespace vessel=opennav::vessel;
void Check(bool value,const char *why) { if(!value) throw std::runtime_error(why); }
struct wxString : std::string {
  using std::string::string;
  wxString()=default;
  wxString(const std::string &value):std::string(value){}
  static wxString FromUTF8(const std::string &value) { return value; }
  void Clear() { clear(); }
};
struct wxJSONValue { std::map<std::string,wxString> fields; wxString &operator[](const char *key) { return fields[key]; } };
struct RoutePoint { std::string id; };
struct Route {
  application::Route data;
  bool registered=true;
  RoutePoint first{"first"};
  void Alive() const { Check(registered,"Stale raw Route pointer dereferenced after callback"); }
  int GetnPoints() const { Alive(); return static_cast<int>(data.points.size()); }
  bool IsVisible() const { Alive(); return data.visible; }
  void SetVisible(bool value) { Alive(); data.visible=value; }
};
RoutePoint anchor1{"anchor1"},anchor2{"anchor2"},replacement_anchor{"replacement-anchor"};
RoutePoint *pAnchorWatchPoint1=nullptr,*pAnchorWatchPoint2=nullptr;
wxString g_AW1GUID,g_AW2GUID;
bool AnchorAlertOn1=false,AnchorAlertOn2=false;
double gLat=57,gLon=16,gCog=90,gSog=3;
std::vector<std::unique_ptr<Route>> routes;
std::function<void()> on_save,on_deactivate,on_notify;
bool save_ok=true,stop_ok=true,gps_valid=true,watch_valid=true,shared_anchor=false;
std::string watch_revision="initial";
int saves=0,stops=0,notifications=0,activations=0,best_reads=0;
Route *active_route=nullptr;
void Thread() {}
bool Position(const vessel::Navigation &,vessel::Time) { return gps_valid && gLat==57 && gLon==16; }
application::Route Copy(Route *route) {
  route->Alive(); auto value=route->data;
  value.active=route==active_route; value.editable=value.editable&&!value.active;
  value.revision += value.visible?"/visible":"/hidden";
  value.revision += value.active?"/active":"/inactive";
  if(shared_anchor) value.revision += pAnchorWatchPoint1?"/watched":"/unwatched";
  return value;
}
Route *Resolve(const application::Route &selected) {
  Route *result=nullptr;
  for(auto &route:routes) if(route->registered && route->data.id==selected.id) {
    if(result) return nullptr;
    result=route.get();
  }
  return result && Copy(result).revision==selected.revision ? result : nullptr;
}
struct Manager {
  Route *GetpActiveRoute() { return active_route; }
  bool IsRouteValid(Route *route) { return route && route->registered; }
  bool DeactivateRoute() {
    ++stops;
    if(!stop_ok) return false;
    active_route=nullptr;
    if(on_deactivate) on_deactivate();
    return true;
  }
  RoutePoint *FindBestActivatePoint(Route *route,double,double,double,double) {
    route->Alive(); ++best_reads; return &route->first;
  }
  bool ActivateRoute(Route *route,RoutePoint *best) {
    route->Alive();
    Check(best==&route->first,"Activation retained a waypoint from an earlier route generation");
    Check(!pAnchorWatchPoint1 && !pAnchorWatchPoint2,"Activation overwrote a callback-created anchor watch");
    Check(!active_route,"Activation overwrote callback-created navigation");
    ++activations; active_route=route; return true;
  }
} manager;
Manager *g_pRouteMan=&manager;
struct NavObj_dB {
  static NavObj_dB &GetInstance() { static NavObj_dB database; return database; }
  bool UpdateRoute(Route *route) { route->Alive(); ++saves; if(on_save) on_save(); return save_ok; }
};
application::AnchorWatchSelection CopyAnchorWatchSelection() {
  application::AnchorWatchSelection result; result.available=watch_valid;
  for(auto *point:{pAnchorWatchPoint1,pAnchorWatchPoint2}) if(point) {
    application::Waypoint copy; copy.id=point->id; copy.revision=watch_revision;
    result.watches.push_back(copy);
  }
  return result;
}
void SendJSONMessageToAllPlugins(const char *,const wxJSONValue &) {
  ++notifications; if(on_notify) on_notify();
}
'''
checks = r'''
Route *AddRoute(const std::string &id) {
  auto value=std::make_unique<Route>(); value->data.id=id; value->data.revision="revision";
  value->data.editable=true; value->data.visible=false; value->data.points.resize(2);
  routes.push_back(std::move(value)); return routes.back().get();
}
void Reset() {
  routes.clear(); on_save={}; on_deactivate={}; on_notify={};
  save_ok=stop_ok=gps_valid=watch_valid=true; shared_anchor=false; watch_revision="initial";
  saves=stops=notifications=activations=best_reads=0; active_route=nullptr; g_pRouteMan=&manager;
  gLat=57; gLon=16;
  pAnchorWatchPoint1=&anchor1; pAnchorWatchPoint2=&anchor2;
  g_AW1GUID="anchor1"; g_AW2GUID="anchor2"; AnchorAlertOn1=AnchorAlertOn2=true;
}
void Mutate(int mutation,Route *target) {
  switch(mutation) {
  case 0: target->registered=false; break;
  case 1: target->registered=false; AddRoute(target->data.id)->data.revision="replacement"; break;
  case 2: target->data.revision+="/edited"; break;
  case 3: target->data.editable=false; break;
  case 4: target->data.points.resize(1); target->data.revision+="/point removed"; break;
  case 5: active_route=AddRoute("callback-active"); break;
  case 6: pAnchorWatchPoint1=&replacement_anchor; g_AW1GUID="replacement-anchor"; break;
  case 7: gps_valid=false; break;
  case 8: gLat=58; break;
  case 9: watch_valid=false; break;
  case 10: g_pRouteMan=nullptr; break;
  }
}
void RunChecks() {
  for(int boundary=0;boundary<3;++boundary) for(int mutation=0;mutation<11;++mutation) {
    Reset(); auto *target=AddRoute("requested");
    if(boundary==1) active_route=AddRoute("previous-active");
    const auto selected=Copy(target); const auto consent=CopyAnchorWatchSelection();
    auto change=[&,target] { Mutate(mutation,target); };
    if(boundary==0) on_save=change;
    if(boundary==1) on_deactivate=change;
    if(boundary==2) on_notify=change;
    const auto result=ActivateRouteTransition(selected,vessel::Navigation{},&consent);
    Check(!result.ok && activations==0 && best_reads==0,"Callback mutation reached point selection/activation");
    if(boundary<2 && mutation!=6)
      Check(pAnchorWatchPoint1==&anchor1 && pAnchorWatchPoint2==&anchor2,"Pre-clear callback failure stopped watches");
    if(boundary==2) Check(notifications==2,"Both actually cleared watch notifications must be delivered");
    if(mutation==6) Check(pAnchorWatchPoint1==&replacement_anchor,"Callback-created watch was overwritten");
  }
  for(bool shared:{false,true}) {
    Reset(); shared_anchor=shared;
    auto *target=AddRoute("requested"); const auto selected=Copy(target);
    const auto consent=CopyAnchorWatchSelection();
    const auto result=ActivateRouteTransition(selected,vessel::Navigation{},&consent);
    Check(result.ok && activations==1 && best_reads==1 && notifications==2,
          "Unchanged route activates once, including watch shared with a route point");
  }
  Reset(); auto *target=AddRoute("requested"); active_route=AddRoute("previous-active");
  auto selected=Copy(target); auto consent=CopyAnchorWatchSelection();
  Check(ActivateRouteTransition(selected,vessel::Navigation{},&consent).ok && stops==1 && activations==1,
        "Normal current-route replacement remains functional");
  Reset(); target=AddRoute("requested"); selected=Copy(target); consent=CopyAnchorWatchSelection();
  save_ok=false; on_save=[target] { target->registered=false; };
  Check(!ActivateRouteTransition(selected,vessel::Navigation{},&consent).ok && notifications==0,
        "Failed save callback deletion cannot dereference route during visibility rollback");
  Reset(); target=AddRoute("requested"); selected=Copy(target);
  pAnchorWatchPoint1=pAnchorWatchPoint2=nullptr; g_AW1GUID.Clear();g_AW2GUID.Clear();
  Check(ActivateRouteTransition(selected,vessel::Navigation{},nullptr).ok && notifications==0 && activations==1,
        "Ordinary no-watch activation stays available");
  std::cout<<"Route activation callback boundaries passed: 33 mutation cases and normal/failure paths\n";
}
int main() {
  try { RunChecks(); return 0; }
  catch(const std::exception &error) { std::cerr<<error.what()<<'\n'; return 1; }
}
'''
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(preamble+'\n'+production+'\n'+checks,encoding='utf-8',newline='\n')

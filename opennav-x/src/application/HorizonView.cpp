#include "application/HorizonView.h"
#include "vessel/AisSelection.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

namespace opennav::application {
namespace {
bool Current(const vessel::Sample &s,vessel::Time now,double low,double high) {
  const auto a=vessel::Assess(s,now);
  return a.value && (a.quality==vessel::Quality::Live || a.quality==vessel::Quality::Aging) &&
      std::isfinite(*a.value) && *a.value>=low && *a.value<=high;
}
bool Position(const vessel::VesselState &s,vessel::Time now) {
  const auto q=PresentPositionHealth(s.navigation,now).state;
  return q==SignalState::Current || q==SignalState::Aging;
}
bool CurrentRoute(const vessel::VesselState &s,vessel::Time now) {
  const auto &r=s.navigation.route;
  return r && Position(s,now) && vessel::AssessRoute(*r,now).state==vessel::RouteState::Valid &&
      r->position_source==s.navigation.latitude_deg.source && r->position_observed_at &&
      *r->position_observed_at<=s.navigation.latitude_deg.observed_at;
}
bool Fresh(vessel::Time observed,vessel::Time now) {
  return observed!=vessel::Time{} && observed<=now && now-observed<std::chrono::seconds(5);
}
std::string Number(double n,int digits) {
  std::ostringstream out;out.imbue(std::locale::classic());
  out<<std::fixed<<std::setprecision(digits)<<n;return out.str();
}
std::string Text(const std::string &s) {
  // Inputs are already normalized, but the view must remain bounded and cannot
  // accept a line break/control sequence as part of a three-line event card.
  if(s.size()>512 || std::any_of(s.begin(),s.end(),[](unsigned char c){return c<32 || c==127;}))return {};
  return s;
}
int Mmsi(const std::string &s) {
  if(s.empty() || s.size()>9)return 0;
  int n=0;const auto parsed=std::from_chars(s.data(),s.data()+s.size(),n);
  return parsed.ec==std::errc{} && parsed.ptr==s.data()+s.size() && n>0 && n<=999999999 ? n : 0;
}
bool AisEventCurrent(const smartnav::AdvisoryEvent &e,const vessel::AisState &ais,vessel::Time now) {
  // Match the existing SmartNav source contract, including each CPA/TCPA
  // sample's own budget. A position is not required merely to display advice.
  const auto metric=[&](const vessel::Sample &s){const auto a=vessel::Assess(s,now);return a.value && *a.value>=0 &&
      (a.quality==vessel::Quality::Live || a.quality==vessel::Quality::Aging || a.quality==vessel::Quality::Estimated);};
  if(!ais.available || ais.simulated || !Fresh(ais.observed_at,now) || ais.targets.size()>2000)return false;
  const auto id=Mmsi(e.identity);if(!id)return false;
  return std::any_of(ais.targets.begin(),ais.targets.end(),[&](const auto &t){return t.mmsi==id &&
      t.origin==vessel::AisOrigin::LocalOpenCPN && t.source==e.source && t.active && !t.lost && !t.doubtful &&
      t.upstream_alarm && metric(t.cpa_nm) && metric(t.tcpa_minutes) &&
      e.observed_at==std::min(t.cpa_nm.observed_at,t.tcpa_minutes.observed_at);});
}
}
bool HorizonActionAllowed(const HorizonAction &a,const vessel::VesselState &s,
                          const vessel::AisState &ais,vessel::Time now) {
  if(a.kind==HorizonActionKind::None || s.replayed || s.simulated)return false;
  if(a.kind==HorizonActionKind::Follow)return Position(s,now);
  if(a.kind==HorizonActionKind::Passage) {
    if(!CurrentRoute(s,now))return false;
    const auto &r=*s.navigation.route;
    if(r.route_id!=a.route_id || r.revision_scope!=a.revision_scope || r.route_revision!=a.route_revision)return false;
    if(a.identity==r.route_id)return true; // Route energy event, still inspection only.
    return std::count_if(r.remaining_steps.begin(),r.remaining_steps.end(),[&](const auto &step){return step.waypoint_id==a.identity;})==1;
  }
  vessel::AisSelection selection;
  if(!Fresh(ais.observed_at,now) || !selection.Select(a.mmsi,ais,now))return false;
  const auto target=std::find_if(ais.targets.begin(),ais.targets.end(),[&](const auto &t){return t.mmsi==a.mmsi;});
  return target!=ais.targets.end() && target->origin==vessel::AisOrigin::LocalOpenCPN &&
      !a.source.empty() && target->source==a.source;
}
HorizonView PresentHorizon(const vessel::VesselState &s,const smartnav::NavigationAdvice &advice,
                          const vessel::AisState &ais,vessel::Time now) {
  HorizonView v;v.historical=s.replayed || s.simulated;
  if(v.historical)v.advisory_label=s.replayed ? "SmartNav · replay" : "SmartNav · test data";
  auto &motion=v.items[0];motion.time=v.historical ? (s.replayed ? "REPLAY" : "TEST DATA") : "NOW";
  motion.marker=HorizonMarker::Now;motion.title="Navigation unavailable";motion.detail="Check vessel input";
  const auto &nav=s.navigation;
  if(Current(nav.sog_kn,now,0,200)) {
    motion.title="Current motion";motion.detail=Number(*nav.sog_kn.value,1)+" kn";
    if(Current(nav.cog_deg,now,0,360)) {
      std::ostringstream course;course.imbue(std::locale::classic());
      course<<std::setfill('0')<<std::setw(3)<<(static_cast<int>(std::lround(*nav.cog_deg.value))%360)<<"° COG · ";
      motion.detail=course.str()+motion.detail;
    }
    if(vessel::Assess(nav.sog_kn,now).quality==vessel::Quality::Aging ||
       (Current(nav.cog_deg,now,0,360) && vessel::Assess(nav.cog_deg,now).quality==vessel::Quality::Aging))motion.title="Motion / aging";
  } else if(vessel::Assess(nav.sog_kn,now).quality==vessel::Quality::Stale) {
    motion.title="Motion stale";motion.detail="Check vessel input";
  }
  if(v.historical) {
    const char *prefix=s.replayed ? "Recorded" : "Test";
    motion.title=std::string(prefix)+(motion.title=="Current motion" ? " motion" :
        motion.title=="Motion / aging" ? " motion / aging" :
        motion.title=="Motion stale" ? " motion stale" : " motion unavailable");
  }
  HorizonAction follow;follow.kind=HorizonActionKind::Follow;
  if(HorizonActionAllowed(follow,s,ais,now))motion.action=follow;
  const bool matching=CurrentRoute(s,now) && advice.route_valid && s.navigation.route &&
      advice.route_id==s.navigation.route->route_id && advice.route_revision==s.navigation.route->route_revision &&
      advice.revision_scope==s.navigation.route->revision_scope;
  size_t slot=1;
  if(advice.calculated_at==now && advice.events.size()<=160 && !v.historical)
    for(const auto &e:advice.events) {
      if(slot==v.items.size())break;
      if(e.kind==smartnav::EventKind::ArrivalSoc || e.kind==smartnav::EventKind::Reserve)continue;
      if(e.observed_at>now || Text(e.title).empty() || Text(e.detail).empty() || Text(e.source).empty() || Text(e.identity).empty())continue;
      if(e.seconds_from_now && (!std::isfinite(*e.seconds_from_now) || *e.seconds_from_now<0))continue;
      HorizonAction a;
      if(e.kind==smartnav::EventKind::AisEncounter) {
        if(!AisEventCurrent(e,ais,now))continue;
        a.kind=HorizonActionKind::Ais;a.mmsi=Mmsi(e.identity);a.identity=e.identity;a.source=e.source;
      } else {
        if(!matching)continue;
        if(e.kind!=smartnav::EventKind::EnergyShortfall && e.source!=s.navigation.route->source)continue;
        a.kind=HorizonActionKind::Passage;a.route_id=advice.route_id;a.revision_scope=advice.revision_scope;
        a.route_revision=advice.route_revision;a.identity=e.identity;a.source=e.source;
      }
      auto &item=v.items[slot++];item.title=e.title;item.detail=e.detail;item.severity=e.severity;
      item.event_identity=e.identity;item.event_source=e.source;
      if(HorizonActionAllowed(a,s,ais,now))item.action=std::move(a);
      item.marker=e.kind==smartnav::EventKind::AisEncounter ? HorizonMarker::Traffic :
          e.kind==smartnav::EventKind::Destination ? HorizonMarker::Arrival : HorizonMarker::Event;
      item.time=e.seconds_from_now ? "~"+Number(*e.seconds_from_now/60,0)+" min" : "TIME UNAVAILABLE";
      if(e.kind==smartnav::EventKind::Destination)item.time+=" · ARRIVAL";
    }
  if(slot==1) {
    v.items[1].time="PASSAGE";v.items[1].title=v.historical ? "Historical data" : "No current advisory";
    v.items[1].detail=v.historical ? "Live actions unavailable" : Text(advice.reason);
    if(v.items[1].detail.empty())v.items[1].detail="Check active route and vessel input";
  }
  return v;
}
} // namespace opennav::application

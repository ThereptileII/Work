#include "application/HorizonView.h"
#include <iostream>
#include <limits>
#include <stdexcept>
#ifdef OPENNAV_HORIZON_GTEST
#include <gtest/gtest.h>
#endif
using namespace opennav;
using namespace std::chrono_literals;
namespace {
const vessel::Time stamp{100s};
int checks=0;
void Check(bool good,const char *why){++checks;if(!good)throw std::runtime_error(why);}
vessel::Sample Sample(double value){return {value,"Selected GPS",stamp,vessel::Validity::Measured};}
struct Inputs {
  vessel::VesselState state;vessel::AisState ais;smartnav::NavigationAdvice advice;
  Inputs() {
    state.navigation.latitude_deg=Sample(58);state.navigation.longitude_deg=Sample(16);
    state.navigation.sog_kn=Sample(6.3);state.navigation.cog_deg=Sample(43);
    auto r=std::make_shared<vessel::RouteProgressSnapshot>();r->route_id="route";r->revision_scope="epoch";r->route_revision=1;
    r->active_waypoint_id="first";r->active_waypoint_index=0;r->waypoint_count=2;r->remaining_distance_nm=5;
    r->observed_at=stamp;r->position_observed_at=stamp;r->state=vessel::RouteState::Valid;r->source="OpenCPN route progress";r->position_source="Selected GPS";
    r->remaining_steps={{"first","Next",58.1,16.1,2,45},{"last","Harbour",58.2,16.2,3,80}};state.navigation.route=r;
    vessel::AisTarget t;t.mmsi=123456789;t.name="Local traffic";t.source="OpenCPN AIS";t.active=true;t.upstream_alarm=true;
    t.latitude_deg=Sample(58.2);t.longitude_deg=Sample(16.3);t.cpa_nm=Sample(.4);t.tcpa_minutes=Sample(12);t.observed_at=stamp;
    ais.available=true;ais.observed_at=stamp;ais.targets={t};Refresh();
  }
  void Refresh(vessel::Time now=stamp){advice=smartnav::Advise(state,{},ais,now);}
  application::HorizonView View(vessel::Time now=stamp){return application::PresentHorizon(state,advice,ais,now);}
};
}
int RunHorizonChecks(){try{
  using A=application::HorizonActionKind;
  auto v=application::PresentHorizon({}, {}, {}, stamp);
  Check(v.items[0].title=="Navigation unavailable"&&v.items[0].action.kind==A::None,"empty input has no invented motion or follow action");
  Check(v.items[1].title=="No current advisory"&&v.items[2].title.empty(),"missing events remain missing");
  Inputs in;v=in.View();
  Check(v.items[0].title=="Current motion"&&v.items[0].detail=="043° COG · 6.3 kn","motion copied without unsupported On course claim");
  Check(v.items[0].action.kind==A::Follow,"fresh coherent position enables existing follow action");
  Check(v.items[1].title=="Local traffic"&&v.items[1].action.mmsi==123456789,"existing SmartNav event order preserved");
  Check(v.items[2].title=="Next"&&v.items[3].title=="Planned course change","bounded first three events preserved without resorting");
  for(const auto &item:v.items)Check(item.time.find(':')==std::string::npos,"no invented absolute ETA clock");
  const auto ais_action=v.items[1].action,route_action=v.items[2].action,follow=v.items[0].action;
  in.state.navigation.sog_kn=Sample(0);in.Refresh();Check(in.View().items[0].detail.find("0.0 kn")!=std::string::npos,"measured zero speed retained");
  for(double bad:{-1.,201.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    in.state.navigation.sog_kn=Sample(bad);in.Refresh();Check(in.View().items[0].title=="Navigation unavailable","malformed speed withheld");
  }
  in=Inputs();in.state.navigation.cog_deg=Sample(361);in.Refresh();Check(in.View().items[0].detail=="6.3 kn","invalid course not silently normalized");
  in.state.navigation.cog_deg=Sample(0);in.Refresh();Check(in.View().items[0].detail.find("000°")==0,"zero course valid");
  in=Inputs();in.Refresh(stamp+2s);Check(in.View(stamp+2s).items[0].title=="Motion / aging","aging motion explicit");
  Check(!application::HorizonActionAllowed(follow,in.state,in.ais,stamp+5s),"click-time GPS expiry disables follow");
  in.Refresh(stamp+5s);Check(in.View(stamp+5s).items[0].title=="Motion stale"&&in.View(stamp+5s).items[1].title=="No current advisory","retained data expires without renewal");
  in=Inputs();const auto observation=in.state.navigation.latitude_deg.observed_at;in.View();Check(in.state.navigation.latitude_deg.observed_at==observation,"presentation cannot renew observations");
  for(int invalid=0;invalid<6;++invalid){in=Inputs();
    if(invalid==0)in.state.navigation.latitude_deg.value.reset();
    if(invalid==1)in.state.navigation.longitude_deg.value=181;
    if(invalid==2)in.state.navigation.longitude_deg.source="Other GPS";
    if(invalid==3)in.state.navigation.longitude_deg.device_id="other";
    if(invalid==4)in.state.navigation.longitude_deg.observed_at+=1ms;
    if(invalid==5)in.state.navigation.longitude_deg.validity=vessel::Validity::Estimated;
    Check(in.View().items[0].action.kind==A::None,"invalid/incoherent position cannot enable follow");
    Check(!application::HorizonActionAllowed(route_action,in.state,in.ais,stamp),"GPS loss invalidates retained route action");
  }
  for(int change=0;change<7;++change){in=Inputs();auto r=std::make_shared<vessel::RouteProgressSnapshot>(*in.state.navigation.route);
    if(change==0)r->route_id="other";if(change==1)r->route_revision++;if(change==2)r->revision_scope="new epoch";
    if(change==3)r->state=vessel::RouteState::RouteChanged;if(change==4)r->remaining_steps.erase(r->remaining_steps.begin());
    if(change==5)r->position_source="Other GPS";if(change==6)r->remaining_steps.push_back(r->remaining_steps.front());
    in.state.navigation.route=r;
    Check(!application::HorizonActionAllowed(route_action,in.state,in.ais,stamp),"changed/reversed/skip/ambiguous route action rejected at click");
  }
  in=Inputs();in.state.navigation.route.reset();Check(!application::HorizonActionAllowed(route_action,in.state,in.ais,stamp),"deleted route safe");
  in=Inputs();in.advice.calculated_at-=1ms;Check(in.View().items[1].title=="No current advisory","retained advice epoch not revived");
  in=Inputs();in.advice.events[0].observed_at=stamp+1ms;Check(in.View().items[1].title!="Local traffic","future event withheld");
  in=Inputs();in.advice.events[0].seconds_from_now=std::numeric_limits<double>::infinity();Check(in.View().items[1].title!="Local traffic","nonfinite event time withheld");
  in=Inputs();in.advice.events[0].seconds_from_now=-1;Check(in.View().items[1].title!="Local traffic","negative event time withheld");
  in=Inputs();in.advice.events[0].title=std::string(513,'x');Check(in.View().items[1].title!="Local traffic","oversized event text rejected");
  in=Inputs();in.advice.events[0].detail="unsafe\nmultiline";Check(in.View().items[1].title!="Local traffic","control characters rejected");
  in=Inputs();in.advice.events.resize(161);Check(in.View().items[1].title=="No current advisory","event collection bounded");
  for(int invalid=0;invalid<8;++invalid){in=Inputs();
    if(invalid==0)in.ais.targets.push_back(in.ais.targets.front());
    if(invalid==1)in.ais.targets.front().origin=vessel::AisOrigin::AisStreamOnline;
    if(invalid==2)in.ais.targets.front().source="other receiver";
    if(invalid==3)in.ais.targets.front().latitude_deg.value=91;
    if(invalid==4)in.ais.targets.front().lost=true;
    if(invalid==5)in.ais.targets.front().latitude_deg.observed_at-=5s;
    if(invalid==6)in.ais.targets.clear();if(invalid==7)in.ais.available=false;
    Check(!application::HorizonActionAllowed(ais_action,in.state,in.ais,stamp),"stale/lost/duplicate/online/replaced AIS cannot be selected");
    if(invalid==0||invalid==3||invalid==5)Check(in.View().items[1].title=="Local traffic"&&in.View().items[1].action.kind==A::None,"valid alarm remains visible when context position unavailable/ambiguous");
    else Check(in.View().items[1].title!="Local traffic","lost/absent/source-replaced AIS alarm withheld");
  }
  for(const auto &bad:{"0","-1","123x","1000000000","+123456789"}){in=Inputs();in.advice.events[0].identity=bad;Check(in.View().items[1].title!="Local traffic","strict MMSI parsing");}
  in=Inputs();in.ais.targets[0].cpa_nm.observed_at-=10s;in.ais.targets[0].tcpa_minutes.observed_at-=10s;
  in.ais.targets[0].cpa_nm.freshness={20s,60s};in.ais.targets[0].tcpa_minutes.freshness={20s,60s};
  in.ais.targets[0].latitude_deg.value.reset();in.Refresh();
  Check(in.View().items[1].title=="Local traffic"&&in.View().items[1].action.kind==A::None,"SmartNav accepted longer-budget CPA remains visible without action-safe position");
  for(bool replay:{false,true}){in=Inputs();in.state.replayed=replay;in.state.simulated=!replay;v=in.View();
    Check(v.historical&&v.items[0].action.kind==A::None&&v.items[1].title=="Historical data","historical data visibly separated and actions unavailable");
    Check(v.advisory_label.find(replay?"replay":"test data")!=std::string::npos,"replay/test provenance visible");
    in.Refresh(stamp+5s);Check(in.View(stamp+5s).items[0].title.find("stale")!=std::string::npos,"historical labeling preserves expired quality");}
  in=Inputs();v=in.View();auto before=v;
  in.advice.events[0].severity=smartnav::Severity::Caution;Check(!(in.View()==before),"severity-only difference changes presentation identity");
  in=Inputs();before=in.View();in.ais.targets[0].mmsi=987654321;in.advice.events[0].identity="987654321";
  Check(!(in.View()==before)&&in.View().items[1].action.mmsi==987654321,"identity-only change updates action despite same labels");
  in=Inputs();in.ais.targets[0].latitude_deg.value.reset();before=in.View();
  in.ais.targets[0].mmsi=987654321;in.advice.events[0].identity="987654321";
  Check(!(in.View()==before)&&in.View().items[1].event_identity=="987654321"&&in.View().items[1].action.kind==A::None,"disabled event still retains its changed owned identity");
  in=Inputs();before=in.View();Check(in.View()==before,"unchanged owned presentation does not require redraw");
  in.state={};in.ais={};in.advice={};Check(before.items[1].action.mmsi==123456789,"retained presentation owns values after source destruction");
  std::cout<<"PASS "<<checks<<" horizon provenance checks\n";return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
#ifdef OPENNAV_HORIZON_GTEST
TEST(OpenNavHorizon, OwnedAdviceAndActions){EXPECT_EQ(RunHorizonChecks(),0);}
#else
int main(){return RunHorizonChecks();}
#endif

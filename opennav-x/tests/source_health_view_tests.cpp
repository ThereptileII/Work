#include "application/SourceHealthView.h"
#include <iostream>
#include <limits>
#include <stdexcept>
#ifdef OPENNAV_HEALTH_GTEST
#include <gtest/gtest.h>
#endif
using namespace opennav;
using namespace std::chrono_literals;
namespace {
const vessel::Time stamp{100s};
using S = application::SignalState;
int checks=0;
void Check(bool ok,const char *why) {++checks;if(!ok)throw std::runtime_error(why);}
vessel::Sample Sample(double value) {return {value,"NMEA2000/128267/1",stamp,vessel::Validity::Measured};}
application::HealthSignal Signal(const application::SourceHealthView &v,const char *id) {
  for(const auto &s:v.signals)if(s.id==id)return s;
  throw std::runtime_error("missing signal");
}
}
int RunSourceHealthChecks() {
  try {
    vessel::VesselState vessel;
    application::OnlineAisState online;
    vessel::AisState onboard;
    adapters::PilotView pilot;
    std::vector<vessel::SourceHealth> sources;
    const auto view=[&](vessel::Time now=stamp){return application::PresentSourceHealth(vessel,sources,onboard,online,pilot,now);};
    Check(view().signals.size()==11,"fixed owned signal rows including separate AIS services");
    for(const auto &s:view().signals)Check(s.state==S::Unavailable,"missing data never healthy");
    vessel.navigation.latitude_deg=Sample(0);
    Check(Signal(view(),"gps").state==S::Unavailable,"latitude alone is not GPS");
    vessel.navigation.longitude_deg=Sample(0);
    Check(Signal(view(),"gps").state==S::Current,"measured zero coordinate is valid");
    for(int bad=0;bad<5;++bad) {
      auto nav=vessel.navigation;
      if(bad==0)nav.longitude_deg.source="different";
      if(bad==1)nav.longitude_deg.observed_at+=1ms;
      if(bad==2)nav.longitude_deg.device_id="other device";
      if(bad==3)nav.longitude_deg.value=181;
      if(bad==4)nav.latitude_deg.value=std::numeric_limits<double>::quiet_NaN();
      Check(application::PresentPositionHealth(nav,stamp).state==S::Invalid,"incoherent or invalid position withheld");
    }
    Check(Signal(view(stamp+2s),"gps").state==S::Aging,"GPS aging boundary");
    Check(Signal(view(stamp+5s),"gps").state==S::Stale,"GPS stale boundary");
    Check(Signal(view(stamp-1ms),"gps").state==S::Uncertain,"future GPS not current");
    vessel.navigation.longitude_deg.validity=vessel::Validity::Estimated;
    Check(Signal(view(),"gps").state==S::Estimated,"estimated longitude cannot look measured");
    vessel.environment.depth_below_transducer_m=Sample(0);
    vessel.battery.soc_percent=Sample(68);
    vessel.battery.soc_percent.validity=vessel::Validity::Estimated;
    Check(Signal(view(),"depth").state==S::Current,"measured zero depth remains distinct from missing");
    Check(Signal(view(),"battery").state==S::Estimated,"estimated SOC is explicit");
    Check(Signal(view(stamp+5s),"battery").state==S::Stale,"stale estimate cannot look fresh");
    Check(Signal(view(stamp+3s),"depth").age==3s,"reading does not refresh observation time");
    auto &depth=vessel.environment.depth_below_transducer_m;
    sources.push_back({vessel::Quantity::Depth,"depth-source",depth,true,40,4.8,100,0});
    Check(Signal(view(),"depth").frequency_hz==4.8&&Signal(view(),"depth").priority==40,"selected matching metadata copied");
    sources[0].selected=false;
    Check(!Signal(view(),"depth").frequency_hz,"unselected source cannot supply cadence");
    sources[0].selected=true;sources[0].sample.observed_at-=1ms;
    Check(!Signal(view(),"depth").priority,"metadata from another observation not attributed");
    sources[0].sample=depth;sources.push_back(sources[0]);
    Check(!Signal(view(),"depth").frequency_hz,"ambiguous duplicate metadata is unavailable");
    sources.resize(1);sources[0].frequency_hz=std::numeric_limits<double>::infinity();
    Check(!Signal(view(),"depth").frequency_hz,"nonfinite cadence is unavailable");
    for(auto n:{std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
      depth.value=n;Check(Signal(view(),"depth").state==S::Invalid,"invalid numeric value cannot be healthy");
    }
    depth=Sample(8.4);depth.freshness={5s,2s};
    Check(Signal(view(),"depth").state==S::Invalid,"bad thresholds do not throw through UI");
    depth=Sample(8.4);depth.source.clear();
    Check(Signal(view(),"depth").state==S::Unavailable,"missing provenance unavailable");
    onboard.available=true;onboard.source="OpenCPN AIS";
    online.enabled=true;online.feed.health.connection=ais::Connection::Connected;
    Check(Signal(view(),"online").status=="Subscribing","connection alone is not confirmed subscription");
    online.feed.health.subscription_confirmed=true;
    Check(Signal(view(),"online").status=="Connected","confirmed supplemental connection shown");
    Check(Signal(view(),"ais").state==S::Unavailable,"internet does not establish onboard receiver health");
    vessel::AisTarget target;
    target.mmsi=123456789;target.active=true;target.latitude_deg=Sample(57);target.longitude_deg=Sample(16);
    target.origin=vessel::AisOrigin::AisStreamOnline;onboard.targets={target};
    Check(Signal(view(),"ais").state==S::Unavailable,"online target accidentally supplied in local model ignored");
    onboard.targets[0].origin=vessel::AisOrigin::LocalOpenCPN;
    Check(Signal(view(),"ais").status=="Targets current","onboard report health uses OpenCPN assessment");
    online.feed.health.connection=ais::Connection::Offline;
    Check(Signal(view(),"ais").state==S::Current&&Signal(view(),"online").status=="Offline","internet failure cannot affect onboard status");
    Check(Signal(view(stamp+5s),"ais").state==S::Stale,"reading AIS model does not refresh target report");
    Check(!Signal(view(),"ais").age,"copy timestamp is not receiver observation age");
    pilot.fresh=true;
    Check(Signal(view(),"pilot").state==S::Unavailable,"fresh flag alone is not pilot feedback");
    pilot.feedback.source="TEST ONLY";pilot.feedback.sequence=1;pilot.feedback.observed_at=stamp;
    pilot.feedback.mode=adapters::PilotMode::Standby;
    Check(Signal(view(),"pilot").state==S::Current,"fresh actual pilot feedback");
    Check(Signal(view(stamp+3s),"pilot").state==S::Stale,"pilot expires at actual adapter threshold");
    vessel.replayed=true;online.feed.health.connection=ais::Connection::Connected;
    Check(view().historical&&Signal(view(),"online").state==S::Unavailable,"replay never implies live supplemental connection");
    auto retained=view();vessel={};onboard={};pilot={};sources.clear();
    Check(Signal(retained,"gps").source=="NMEA2000/128267/1","owned display survives input destruction");
    std::cout<<"PASS "<<checks<<" source-health provenance checks\n";return 0;
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
#ifdef OPENNAV_HEALTH_GTEST
TEST(OpenNavSourceHealth, PerMeasurementAndIndependentAis) { EXPECT_EQ(RunSourceHealthChecks(),0); }
#else
int main() {return RunSourceHealthChecks();}
#endif

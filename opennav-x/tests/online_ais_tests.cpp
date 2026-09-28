#include "ais/TargetCache.h"
#include "ais/Subscription.h"
#include "vessel/AisSelection.h"
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace opennav;
using namespace std::chrono_literals;
namespace {
unsigned checks = 0;
void Check(bool ok, const char *reason) {
  ++checks; if (!ok) throw std::runtime_error(reason);
}
ais::PositionReport Position(vessel::Time time, int mmsi = 265000001) {
  return {mmsi, 58.1, 16.2, 6.3, 43, 41, 0, time};
}
}
int main() {
 try {
  const vessel::Time t{100s};
  ais::TargetCache cache;
  Check(cache.Read(t).targets.empty(), "No position must remain empty");
  ais::StaticReport info;
  info.mmsi=265000001;info.name="SANITIZED TEST VESSEL";info.observed_at=t;
  Check(cache.Observe(info,t),"Static accepted independently");
  Check(cache.Read(t).targets.empty(),"Static metadata cannot create a zero-position vessel");
  Check(cache.Observe(Position(t),t),"Class A/B normalized position accepted");
  const auto retained=cache.Read(t);
  Check(retained.targets.size()==1&&retained.targets[0].name==*info.name,"Static then position merges");
  Check(retained.targets[0].origin==vessel::AisOrigin::AisStreamOnline&&retained.targets[0].source==ais::OnlineSource,"Online provenance explicit");
  Check(!retained.targets[0].cpa_nm.value&&!retained.targets[0].range_nm.value&&!retained.targets[0].upstream_alarm,"No invented local navigation metrics");
  Check(vessel::AisSelection::CurrentPosition(retained.targets[0],t),"Current owned position usable for display selection");
  Check(ais::Age(t,t+14999ms)==ais::TargetAge::Live,"Live upper boundary");
  Check(ais::Age(t,t+15s)==ais::TargetAge::Aging,"Aging boundary");
  Check(ais::Age(t,t+60s)==ais::TargetAge::Stale,"Stale boundary");
  Check(ais::Age(t,t+2min)==ais::TargetAge::Lost,"Lost boundary");
  Check(ais::Age(t,t+10min)==ais::TargetAge::Expired,"Expiry boundary");
  Check(ais::Age(t,t-1ms)==ais::TargetAge::Invalid,"Future observation rejected");
  Check(ais::Age(vessel::Time::min(),vessel::Time::max())==ais::TargetAge::Expired,"Extreme clock difference cannot overflow");
  Check(ais::Age({},t)==ais::TargetAge::Invalid,"Missing timestamp invalid");
  info.observed_at=t+61s;info.callsign="TEST123";
  Check(cache.Observe(info,t+61s),"Static refresh accepted");
  const auto stale=cache.Read(t+61s);
  Check(stale.observed_at==t&&stale.targets[0].latitude_deg.observed_at==t,"Static/UI copy never renews position");
  Check(vessel::Assess(stale.targets[0].latitude_deg,t+61s).quality==vessel::Quality::Stale,"Retained value visibly stale");
  Check(!vessel::AisSelection::CurrentPosition(stale.targets[0],t+61s),"Stale selection is not current");
  Check(cache.Read(t+2min).targets[0].lost&&!cache.Read(t+2min).targets[0].active,"Lost state explicit");
  Check(cache.Read(t+10min).targets.empty(),"Online traffic ages off chart");
  Check(!cache.Observe(Position(t-1s),t),"Out-of-order position rejected");
  Check(!cache.Observe(Position(t),t),"Duplicate observation cannot refresh receipt age");
  Check(!cache.Observe(Position(t+1s),t),"Future observation cannot enter cache");
  for (int bad=0;bad<12;++bad) {
    auto p=Position(t+62s,265000002);
    switch(bad) {
      case 0:p.mmsi=0;break; case 1:p.mmsi=99999999;break;
      case 2:p.latitude=91;break; case 3:p.longitude=181;break;
      case 4:p.sog=102.3;break; case 5:p.cog=360;break;
      case 6:p.heading=511;break; case 7:p.navigation_status=15;break;
      case 8:p.latitude=std::numeric_limits<double>::quiet_NaN();break;
      case 9:p.longitude=std::numeric_limits<double>::infinity();break;
      case 10:p.sog=-1;break; case 11:p.heading=-1;break;
    }
    Check(!cache.Observe(p,t+62s),"Invalid/sentinel/nonfinite input rejected atomically");
  }
  Check(cache.Size()==1,"Invalid input allocated no target");
  auto empty=Position(t+62s);empty.sog.reset();empty.cog.reset();empty.heading.reset();
  Check(cache.Observe(empty,t+62s),"Missing motion retains valid geographic position");
  Check(!cache.Read(t+62s).targets[0].sog_kn.value,"Missing SOG stays missing not zero");
  auto anti=Position(t+63s,265000002);anti.longitude=-179.99;
  Check(cache.Observe(anti,t+63s),"Antimeridian coordinate not reprojected");
  info.observed_at=t+63s;info.name=std::string(129,'a');
  Check(!cache.Observe(info,t+63s),"Overlong static string rejected");
  info.name="unsafe\nlog";Check(!cache.Observe(info,t+63s),"Control characters rejected");
  info.name="UPDATED";info.length_m=1500;
  Check(!cache.Observe(info,t+63s),"Invalid dimensions rejected");
  Check(retained.targets[0].name=="SANITIZED TEST VESSEL"&&retained.targets[0].sog_kn.value==6.3,"Retained snapshot survives cache edits");

  vessel::AisState onboard=retained;onboard.source="OpenCPN";
  auto &local=onboard.targets[0];local.origin=vessel::AisOrigin::LocalOpenCPN;
  local.source="OpenCPN";local.latitude_deg.value=59;local.upstream_alarm=true;
  local.cpa_nm={.1,"OpenCPN",t,vessel::Validity::Measured};
  ais::ProviderSnapshot online;online.targets=cache.Read(t+63s);
  auto merged=ais::Aggregate(onboard,online);
  Check(merged.display.targets.size()==2,"MMSI deduplication with online-only addition");
  Check(merged.display.targets[0].latitude_deg.value==59&&merged.display.targets[0].latitude_deg.observed_at==t,"Older local report remains source-authoritative");
  Check(merged.display.targets[0].upstream_alarm&&merged.display.targets[0].cpa_nm.value==.1,"Local safety semantics preserved as a whole");
  Check(merged.onboard.source=="OpenCPN"&&merged.online.targets.source==ais::OnlineSource,"Provider health remains separate");
  local.lost=true;merged=ais::Aggregate(onboard,online);
  Check(merged.display.targets[0].lost,"Online traffic cannot hide onboard lost state");
  onboard.targets.clear();onboard.available=false;merged=ais::Aggregate(onboard,online);
  Check(merged.display.targets.size()==2&&!merged.onboard.available,"Online display does not imply local receiver availability");
  online.targets.targets[0].upstream_alarm=true;online.targets.targets[0].cpa_nm.value=.01;
  merged=ais::Aggregate(onboard,online);
  Check(!merged.display.targets[0].upstream_alarm&&!merged.display.targets[0].cpa_nm.value,"Online cannot smuggle CPA or upstream alarm");
  online.targets.targets.push_back(online.targets.targets[0]);merged=ais::Aggregate(onboard,online);
  Check(merged.display.targets.size()==1,"Ambiguous online identity withheld");
  online.targets.targets.resize(2001);merged=ais::Aggregate(onboard,online);
  Check(merged.display.targets.empty(),"Oversized supplemental snapshot withheld");

  ais::TargetCache bounded;
  for(std::size_t i=0;i<ais::TargetCache::Capacity;++i)
    Check(bounded.Observe(Position(t,265000000+static_cast<int>(i)),t),"Bounded set capacity accepts unique identities");
  Check(!bounded.Observe(Position(t,366000000),t),"Cache cannot grow past bound");
  Check(bounded.Observe(Position(t+11min,366000000),t+11min)&&bounded.Size()==1,"Expired records reclaimed before allocation");
  auto boxes=ais::SubscriptionArea({57,58,16,17});
  Check(boxes.size()==1&&boxes[0].south<57&&boxes[0].east>17,"Viewport subscription gets a geographic margin");
  boxes=ais::SubscriptionArea({57,58,179,-179});
  Check(boxes.size()==2&&boxes[0].east==180&&boxes[1].west== -180,"Antimeridian split does not subscribe to the opposite hemisphere");
  boxes=ais::SubscriptionArea({-89,89,-179,179});
  Check(boxes.size()==1&&boxes[0].west== -180&&boxes[0].east==180&&boxes[0].south== -90&&boxes[0].north==90,"World and polar bounds are clamped");
  Check(ais::SubscriptionArea({58,57,16,17}).empty(),"Reversed latitude bounds rejected");
  Check(ais::SubscriptionArea({57,58,std::numeric_limits<double>::quiet_NaN(),17}).empty(),"Nonfinite viewport rejected");
  ais::SubscriptionPolicy subscriptions;
  Check(subscriptions.Pending(t,true).empty(),"No initial area means no fabricated subscription");
  Check(subscriptions.ObserveViewport({57,58,16,17}),"Valid chart area selected");
  Check(!subscriptions.Pending(t,true).empty(),"Full subscription ready immediately on connect");
  subscriptions.Sent(t);
  Check(!subscriptions.IsConfirmed(),"Send alone is not subscription confirmation");
  subscriptions.ObserveViewport({59,60,18,19});
  Check(subscriptions.Pending(t+10s,false).empty(),"Cannot overlap subscription acknowledgements");
  subscriptions.Confirmed();Check(subscriptions.IsConfirmed(),"Explicit confirmation establishes status");
  Check(subscriptions.Pending(t+4999ms,false).empty(),"Rapid chart movement is coalesced");
  Check(!subscriptions.Pending(t+5s,false).empty(),"Latest area published after controlled cadence");
  subscriptions.ObserveViewport({57.05,58.05,16.05,17.05});
  Check(subscriptions.Pending(t+5s,false).empty(),"Small pan inside margin needs no resubscription");
  Check(!subscriptions.Pending(t+6s,true).empty(),"Reconnect sends full latest subscription again");
  Check(ais::ReconnectDelay(0,0)==2s&&ais::ReconnectDelay(1,0)==4s,"Exponential retry starts gently");
  Check(ais::ReconnectDelay(9,0)==5min&&ais::ReconnectDelay(10,123)==15min,"Repeated failures enter bounded cooldown");
  Check(ais::ReconnectDelay(4,123)>ais::ReconnectDelay(4,0),"Injected jitter spreads retries deterministically");
  std::cout<<checks<<" online AIS cache/provenance/precedence checks passed\n";
 } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}

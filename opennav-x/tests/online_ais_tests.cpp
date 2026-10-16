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
  Check(ais::Age(t,t+3min-1ms)==ais::TargetAge::Live,"Live upper boundary");
  Check(ais::Age(t,t+3min)==ais::TargetAge::Aging,"Aging boundary");
  Check(ais::Age(t,t+10min-1ms)==ais::TargetAge::Aging,"Aging upper boundary");
  Check(ais::Age(t,t+10min)==ais::TargetAge::Stale,"Stale boundary");
  Check(ais::Age(t,t+20min-1ms)==ais::TargetAge::Stale,"Stale upper boundary");
  Check(ais::Age(t,t+20min)==ais::TargetAge::Lost,"Lost boundary");
  Check(ais::Age(t,t+30min-1ms)==ais::TargetAge::Lost,"Lost upper boundary");
  Check(ais::Age(t,t+30min)==ais::TargetAge::Expired,"Expiry boundary");
  Check(vessel::Assess(retained.targets[0].latitude_deg,t+2min).quality==vessel::Quality::Live,
        "Sparse online position stays live under the display policy");
  Check(vessel::Assess(retained.targets[0].latitude_deg,t+3min).quality==vessel::Quality::Aging,
        "Sample and chart aging thresholds agree");
  Check(vessel::AisSelection::CurrentPosition(retained.targets[0],t+10min-1ms),
        "Aging online position remains inspectable before stale boundary");
  Check(ais::Age(t,t-1ms)==ais::TargetAge::Invalid,"Future observation rejected");
  Check(ais::Age(vessel::Time::min(),vessel::Time::max())==ais::TargetAge::Expired,"Extreme clock difference cannot overflow");
  Check(ais::Age({},t)==ais::TargetAge::Invalid,"Missing timestamp invalid");
  info.observed_at=t+61s;info.callsign="TEST123";
  Check(cache.Observe(info,t+61s),"Static refresh accepted");
  const auto stale=cache.Read(t+10min);
  Check(stale.observed_at==t&&stale.targets[0].latitude_deg.observed_at==t,"Static/UI copy never renews position");
  Check(vessel::Assess(stale.targets[0].latitude_deg,t+10min).quality==vessel::Quality::Stale,"Retained value visibly stale");
  Check(!vessel::AisSelection::CurrentPosition(stale.targets[0],t+10min),"Stale selection is not current");
  Check(cache.Read(t+20min).targets[0].lost&&!cache.Read(t+20min).targets[0].active,"Lost state explicit");
  Check(cache.Read(t+30min).targets.empty(),"Online traffic ages off chart");
  Check(!cache.Observe(Position(t,265000003),t+30min),"Expired incoming position cannot allocate a target");
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

  ais::TargetCache partial;
  Check(partial.Observe(Position(t),t),"Position before static fields accepted");
  ais::StaticReport newest;
  newest.mmsi=265000001;newest.observed_at=t+30s;
  newest.callsign="NEW CALL";newest.ship_type=70;newest.length_m=40;
  Check(partial.Observe(newest,t+30s),"Newer part B fields arrive before name");
  ais::StaticReport older;
  older.mmsi=newest.mmsi;older.observed_at=t+5s;older.name="DELAYED NAME";
  older.callsign="OLD CALL";older.ship_type=36;older.length_m=20;
  older.destination="PORT";older.beam_m=8;
  Check(partial.Observe(older,t+31s),"Older report fills missing fields independently");
  auto merged_static=partial.Read(t+31s);
  const auto &named=merged_static.targets[0];
  Check(partial.Size()==1&&merged_static.targets.size()==1,"Same MMSI remains one target");
  Check(named.name=="DELAYED NAME"&&named.callsign.value=="NEW CALL"&&
        named.callsign.observed_at==t+30s,"Delayed name does not regress newer callsign");
  Check(named.ship_type.value==70&&named.length_m.value==40&&named.beam_m.value==8&&
        named.beam_m.observed_at==t+5s,"Each numeric static field retains its own observation");
  Check(named.destination.value=="PORT"&&named.destination.observed_at==t+5s,
        "Oldest partial report can fill destination without rejuvenating it");
  Check(named.observed_at==t&&named.latitude_deg.observed_at==t&&
        merged_static.observed_at==t,"Static merges never renew position clocks");
  Check(!partial.Observe(older,t+32s),"Duplicate static fields do not refresh receipt age");
  older.observed_at=t+4s;older.name="OLDER NAME";older.destination="OLDER PORT";
  Check(!partial.Observe(older,t+32s),"Older same-field values cannot regress accepted static data");
  older.observed_at=t+6s;older.name="NEWER NAME";
  Check(partial.Observe(older,t+32s),"Newer name may merge despite newer unrelated fields");
  Check(partial.Read(t+32s).targets[0].name=="NEWER NAME","Latest typed name retained");
  ais::StaticReport blank;
  blank.mmsi=newest.mmsi;blank.observed_at=t+40s;
  blank.name=" @@ @ ";blank.callsign=" ";blank.destination="";
  Check(!partial.Observe(blank,t+40s),"Empty and AIS-padded fields cannot erase known text");
  Check(partial.Read(t+40s).targets[0].name=="NEWER NAME"&&
        partial.Read(t+40s).targets[0].callsign.value=="NEW CALL","Known text survives empty reports");
  ais::StaticReport metadata;
  metadata.mmsi=newest.mmsi;metadata.observed_at=t+41s;metadata.name="CACHED NAME";
  metadata.name_origin=ais::NameOrigin::MetadataFallback;
  Check(!partial.Observe(metadata,t+41s),"Newer metadata cannot replace a typed name");

  ais::TargetCache fallback;
  Check(fallback.Observe(metadata,t+41s),"Metadata supplies an otherwise missing name");
  Check(fallback.Read(t+41s).targets.empty(),"Metadata name alone cannot create chart position");
  Check(fallback.Observe(Position(t),t+41s),"Later receipt attaches actual position to metadata MMSI");
  Check(fallback.Read(t+41s).targets[0].name=="CACHED NAME","Metadata fallback displayed by MMSI");
  auto previous_metadata=metadata;previous_metadata.observed_at=t+40s;
  previous_metadata.name="OLD CACHED NAME";
  Check(!fallback.Observe(previous_metadata,t+42s),"Older metadata cannot regress metadata name");
  Check(!fallback.Observe(metadata,t+42s),"Duplicate metadata does not refresh static age");
  ais::StaticReport typed;
  typed.mmsi=metadata.mmsi;typed.observed_at=t+10s;typed.name="  ACTUAL NAME@@ ";
  Check(fallback.Observe(typed,t+42s),"Typed name supersedes newer cached metadata");
  Check(fallback.Read(t+42s).targets[0].name=="ACTUAL NAME","Typed name normalized without synthesis");
  Check(!fallback.Observe(metadata,t+43s),"Metadata cannot take back typed name authority");
  typed.observed_at=t+9s;typed.name="OLD TYPED";
  Check(!fallback.Observe(typed,t+43s),"Older typed name cannot regress accepted typed name");
  Check(fallback.Read(t+43s).targets[0].latitude_deg.observed_at==t,
        "Name-source changes preserve original position time");

  vessel::AisState onboard=retained;onboard.source="OpenCPN";
  auto &local=onboard.targets[0];local.origin=vessel::AisOrigin::LocalOpenCPN;
  local.source="OpenCPN";local.latitude_deg.value=59;local.upstream_alarm=true;
  local.cpa_nm={.1,"OpenCPN",t,vessel::Validity::Measured};
  local.latitude_deg.freshness=local.longitude_deg.freshness={15s,60s};
  Check(!vessel::AisSelection::CurrentPosition(local,t+60s),"Local AIS retains its shorter stale policy");
  Check(vessel::AisSelection::CurrentPosition(retained.targets[0],t+60s),"Online display policy does not alter local AIS");
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
  Check(bounded.Observe(Position(t+31min,366000000),t+31min)&&bounded.Size()==1,"Expired records reclaimed before allocation");
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

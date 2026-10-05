#include "application/FooterView.h"
#include "vessel/RouteProgress.h"
#include <iostream>
#include <limits>
#include <stdexcept>
#ifdef OPENNAV_FOOTER_GTEST
#include <gtest/gtest.h>
#endif
using namespace opennav;
using namespace std::chrono_literals;
namespace {
const vessel::Time stamp{100s};
using S=application::SignalState;
int checks=0;
void Check(bool ok,const char *why) {++checks;if(!ok)throw std::runtime_error(why);}
vessel::Sample Reading(double value) {return {value,"Selected navigation / identity unavailable",stamp,vessel::Validity::Measured};}
}
int RunFooterChecks() {
  try {
    vessel::VesselState state;application::AnchorState anchor;
    vessel::AisState onboard;application::OnlineAisState online;
    const auto view=[&](vessel::Time now=stamp) {return application::PresentFooter(state,anchor,
      application::PresentSourceHealth(state,{},onboard,online,{},now),now);};
    Check(view().navigation_state=="NO POSITION"&&view().position=="GPS POSITION UNAVAILABLE","empty product has no invented underway/position");
    Check(view().cog=="—"&&view().xte=="—","missing COG and XTE not zero");
    Check(view().health_summary=="0 live signals"&&view().health_state==S::Unavailable,"no fabricated nine-of-ten source health");
    state.navigation.latitude_deg=Reading(58.341033333333);
    Check(view().position_state==S::Unavailable,"one coordinate cannot establish position");
    state.navigation.longitude_deg=Reading(16.803633333333);
    Check(view().position=="58° 20.462′ N   016° 48.218′ E","prototype coordinate precision uses owned measured values");
    Check(view().navigation_state=="EXPLORING","position alone never implies underway");
    Check(view().health_summary=="1 live signal"&&view().health_source=="Vessel data","generic selected-nav provenance not falsely NMEA 2000");
    state.navigation.cog_deg=Reading(43);
    Check(view().cog=="043°","course is padded and comes from COG");
    state.navigation.heading_true_deg=Reading(180);
    Check(view().cog=="043°","heading never substituted for COG");
    state.navigation.cog_deg=Reading(0);Check(view().cog=="000°","measured zero course remains valid");
    state.navigation.cog_deg=Reading(359.9);Check(view().cog=="000°","rounded north wraps at 360");
    state.navigation.cog_deg=Reading(360);Check(view().cog=="000°","accepted equivalent north normalizes");
    for(double bad:{-1.,361.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
      state.navigation.cog_deg=Reading(bad);Check(view().cog=="—"&&view().cog_state==S::Invalid,"malformed COG is withheld");
    }
    state.navigation.cog_deg=Reading(43);
    Check(view(stamp+2s).cog=="043° AGING"&&view(stamp+2s).position.find("AGING")!=std::string::npos,"aging never appears current");
    Check(view(stamp+5s).cog=="STALE"&&view(stamp+5s).navigation_state=="NO POSITION"&&view(stamp+5s).position=="GPS POSITION STALE","expired values are visibly withheld");
    Check(view(stamp+5s).live_signals==0&&view(stamp+5s).stale_signals==2,"health expiration independent of UI reads");
    Check(state.navigation.cog_deg.observed_at==stamp,"presentation never renews observations");
    Check(view(stamp-1ms).position_state==S::Uncertain&&view(stamp-1ms).position=="GPS POSITION UNCERTAIN","future position withheld");
    for(int bad=0;bad<5;++bad) {
      const auto saved=state.navigation;
      if(bad==0)state.navigation.longitude_deg.source="different receiver";
      if(bad==1)state.navigation.longitude_deg.observed_at+=1ms;
      if(bad==2)state.navigation.longitude_deg.value=181;
      if(bad==3)state.navigation.latitude_deg.value=std::numeric_limits<double>::quiet_NaN();
      if(bad==4)state.navigation.longitude_deg.device_id="different device";
      Check(view().position_state==S::Invalid&&view().position=="GPS POSITION INVALID","incoherent coordinate pair withheld");state.navigation=saved;
    }
    state.navigation.longitude_deg.validity=vessel::Validity::Estimated;
    Check(view().position=="GPS POSITION ESTIMATED"&&view().navigation_state=="NO POSITION","estimated coordinate does not imply measured navigation");
    state.navigation.longitude_deg=Reading(0);state.navigation.latitude_deg=Reading(-0.0);
    Check(view().position=="00° 00.000′ N   000° 00.000′ E","measured zero is valid, signed zero is not a hemisphere");
    state.navigation.latitude_deg=Reading(-89.9999999);state.navigation.longitude_deg=Reading(-179.9999999);
    Check(view().position=="90° 00.000′ S   180° 00.000′ W","rounded minute carries at polar/antimeridian boundaries");
    state.navigation.latitude_deg=Reading(58);state.navigation.longitude_deg=Reading(16);
    auto route=std::make_shared<vessel::RouteProgressSnapshot>();route->route_id="route";route->revision_scope="test";
    route->route_revision=1;route->active_waypoint_id="point";route->active_waypoint_index=0;route->waypoint_count=1;
    route->remaining_distance_nm=2;route->observed_at=stamp;route->position_observed_at=stamp;route->state=vessel::RouteState::Valid;
    route->source="OpenCPN route progress";route->position_source=state.navigation.latitude_deg.source;state.navigation.route=route;
    Check(view().navigation_state=="ROUTE ACTIVE","valid owned route activates footer route state");
    Check(view().xte == "—", "valid route with no recorded XTE cannot invent it");
    route->cross_track_error_nm = .03;
    route->cross_track_direction = vessel::CrossTrackDirection::Left;
    Check(view().xte == "← 0.030 NM" && view().xte_state == S::Current &&
              view().xte_hint == "Steer left toward route (OpenCPN)",
          "left arrow explicitly means native direction to steer toward route");
    route->cross_track_direction = vessel::CrossTrackDirection::Right;
    Check(view().xte == "0.030 NM →" && view().xte_hint == "Steer right toward route (OpenCPN)",
          "right arrow preserves native steering direction");
    route->distance_units_per_nm = 1852.; route->distance_unit = "m";
    Check(view().xte == "56 m →", "owned OpenCPN preference controls XTE units");
    route->cross_track_error_nm = 0.;
    Check(view().xte == "0 m" && view().xte_hint == "On route; no cross-track correction",
          "observed zero has no arbitrary corrective arrow");
    route->cross_track_error_nm = .03;
    route->distance_units_per_nm = 1.; route->distance_unit = "NM";
    Check(view(stamp + 2s).xte == "0.030 NM → AGING" && view(stamp + 2s).xte_state == S::Aging,
          "aging route XTE never claims current quality");
    for (int bad = 0; bad < 4; ++bad) {
      const auto saved = state.navigation;
      if (bad == 0) state.navigation.latitude_deg.source = state.navigation.longitude_deg.source = "other GPS";
      if (bad == 1) state.navigation.latitude_deg.observed_at = state.navigation.longitude_deg.observed_at = stamp - 1ms;
      if (bad == 2) state.navigation.latitude_deg.value.reset();
      if (bad == 3) state.navigation.longitude_deg.validity = vessel::Validity::Uncertain;
      Check(view().xte == "—", "current GPS loss, source change, older fix and uncertainty withhold XTE");
      state.navigation = saved;
    }
    state.navigation.latitude_deg.observed_at = state.navigation.longitude_deg.observed_at = stamp + 1s;
    Check(view(stamp + 1s).xte == "0.030 NM →", "newer same-source GPS may follow coherent route pass");
    Check(view(stamp + 5s).xte == "STALE" && view(stamp + 5s).xte_state == S::Stale,
          "newer GPS cannot refresh an expired route XTE publication");
    state.navigation.latitude_deg.observed_at = state.navigation.longitude_deg.observed_at = stamp;
    anchor.waypoint_id = "watch";
    Check(view().xte == "—", "anchor mode cannot expose conflicting route guidance");
    anchor = {};
    route->state=vessel::RouteState::RouteChanged;
    Check(view().navigation_state=="ROUTE WAITING","editing/reversal/transition cannot look coherent");
    Check(view().xte == "—", "route transition clears cross-track presentation");
    route->state=vessel::RouteState::NoActiveRoute;
    Check(view().navigation_state=="EXPLORING","deactivation removes route-active state");
    route->state=vessel::RouteState::Valid;route->observed_at-=10s;route->position_observed_at=route->observed_at;
    Check(view().navigation_state=="ROUTE WAITING","retained route does not refresh with new GPS");
    state.navigation.route.reset();Check(view().navigation_state=="EXPLORING","deleted route snapshot not dereferenced");
    anchor.waypoint_id="anchor";Check(view().navigation_state=="ANCHOR WATCH","selected anchor watch is explicit, not a claimed motion state");anchor={};
    online.enabled=true;online.feed.health.connection=ais::Connection::Connected;online.feed.health.subscription_confirmed=true;
    const auto before=view().live_signals;
    Check(before==2,"online subscription cannot create local receiver or extra onboard health");
    online.feed.health.connection=ais::Connection::Offline;
    Check(view().live_signals==before,"internet loss cannot remove local health");
    application::SourceHealthView duplicates;application::HealthSignal h;h.id="gps";h.state=S::Current;
    duplicates.signals={h,h};h.id="online";duplicates.signals.push_back(h);h.id="unknown";duplicates.signals.push_back(h);
    Check(application::PresentFooter(state,anchor,duplicates,stamp).live_signals==1,"counts named measurements once, not arbitrary connections");
    const auto retained=view();state={};Check(retained.position=="58° 00.000′ N   016° 00.000′ E","owned view survives input destruction");
    for(bool replay:{false,true}) {
      state.simulated=!replay;state.replayed=replay;const auto v=view();
      Check(v.historical&&v.live_signals==0&&v.health_summary=="Inspect quality","historical data never claims live signals");
      Check(v.navigation_state==(replay?"REPLAY":"TEST DATA"),"test/replay state unmistakable");
    }
    Check(view().xte=="—","absent XTE remains unavailable in historical source modes");
    std::cout<<"PASS "<<checks<<" footer provenance checks\n";return 0;
  }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
#ifdef OPENNAV_FOOTER_GTEST
TEST(OpenNavFooter, OwnedNavigationAndHealth) {EXPECT_EQ(RunFooterChecks(),0);}
#else
int main(){return RunFooterChecks();}
#endif

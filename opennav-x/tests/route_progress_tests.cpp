#include "integration/RouteProgressInput.h"
#include "smartnav/Energy.h"
#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>

using namespace opennav;
using namespace vessel;
using namespace integration;
using namespace std::chrono_literals;
const Time t{100s};
void Require(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
RouteRead Fixture(std::size_t active = 0) {
  RouteRead r;
  r.route.active = true; r.route.id = "route-A";
  r.route.points = {{"a",56,12,0},{"b",57,13,3},{"c",58,14,4}};
  r.route.active_index = active; r.route.active_point_id = r.route.points[active].id;
  r.position.latitude_deg = {55,"selected GPS",t,Validity::Measured};
  r.position.longitude_deg = {11,"selected GPS",t,Validity::Measured};
  r.upstream_position_valid = true; r.upstream_latitude_deg = 55; r.upstream_longitude_deg = 11;
  r.range_to_active_nm = 2;
  return r;
}
RouteProgress Evaluate(const RouteRead& r, Time now = t) {
  RouteProgressInput input("test session"); input.Complete(r,r,now); return input.Current();
}
void Invalid(const RouteProgress& s, RouteState state) {
  Require(s->state == state, RouteStateName(s->state));
  Require(!s->remaining_distance_nm, "Invalid route cannot become zero distance");
  Require(!RouteDistanceSample(*s,t+1s).value, "Invalid route cannot enter advisory consumer");
}
void Change(const std::function<void(RouteRead&)>& edit, RouteState expected) {
  RouteProgressInput input("test session"); auto a=Fixture(); input.Complete(a,a,t);
  const auto retained=input.Current(); auto b=a; edit(b);
  input.Complete(a,b,t+1s); Invalid(input.Current(),expected);
  Require(retained->remaining_distance_nm == 9 && retained->route_id == "route-A", "Retained snapshot owns values");
}
int main() {
  std::size_t passed=0;
  auto test=[&](const char* name, const std::function<void()>& run) {
    try { run(); ++passed; std::cout << "PASS " << name << '\n'; }
    catch(const std::exception& e) { throw std::runtime_error(std::string(name)+": "+e.what()); }
  };
  try {
    static_assert(std::is_const_v<RouteProgress::element_type>, "Public snapshots are immutable");
    test("first waypoint", [] { Require(Evaluate(Fixture(0))->remaining_distance_nm==9,"Current range plus both later legs"); });
    test("middle waypoint", [] { Require(Evaluate(Fixture(1))->remaining_distance_nm==6,"Exclude incoming active leg"); });
    test("final waypoint", [] { Require(Evaluate(Fixture(2))->remaining_distance_nm==2,"Only current range remains"); });
    test("arrival and advance reject old range then recover", [] {
      RouteProgressInput input("test"); auto a=Fixture(); input.Complete(a,a,t); auto b=Fixture(1); b.range_to_active_nm=0;
      input.Complete(a,b,t+1s); Invalid(input.Current(),RouteState::ActivePointChanged);
      b.range_to_active_nm=3; input.Complete(b,b,t+2s); Require(input.Current()->remaining_distance_nm==7,"Next normal pass uses new point range");
    });
    test("skip between passes", [] {
      RouteProgressInput input("test"); auto a=Fixture(); input.Complete(a,a,t); auto b=Fixture(2); b.range_to_active_nm=0;
      input.Complete(b,b,t+1s); Invalid(input.Current(),RouteState::ActivePointChanged);
      b.range_to_active_nm=7; input.Complete(b,b,t+2s); Require(input.Current()->remaining_distance_nm==7,"Stable post-skip pass recovers");
    });
    test("deactivation", [] { Change([](auto& r){r.route={};},RouteState::NoActiveRoute); });
    test("deletion and retained lifetime", [] { Change([](auto& r){r.route.registered=false;r.route.points.clear();},RouteState::InvalidRoute); });
    test("reversal", [] { Change([](auto& r){std::reverse(r.route.points.begin(),r.route.points.end()); r.route.active_index=2;},RouteState::RouteChanged); });
    test("edit while active", [] { Change([](auto& r){r.route.points[1].latitude_deg=56.5; r.route.points[1].incoming_leg_nm=8;},RouteState::RouteChanged); });
    test("repeated identity including inactive point", [] {auto r=Fixture(); r.route.points[2].id="b"; Invalid(Evaluate(r),RouteState::AmbiguousPoint); });
    test("invalid active waypoint", [] {auto r=Fixture(); r.route.active_index.reset(); Invalid(Evaluate(r),RouteState::InvalidActivePoint); });
    test("negative leg including travelled leg", [] {auto r=Fixture(2); r.route.points[1].incoming_leg_nm=-1; Invalid(Evaluate(r),RouteState::InvalidLeg); });
    test("NaN infinity negative range", [] {
      for(double value:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),-1.0}) {
        auto r=Fixture(); r.range_to_active_nm=value; Invalid(Evaluate(r),RouteState::InvalidRange);
        r=Fixture(); r.route.points[1].incoming_leg_nm=value; Invalid(Evaluate(r),RouteState::InvalidLeg);
      }
    });
    test("overflow", [] {auto r=Fixture(); r.range_to_active_nm=std::numeric_limits<double>::max(); r.route.points[1].incoming_leg_nm=std::numeric_limits<double>::max(); Invalid(Evaluate(r),RouteState::ArithmeticLimit); });
    test("missing position", [] {auto r=Fixture(); r.position={}; Invalid(Evaluate(r),RouteState::MissingPosition); });
    test("stale position and retained snapshot age", [] {
      auto r=Fixture(); Invalid(Evaluate(r,t+5s),RouteState::StalePosition);
      const auto s=Evaluate(r); Require(AssessRoute(*s,t+2s).quality==Quality::Aging,"Aging matches Vessel Data");
      Require(AssessRoute(*s,t+5s).state==RouteState::StalePosition,"Stale at exact boundary");
      Require(!RouteDistanceSample(*s,t+5s).value,"Retained snapshot cannot feed energy when stale");
    });
    test("out of order time and position", [] {
      RouteProgressInput input("test"); auto r=Fixture(); input.Complete(r,r,t+1s);
      input.Complete(r,r,t); Invalid(input.Current(),RouteState::OutOfOrder);
      r.position.latitude_deg.observed_at=t-1s; r.position.longitude_deg.observed_at=t-1s;
      input.Complete(r,r,t+2s); Invalid(input.Current(),RouteState::OutOfOrder);
    });
    test("revision and route identity", [] {
      RouteProgressInput input("session"); auto r=Fixture(); input.Complete(r,r,t); auto s=input.Current();
      input.Complete(r,r,t+1s); Require(input.Current()->route_revision==s->route_revision,"Unchanged geometry retains revision");
      r.route.id="route-B"; input.Complete(r,r,t+2s); Invalid(input.Current(),RouteState::RouteChanged);
      Require(input.Current()->route_revision>s->route_revision,"Identity change advances revision");
      input.Complete(r,r,t+3s); Require(input.Current()->remaining_distance_nm==9,"New coherent route accepted");
    });
    test("no active route", [] {auto r=Fixture();r.route={};Invalid(Evaluate(r),RouteState::NoActiveRoute); });
    test("changed position during upstream pass", [] {Change([](auto& r){r.position.latitude_deg.observed_at=t+1s;},RouteState::PositionChanged); });
    test("selected position must match upstream", [] {auto r=Fixture();r.upstream_latitude_deg=56;Invalid(Evaluate(r),RouteState::PositionMismatch); });
    test("future uncertain and unsourced position", [] {
      auto r=Fixture(); r.position.latitude_deg.observed_at=t+1s; r.position.longitude_deg.observed_at=t+1s; Invalid(Evaluate(r),RouteState::UncertainPosition);
      r=Fixture();r.position.latitude_deg.validity=Validity::Estimated;Invalid(Evaluate(r),RouteState::UncertainPosition);
      r=Fixture();r.position.latitude_deg.source.clear();Invalid(Evaluate(r),RouteState::MissingPosition);
    });
    test("nested event rejects edit-then-restore", [] {auto r=Fixture();r.interrupted=true;Invalid(Evaluate(r),RouteState::InterruptedPass); });
    test("consumer read invalidates without freshening", [] {
      RouteProgressInput input("test"); auto r=Fixture(); input.Complete(r,r,t);
      auto old=input.Current(); input.CheckCurrent(r.route,t+2s); Require(input.Current()==old,"Read must not republish unchanged distance");
      r.route.active_point_id="c";r.route.active_index=2;input.CheckCurrent(r.route,t+3s);
      Invalid(input.Current(),RouteState::ActivePointChanged);Require(input.Current()->observed_at==t,"Invalidation keeps measurement time");
    });
    test("valid zero is explicit geometry not missing route", [] {auto r=Fixture(2);r.range_to_active_nm=0;Require(Evaluate(r)->remaining_distance_nm==0,"Explicit coherent zero supported"); });
    test("position cannot postdate its progress observation", [] {
      auto snapshot=*Evaluate(Fixture());snapshot.position_observed_at=t+1s;
      Require(AssessRoute(snapshot,t+2s).state==RouteState::UncertainPosition,"Inconsistent clocks cannot age into validity");
    });
    test("test-only energy consumption", [] {
      const auto s=Evaluate(Fixture()); smartnav::EnergyInputs e;
      e.distance_remaining_nm=RouteDistanceSample(*s,t);
      e.soc_percent={80,"battery",t,Validity::Measured};e.sog_kn={5,"GPS",t,Validity::Measured};e.total_discharge_kw={2,"net discharge",t,Validity::Measured};
      smartnav::EnergyModel model{20,20,0.5,"synthetic pack"};auto result=smartnav::PredictEnergy(model,e,t);
      Require(result.arrival.estimate && result.arrival.estimate->soc_percent==62,"9 NM fixture consumes existing energy contract");
      e.distance_remaining_nm=RouteDistanceSample(*s,t+5s);Require(!smartnav::PredictEnergy(model,e,t+5s).arrival.estimate,"Stale distance withheld");
    });
    std::cout<<passed<<" route contract scenarios passed; geometry compared in upstream integration tests\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}

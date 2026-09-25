#include "integration/RouteProgressInput.h"
#include "smartnav/Advisories.h"
#include "smartnav/HazardLookAhead.h"
#include "smartnav/VesselEnergy.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace opennav;
using namespace std::chrono_literals;
const vessel::Time epoch{100s};
void Check(bool value, const char *reason) {
  if (!value)
    throw std::runtime_error(reason);
}
vessel::Sample Sample(double v) {
  return {v, "test selected input", epoch, vessel::Validity::Measured};
}
integration::RouteRead Read() {
  integration::RouteRead r;
  r.route.active = true;
  r.route.id = "route";
  r.route.name = "Test passage";
  r.route.active_index = 0;
  r.route.active_point_id = "a";
  r.route.points = {{"a", 10, 179.9, 0, "First", {}},
                    {"b", 10, -179.9, 2, "Second", 32},
                    {"c", 11, -179.9, 3, "Final", 90}};
  r.position.latitude_deg = Sample(10);
  r.position.longitude_deg = Sample(179.8);
  r.upstream_position_valid = true;
  r.upstream_latitude_deg = 10;
  r.upstream_longitude_deg = 179.8;
  r.range_to_active_nm = 1;
  r.bearing_to_active_true_deg = 0;
  return r;
}
vessel::VesselState Fixture() {
  auto r = Read();
  integration::RouteProgressInput input("test");
  input.Complete(r, r, epoch);
  vessel::VesselState s;
  s.navigation = r.position;
  s.navigation.route = input.Current();
  s.navigation.sog_kn = Sample(6);
  s.navigation.cog_deg = Sample(0);
  s.battery.soc_percent = Sample(80);
  s.battery.net_discharge_kw = Sample(2);
  return s;
}
smartnav::NavigationAdvice Advice(const vessel::VesselState &s,
                                  vessel::Time now = epoch) {
  return smartnav::Advise(
      s, smartnav::PredictVesselEnergy({20, 20, .5, "test capacity"}, s, now),
      {}, now);
}
std::size_t Count(const smartnav::NavigationAdvice &a,
                  smartnav::EventKind kind) {
  std::size_t count = 0;
  for (const auto &e : a.events)
    if (e.kind == kind)
      ++count;
  return count;
}
void Turns() {
  auto s = Fixture();
  const auto a = Advice(s);
  Check(a.route_valid && a.route_revision == 1, "Route provenance");
  Check(Count(a, smartnav::EventKind::Turn) == 2, "Two planned turns");
  Check(a.events[0].seconds_from_now == 600 && a.events[0].title == "First",
        "First waypoint timing");
  Check(a.events[1].course_change_deg == 32 &&
            a.events[1].course_true_deg == 32,
        "Next course from stored leg");
  Check(a.events[3].seconds_from_now == 1800 &&
            a.events[3].course_change_deg == 58,
        "Future leg turn timing");
  const auto held = s.navigation.route;
  auto changed = Read();
  changed.route.points[1].incoming_course_true_deg = 50;
  integration::RouteProgressInput input("test");
  input.Complete(Read(), Read(), epoch);
  input.Complete(Read(), changed, epoch + 1s);
  s.navigation.route = input.Current();
  Check(!Advice(s, epoch + 1s).route_valid,
        "Course edit invalidates coherent publication");
  Check(held->remaining_steps[1].course_true_deg == 32,
        "Owned steps survive edit");
  s = Fixture();
  s.navigation.cog_deg = Sample(359);
  auto r = std::make_shared<vessel::RouteProgressSnapshot>(*s.navigation.route);
  r->remaining_steps[1].course_true_deg = 1;
  s.navigation.route = r;
  Check(Advice(s).events[1].course_change_deg == 2,
        "North wrap is shortest course change");
  s.navigation.sog_kn = Sample(.1);
  auto slow = Advice(s);
  Check(slow.route_valid && !slow.events[0].seconds_from_now &&
            Count(slow, smartnav::EventKind::Turn) == 0,
        "Stationary timing withheld");
  s = Fixture();
  s.navigation.cog_deg = {};
  Check(Count(Advice(s), smartnav::EventKind::Turn) == 0,
        "Missing COG withholds turns");
  Check(!Advice(Fixture(), epoch + 5s).route_valid,
        "Retained route cannot become fresh");
  s = Fixture();
  r = std::make_shared<vessel::RouteProgressSnapshot>(*s.navigation.route);
  r->remaining_steps[1].distance_from_previous_nm = 99;
  s.navigation.route = r;
  Check(!Advice(s).route_valid, "Step total mismatch rejects timeline");
  r->remaining_steps[1].distance_from_previous_nm =
      std::numeric_limits<double>::infinity();
  Check(!Advice(s).route_valid, "Nonfinite planned distance rejected");
  s = Fixture();
  r = std::make_shared<vessel::RouteProgressSnapshot>(*s.navigation.route);
  r->remaining_steps[1].waypoint_id = "a";
  s.navigation.route = r;
  Check(!Advice(s).route_valid, "Repeated identity rejects timeline");
  s = Fixture();
  s.navigation.route.reset();
  Check(!Advice(s).route_valid, "Inactive route withheld");
}
void Failures() {
  auto s=Fixture();s.navigation.latitude_deg={};
  Check(!Advice(s).route_valid,"Immediate GPS loss withholds cached route advice");
  s=Fixture();s.navigation.latitude_deg.freshness={500ms,1500ms};
  Check(!Advice(s,epoch+2s).route_valid,"Position source limit stricter than route default");
  s=Fixture();s.navigation.longitude_deg.source="replacement input";
  Check(!Advice(s).route_valid,"Mixed position source rejected");
  s=Fixture();s.navigation.latitude_deg.observed_at+=1s;s.navigation.longitude_deg.observed_at+=1s;
  Check(Advice(s,epoch+1s).route_valid,"Newer coherent selected fix can accompany still-fresh progress");
  s=Fixture();s.navigation.sog_kn.value=0;
  Check(Count(Advice(s),smartnav::EventKind::Turn)==0,"Stopped vessel no turn timing");
  s=Fixture();s.navigation.cog_deg.validity=vessel::Validity::Uncertain;
  Check(Count(Advice(s),smartnav::EventKind::Turn)==0,"Uncertain course no turn angle");
  auto r=std::make_shared<vessel::RouteProgressSnapshot>(*s.navigation.route);
  r->state=vessel::RouteState::ActivePointChanged;s.navigation.route=r;
  Check(!Advice(s).route_valid,"Skipped/advancing waypoint no transient advice");
  r->state=vessel::RouteState::RouteChanged;
  Check(!Advice(s).route_valid,"Edited/reversed route no transient advice");
}
void EnergyAndAis() {
  auto s = Fixture();
  Check(Count(Advice(s), smartnav::EventKind::ArrivalSoc) == 1,
        "Destination energy event");
  s.battery.net_discharge_kw = Sample(19);
  auto a = Advice(s);
  Check(Count(a, smartnav::EventKind::EnergyShortfall) == 1 &&
            Count(a, smartnav::EventKind::Reserve) == 1,
        "Shortfall plus projected reserve");
  s.battery.net_discharge_kw = Sample(13);
  a = Advice(s);
  Check(Count(a, smartnav::EventKind::Reserve) == 2,
        "Reserve destination and crossing events");
  auto e = smartnav::PredictVesselEnergy({20, 20, .5, "test"}, s, epoch);
  auto other = Fixture();
  Check(Count(smartnav::Advise(other, e, {}, epoch),
              smartnav::EventKind::ArrivalSoc) == 0,
        "Cannot mix another route publication");
  Check(Count(smartnav::Advise(s, e, {}, epoch + 1s),
              smartnav::EventKind::ArrivalSoc) == 0,
        "Cached prediction epoch withheld");
  s.battery.soc_percent.observed_at = epoch - 5s;
  Check(Count(Advice(s), smartnav::EventKind::ArrivalSoc) == 0,
        "Stale SOC suppresses energy event");
  vessel::AisState ais;
  ais.available = true;
  ais.observed_at = epoch;
  vessel::AisTarget target;
  target.mmsi = 123456789;
  target.name = "TEST TARGET";
  target.source = "OpenCPN test AIS";
  target.active = true;
  target.upstream_alarm = true;
  target.cpa_nm = Sample(.2);
  target.tcpa_minutes = Sample(4);
  ais.targets.push_back(target);
  a = smartnav::Advise(s, {}, ais, epoch);
  Check(Count(a, smartnav::EventKind::AisEncounter) == 1,
        "Uses upstream AIS alarm and CPA/TCPA");
  Check(a.events.front().seconds_from_now == 240, "AIS CPA time unchanged");
  ais.targets[0].upstream_alarm = false;
  Check(Count(smartnav::Advise(s, {}, ais, epoch),
              smartnav::EventKind::AisEncounter) == 0,
        "No independent alarm threshold");
  ais.targets[0] = target;
  ais.targets[0].lost = true;
  Check(Count(smartnav::Advise(s, {}, ais, epoch),
              smartnav::EventKind::AisEncounter) == 0,
        "Lost AIS withheld");
  ais.targets[0] = target;
  ais.targets[0].tcpa_minutes = Sample(-1);
  Check(Count(smartnav::Advise(s, {}, ais, epoch),
              smartnav::EventKind::AisEncounter) == 0,
        "Past CPA not future event");
  ais.targets[0] = target;
  Check(Count(smartnav::Advise(s, {}, ais, epoch + 5s),
              smartnav::EventKind::AisEncounter) == 0,
        "Stale AIS withheld");
}
class ChartFixture final : public smartnav::IChartCorridor {
public:
  bool wrong_revision = false, missing_datum = false, empty = false, fail = false, excessive = false;
  smartnav::CorridorEvidence
  Inspect(const smartnav::PathCorridor &path) const override {
    if(fail)throw std::runtime_error("fixture provider failed");
    smartnav::CorridorEvidence e{path,
                                 smartnav::Coverage::Complete,
                                 "TEST chart intersection",
                                 "TEST survey",
                                 {}};
    if (wrong_revision)
      ++e.query.route_revision;
    if (!empty)
      e.objects = {
          {"shoal", "TEST depth area", "TEST ENC",
           missing_datum ? "" : "chart datum", 2.4, false},
          {"deep", "TEST depth area", "TEST ENC", "chart datum", 20, false},
          {"obstruction",
           "TEST unknown obstruction",
           "TEST ENC",
           "chart datum",
           {},
           true}};
    if(excessive)e.objects.resize(5000,e.objects.front());
    return e;
  }
};
void Hazards() {
  auto s = Fixture();
  smartnav::HazardConfiguration c{2, .5, 50};
  auto p = smartnav::BuildCorridor(s, c, epoch);
  Check(p && p->path.size() == 4, "Future corridor includes all route points");
  Check(p->path[1].longitude_deg == 179.9 && p->path[2].longitude_deg == -179.9,
        "Antimeridian coordinates copied unchanged");
  ChartFixture f;
  auto a = smartnav::LookAhead(*p, c, f, epoch);
  Check(a.potential_hazards.size() == 2 &&
            a.coverage == smartnav::Coverage::Complete,
        "Draft plus margin and unknown obstruction");
  f.empty = true;
  a = smartnav::LookAhead(*p, c, f, epoch);
  Check(a.message.find("not proof") != std::string::npos,
        "No hazards is never proof of safety");
  f.empty = false;
  f.missing_datum = true;
  Check(smartnav::LookAhead(*p, c, f, epoch).coverage ==
            smartnav::Coverage::Partial,
        "Missing datum degrades coverage");
  f.wrong_revision = true;
  Check(smartnav::LookAhead(*p, c, f, epoch).coverage ==
            smartnav::Coverage::Unavailable,
        "Wrong query provenance rejected");
  smartnav::UnavailableChartCorridor none;
  Check(smartnav::LookAhead(*p, c, none, epoch).coverage ==
            smartnav::Coverage::Unavailable,
        "Unintegrated chart query unavailable");
  Check(!smartnav::BuildCorridor(s, c, epoch + 5s),
        "Stale route has no corridor");
  s.simulated = true;
  Check(!smartnav::BuildCorridor(s, c, epoch),
        "Synthetic trip never queries live chart geometry");
  s = Fixture();s.replayed=true;
  Check(!smartnav::BuildCorridor(s,c,epoch),"Historical playback cannot query current charts");
  s=Fixture();s.navigation.latitude_deg.freshness={500ms,1500ms};
  Check(!smartnav::BuildCorridor(s,c,epoch+2s),"Strict position freshness applies to corridor");
  f.wrong_revision=false;f.missing_datum=false;f.fail=true;
  Check(smartnav::LookAhead(*p,c,f,epoch).coverage==smartnav::Coverage::Unavailable,"Provider failure unavailable");
  f.fail=false;f.excessive=true;
  const auto bounded=smartnav::LookAhead(*p,c,f,epoch);
  Check(bounded.coverage==smartnav::Coverage::Partial && bounded.potential_hazards.size()<=4096,"Oversized chart evidence bounded and partial");
  s = Fixture();
  c.draft_m = std::numeric_limits<double>::quiet_NaN();
  Check(!smartnav::BuildCorridor(s, c, epoch), "Missing draft fails closed");
}
int main(int argc, char **argv) {
  try {
    const std::string group = argc > 1 ? argv[1] : "";
    if (group == "failures")
      Failures();
    else if (group == "turns")
      Turns();
    else if (group == "events")
      EnergyAndAis();
    else if (group == "hazards")
      Hazards();
    else
      throw std::runtime_error("Unknown group");
    std::cout << "PASS " << group << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
